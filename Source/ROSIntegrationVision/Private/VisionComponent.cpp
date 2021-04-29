// Author Tim Fronsee <tfronsee21@gmail.com>

#include "VisionComponent.h"

#include <cmath>
#include <thread>

#include "ROSTime.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

#include "PacketBuffer.h"
#include "ROSIntegrationGameInstance.h"

#include "EngineUtils.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "UObject/ConstructorHelpers.h"

#if PLATFORM_WINDOWS
  #define _USE_MATH_DEFINES
#endif

// Private data container so that internal structures are not visible to the outside
class ROSINTEGRATIONVISION_API UVisionComponent::PrivateData
{
public:
	TSharedPtr<PacketBuffer> Buffer;
	// TCPServer Server;
	std::mutex WaitColor;
	std::condition_variable CVColor;
	std::thread ThreadColor;
	bool DoColor;
};

UVisionComponent::UVisionComponent() :
Width(960),
Height(540),
ServerPort(10000)
{
    Priv = new PrivateData();
    FieldOfView = 90.0;
    PrimaryComponentTick.bCanEverTick = true;

    auto owner = GetOwner();
    if (owner)
    {
        Color = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("ColorCapture"));
        Color->SetupAttachment(this);
        Color->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        Color->TextureTarget = CreateDefaultSubobject<UTextureRenderTarget2D>(TEXT("ColorTarget"));
		Color->TextureTarget->TargetGamma = 2.0;
        Color->TextureTarget->InitAutoFormat(Width, Height);
        Color->FOVAngle = FieldOfView;
    }
    else 
	{
        UE_LOG(LogTemp, Warning, TEXT("No owner!"));
    }
}

UVisionComponent::~UVisionComponent()
{
    delete Priv;
}

void UVisionComponent::Pause(const bool _Pause)
{
    Paused = _Pause;
}

bool UVisionComponent::IsPaused() const
{
    return Paused;
}

void UVisionComponent::InitializeTopics()
{
	// Establish ROS communication
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>
		(GetOwner()->GetGameInstance());

	if (rosinst && rosinst->bConnectToROS)
	{
		CameraInfoPublisher = NewObject<UTopic>(UTopic::StaticClass());
		ImagePublisher = NewObject<UTopic>(UTopic::StaticClass());

		CameraInfoPublisher->Init(rosinst->ROSIntegrationCore, CameraInfoTopicName, TEXT("sensor_msgs/CameraInfo"));
		CameraInfoPublisher->Advertise();

		ImagePublisher->Init(rosinst->ROSIntegrationCore, ImageTopicName, TEXT("sensor_msgs/Image"));
		ImagePublisher->Advertise();
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("UnrealROSInstance not existing."));
	}
}

void UVisionComponent::PublishImages() {
	// Check if paused
	if (Paused) {
		return;
	}

	FROSTime time = FROSTime::Now();

	if (ImagePublisher && ImagePublisher->IsAdvertising()) {
		auto owner = GetOwner();
		owner->UpdateComponentTransforms();

		FDateTime Now = FDateTime::UtcNow();
		Priv->Buffer->HeaderWrite->TimestampCapture = Now.ToUnixTimestamp() * 1000000000 + Now.GetMillisecond() * 1000000;

		FVector Translation = GetComponentLocation();
		FQuat Rotation = GetComponentQuat();
		// Convert to meters and ROS coordinate system
		Priv->Buffer->HeaderWrite->Translation.X = Translation.X / 100.0f;
		Priv->Buffer->HeaderWrite->Translation.Y = -Translation.Y / 100.0f;
		Priv->Buffer->HeaderWrite->Translation.Z = Translation.Z / 100.0f;
		Priv->Buffer->HeaderWrite->Rotation.X = -Rotation.X;
		Priv->Buffer->HeaderWrite->Rotation.Y = Rotation.Y;
		Priv->Buffer->HeaderWrite->Rotation.Z = -Rotation.Z;
		Priv->Buffer->HeaderWrite->Rotation.W = Rotation.W;

		// Read color image and notify processing thread
		Priv->WaitColor.lock();
		ReadImage(Color->TextureTarget, ImageColor);
		Priv->WaitColor.unlock();
		Priv->DoColor = true;
		Priv->CVColor.notify_one();

		Priv->Buffer->StartReading();
		uint32_t xSize = Priv->Buffer->HeaderRead->Size;
		uint32_t xSizeHeader = Priv->Buffer->HeaderRead->SizeHeader; // Size of the header
		uint32_t xWidth = Priv->Buffer->HeaderRead->Width; // Width of the images
		uint32_t xHeight = Priv->Buffer->HeaderRead->Height; // Height of the images
		uint32_t xBytes = Priv->Buffer->HeaderRead->Bytes;

		// Get the data offsets for the different types of images that are in the buffer
		const uint32_t& OffsetColor = Priv->Buffer->OffsetImage;
		UE_LOG(LogTemp, Verbose, TEXT("Buffer Offsets: %d"), OffsetColor);

		TSharedPtr<ROSMessages::sensor_msgs::Image> ImageMessage(new ROSMessages::sensor_msgs::Image());

		ImageMessage->header.seq = 0;
		ImageMessage->header.time = time;
		ImageMessage->header.frame_id = ImageOpticalFrame;
		ImageMessage->height = Height;
		ImageMessage->width = Width;
		ImageMessage->encoding = TEXT("bgr8");
		ImageMessage->step = Width * xBytes;
		ImageMessage->data = &Priv->Buffer->Read[OffsetColor];
		ImagePublisher->Publish(ImageMessage);

		Priv->Buffer->DoneReading();
	}

	// Construct and publish CameraInfo
	if (CameraInfoPublisher && CameraInfoPublisher->IsAdvertising()) {
		const float FOVX = Height > Width ? FieldOfView * Width / Height : FieldOfView;
		const float FOVY = Width > Height ? FieldOfView * Height / Width : FieldOfView;
		double halfFOVX = FOVX * PI / 360.0; // was M_PI on gcc
		double halfFOVY = FOVY * PI / 360.0; // was M_PI on gcc
		const double cX = Width / 2.0;
		const double cY = Height / 2.0;

		const double K0 = cX / std::tan(halfFOVX);
		const double K2 = cX;
		const double K4 = K0;
		const double K5 = cY;
		const double K8 = 1;

		const double P0 = K0;
		const double P2 = K2;
		const double P5 = K4;
		const double P6 = K5;
		const double P10 = 1;

		TSharedPtr<ROSMessages::sensor_msgs::CameraInfo> CamInfo(new ROSMessages::sensor_msgs::CameraInfo());
		CamInfo->header.seq = 0;
		CamInfo->header.time = time;
		CamInfo->header.frame_id = ImageOpticalFrame;
		CamInfo->height = Height;
		CamInfo->width = Width;
		CamInfo->distortion_model = TEXT("plumb_bob");
		CamInfo->D[0] = 0;
		CamInfo->D[1] = 0;
		CamInfo->D[2] = 0;
		CamInfo->D[3] = 0;
		CamInfo->D[4] = 0;

		CamInfo->K[0] = K0;
		CamInfo->K[1] = 0;
		CamInfo->K[2] = K2;
		CamInfo->K[3] = 0;
		CamInfo->K[4] = K4;
		CamInfo->K[5] = K5;
		CamInfo->K[6] = 0;
		CamInfo->K[7] = 0;
		CamInfo->K[8] = K8;

		CamInfo->R[0] = 1;
		CamInfo->R[1] = 0;
		CamInfo->R[2] = 0;
		CamInfo->R[3] = 0;
		CamInfo->R[4] = 1;
		CamInfo->R[5] = 0;
		CamInfo->R[6] = 0;
		CamInfo->R[7] = 0;
		CamInfo->R[8] = 1;

		CamInfo->P[0] = P0;
		CamInfo->P[1] = 0;
		CamInfo->P[2] = P2;
		CamInfo->P[3] = P0 * TranslateX;
		CamInfo->P[4] = 0;
		CamInfo->P[5] = P5;
		CamInfo->P[6] = P6;
		CamInfo->P[7] = 0;
		CamInfo->P[8] = 0;
		CamInfo->P[9] = 0;
		CamInfo->P[10] = P10;
		CamInfo->P[11] = 0;

		CamInfo->binning_x = 0;
		CamInfo->binning_y = 0;

		CamInfo->roi.x_offset = 0;
		CamInfo->roi.y_offset = 0;
		CamInfo->roi.height = 0;
		CamInfo->roi.width = 0;
		CamInfo->roi.do_rectify = false;

		CameraInfoPublisher->Publish(CamInfo);
	}
}

void UVisionComponent::InitializeComponent()
{
    Super::InitializeComponent();
}

void UVisionComponent::BeginPlay()
{
	Super::BeginPlay();
    // Initializing buffers for reading images from the GPU
	ImageColor.AddUninitialized(Width * Height);

	// Reinit renderer
	Color->TextureTarget->InitAutoFormat(Width, Height);

	AspectRatio = Width / (float)Height;

	// Setting flags for each camera
	ShowFlagsLit(Color->ShowFlags);

	// Creating double buffer and setting the pointer of the server object
	Priv->Buffer = TSharedPtr<PacketBuffer>(new PacketBuffer(Width, Height, 3, FieldOfView));

	Running = true;
	Paused = false;

	Priv->DoColor = false;

	// Starting threads to process image data
	Priv->ThreadColor = std::thread(&UVisionComponent::ProcessColor, this);
}

void UVisionComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *TickFunction)
{
    Super::TickComponent(DeltaTime, TickType, TickFunction);
}

void UVisionComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
    Running = false;

    // Stopping processing threads
	Priv->DoColor = true;
    Priv->CVColor.notify_one();

    Priv->ThreadColor.join();
}

void UVisionComponent::ShowFlagsBasicSetting(FEngineShowFlags &ShowFlags) const
{
	ShowFlags = FEngineShowFlags(EShowFlagInitMode::ESFIM_All0);
	ShowFlags.SetRendering(true);
	ShowFlags.SetStaticMeshes(true);
	ShowFlags.SetLandscape(true);
	ShowFlags.SetInstancedFoliage(true);
	ShowFlags.SetInstancedGrass(true);
	ShowFlags.SetInstancedStaticMeshes(true);
}

void UVisionComponent::ShowFlagsLit(FEngineShowFlags &ShowFlags) const
{
	ShowFlagsBasicSetting(ShowFlags);
	ShowFlags = FEngineShowFlags(EShowFlagInitMode::ESFIM_Game);
	ApplyViewMode(VMI_Lit, true, ShowFlags);
	ShowFlags.SetMaterials(true);
	ShowFlags.SetLighting(true);
	ShowFlags.SetPostProcessing(true);
	// ToneMapper needs to be enabled, otherwise the screen will be very dark
	ShowFlags.SetTonemapper(true);
	// TemporalAA needs to be disabled, otherwise the previous frame might contaminate current frame.
	// Check: https://answers.unrealengine.com/questions/436060/low-quality-screenshot-after-setting-the-actor-pos.html for detail
	ShowFlags.SetTemporalAA(false);
	ShowFlags.SetAntiAliasing(true);
	ShowFlags.SetEyeAdaptation(false); // Eye adaption is a slow temporal procedure, not useful for image capture
}

void UVisionComponent::ReadImage(UTextureRenderTarget2D *RenderTarget, TArray<FFloat16Color> &ImageData) const
{
	FTextureRenderTargetResource *RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
	RenderTargetResource->ReadFloat16Pixels(ImageData);
}

void UVisionComponent::ToColorImage(const TArray<FFloat16Color> &ImageData, uint8 *Bytes) const
{
	const FFloat16Color *itI = ImageData.GetData();
	uint8_t *itO = Bytes;

	// Converts Float colors to bytes
	for (size_t i = 0; i < ImageData.Num(); ++i, ++itI, ++itO)
	{
		*itO = (uint8_t)std::round((float)itI->B * 255.f);
		*++itO = (uint8_t)std::round((float)itI->G * 255.f);
		*++itO = (uint8_t)std::round((float)itI->R * 255.f);
	}
	return;
}

void UVisionComponent::ProcessColor()
{
	while (true)
	{
		std::unique_lock<std::mutex> WaitLock(Priv->WaitColor);
		Priv->CVColor.wait(WaitLock, [this] {return Priv->DoColor; });
		Priv->DoColor = false;
		if (!this->Running) break;
		ToColorImage(ImageColor, Priv->Buffer->Image);

		// Complete Buffer
		Priv->Buffer->DoneWriting();
	}
}
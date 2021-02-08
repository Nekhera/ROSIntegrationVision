// Fill out your copyright notice in the Description page of Project Settings.

#include "DepthComponent.h"

#include <cmath>

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
class ROSINTEGRATIONVISION_API UDepthComponent::PrivateData
{
public:
	TSharedPtr<PacketBuffer> Buffer;
	// TCPServer Server;
	std::mutex WaitDepth;
	std::condition_variable CVDepth;
	std::thread ThreadDepth;
	bool DoDepth;
};

UDepthComponent::UDepthComponent() :
Width(480),
Height(270),
ServerPort(10000)
{
    Priv = new PrivateData();
    FieldOfView = 90.0;
    PrimaryComponentTick.bCanEverTick = true;

    auto owner = GetOwner();
    if (owner)
    {
        Depth = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("DepthCapture"));
        Depth->SetupAttachment(this);
        Depth->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
        Depth->TextureTarget = CreateDefaultSubobject<UTextureRenderTarget2D>(TEXT("DepthTarget"));
        Depth->TextureTarget->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA16f;
        Depth->TextureTarget->InitAutoFormat(Width, Height);
        Depth->FOVAngle = FieldOfView;
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("No owner!"));
    }
}

UDepthComponent::~UDepthComponent()
{
    delete Priv;
}

void UDepthComponent::Pause(const bool _Pause)
{
    Paused = _Pause;
}

bool UDepthComponent::IsPaused() const
{
    return Paused;
}

void UDepthComponent::InitializeTopics()
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

void UDepthComponent::PublishImages() {
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

		Priv->WaitDepth.lock();
		ReadImage(Depth->TextureTarget, ImageDepth);
		Priv->WaitDepth.unlock();
		Priv->DoDepth = true;
		Priv->CVDepth.notify_one();

		Priv->Buffer->StartReading();
		const uint32_t& OffsetDepth = Priv->Buffer->OffsetImage;

		// * - Depth image data (width * height * 2 Bytes (Float16))
		uint8_t* DepthPtr = &Priv->Buffer->Read[OffsetDepth];
		uint32_t TargetDepthBufSize = Width * Height * 4;
		uint8_t* TargetDepthBuf = new uint8_t[TargetDepthBufSize]; // Allocate a byte for every pixel * 4 Bytes for a single 32Bit Float

		convertDepth((uint16_t*)DepthPtr, (__m128*)TargetDepthBuf);
		// convertDepth((uint16_t *)packet.pDepth, (__m128*)&msgDepth->data[0]);
		UE_LOG(LogTemp, Verbose, TEXT("Buffer Offsets: %d"), OffsetDepth);

		TSharedPtr<ROSMessages::sensor_msgs::Image> DepthMessage(new ROSMessages::sensor_msgs::Image());

		DepthMessage->header.seq = 0;
		DepthMessage->header.time = time;
		DepthMessage->header.frame_id = ImageOpticalFrame;
		DepthMessage->height = Height;
		DepthMessage->width = Width;
		DepthMessage->encoding = TEXT("32FC1");
		DepthMessage->step = Width * 4;
		DepthMessage->data = TargetDepthBuf;
		ImagePublisher->Publish(DepthMessage);

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
		CamInfo->P[3] = P0 * 0.08;
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

void UDepthComponent::InitializeComponent()
{
	Super::InitializeComponent();
}

void UDepthComponent::BeginPlay()
{
	Super::BeginPlay();
	// Initializing buffers for reading images from the GPU
	ImageDepth.AddUninitialized(Width * Height);

	// Reinit renderer
	Depth->TextureTarget->InitAutoFormat(Width, Height);

	AspectRatio = Width / (float)Height;

	// Creating double buffer and setting the pointer of the server object
	Priv->Buffer = TSharedPtr<PacketBuffer>(new PacketBuffer(Width, Height, 2, FieldOfView));

	Running = true;
	Paused = false;

	Priv->DoDepth = false;

	// Starting threads to process image data
	Priv->ThreadDepth = std::thread(&UDepthComponent::ProcessDepth, this);
}

void UDepthComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* TickFunction)
{
	Super::TickComponent(DeltaTime, TickType, TickFunction);
}

void UDepthComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
	Running = false;

	// Stopping processing threads
	Priv->DoDepth = true;
	Priv->CVDepth.notify_one();

	Priv->ThreadDepth.join();
}

void UDepthComponent::ReadImage(UTextureRenderTarget2D* RenderTarget, TArray<FFloat16Color>& ImageData) const
{
	FTextureRenderTargetResource* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
	RenderTargetResource->ReadFloat16Pixels(ImageData);
}

void UDepthComponent::ToDepthImage(const TArray<FFloat16Color>& ImageData, uint8* Bytes) const
{
	const FFloat16Color* itI = ImageData.GetData();
	uint16_t* itO = reinterpret_cast<uint16_t*>(Bytes);

	// Just copies the encoded Float16 values
	for (size_t i = 0; i < ImageData.Num(); ++i, ++itI, ++itO)
	{
		*itO = itI->R.Encoded;
	}
	return;
}

void UDepthComponent::ProcessDepth()
{
	while (true)
	{
		std::unique_lock<std::mutex> WaitLock(Priv->WaitDepth);
		Priv->CVDepth.wait(WaitLock, [this] {return Priv->DoDepth; });
		Priv->DoDepth = false;
		if (!this->Running) break;
		ToDepthImage(ImageDepth, Priv->Buffer->Image);

		// Complete Buffer
		Priv->Buffer->DoneWriting();
	}
}

void UDepthComponent::convertDepth(const uint16_t* in, __m128* out) const
{
	const size_t size = (Width * Height) / 4;
	for (size_t i = 0; i < size; ++i, in += 4, ++out)
	{
		// Divide by 100 here in order to convert UU (cm) into ROS units (m)
		*out = _mm_cvtph_ps(
			_mm_div_epi16(
				_mm_set_epi16(0, 0, 0, 0, *(in + 3), *(in + 2), *(in + 1), *(in + 0)),
				_mm_set_epi16(100, 100, 100, 100, 100, 100, 100, 100)
			)
		);// / 100;
	}
}
#pragma once 

#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"

#include "RI/Topic.h"

#include "DepthComponent.generated.h"

UCLASS()
class ROSINTEGRATIONVISION_API UDepthComponent : public UCameraComponent {

    GENERATED_BODY()

public:

    UDepthComponent();
    ~UDepthComponent();
    void Pause(const bool _Pause = true);
    bool IsPaused() const;

    UFUNCTION(BlueprintCallable, Category = "ROS")
        void InitializeTopics();
    UFUNCTION(BlueprintCallable, Category = "ROS")
        void PublishImages();

    UPROPERTY(EditAnywhere, Category = "Depth Component")
        uint32 Width;
    UPROPERTY(EditAnywhere, Category = "Depth Component")
        uint32 Height;
    UPROPERTY(EditAnywhere, Category = "Depth Component")
        int32 ServerPort;

    UPROPERTY(Transient, EditAnywhere, BlueprintReadWrite, Category = "Depth Component")
        USceneCaptureComponent2D* Depth;

    UPROPERTY(BlueprintReadWrite, Category = "Depth Component")
        FString ImageFrame = TEXT("camera_frame");
    UPROPERTY(BlueprintReadWrite, Category = "Depth Component")
        FString ImageOpticalFrame = TEXT("camera_frame_optical");

    UPROPERTY(EditAnywhere, Category = "Depth Component")
        FString CameraInfoTopicName = TEXT("/unreal_ros/camera_info");
    UPROPERTY(EditAnywhere, Category = "Depth Component")
        FString ImageTopicName = TEXT("/unreal_ros/image_depth");

    UPROPERTY(Transient, EditAnywhere, Category = "Depth Component")
        UTopic* CameraInfoPublisher;
    UPROPERTY(Transient, EditAnywhere, Category = "Depth Component")
        UTopic* ImagePublisher;

protected:

    virtual void InitializeComponent() override;
    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime,
        enum ELevelTick TickType,
        FActorComponentTickFunction* TickFunction) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:

    // Private data container
    class PrivateData;
    PrivateData* Priv;

    UMaterialInstanceDynamic* MaterialDepthInstance;

    TArray<FFloat16Color> ImageDepth;
    TArray<uint8> DataDepth;
    bool Running, Paused;

    void ReadImage(UTextureRenderTarget2D* RenderTarget, TArray<FFloat16Color>& ImageData) const;
    void ToDepthImage(const TArray<FFloat16Color>& ImageData, uint8* Bytes) const;
    void ProcessDepth();
    // in must hold Width*Height*2(float) Bytes
    void convertDepth(const uint16_t* in, __m128* out) const;
};

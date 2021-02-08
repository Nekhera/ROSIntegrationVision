// Author: Tim Fronsee <tfronsee21@gmail.com> 
#pragma once 

#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"

#include "RI/Topic.h"

#include "VisionComponent.generated.h"

UCLASS()
class ROSINTEGRATIONVISION_API UVisionComponent : public UCameraComponent {
    
    GENERATED_BODY()

public:
    UVisionComponent();
    ~UVisionComponent();
    void Pause(const bool _Pause = true);
    bool IsPaused() const;

    UFUNCTION(BlueprintCallable, Category = "ROS")
        void InitializeTopics();
    UFUNCTION(BlueprintCallable, Category = "ROS")
        void PublishImages();

    UPROPERTY(EditAnywhere, Category = "Vision Component")
        float TranslateX;
    UPROPERTY(EditAnywhere, Category = "Vision Component")
        uint32 Width;
    UPROPERTY(EditAnywhere, Category = "Vision Component")
        uint32 Height;
    UPROPERTY(EditAnywhere, Category = "Vision Component")
        int32 ServerPort;

    // The cameras for color, depth and objects;
    UPROPERTY(Transient, EditAnywhere, BlueprintReadWrite, Category = "Vision Component")
        USceneCaptureComponent2D* Color;

    UPROPERTY(BlueprintReadWrite, Category = "Vision Component")
        FString ImageFrame = TEXT("camera_frame");
    UPROPERTY(BlueprintReadWrite, Category = "Vision Component")
        FString ImageOpticalFrame = TEXT("camera_frame_optical");

    UPROPERTY(EditAnywhere, Category = "Vision Component")
        FString CameraInfoTopicName = TEXT("/unreal_ros/camera_info");
    UPROPERTY(EditAnywhere, Category = "Vision Component")
        FString ImageTopicName = TEXT("/unreal_ros/image_color");

    UPROPERTY(Transient, EditAnywhere, Category = "Vision Component")
        UTopic* CameraInfoPublisher;
    UPROPERTY(Transient, EditAnywhere, Category = "Vision Component")
        UTopic* ImagePublisher;

protected:
  
    virtual void InitializeComponent() override;
    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, 
                               enum ELevelTick TickType,
                               FActorComponentTickFunction *TickFunction) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    
    // Private data container
    class PrivateData;
    PrivateData *Priv;
  
    TArray<FFloat16Color> ImageColor;
    TArray<uint8> DataColor;
    bool Running, Paused;
  
    void ShowFlagsBasicSetting(FEngineShowFlags &ShowFlags) const;
    void ShowFlagsLit(FEngineShowFlags &ShowFlags) const;
    void ReadImage(UTextureRenderTarget2D *RenderTarget, TArray<FFloat16Color> &ImageData) const;
    void ToColorImage(const TArray<FFloat16Color> &ImageData, uint8 *Bytes) const;
    void ProcessColor();

};

// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "VisionComponent.h"

#include "VisionActor.generated.h"

UCLASS()
class ROSINTEGRATIONVISION_API AVisionActor : public AActor
{
	GENERATED_BODY()

public:
	AVisionActor();
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vision Actor")
	UVisionComponent* vision;
};

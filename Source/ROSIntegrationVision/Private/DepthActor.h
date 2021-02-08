// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "DepthComponent.h"

#include "DepthActor.generated.h"

UCLASS()
class ROSINTEGRATIONVISION_API ADepthActor : public AActor
{
	GENERATED_BODY()

public:
	ADepthActor();
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Depth Actor")
	UDepthComponent* depth;
};

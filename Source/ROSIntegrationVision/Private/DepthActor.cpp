// Fill out your copyright notice in the Description page of Project Settings.

#include "DepthActor.h"
#include "ROSIntegrationGameInstance.h"

// Sets default values
ADepthActor::ADepthActor() : AActor()
{
    UE_LOG(LogTemp, Warning, TEXT("DepthActor CTOR"));

    // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = false;

    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(RootComponent);

    depth = CreateDefaultSubobject<UDepthComponent>(TEXT("Depth"));
    depth->SetupAttachment(RootComponent);
}

// Called when the game starts or when spawned
void ADepthActor::BeginPlay()
{
    Super::BeginPlay();
}

// Called every frame
void ADepthActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}
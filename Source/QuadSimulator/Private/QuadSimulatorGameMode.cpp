// Copyright 1998-2014 Epic Games, Inc. All Rights Reserved.

#include "QuadSimulator.h"
#include "QuadSimulatorGameMode.h"
#include "QuadSimulatorPawn.h"

AQuadSimulatorGameMode::AQuadSimulatorGameMode(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// set default pawn class to our flying pawn
	DefaultPawnClass = AQuadSimulatorPawn::StaticClass();
}

// Copyright 1998-2014 Epic Games, Inc. All Rights Reserved.

#include "QuadSimulator.h"
#include "QuadSimulatorPawn.h"

AQuadSimulatorPawn::AQuadSimulatorPawn(const FObjectInitializer& ObjectInitializer) 
	: Super(ObjectInitializer)
{
	// Structure to hold one-time initialization
	struct FConstructorStatics
	{
		ConstructorHelpers::FObjectFinderOptional<UStaticMesh> PlaneMesh;
		FConstructorStatics()
			: PlaneMesh(TEXT("/Game/Meshes/Quad/QuadModel"))
		{
		}
	};
	static FConstructorStatics ConstructorStatics;

	// Create static mesh component
	PlaneMesh = ObjectInitializer.CreateDefaultSubobject<UStaticMeshComponent>(this, TEXT("PlaneMesh0"));
	PlaneMesh->SetStaticMesh(ConstructorStatics.PlaneMesh.Get());
	RootComponent = PlaneMesh;

	// Create a spring arm component
	SpringArm = ObjectInitializer.CreateDefaultSubobject<USpringArmComponent>(this, TEXT("SpringArm0"));
	SpringArm->AttachTo(RootComponent);
	SpringArm->TargetArmLength = 160.0f; // The camera follows at this distance behind the character	
	SpringArm->SocketOffset = FVector(0.f,0.f,60.f);
	SpringArm->bEnableCameraLag = false;
	SpringArm->CameraLagSpeed = 15.f;

	// Create camera component 
	Camera = ObjectInitializer.CreateDefaultSubobject<UCameraComponent>(this, TEXT("Camera0"));
	Camera->AttachTo(SpringArm, USpringArmComponent::SocketName);
	Camera->bUsePawnControlRotation = false; // Don't rotate camera with controller

	// Set handling parameters
	Acceleration = 500.f;
	TurnSpeed = 50.f;
	MaxSpeed = 4000.f;
	MinSpeed = 500.f;
	CurrentForwardSpeed = 500.f;

}



void AQuadSimulatorPawn::PreInitializeComponents()
{
	Super::PreInitializeComponents();

	initializePhysicsParameters();
	initializeQuadParameters();
	applyInitialConditions();
	
	// Schedule the dynamics update rate
	GetWorld()->GetTimerManager().SetTimer(this, &AQuadSimulatorPawn::UpdateDynamics, DynamicsUpdateRate, true);
}

void AQuadSimulatorPawn::initializePhysicsParameters()
{
	// 50 Hz for the dynamics update rate
	DynamicsUpdateRate = 0.02f;
	G = 9.8f;
	GravAccel_IF.Set(0.f, 0.f, -1.f*G);
}

void AQuadSimulatorPawn::initializeQuadParameters()
{
	Mass = 0.2f;
	Thrust = 0.f;
	ThrustCoefficient = 9.8f;
	DragCoefficientX = 0.1f; DragCoefficientY = 0.1f; DragCoefficientZ = 0.1f;
	//DragCoefficientX = 0.f; DragCoefficientY = 0.f; DragCoefficientZ = 0.f;
	DragCoefficientMatrix.M[0][0] = DragCoefficientX; DragCoefficientMatrix.M[1][1] = DragCoefficientY; DragCoefficientMatrix.M[2][2] = DragCoefficientZ;
	AngularPosition0_IF.Set(FMath::DegreesToRadians(-15.f), FMath::DegreesToRadians(0.f), FMath::DegreesToRadians(0.f));
	LinearAccel_IF.Set(0.f, 0.f, 0.f);
	LinearVel0_IF.Set(0.f, 0.f, 0.f);
	// Location is in centimeters?
	Position0_IF = GetActorLocation()/100.f;
}

void AQuadSimulatorPawn::applyInitialConditions()
{
	Throttle = 0.f;
	AngularPosition_IF = AngularPosition0_IF;
	LinearVel_IF = LinearVel0_IF;
	Position_IF = Position0_IF;
}

void AQuadSimulatorPawn::UpdateDynamics()
{
	Roll = AngularPosition_IF.X; Pitch = AngularPosition_IF.Y; Yaw = AngularPosition_IF.Z;

	// Calculate the thrust from throtlle input.
	Thrust = ThrustCoefficient*Throttle;
	FVector ThrustAccel_IF = calculateThrustAccel(Thrust);

	// Calculate drag force acceleration. Should move this to a function, and also have a re-usable convert 
	// from 4 vector to 3 vector by ignoring the W component.
	FVector4 DragAccel_IF_Vector4 = DragCoefficientMatrix.TransformVector(LinearVel_IF);
	DragAccel_IF.X = DragAccel_IF_Vector4.X; DragAccel_IF.Y = DragAccel_IF_Vector4.Y; DragAccel_IF.Z = DragAccel_IF_Vector4.Z;

	// Calculate the linear accleration from gravitational accel, thrust accel, and drag accel.
	LinearAccel_IF = GravAccel_IF + ThrustAccel_IF - (1/Mass)*DragAccel_IF;
	UE_LOG(LogTemp, Warning, TEXT("Quad Accel X: %f"), LinearAccel_IF.X);
	UE_LOG(LogTemp, Warning, TEXT("Quad Accel Y: %f"), LinearAccel_IF.Y);
	UE_LOG(LogTemp, Warning, TEXT("Quad Accel Z: %f"), LinearAccel_IF.Z);
	// Dirty hack to stop falling below ground
	if (Position_IF.Z <= 0.f && LinearAccel_IF.Z <= 0.f)
	{
		Position_IF.Z = 0.f;
	}
	else
	{
		LinearVel_IF += LinearAccel_IF*DynamicsUpdateRate;
		Position_IF += LinearVel_IF*DynamicsUpdateRate;
	}

	SetActorLocation(Position_IF*100.f);

	FRotator QuadRotation(FMath::RadiansToDegrees(Pitch), FMath::RadiansToDegrees(Yaw), FMath::RadiansToDegrees(Roll));
	UE_LOG(LogTemp, Warning, TEXT("Pitch Math Frame :  %f"), QuadRotation.Pitch);
	UE_LOG(LogTemp, Warning, TEXT("Quad Pitch Sim Frame :  %f"), convertToSimRotation(QuadRotation).Pitch);
//	AddActorWorldRotation(convertToSimRotation(QuadRotation));
	SetActorRotation(convertToSimRotation(QuadRotation));
}

FVector AQuadSimulatorPawn::calculateThrustAccel(float thrust)
{
	float specificThrust = thrust / Mass;
	FVector thrustVectorInIF;
	thrustVectorInIF.X = FMath::Cos(Yaw)*FMath::Sin(Pitch)*FMath::Cos(Roll) + FMath::Sin(Yaw)*FMath::Sin(Roll);
	thrustVectorInIF.Y = FMath::Sin(Yaw)*FMath::Sin(Pitch)*FMath::Cos(Roll) - FMath::Cos(Yaw)*FMath::Sin(Roll);
	thrustVectorInIF.Z = FMath::Cos(Pitch)*FMath::Cos(Roll);
	return specificThrust*thrustVectorInIF;
}

FRotator AQuadSimulatorPawn::convertToSimRotation(FRotator rotatorMathFrame)
{
	FRotator rotatorSimFrame;
	rotatorSimFrame.Roll = -1.0f*rotatorMathFrame.Roll;
	rotatorSimFrame.Pitch = -1.0f*rotatorMathFrame.Pitch;
	rotatorSimFrame.Yaw = -1.0f*rotatorMathFrame.Yaw;
	return rotatorSimFrame;
}

void AQuadSimulatorPawn::Tick(float DeltaSeconds)
{

	/**
	const FVector LocalMove = FVector(CurrentForwardSpeed * DeltaSeconds, 0.f, 0.f);

	// Move plan forwards (with sweep so we stop when we collide with things)
	AddActorLocalOffset(LocalMove, true);

	// Calculate change in rotation this frame
	FRotator DeltaRotation(0,0,0);
	DeltaRotation.Pitch = CurrentPitchSpeed * DeltaSeconds;
	DeltaRotation.Yaw = CurrentYawSpeed * DeltaSeconds;
	DeltaRotation.Roll = CurrentRollSpeed * DeltaSeconds;


	// Rotate plane
	AddActorLocalRotation(DeltaRotation);

	*/

	// Call any parent class Tick implementation
	Super::Tick(DeltaSeconds);
}

void AQuadSimulatorPawn::ReceiveHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
	Super::ReceiveHit(MyComp, Other, OtherComp, bSelfMoved, HitLocation, HitNormal, NormalImpulse, Hit);

	// Set velocity to zero upon collision
	CurrentForwardSpeed = 0.f;
}


void AQuadSimulatorPawn::SetupPlayerInputComponent(class UInputComponent* InputComponent)
{
	check(InputComponent);

	// Bind our control axis' to callback functions
	InputComponent->BindAxis("Thrust", this, &AQuadSimulatorPawn::ThrustInput);
	InputComponent->BindAxis("MoveUp", this, &AQuadSimulatorPawn::MoveUpInput);
	InputComponent->BindAxis("MoveRight", this, &AQuadSimulatorPawn::MoveRightInput);
}

void AQuadSimulatorPawn::ThrustInput(float Val)
{
	// Is there no input?
	bool bHasInput = !FMath::IsNearlyEqual(Val, 0.f);
	// If input is not held down, reduce speed
	float CurrentAcc = bHasInput ? (Val * Acceleration) : (-0.5f * Acceleration);
	// Calculate new speed
	float NewForwardSpeed = CurrentForwardSpeed + (GetWorld()->GetDeltaSeconds() * CurrentAcc);
	// Clamp between MinSpeed and MaxSpeed
	CurrentForwardSpeed = FMath::Clamp(NewForwardSpeed, MinSpeed, MaxSpeed);

	Throttle = Val;
}

void AQuadSimulatorPawn::MoveUpInput(float Val)
{
	// Target pitch speed is based in input
	float TargetPitchSpeed = (Val * TurnSpeed * -1.f);

	// When steering, we decrease pitch slightly
	TargetPitchSpeed += (FMath::Abs(CurrentYawSpeed) * -0.2f);

	// Smoothly interpolate to target pitch speed
	CurrentPitchSpeed = FMath::FInterpTo(CurrentPitchSpeed, TargetPitchSpeed, GetWorld()->GetDeltaSeconds(), 2.f);
}

void AQuadSimulatorPawn::MoveRightInput(float Val)
{
	// Target yaw speed is based on input
	float TargetYawSpeed = (Val * TurnSpeed);

	// Smoothly interpolate to target yaw speed
	CurrentYawSpeed = FMath::FInterpTo(CurrentYawSpeed, TargetYawSpeed, GetWorld()->GetDeltaSeconds(), 2.f);

	// Is there any left/right input?
	const bool bIsTurning = FMath::Abs(Val) > 0.2f;

	// If turning, yaw value is used to influence roll
	// If not turning, roll to reverse current roll value
	float TargetRollSpeed = bIsTurning ? (CurrentYawSpeed * 0.5f) : (GetActorRotation().Roll * -2.f);

	// Smoothly interpolate roll speed
	CurrentRollSpeed = FMath::FInterpTo(CurrentRollSpeed, TargetRollSpeed, GetWorld()->GetDeltaSeconds(), 2.f);
}
// Copyright 1998-2014 Epic Games, Inc. All Rights Reserved.
#pragma once
#include "GameFramework/Pawn.h"
#include "Engine.h"
#include "QuadSimulatorPawn.generated.h"


UCLASS(config=Game)
class AQuadSimulatorPawn : public APawn
{
	GENERATED_BODY()

	/** StaticMesh component that will be the visuals for our flying pawn */
	UPROPERTY(Category = Mesh, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	class UStaticMeshComponent* PlaneMesh;

	/** Spring arm that will offset the camera */
	UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	class USpringArmComponent* SpringArm;

	/** Camera component that will be our viewpoint */
	UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	class UCameraComponent* Camera;
public:
	AQuadSimulatorPawn(const FObjectInitializer& ObjectInitializer);

	// Begin AActor overrides
	virtual void PreInitializeComponents() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void ReceiveHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;
	// End AActor overrides

protected:

	// Begin APawn overrides
	virtual void SetupPlayerInputComponent(class UInputComponent* InputComponent) override; // Allows binding actions/axes to functions
	// End APawn overrides

	/** Bound to the thrust axis */
	void ThrustInput(float Val);
	
	/** Bound to the vertical axis */
	void MoveUpInput(float Val);

	/** Bound to the horizontal axis */
	void MoveRightInput(float Val);

private:


	void UpdateDynamics();

	/** Function called once to initialize parameters related to the math/physics model */
	void initializePhysicsParameters();

	/** Function called once to initialize physical parameters related to the quad */
	void initializeQuadParameters();

	/** Function called once at initialization to apply any initial conditions to the quad */
	void applyInitialConditions();

	/** Calculates the inertial frame acceleration due to the quad copter's thrust. */
	FVector calculateThrustAccel(float thrust);

	/** How quickly forward speed changes */
	UPROPERTY(Category=Plane, EditAnywhere)
	float Acceleration;

	/** How quickly pawn can steer */
	UPROPERTY(Category=Plane, EditAnywhere)
	float TurnSpeed;

	/** Max forward speed */
	UPROPERTY(Category = Pitch, EditAnywhere)
	float MaxSpeed;

	/** Min forward speed */
	UPROPERTY(Category=Yaw, EditAnywhere)
	float MinSpeed;

	/** Rate at which the dynamics is calculated at in seconds */
	float DynamicsUpdateRate;

	/** Current forward speed */
	float CurrentForwardSpeed;

	/** Current yaw speed */
	float CurrentYawSpeed;

	/** Current pitch speed */
	float CurrentPitchSpeed;

	/** Current roll speed */
	float CurrentRollSpeed;

	/** Quad copter mass */
	float Mass;

	/** Gravitational acceleration */
	float G;

	/** Gravitational acceleration vector */
	FVector GravAccel_IF;

	/** Total thrust provided by the four quad rotors */
	float Thrust;

	/** Quadcopter thrust coefficient. Relates thrust to propeller RPM.*/
	float ThrustCoefficient;

	/** Throttle input */
	float Throttle;

	/** Quad copter position in the inertial (world) frame */
	FVector Position_IF;

	/** Quad copter initial position in the inertial (world) frame */
	FVector Position0_IF;

	/** Angular velocity in the inertial (world) frame */
	FVector AngularVel_IF;

	/** Initial angular velocity in the inertial (world) frame */
	FVector AngularVel0_IF;

	/** Angular acceleration in the inertial (world) frame */
	FVector AngularAccel_IF;

	/** Angular velocity in the body (local) frame */
	FVector AngularVel_BF;

	/** Initial angular velocity in the body (local) frame */
	FVector AngularVel0_BF;

	/** Angular acceleration in the body (local) frame */
	FVector AngularAccel_BF;

	/** Linear velocity in the inertial (world) frame */
	FVector LinearVel_IF;

	/** Initial linear velocity in the inertial (world) frame */
	FVector LinearVel0_IF;

	/** Linear acceleration in the inertial (world) frame */
	FVector LinearAccel_IF;

	/** Quad copter roll (radians) */
	float Roll;

	/** Quad copter roll rate  (radians/s)*/
	float RollRate;

	/** Quad copter pitch  (radians)*/
	float Pitch;

	/** Quad copter pitch rate (radians/s) */
	float PitchRate;

	/** Quad copter yaw  (radians)*/
	float Yaw;

	/** Quad copter yaw rate  (radians/s)*/
	float YawRate;

public:
	/** Returns PlaneMesh subobject **/
	FORCEINLINE class UStaticMeshComponent* GetPlaneMesh() const { return PlaneMesh; }
	/** Returns SpringArm subobject **/
	FORCEINLINE class USpringArmComponent* GetSpringArm() const { return SpringArm; }
	/** Returns Camera subobject **/
	FORCEINLINE class UCameraComponent* GetCamera() const { return Camera; }
};

//
// Created by marvel on 10/8/24.
//


#pragma once
#include "tinysplinecxx.h"
#include "Core/CoreMinimal.h"
#include "Math/FTransform.h"
#include "ReferenceSurface.h"
#include "LinkageOptimize/SurfaceVoxel.h"
#include "SpacialMechanism/SurfaceJoint.h"
#include "SpacialMechanism/SurfaceJointComponent.h"

struct JointSequence
{
    char Type;
    bool bIsRoot = false;

    Vector2d UV = Vector2d::Zero();

	TArray<FVector2> PinUV;
	TArray<FVector2> PinInMeshUV;

	TArray<FVector2> SocketUV;
	TArray<FVector2> SocketInMeshUV;

	int Pin = -1;
	int Socket = -1;

    FTransform InitTransform = FTransform::Identity();
    TArray<FTransform> TransformSequence;

    tinyspline::BSpline LinkageSpline;
    TArray<FVector> LinkageSample;

    ObjectPtr<SurfaceJointActor> JointActor = nullptr;
    ObjectPtr<class StaticMesh>	 LinkageSweptVolume;

	[[nodiscard]] SurfaceJointComponent* GetComponent() const
	{
		return JointActor->GetJointComponent().get();
	}
};



struct MechanismSequence
{
    String Type;
    int EffectorParentIndex = -1;
    int EffectorIndex = -1;

	int OriginalTime = -1; // The motion time duration of the mechanism
	int MobilityAllowedTime = -1; // The maximum allowed time for the mechanism to statify the mobility
	int EffectorTime = -1; // Best time needed for the effector to perform the task


	TArray<JointSequence> Joints; // Joint sequences

	TArray<FVector> TargetPoints; // Target points
	TArray<FVector> EffectorTarjectory; // Effector world trajectory

    ObjectPtr<ReferenceSurface> RSurface; // Original surface parametric
	SurfaceVoxel* Voxel;

	void CheckValid()
	{
		ASSERT(!Type.empty());
		ASSERT(EffectorIndex != -1);
		ASSERT(EffectorParentIndex != -1);
		ASSERT(RSurface != nullptr);
		ASSERT(!Joints.empty());
	}

	[[nodiscard]] int JointNumWithEffector() const
	{
		return Type.size();
	}

	[[nodiscard]] int JointNumWithoutEffector() const
	{
		return Type.size() - 1;
	}
};
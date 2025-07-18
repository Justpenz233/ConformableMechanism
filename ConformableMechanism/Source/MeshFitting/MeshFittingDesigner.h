//
// Created by MarvelLi on 2024/2/26.
//

#pragma once
#include "Optimization/ReferenceSurface.h"
#include <Optimization/MechanismMobility.h>

namespace tinyspline {
class BSpline;
}

struct SurfaceLinkageConfiguration
{
	String Type;

	MechanismMobility Mobility;
	int EffectorParentIndex = -1;
	int EffectorIndex = -1;

	int OriginalTime = -1;
	int MobilityAllowTime = -1;
	int EffectorBestTime = -1;

	ObjectPtr<ReferenceSurface> AttachedSurface; // Original surface parametric

	TArray<Vector2d> UV; // UV of joints
	TArray<FTransform> InitTransform; // Init transforms for joints
	TArray<FVector> EffectorTarjectory; // Effector world trajectory

	// Surface mesh after collision avoidance
	TArray<TArray<FTransform>> JointTransforms; // Transforms for each joints in world space

	ObjectPtr<class StaticMesh> SurfaceMesh = nullptr;
	TArray<ObjectPtr<class StaticMesh>> LinkageSweptVolume;

	// Joint mesh
	TArray<ObjectPtr<class SurfaceJointActor>> JointActor = {nullptr, nullptr, nullptr, nullptr, nullptr};

	TArray<TArray<FVector>> LinkagePath; // optimized linkages path in 3D space, represent linkage geometry
	TArray<FVector> EffectorGeometry;

	double AnglePerTimeClip() const
	{
		ASSERT(OriginalTime != -1);
		return 2. * M_PI / OriginalTime;
	}
};

class MeshFittingDesigner
{
public:
	using Parameter = TArray<double>;

	static Parameter RandomParameter();

	static Parameter RandomValidParameter(String Type, ObjectPtr<ReferenceSurface> Surface, const TArray<ObjectPtr<class TargetPointsActor>>& TargetPoints);

	static TArray<double> SolveJoint(String Type, ObjectPtr<ReferenceSurface> InSurface, const TArray<FVector>& InTargetPoints, int Generation, int PopulationSize);

	/**
	 * Spawn actors for the linkage and joint mesh,
	 * for further used as container for geometry
	 * @param Config The original configuration, would assign actor and new surface mesh
	 */
	static void SpawnActor(SurfaceLinkageConfiguration& Config);
	
	/**
	 * Post process the configuration, including generate the joint mesh and the support surface mesh
	 * @param Config The original configuration, would be modified by give actor and new surface mesh
	 */
	static void PostProcess(SurfaceLinkageConfiguration& Config);


	static ObjectPtr<StaticMesh> CutSurface(SurfaceLinkageConfiguration& Config);
};

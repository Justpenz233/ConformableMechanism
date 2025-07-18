//
// Created by MarvelLi on 2024/4/22.
//

#pragma once
#include <pagmo/types.hpp>
#include "tinysplinecxx.h"
#include "Math/FTransform.h"
#include "SpacialMechanism/SurfaceJoint.h"

class ReferenceSurface;
class SurfaceVoxel;
class ParametricMeshActor;
class SurfaceLinkageConfiguration;
/**
 * Assume we have ControlPointCount control points to represent each linkage geometry,
 * Given a initial guess, the evolution will optimize the delta position of control points
 */

struct SurfaceLinkageProblem
{
public:

	SurfaceLinkageProblem()= default;
	explicit SurfaceLinkageProblem(const SurfaceLinkageConfiguration& Config,
		SurfaceVoxel* InVoxels = nullptr, bool bDebugLog = false);
	~SurfaceLinkageProblem() = default;
	SurfaceLinkageProblem(SurfaceLinkageProblem&&) = default;
	SurfaceLinkageProblem(const SurfaceLinkageProblem&) = default;
	SurfaceLinkageProblem& operator=(const SurfaceLinkageProblem& other);


	[[nodiscard]] pagmo::vector_double fitness(const pagmo::vector_double& dv) const;
	[[nodiscard]] std::pair<pagmo::vector_double, pagmo::vector_double> get_bounds() const;
	[[nodiscard]] pagmo::vector_double::size_type get_nobj() const { return MultiTarget ? 1 : 2; }

	[[nodiscard]] static std::pair<int, double> CheckLinkageRuntimeCollision(
		const TArray<FVector>& LinkageA, const TArray<FTransform>& TransformA, const FTransform& InitTransformA,
		const TArray<FVector>& LinkageB, const TArray<FTransform>& TransformB, const FTransform& InitTransformB) ;


	[[nodiscard]] static std::pair<int, double> CheckLinkageJointRuntimeCollision(
		const TArray<FVector>& LinkageA, const TArray<FTransform>& TransformA, const FTransform& InitTransformA,
		const FBox& Box, const TArray<FTransform>& TransformB, int DelteaIndex) ;


	void CheckParameterValid() const;
	void Init(const SurfaceLinkageConfiguration& Config);
	static std::pair<TArray<TArray<FVector>>, TArray<double>>Solve(const SurfaceLinkageConfiguration& Config, SurfaceVoxel* Voxels);

	/**
	 * This term simulate a force to push away two intersection curve segments
	 * @return
	 */
	static double SegmentCollisionForce(double Distance, double Threshold);

	/**
	 * Parse the parameter vector to linkage spline
	 * @param dv parameter vector
	 * @param LogControlPoint if Log the control point
	 * @return the linkage spline, joint index and socket index
	 */
	std::tuple<TArray<tinyspline::BSpline>, TArray<int>, TArray<int>> ParamToSpline(const pagmo::vector_double& dv, bool LogControlPoint = false) const;

	static TArray<TArray<FVector>> LinkageSplineSample(const TArray<tinyspline::BSpline>& LinkageSpline, ParametricMeshActor* Surface, int Samples, bool bShowControlPoints = false) ;
	static TArray<FVector2> LinkageSplineSample2D(const tinyspline::BSpline& LinkageSpline, int Samples);
	static TArray<FVector> LinkageSplineSample(const tinyspline::BSpline& LinkageSpline, ParametricMeshActor* Surface, int Samples, bool bShowControlPoints = false);
	void LinkageConnectJoint(TArray<TArray<FVector>>& Linkages, const TArray<int> &JointIndex, const TArray<int>& SocketIndex) const;

	// Merge all the self intersection of a linkage
	static TArray<FVector> PostProcessLinkage(TArray<FVector> LinkagePath) ;

	// x <= a
	static double Penalty(double x, double a);

	/**
	 * @param V the value to be remapped
	 * @return the remapped value
	 */
	FORCEINLINE static void RemapV(double& V){
		V = std::fmod(V, 1.);
		if (V < 0) V += 1;
	}
	bool MultiTarget = true;
protected:
	int JointNum;
	int EffectorIndex;
	int EffectorParentIndex;
	TArray<FVector2> JointUV;
	TArray<TArray<FVector2>> JointPortInMeshUV;
	TArray<TArray<FVector2>> JointPortUV;
	TArray<TArray<FVector2>> SocketPortInMeshUV;
	TArray<TArray<FVector2>> SocketPortUV;
	TArray<TArray<int>> JointPortIndex;
	TArray<TArray<int>> SocketPortIndex;

	TArray<SurfaceJointActor*> Joints;
	TArray<TArray<FTransform>> Transform; // Transforms for the linkage
	TArray<FTransform> InitTransform; // Initial transform for the linkage
	ReferenceSurface* Surface{};
	SurfaceVoxel* Voxels;

	bool DebugLog = false;
};
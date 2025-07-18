//
// Created by MarvelLi on 2024/5/21.
//

#pragma once
#include "Actors/ParametricMeshActor.h"
#include "Algorithm/GraphTheory.h"
#include "Object/Object.h"
#include "Optimization/ReferenceSurface.h"
#include "bvh/v2/bvh.h"
#include <bvh/v2/tri.h>
#include <hpp/fcl/BV/AABB.h>
#include <hpp/fcl/broadphase/broadphase.h>

class SurfaceVoxel : public Object
{
public:
	enum CollisionType
	{
		NoCollision = 0,
		BasicCollision = 1,
		Disconnected = 2,
		WeakSturecture = 3,
		InitCollision = 4
	};

	TArray<FBox> CollisionBoxes;
	TArray<double> BoxScore;

	FORCEINLINE double GetSumScores() const { return CollisionBoxSumScores; }

	explicit SurfaceVoxel(ReferenceSurface* InSurface)
	: Surface(InSurface)
	{
		Build();
	}

	/**
	 * Calculate which cubes intersect with the given line segment
	 * @return the indices of the intersected cubes
	 */
	TArray<int> IntersectCubes(const FVector& Start, const FVector& End) const;

	TArray<int> IntersectBox(const FBox& Box, const FTransform& BoxTransform) const;
	TArray<int> IntersectBoundingVolume(const TFunction<bool(FVector)>& BV, const FTransform& BoxTransform) const;

	/**
	 * Calculate which cubes intersect with the given linkage motion.
	 * Will classify the collision type for each cube
	 * @param LinkageSample Calculate the linkage motion
	 * @param Transform the transform of each linkage motion
	 * @param InitTransform the initial transform of each linkage motion
	 * @param JointBV
	 * @return the collision type for each cube
	 */
	TArray<int> LinkageRuntimeCollision(
		const TArray<TArray<FVector>>&	  LinkageSample,
		const TArray<TArray<FTransform>>& Transform,
		const TArray<FTransform>& InitTransform, const TArray<TFunction<bool(FVector)>>& JointBV) const;

	int Size() const;

	struct CollisionScores
	{
		double BasicCollisionScore = 0.;
		double DisconnectedScore = 0.;
		double WeakSturectureScore = 0.;
		double Sum() const { return BasicCollisionScore + DisconnectedScore + WeakSturectureScore; }
	};
	SurfaceVoxel::CollisionScores CalculateScore(const TArray<int>& CollisionDetected) const;


	/**
	 * Draw the voxel grid used for debug, with score information
	 * @param World the world to draw the voxel grid
	 */
	void DebugDraw(World* World);

	/**
	 * Draw the collision boxes that are collided with the mechanism
	 * @param World the world to draw the collision boxes
	 * @param CollisionDetected the collision detection result
	 */
	void DebugDrawCollision(World* World, const TArray<int>& CollisionDetected) const;

	/**
	 * Detect the fragile part of the surface
	 * @param Threshold the threshold to detect the fragile part
	 * @param CollisionState the collision detection result
	 * @return The collision tag for each triangle
	 */
	TArray<int> DetectFragilePart(double Threshold, const TArray<int>& CollisionState) const;

	/**
	 * Given the collision detection result, expand the collision boxes by the graph theory
	 * @return new collision detection result
	 */
	TArray<bool> ExpandCollisionByGraph(TArray<bool> CollisionDetected) const;

protected:
	void ExpandCollisionByGraph(TArray<int>& CollisionDetected) const;

	using Scalar = double;
	using Vec3   = bvh::v2::Vec<Scalar, 3>;
	using BBox   = bvh::v2::BBox<Scalar, 3>;
	using Node   = bvh::v2::Node<Scalar, 3>;
	using Bvh    = bvh::v2::Bvh<Node>;
	using Ray    = bvh::v2::Ray<Scalar, 3>;

	void Build();

	void BuildBVH();
	Bvh bvh;

	// Zero thickness surface mesh
	ObjectPtr<StaticMesh> SurfaceMesh = nullptr;

	ReferenceSurface* Surface = nullptr;
	Algorithm::GraphTheory::Graph<double> Graph;
	double CollisionBoxSumScores = 0.;
};
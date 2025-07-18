//
// Created by MarvelLi on 2024/12/7.
//

#pragma once
#include "SurfaceVoxel.h"
#include "MeshFitting/MeshFittingDesigner.h"
#include "Misc/Config.h"

#include <pagmo/algorithms/maco.hpp>
#include <pagmo/algorithms/moead_gen.hpp>
#include <pagmo/algorithms/nspso.hpp>
#include <pagmo/algorithms/pso_gen.hpp>

class EffectorLinkageProblem
{
public:
	EffectorLinkageProblem() = default;

	EffectorLinkageProblem(const SurfaceLinkageConfiguration* InConfig, const FVector& InEndPoint, SurfaceVoxel* InVoxels, const TArray<bool>& InCollided);

	[[nodiscard]] pagmo::vector_double fitness(const pagmo::vector_double& dv) const;
	[[nodiscard]] std::pair<pagmo::vector_double, pagmo::vector_double> get_bounds() const;
	[[nodiscard]] pagmo::vector_double::size_type get_nobj() const { return 2; }


	double CollisionScore(const TArray<FVector>& LinkageGeometry) const;
	double SurfaceCutScore(const TArray<FVector>& LinkageGeometry) const;
	static double GeometryScore(const TArray<FVector>& LinkageGeometry);

	TArray<FVector> EffectorLinkageGeometry(const pagmo::vector_double& dv) const;

protected:
	const SurfaceLinkageConfiguration* Config = nullptr;

	FBox BBox;
	SurfaceVoxel* Voxels;
	TArray<bool> Collided;

	FVector EndPoint;

	int Samples = GConfig.Get<int>("EffectorProblem", "Samples");
	int ControlPointsNum = GConfig.Get<int>("EffectorProblem", "ControlPointsNum");
	double MinIndexPercent = GConfig.Get<double>("EffectorProblem", "MinIndexPercent");
	double MaxIndexPercent = GConfig.Get<double>("EffectorProblem", "MaxIndexPercent");


public:
	static void Solve(const SurfaceLinkageConfiguration& Config, SurfaceVoxel* Voxels, const FVector& EndPoint);
};

//
// Created by MarvelLi on 2024/12/7.
//

#include "EffectorLinkageProblem.h"

#include "Fcl.h"
#include "tinysplinecxx.h"
#include "SpacialMechanism/SurfaceJoint.h"

#include <pagmo/island.hpp>
#include <pagmo/algorithms/cmaes.hpp>
#include <pagmo/algorithms/nsga2.hpp>
#include <pagmo/batch_evaluators/thread_bfe.hpp>

EffectorLinkageProblem::EffectorLinkageProblem(const SurfaceLinkageConfiguration* InConfig, const FVector& InEndPoint, SurfaceVoxel* InVoxels, const TArray<bool>& InCollided)
: Config(InConfig), EndPoint(InEndPoint), Voxels(InVoxels), Collided(InCollided)
{
	ASSERT(Config->AttachedSurface != nullptr);
	auto SurfaceTransform = Config->AttachedSurface->GetFTransform();
	auto Mesh = Config->AttachedSurface->GetSurfaceMesh();
	auto V = Mesh->GetVertices();
	for(int i = 0;i < V.rows();i ++)
		BBox += SurfaceTransform * V.row(i);

	// double extend
	auto S = BBox.GetSize();
	auto Center = BBox.GetCenter();
	BBox = FBox(FVector(Center - S), FVector(Center + S));
}

TArray<FVector> EffectorLinkageProblem::EffectorLinkageGeometry(const pagmo::vector_double& dv) const
{
	TArray<FVector> ControlPoints(ControlPointsNum + 2);

	// First Control Points
	{
		auto Index = double(Config->LinkagePath[Config->EffectorParentIndex].size()) * (dv[0] * (MaxIndexPercent - MinIndexPercent) + MinIndexPercent);
		ControlPoints[0] = Config->LinkagePath[Config->EffectorParentIndex][Index];
	}

	// Last Control Points
	ControlPoints.back() = EndPoint;

	FVector BBoxMin = BBox.Min;
	FVector BBoxMax = BBox.Max;
	// Assign other control points
	for (int i = 0; i < ControlPointsNum; i++)
	{
		ControlPoints[i + 1].x() = dv[1 + i * 3];
		ControlPoints[i + 1].y() = dv[1 + i * 3 + 1];
		ControlPoints[i + 1].z() = dv[1 + i * 3 + 2];

		// Remap to bbox space
		ControlPoints[i + 1] = ControlPoints[i + 1].cwiseProduct(BBoxMax - BBoxMin) + BBoxMin;
	}

	// Set control points for geometry
	auto LinkageGeometry = tinyspline::BSpline(ControlPointsNum + 2, 3, std::min(3, ControlPointsNum + 1));
	{
		TArray<double> CntPntData;
		for(auto i : ControlPoints)
		{
			CntPntData.push_back(i.x());
			CntPntData.push_back(i.y());
			CntPntData.push_back(i.z());
		}
		LinkageGeometry.setControlPoints(CntPntData);
	}

	TArray<FVector> Linkage;
	// Sample the b-spline, this part could do equal length sample
	{
		auto LinkageSample = LinkageGeometry.sample(Samples);
		for (int i = 0; i < LinkageSample.size(); i += 3)
			Linkage.emplace_back(
				LinkageSample[i],
				LinkageSample[i + 1],
				LinkageSample[i + 2]);
	}
	return Linkage;
}
double Penalty(double x, double a)
{
	return std::max(0., x - a) * 1e4;
}

double EffectorLinkageProblem::CollisionScore(const TArray<FVector>& LinkageGeometry) const
{
	static int CheckTimeStamp = GConfig.Get<int>("LinkageProblem", "TimeSteps");
	static auto MeshScale = GConfig.Get<double>("JointMesh", "Scale");
	static auto LinkageRadius = GConfig.Get<double>("JointMesh", "LinkageRadius") * MeshScale;
	static auto EffectorRadius = GConfig.Get<double>("EffectorJoint", "LinkageRadius") * MeshScale;

	int TimeDelta = Max(Config->JointTransforms[0].size() / CheckTimeStamp, 1);
	int CollisionNum = 0;
	double Penuality = 0.;

	Matrix4d EInverse = Config->InitTransform[Config->EffectorParentIndex].GetMatrix().inverse();
	auto& ETransform = Config->JointTransforms[Config->EffectorParentIndex];
	for(int t = 0; t < Config->JointTransforms[0].size(); t += TimeDelta)
	{
		for(int i = 0; i < Config->JointTransforms.size() - 1; i ++)
		{
			if (i == Config->EffectorParentIndex) continue;

			Matrix4d Inverse = Config->InitTransform[i].GetMatrix().inverse();
			for(int j = 0;j < Config->LinkagePath[i].size() - 1; j ++)
			{
				FVector Start = Config->JointTransforms[i][t] * (Inverse * Config->LinkagePath[i][j].homogeneous()).head(3);
				FVector End = Config->JointTransforms[i][t] * (Inverse * Config->LinkagePath[i][j + 1].homogeneous()).head(3);

				for(int k = 0;k < LinkageGeometry.size() - 1;k ++)
				{
					FVector S = ETransform[t] * (EInverse * LinkageGeometry[k].homogeneous()).head(3);
					FVector E = ETransform[t] * (EInverse * LinkageGeometry[k + 1].homogeneous()).head(3);

					if (auto Dis = CylinderCylinderDistance(Start, End, S, E, LinkageRadius); Dis < 0)
					{
						CollisionNum ++;
						Penuality += Penalty(abs(Dis), 0.);
					}
				}
			}
		}
	}

	return Penuality + CollisionNum * 1e4;
}

double EffectorLinkageProblem::SurfaceCutScore(const TArray<FVector>& LinkageGeometry) const
{
	auto& ETransform = Config->JointTransforms[Config->EffectorParentIndex];
	Matrix4d EInverse = Config->InitTransform[Config->EffectorParentIndex].GetMatrix().inverse();

	int TimeStep = ETransform.size();
	TArray<bool> CollisionDetected = Collided;
	auto LocalGeometry = [&](int index) -> FVector {
		return (EInverse * LinkageGeometry[index].homogeneous()).head(3);
	};

	for (int Time = 0; Time < TimeStep; Time++)
	{
		for (int i = 0; i < LinkageGeometry.size() - 1; i++)
		{
			FVector Start = ETransform[Time] * LocalGeometry(i);
			FVector End = ETransform[Time] * LocalGeometry(i + 1);
			auto IntersectedCubes = Voxels->IntersectCubes(Start, End);
			for (auto CubeID : IntersectedCubes)
				CollisionDetected[CubeID] = true;
		}
	}
	auto ExpandedCollision = Voxels->ExpandCollisionByGraph(CollisionDetected);
	double Score = 0;
	for(int i = 0;i < ExpandedCollision.size(); i ++)
		if (ExpandedCollision[i])
			Score += Voxels->BoxScore[i];
	return Score / Voxels->GetSumScores();
}

double EffectorLinkageProblem::GeometryScore(const TArray<FVector>& LinkageGeometry)
{
	double Score = 0.;
	for (int i = 2; i < LinkageGeometry.size(); i++)
		Score += (LinkageGeometry[i] - LinkageGeometry[i - 1]).norm();
	return Score;
}

pagmo::vector_double EffectorLinkageProblem::fitness(const pagmo::vector_double& dv) const
{
	ASSERT(Config != nullptr);
	// First construct Effector linkage geometry
	// Maybe we need to check self collision here
	auto EffectorGeometry = EffectorLinkageGeometry(dv);

	// Second check collision with other linkage and joint mesh
	// auto Collision = CollisionScore(EffectorGeometry);
	auto Collision = 0.;

	// Third check collision with surface and calculate score
	auto SurfaceCut = SurfaceCutScore(EffectorGeometry);

	// Plus effector straight score, evaluate how straight the effector is
	auto Geometry = GeometryScore(EffectorGeometry);

	return { Collision + SurfaceCut, Geometry };
}
std::pair<pagmo::vector_double, pagmo::vector_double> EffectorLinkageProblem::get_bounds() const
{
	TArray<double> L, R;
	for(int i = 0;i < ControlPointsNum * 3 + 1; i ++)
	{ L.push_back(0.); R.push_back(1.); }
	return { L, R };
}

void EffectorLinkageProblem::Solve(const SurfaceLinkageConfiguration& Config, SurfaceVoxel* Voxels, const FVector& EndPoint)
{
	pagmo::population::size_type	 PopulationNum = GConfig.Get<int>("EffectorProblem", "Population");
	int GenerationNum = GConfig.Get<int>("EffectorProblem", "Generation");
	TArray<TFunction<bool(FVector)>> JointBV;
	for (const auto & i : Config.JointActor)
		JointBV.push_back(i->GetJointComponent()->GetJointBoundingVolume());

	auto CollisionDetected =
		Voxels->LinkageRuntimeCollision(Config.LinkagePath, Config.JointTransforms, Config.InitTransform, JointBV);

	TArray<bool> Collided;
	for (int i : CollisionDetected)
		Collided.push_back(i != 0);

	EffectorLinkageProblem Problem(&Config, EndPoint, Voxels, Collided);
	pagmo::problem Prob{ Problem };
	pagmo::nsga2 cmaes_instance(GenerationNum);
	cmaes_instance.set_bfe(pagmo::bfe(pagmo::thread_bfe()));
	pagmo::algorithm algo{ cmaes_instance };
	algo.set_verbosity(1);
	pagmo::population Population{Problem, pagmo::bfe(pagmo::thread_bfe()), PopulationNum};
	pagmo::island Island(algo, Population);
	Island.evolve(1);
	Island.wait();

	TArray<std::pair<TArray<double>, TArray<double>>> Result;

	for (int i = 0; i < Island.get_population().size(); i++)
	{
		auto f = Island.get_population().get_f()[i];
		auto x = Island.get_population().get_x()[i];
		Result.push_back({ f, x });
	}
	// Sort the result
	std::sort(Result.begin(), Result.end(), [](const auto& L, const auto& R) {
		return L.first[0] == R.first[0] ? L.first[1] < R.first[1] : L.first[0] < R.first[0];
	});
	for (int i = 0; i < Result.size(); i++)
	{
		LOG_INFO("F: {} X: {}", Result[i].first, Result[i].second);
	}
}
//
// Created by MarvelLi on 2024/4/22.
//

#include "SurfaceLinkageProblem.h"
#include <igl/segment_segment_intersect.h>
#include "Actors/ParametricMeshActor.h"
#include "Misc/Config.h"
#include "SurfaceVoxel.h"
#include "Math/Geometry.h"
#include "Math/Intersect.h"
#include "Math/Random.h"
#include "Fcl.h"
#include "Optimization/ReferenceSurface.h"
#include "spdlog/stopwatch.h"

SurfaceLinkageProblem::SurfaceLinkageProblem(
	const SurfaceLinkageConfiguration& Config, SurfaceVoxel* InVoxels, bool bDebugLog)
{
	Voxels = InVoxels;
	DebugLog = bDebugLog;
	Init(Config);
}
SurfaceLinkageProblem& SurfaceLinkageProblem::operator=(const SurfaceLinkageProblem& other)
{
	JointUV = other.JointUV;
	JointPortInMeshUV = other.JointPortInMeshUV;
	JointPortUV = other.JointPortUV;
	SocketPortInMeshUV = other.SocketPortInMeshUV;
	SocketPortUV = other.SocketPortUV;
	Joints = other.Joints;
	Transform = other.Transform;
	InitTransform = other.InitTransform;
	Surface = other.Surface;
	Voxels = other.Voxels;
	return *this;
}
void SurfaceLinkageProblem::CheckParameterValid() const
{
	ASSERT(Joints.size() >= 4);
	ASSERT(JointUV.size() >= Joints.size() - 1);
	ASSERT(JointPortUV.size() == Joints.size() - 1);
	ASSERT(SocketPortUV.size() == Joints.size() - 1);
	ASSERT(JointPortInMeshUV.size() == Joints.size() - 1);
	ASSERT(SocketPortInMeshUV.size() == Joints.size() - 1);
	ASSERT(Transform.size() == Joints.size());
	ASSERT(InitTransform.size() == Joints.size());

}
pagmo::vector_double SurfaceLinkageProblem::fitness(const pagmo::vector_double& dv) const
{
	/***
	 * Initialize the control points of the linkage geometry
	 * Sample the path to a discrete curve
	 */
	auto [LinkageSpline, JointPort, SocketPort] = ParamToSpline(dv, DebugLog);
	static int SampleCount = GConfig.Get<int>("LinkageProblem", "SplineSample");

	// Sampled points on the linkage geometry
	TArray<TArray<FVector>> Linkage = LinkageSplineSample(LinkageSpline, Surface, SampleCount);
	int LinkageNum = Linkage.size();
	static auto MeshScale = GConfig.Get<double>("JointMesh", "Scale");
	static auto LinkageRadius = GConfig.Get<double>("JointMesh", "LinkageRadius") * MeshScale;

	int ExceedCount = 0; double ExceedForce = 0;
	{
		for (const auto& LinkageI : LinkageSpline)
		{
			auto Samples = LinkageSplineSample2D(LinkageI, 512);
			for (const auto& Sample : Samples)
				if (Sample.x() > 1. || Sample.x() < 0. || Sample.y() > 1. || Sample.y() < 0.)
				{
					ExceedCount ++;
					ExceedForce += Penalty(Sample.x(), 1) + Penalty(Sample.y(), 1);
					ExceedForce += Penalty(-Sample.x(), 0) + Penalty(-Sample.y(), 0);
					if (DebugLog)[[unlikely]] LOG_INFO("Exceed uv: {}", Sample);
				}
		}
		if (DebugLog && ExceedCount != 0)[[unlikely]] LOG_INFO("Exceed count: {}", ExceedCount);
	}
	int SelfLinkageIntersection = 0.;
	double SelfIntersectionForce = 0.;
	for(int i = 0;i <= LinkageNum;i ++)
	{
		auto JointBox = Joints[i]->GetJointComponent()->GetJointBoundingBox();
		auto Transform = InitTransform[i];
		if (i < LinkageNum)
		for(int j = 2;j < Linkage[i].size() - 1;j ++)
		{
			auto Start0 = Transform.ToLocalSpace(Linkage[i][j]); auto End0 = Transform.ToLocalSpace(Linkage[i][j + 1]);
			if (auto Dis = CylinderBoxDistance(Start0, End0, LinkageRadius, JointBox); Dis < 0)
			{
				SelfLinkageIntersection ++;
				SelfIntersectionForce += Penalty(abs(Dis), 0.);
				if (DebugLog)[[unlikely]]
					LOG_INFO("Linkage {} and Joint {} collision: {}", i, i, Dis);
			}
		}

		if (i > 0)
		for(int j = 0;j < Linkage[i - 1].size() - 2;j ++)
		{
			auto Start1 = Transform.ToLocalSpace(Linkage[i - 1][j]); auto End1 = Transform.ToLocalSpace(Linkage[i - 1][j + 1]);
			if (auto Dis = CylinderBoxDistance(Start1, End1, LinkageRadius, JointBox); Dis < 0)
			{
				SelfLinkageIntersection ++;
				SelfIntersectionForce += Penalty(abs(Dis), 0.);
				if (DebugLog)[[unlikely]]
					LOG_INFO("Linkage {} and Joint {} collision: {}", i - 1, i, Dis);
			}
		}
	}
	LinkageConnectJoint(Linkage, JointPort, SocketPort);

	/***
	 * First, check if each linkage geometry have self intersection at the first frame
	 * No need to consider the transformation
	 */
	static bool PostProcessLinkage = GConfig.Get<bool>("LinkageProblem", "PostProcessLinkage");
	if(!PostProcessLinkage)
	{
		for (int i = 0; i < LinkageNum;i ++)
		{
			for(int k = 0; k < Linkage[i].size() - 2; k ++)
			{
				for(int l = k + 3; l < Linkage[i].size() - 2;l ++)
				{
					auto Start0 = Linkage[i][k]; auto End0 = Linkage[i][k + 1];
					auto Start1 = Linkage[i][l]; auto End1 = Linkage[i][l + 1];
					if(auto Dis = CylinderCylinderCollide(Start0, End0, Start1, End1, LinkageRadius); Dis > 1e-2)
					{
						SelfLinkageIntersection ++;
						SelfIntersectionForce += Penalty(Dis, 0);
						if (DebugLog)[[unlikely]]
						LOG_INFO("Linkage {} : seg {} and  {} collision: {}", i, k, l, Dis);
					}
				}
			}
		}
	}

	/***
	 * Then check if the linakge intersect joint
	 */
	TArray<FBox> JointBoxes;
	for(int JointIndex = 0; JointIndex < LinkageNum + 1; JointIndex ++)
		JointBoxes.push_back(Joints[JointIndex]->GetJointComponent()->GetJointBoundingBox());

	Linkage.push_back(TArray<FVector>{InitTransform[EffectorParentIndex].GetLocation(), InitTransform[EffectorIndex].GetLocation()});
	// Simulation check collision
	double RuntimeCollisionForce = 0;
	int RuntimeCollisionCount = 0;

	for (int i = 0; i < LinkageNum; i++)
	{
		for (int j = i + 1; j < LinkageNum; j++)
		{
			auto [CCount, CScore] = CheckLinkageRuntimeCollision(
				Linkage[i], Transform[i], InitTransform[i],
				Linkage[j], Transform[j], InitTransform[j]);
			RuntimeCollisionCount += CCount;
			RuntimeCollisionForce += CScore;
			if (DebugLog && CCount != 0)[[unlikely]] LOG_INFO("Linkage {} and Linkage {} collision count: {}", i, j, CCount);
		}
	}


	for (int i = 0;i < Linkage.size() - 1;i ++)
	{
		for (int j = 0;j < JointBoxes.size(); j ++)
		{
			if (j == i || j == i + 1) continue;
			auto [CCount, CScore] = CheckLinkageJointRuntimeCollision(
				   Linkage[i], Transform[i], InitTransform[i],
				   JointBoxes[j], Transform[j], 3);
			RuntimeCollisionCount += CCount; RuntimeCollisionForce += CScore;
			if (DebugLog && CCount != 0)[[unlikely]] LOG_INFO("Linkage {} and Joint {} collision count: {}", i, j, CCount);
		}
	}


	/***
	 * Last step, calculate the collision with the surface gemoetry
	 * By using the box score
	 * Use bvh traversal to accelerate the collision detection
	 */
	TArray<TFunction<bool(FVector)>> JointBV;
	for (int i = 0; i < LinkageNum + 1; i ++)
		JointBV.push_back(Joints[i]->GetJointComponent()->GetJointBoundingVolume());
	double CollisionScore = 0;
	auto CollisionDetected =
		Voxels->LinkageRuntimeCollision(Linkage, Transform, InitTransform, JointBV);

	auto Score = Voxels->CalculateScore(CollisionDetected);
	if(DebugLog)[[unlikely]]
	{
		TArray<int> BasicCollide;
		TArray<int> Disconnected;
		TArray<int> WeakStructure;
		for(int i = 0; i < CollisionDetected.size(); i ++)
		{
			if(CollisionDetected[i] == 1) BasicCollide.push_back(i);
			if(CollisionDetected[i] == 2) Disconnected.push_back(i);
			if(CollisionDetected[i] == 3) WeakStructure.push_back(i);
		}
		LOG_INFO("Basic Collide: {}", BasicCollide);
		LOG_INFO("Disconnected: {}", Disconnected);
		LOG_INFO("Weak Structure: {}", WeakStructure);
	}


	// Detect if driven joint box was isolated
	bool DrivenIsolated = false;
	{
		DrivenIsolated = true;
		// When all the areas are disconnected, the driven joint is isolated
		// auto DrivenBoxCollide = Voxels->IntersectBoundingVolume(JointBV[0], InitTransform[0]);
		// for(auto DBC : DrivenBoxCollide)
		// {
			// ASSERT(DBC < CollisionDetected.size());
			// if(CollisionDetected[DBC] == 0)
			// {
				// DrivenIsolated = false;
				// break;
			// }
		// }
	}


	static bool bDebugShow = GConfig.Get<bool>("SurfaceGraph", "DebugShow");
	if(bDebugShow)
		Voxels->DebugDrawCollision(GWorld, CollisionDetected);

	CollisionScore = Score.Sum() / Voxels->GetSumScores();
	// CollisionScore = -Score.WeakSturectureScore / Voxels->GetSumScores();

	// calculate the linkage length
	double LinkageLength = 0;
	for (int i = 0; i < LinkageNum; i ++)
	{
		for (int j = 0; j < Linkage[i].size() - 1; j ++)
			LinkageLength += (Linkage[i][j + 1] - Linkage[i][j]).norm();
	}

	/**
	 * Calculate fitness value
	 */
	int Intersection = ExceedCount + SelfLinkageIntersection  + RuntimeCollisionCount + DrivenIsolated;
	double IntersectionForce = RuntimeCollisionForce + ExceedForce + SelfIntersectionForce;
	double BasicTerm = CollisionScore * 10;
	double PunishTerm = Intersection * 1e4 + IntersectionForce;
	constexpr double LinkageLengthWeight = 0.25;
	if(DebugLog)
	{
		LOG_INFO("Driven Isolated: {}", DrivenIsolated);
		LOG_INFO("Intersection: {} SelfLinkageIntersection: {} RuntimeCollisionCount: {}",
			Intersection, SelfLinkageIntersection, RuntimeCollisionCount);
		LOG_INFO("Collision score: {}", CollisionScore);
		LOG_INFO("Linkage length{}", LinkageLength);
		LOG_INFO("Linkage Fitness: {}", BasicTerm + PunishTerm + LinkageLength * LinkageLengthWeight);
	}
	if (MultiTarget)
		return {BasicTerm + PunishTerm, LinkageLength};
	// return {BasicTerm + PunishTerm};
	return {BasicTerm + PunishTerm + LinkageLength * LinkageLengthWeight};

}
std::pair<int, double> SurfaceLinkageProblem::CheckLinkageRuntimeCollision(
	const TArray<FVector>& LinkageA, const TArray<FTransform>& TransformA, const FTransform& InitTransformA,
	const TArray<FVector>& LinkageB, const TArray<FTransform>& TransformB, const FTransform& InitTransformB)
{
	static int	CheckTimeStamp = GConfig.Get<int>("LinkageProblem", "TimeSteps");
	static auto MeshScale = GConfig.Get<double>("JointMesh", "Scale");
	static auto LinkageRadius = GConfig.Get<double>("JointMesh", "LinkageRadius") * MeshScale;
	static auto Tolerance = GConfig.Get<double>("JointMesh", "CollisionTolerance") * MeshScale;
	static auto LinkageMinDistance = LinkageRadius + Tolerance;

	int TotalTime = std::min(TransformA.size(), TransformB.size());
	int TimeDelta = std::max(TotalTime / CheckTimeStamp, 1);
	int CollisionCount = 0;
	double CollisionForce = 0;
	Matrix4d InitARelative = InitTransformA.GetMatrix().inverse();
	Matrix4d InitBRelative = InitTransformB.GetMatrix().inverse();
	auto ToLocal =
		[&](const FVector& Pos, const Matrix4d& InverseMatrix)-> FVector
		{
			return (InverseMatrix * Pos.homogeneous()).head(3);
		};

	for (int Time = 0; Time < TotalTime; Time += TimeDelta)
	{
		for(int i = 0; i < LinkageA.size() - 1; i ++)
		{
			for (int j = 0;j < LinkageB.size() - 1; j ++)
			{
				auto StartA = TransformA[Time] * ToLocal(LinkageA[i], InitARelative);
				auto EndA = TransformA[Time] * ToLocal(LinkageA[i + 1], InitARelative);
				auto StartB = TransformB[Time] * ToLocal(LinkageB[j], InitBRelative);
				auto EndB = TransformB[Time] * ToLocal(LinkageB[j + 1], InitBRelative);

				auto dis = std::get<0>(SegmentSegmentDistance(StartA, EndA, StartB, EndB));
				if (dis < LinkageMinDistance)
				{
					CollisionCount ++;
					CollisionForce += Penalty(LinkageMinDistance - dis, 0);
				}
			}
		}
	}
	return { CollisionCount, CollisionForce };
}

std::pair<int, double> SurfaceLinkageProblem::CheckLinkageJointRuntimeCollision(
	const TArray<FVector>& LinkageA, const TArray<FTransform>& TransformA, const FTransform& InitTransformA,
	const FBox& Box, const TArray<FTransform>& TransformB, int DelteaIndex)
{
	static int	CheckTimeStamp = GConfig.Get<int>("LinkageProblem", "TimeSteps");
	static auto MeshScale = GConfig.Get<double>("JointMesh", "Scale");
	static auto LinkageRadius = GConfig.Get<double>("JointMesh", "LinkageRadius") * MeshScale;
	static auto Tolerance = GConfig.Get<double>("JointMesh", "CollisionTolerance") * MeshScale;

	int TotalTime = std::min(TransformA.size(), TransformB.size());
	int TimeDelta = std::max(TotalTime / CheckTimeStamp, 1);
	int CollisionCount = 0;
	double CollisionForce = 0;
	Matrix4d InitARelative = InitTransformA.GetMatrix().inverse();

	auto ToLocal =
		[&](const FVector& Pos, const Matrix4d& InverseMatrix)-> FVector
		{
			return (InverseMatrix * Pos.homogeneous()).head(3);
		};

	for (int Time = 0; Time < TotalTime; Time += TimeDelta)
	{
		for(int i = DelteaIndex; i < LinkageA.size() - 1 - DelteaIndex; i ++)
		{
			auto StartA = TransformB[Time].ToLocalSpace(TransformA[Time] * ToLocal(LinkageA[i], InitARelative));
			auto EndA = TransformB[Time].ToLocalSpace(TransformA[Time] * ToLocal(LinkageA[i + 1], InitARelative));

			double dis = CapsuleBoxDistance(StartA, EndA, LinkageRadius, Box);
			if (dis < Tolerance)
			{
				CollisionCount ++;
				CollisionForce += SegmentCollisionForce(dis, Tolerance);
			}
		}
	}

	return { CollisionCount, CollisionForce };
}

std::tuple<TArray<tinyspline::BSpline>, TArray<int>, TArray<int>>
SurfaceLinkageProblem::ParamToSpline(const pagmo::vector_double& dv, bool LogControlPoint) const
{
	int LinkageNum = JointNum - 1;
	TArray<tinyspline::BSpline> LinkageGeometry(LinkageNum); // Original linkage geometry in BSpline
	int ControlPointNum = dv.size() / (LinkageNum * 2);
	TArray<int> SocketPort(LinkageNum + 1, 0);
	TArray<int> JointPort(LinkageNum + 1, 0);

	ASSERT(dv.size() % 2 == 0);

	for(int i = 0;i < LinkageNum;i ++)
	{
		LinkageGeometry[i] = tinyspline::BSpline(2 + ControlPointNum,
			2, std::min(3, ControlPointNum + 1),
			tinyspline::BSpline::Clamped);
		TArray<FVector2> ControlPoint(2 + ControlPointNum);

		for (int j = 0; j < ControlPointNum; j ++)
		{
			ControlPoint[j + 1] = {
				dv[j * 2 + i * ControlPointNum * 2],
				dv[j * 2 + i * ControlPointNum * 2 + 1]};
		}
		auto MinDistance = std::numeric_limits<double>::max();
		for(int j = 0; j < JointPortUV[i].size(); j++)
			if (auto Distance = (Surface->Sample(ControlPoint[1]) - Surface->Sample(JointPortUV[i][j])).norm(); Distance < MinDistance)
			{
				MinDistance = Distance;
				JointPort[i] = j;
			}
		MinDistance = std::numeric_limits<double>::max();
		for (int j = 0; j < SocketPortUV[i + 1].size(); j ++)
			if (auto Distance = (Surface->Sample(ControlPoint[ControlPointNum]) - Surface->Sample(SocketPortUV[i + 1][j])).norm(); Distance < MinDistance)
			{
				MinDistance = Distance;
				SocketPort[i + 1] = j;
			}
		int JointIndex = JointPort[i]; int SocketIndex = SocketPort[i + 1];
		ControlPoint[0] = JointPortUV[i][JointIndex];
		ControlPoint[ControlPointNum + 1] = SocketPortUV[i + 1][SocketIndex];

		// GWorld->DebugDrawPoint(Surface->Sample(JointUV[i]), 5, RGB(0, 0, 100*i));
		// GWorld->DebugDrawPoint(Surface->Sample(JointPortInMeshUV[i][JointIndex]), 3, RGB(255, 0, 0));
		// GWorld->DebugDrawPoint(Surface->Sample(JointPortUV[i][JointIndex]), 3, RGB(255, 0, 0));
		// GWorld->DebugDrawPoint(Surface->Sample(SocketPortInMeshUV[i + 1][SocketIndex]), 3, RGB(0, 255, 0));
		// GWorld->DebugDrawPoint(Surface->Sample(SocketPortUV[i + 1][SocketIndex]), 3, RGB(0, 255, 0));

		if(LogControlPoint) [[unlikely]]
		{
			LOG_INFO("Linkage {} control points: ", i);
			for (int j = 0; j < ControlPoint.size(); j ++)
			{
				auto Pos = Surface->Sample(ControlPoint[j]);
				LOG_INFO("UV pos: {}, World pos: {}", ControlPoint[j], Pos);
			}
		}

		TArray<double> ControlPointData(ControlPoint.size() * 2);
		for (int j = 0; j < ControlPoint.size(); j ++)
		{
			ControlPointData[j * 2] = ControlPoint[j].x();
			ControlPointData[j * 2 + 1] = ControlPoint[j].y();
		}
		LinkageGeometry[i].setControlPoints(ControlPointData);
	}
	return {LinkageGeometry, JointPort, SocketPort};
}


TArray<TArray<FVector>> SurfaceLinkageProblem::LinkageSplineSample(const TArray<tinyspline::BSpline>& LinkageSpline, ParametricMeshActor* Surface, int Samples, bool bShowControlPoints)
{
	int						LinkageNum = LinkageSpline.size();
	TArray<TArray<FVector>> Linkage(LinkageNum); // Sampled points on the linkage geometry
	for (int i = 0; i < LinkageNum; i++)
		Linkage[i] = LinkageSplineSample(LinkageSpline[i], Surface, Samples, bShowControlPoints);

	return Linkage;
}
TArray<FVector2> SurfaceLinkageProblem::LinkageSplineSample2D(const tinyspline::BSpline& LinkageSpline, int Samples)
{
	auto LinkageSample = LinkageSpline.sample(Samples);
	TArray<FVector2> Linkage;
	for(int i = 0;i < LinkageSample.size() / 2; i ++)
	{
		auto u = LinkageSample[i * 2];
		auto v = LinkageSample[i * 2 + 1];
		Linkage.emplace_back(u, v);
	}
	return Linkage;
}

TArray<FVector> SurfaceLinkageProblem::LinkageSplineSample(const tinyspline::BSpline& LinkageSpline, ParametricMeshActor* Surface, int Samples, bool bShowControlPoints)
{
	TArray<FVector> Linkage; // Sampled points on the linkage geometry
	static bool				bPostProcessLinkage = GConfig.Get<bool>("LinkageProblem", "PostProcessLinkage");
	if (bShowControlPoints) [[unlikely]]
	{
		FColor Color = Random::RandomFVector();
		int	   num = LinkageSpline.numControlPoints();
		for (int i = 0; i < num; i++)
		{
			auto CP = LinkageSpline.controlPointVec2At(i);
			auto Pos = Surface->Sample(CP.x(), CP.y());
			GWorld->DebugDrawPoint(Pos, 10, Color);
		}
	}

	auto LinkageSample = LinkageSpline.sample(2048);
	double ChordLength = 0.;
	for(int i = 0;i < LinkageSample.size() / 2; i ++)
	{
		auto u = LinkageSample[i * 2];
		auto v = LinkageSample[i * 2 + 1];
		auto Pos = Surface->Sample(u, v);
		Linkage.push_back(Pos);
		ASSERT(!Pos.hasNaN());
	}

	for (int i = 0; i < Linkage.size() - 1;i ++)
		ChordLength += (Linkage[i + 1] - Linkage[i]).norm();

	double ChordPerSample = ChordLength / Samples;
	TArray<FVector> Result;
	Result.push_back(Linkage[0]);
	double CurrentChordLength = 0.;
	for(int i = 1;i < Linkage.size() - 1;i ++)
	{
		if (CurrentChordLength > ChordPerSample)
		{
			Result.push_back(Linkage[i]);
			CurrentChordLength = 0.;
		}
		else
			CurrentChordLength += (Linkage[i + 1] - Linkage[i]).norm();
	}
	Result.push_back(Linkage.back());
	return Result;
}

void SurfaceLinkageProblem::LinkageConnectJoint(TArray<TArray<FVector>>& Linkages, const TArray<int>& JointIndex, const TArray<int>& SocketIndex) const
{
	ASSERT(JointIndex.size() >= Linkages.size());
	ASSERT(SocketIndex.size() >= Linkages.size());
	for(int i = 0;i < std::min(Linkages.size(), SocketIndex.size() - 1);i ++)
	{
		ASSERTMSG(JointPortIndex[i].size() > JointIndex[i], "PortIndex size not match");
		ASSERTMSG(SocketPortIndex[i + 1].size() > SocketIndex[i + 1], "PortIndex size not match");
		auto& Link = Linkages[i];
		auto JointPos = Joints[i]->GetJointComponent()->GetPinPortInMeshWorldLocation(JointPortIndex[i][JointIndex[i]]);
		// JointPos = Surface->Sample(Surface->Projection(JointPos));
		auto SocketPos = Joints[i + 1]->GetJointComponent()->GetSocketPortInMeshWorldLocation(SocketPortIndex[i + 1][SocketIndex[i + 1]]);
		// SocketPos = Surface->Sample(Surface->Projection(SocketPos));
		Link.insert(Link.begin(), JointPos);
		Link.push_back(SocketPos);
		// Link = Curve::SampleWithEqualChordLength(Link, Link.size());
	}
}

TArray<FVector> SurfaceLinkageProblem::PostProcessLinkage(TArray<FVector> LinkagePath)
{
	static auto LinkageRadius = GConfig.Get<double>("JointMesh", "LinkageRadius");
	double		MinDistance = LinkageRadius * 4;
	int			Sample = LinkagePath.size();
	auto		MergeOnce = [&]() {
		   for (int i = 0; i < LinkagePath.size() - 2; i++)
		   {
			   for (int j = LinkagePath.size() - 2; j >= i + 2; j--)
			   {
				   const auto& StartA = LinkagePath[i];
				   const auto& EndA = LinkagePath[i + 1];
				   const auto& StartB = LinkagePath[j];
				   const auto& EndB = LinkagePath[j + 1];
				   if ((EndA - StartA).dot(EndB - StartB) >= 0)
					   continue;
				   auto [dis, _, __, ___] = SegmentSegmentDistance(StartA, EndA, StartB, EndB);
				   if (dis < MinDistance)
				   {
					   LinkagePath.erase(LinkagePath.begin() + i + 1, LinkagePath.begin() + j);
					   return true;
				   }
			   }
		   }
		   return false;
	};

	while (MergeOnce()){}

	return Curve::SampleWithEqualChordLength(LinkagePath, Sample);
}
double SurfaceLinkageProblem::Penalty(double x, double a)
{
	return std::max(0., x - a) * 1e4;
}

std::pair<pagmo::vector_double, pagmo::vector_double> SurfaceLinkageProblem::get_bounds() const
{
	pagmo::vector_double L;
	pagmo::vector_double R;
	static int ControlPointNum = GConfig.Get<int>("LinkageProblem", "ControlPointNum");
	for (int i = 0; i < ControlPointNum; i ++)
	{
		for(int j = 0;j < JointNum - 1; j ++)
		{
			L.push_back(0.); R.push_back(1.);
			L.push_back(0.); R.push_back(1.);
		}
	}
	return { L, R };
}

#include <pagmo/problem.hpp>
#include <pagmo/algorithms/cmaes.hpp>
#include <pagmo/algorithms/de1220.hpp>
#include <pagmo/algorithms/gaco.hpp>
#include <pagmo/batch_evaluators/thread_bfe.hpp>
#include <pagmo/island.hpp>
#include "MeshFitting/MeshFittingDesigner.h"

void SurfaceLinkageProblem::Init(const SurfaceLinkageConfiguration& Config)
{
	/**
	 * First set global variables for solving linkage problem
	 * Given attached surface and each linkage transform
	 * Given shortest geodetic path as initial guess
	 * Given CollisionBoxBVH for collision detection and calculate score
	 * */
	JointNum = Config.Type.size() - 1;
	EffectorIndex = Config.EffectorIndex;
	EffectorParentIndex = Config.EffectorParentIndex;
	Surface = Config.AttachedSurface.get();
	JointUV = Config.UV;
	for (const auto& Joint : Config.JointActor)
		Joints.push_back(Joint.get());

	ASSERTMSG(Surface != nullptr, "Surface is nullptr");
	ASSERTMSG(Transform.empty(), "Transform is not empty");
	ASSERTMSG(InitTransform.empty(), "InitTransform is not empty");

	static auto AwayFromSurfaceTolerance = GConfig.Get<double>("LinkageProblem", "AwayFromSurfaceTolerance");

	JointPortInMeshUV.resize(JointNum); JointPortUV.resize(JointNum);
	SocketPortInMeshUV.resize(JointNum); SocketPortUV.resize(JointNum);
	JointPortIndex.resize(JointNum); SocketPortIndex.resize(JointNum);

	for (int i = 0; i < JointNum; i++)
	{
		auto JointPortInMeshLocation = Joints[i]->GetJointComponent()->GetPinPortInMeshLocation();
		auto JointPortNum = JointPortInMeshLocation.size();
		TArray<std::pair<double, int>> JointProjectionDistance;
		for (int j = 0; j < JointPortNum; j++)
		{
			FVector	 JointPortInMeshWorld = Joints[i]->GetFTransform() * JointPortInMeshLocation[j];
			FVector2 JointPortInMeshWorldUV = Surface->Projection(JointPortInMeshWorld);
			JointProjectionDistance.emplace_back((Surface->Sample(JointPortInMeshWorldUV) - JointPortInMeshWorld).norm(), j);
		}
		std::ranges::sort(JointProjectionDistance);

		auto PushJoint = [&](int index) {
			FVector	 JointPortInMeshWorld = Joints[i]->GetFTransform() * JointPortInMeshLocation[index];
			FVector2 JointPortInMeshWorldUV = Surface->Projection(JointPortInMeshWorld);
			FVector JointPortWorld = Joints[i]->GetJointComponent()->GetPinPortWorldLocation(index);
			FVector2 JointPortWorldUV = Surface->Projection(JointPortWorld);
			JointPortInMeshUV[i].push_back(JointPortInMeshWorldUV);
			JointPortUV[i].push_back(JointPortWorldUV);
			JointPortIndex[i].push_back(index);
			// GWorld->DebugDrawPoint(JointPortInMeshWorld, 5, RGB(255, 0, 0));
		};
		for (auto [distance, index] : JointProjectionDistance)
		{
			if(distance > AwayFromSurfaceTolerance) break;
			PushJoint(index);
		}
		if (JointPortIndex[i].empty()) PushJoint(JointProjectionDistance[0].second);


		auto SocketPortInMeshLocation = Joints[i]->GetJointComponent()->GetSocketPortInMeshLocation();
		auto SocketPortNum = SocketPortInMeshLocation.size();
		TArray<std::pair<double, int>> SocketProjectionDistance;
		for (int j = 0; j < SocketPortNum; j++)
		{
			FVector SocketPortInMeshWorld = Joints[i]->GetFTransform() * SocketPortInMeshLocation[j];
			FVector2 SocketPortInMeshWorldUV = Surface->Projection(SocketPortInMeshWorld);
			SocketProjectionDistance.emplace_back((Surface->Sample(SocketPortInMeshWorldUV) - SocketPortInMeshWorld).norm(), j);
		}
		auto PushSocket = [&](int index) {
			FVector SocketPortInMeshWorld = Joints[i]->GetFTransform() * SocketPortInMeshLocation[index];
			FVector2 SocketPortInMeshWorldUV = Surface->Projection(SocketPortInMeshWorld);
			FVector SocketPortWorld = Joints[i]->GetJointComponent()->GetSocketPortWorldLocation(index);
			FVector2 SocketPortWorldUV = Surface->Projection(SocketPortWorld);
			SocketPortInMeshUV[i].push_back(SocketPortInMeshWorldUV);
			SocketPortUV[i].push_back(SocketPortWorldUV);
			SocketPortIndex[i].push_back(index);
			// GWorld->DebugDrawPoint(SocketPortInMeshWorld, 5, RGB(0, 255, 0));
			// GWorld->DebugDrawPoint(Surface->Sample(SocketPortWorldUV), 5, RGB(0, 255, 0));
		};
		std::ranges::sort(SocketProjectionDistance);
		for (auto [distance, index] : SocketProjectionDistance)
		{
			if(distance > AwayFromSurfaceTolerance) break;
			PushSocket(index);
		}
		if (SocketPortIndex[i].empty()) PushSocket(SocketProjectionDistance[0].second);
		// Check NAN
		for (const auto& t : JointPortInMeshUV[i])
			ASSERTMSG(!t.hasNaN(), "JointPortInMeshUV has NaN");
		for (const auto& t : JointPortUV[i])
			ASSERTMSG(!t.hasNaN(), "JointPortUV has NaN");
		for (const auto& t : SocketPortInMeshUV[i])
			ASSERTMSG(!t.hasNaN(), "SocketPortInMeshUV has NaN");
		for (const auto& t : SocketPortUV[i])
			ASSERTMSG(!t.hasNaN(), "SocketPortUV has NaN");
	}

	for (const auto& i : Config.JointTransforms)
	{
		ASSERTMSG(!i.empty(), "JointTransforms is empty");
		Transform.push_back(i);
	}
	Transform.push_back(Config.JointTransforms[EffectorParentIndex]); // For effector
	InitTransform = Config.InitTransform;
}
#include <Solver/RandomValidX.h>

std::pair<TArray<TArray<FVector>>, TArray<double>> SurfaceLinkageProblem::Solve(
	const SurfaceLinkageConfiguration& Config,
	SurfaceVoxel*					   Voxels)
{
	SurfaceLinkageProblem Problem(Config, Voxels);

	static constexpr int EvolutionRound = 1;
	static const uint PopulationSize = GConfig.Get<int>("Optimizer", "Population");
	static const int  Generation = GConfig.Get<int>("Optimizer", "Generation");
	pagmo::vector_double BestF;
	pagmo::vector_double BestX;

	pagmo::problem prob{ Problem };

	pagmo::cmaes cmaesInstance(50, -1, -1, -1, -1, 0.5, 1e-6, 1e-3, true, true);
	cmaesInstance.set_bfe(pagmo::bfe(pagmo::thread_bfe()));
	pagmo::algorithm algo{ cmaesInstance };
	algo.set_verbosity(1);
	spdlog::stopwatch TotalTimer;
	spdlog::stopwatch RoundTimer;
	pagmo::population pop{ prob, pagmo::bfe(pagmo::thread_bfe()), PopulationSize };
	pagmo::island	  Island(algo, pop);
	for (int i = 1; i <= PopulationSize / 50; i++)
	{
		Island.evolve();
		Island.wait();
		auto F = Island.get_population().champion_f();
		auto X = Island.get_population().champion_x();
		if (BestF.empty() || BestF[0] > F[0])
		{
			BestF = F;
			BestX = X;
		}
		LOG_INFO("----------------------------------------");
		LOG_INFO("Round : {} / {}", i, PopulationSize / 50);
		LOG_INFO("Round Time: {} s / Total Time: {}s", RoundTimer, TotalTimer);
		LOG_INFO("Best F: {} ", BestF);
		LOG_INFO("Best X: {} ", BestX);
		LOG_INFO("----------------------------------------");
	}
	Problem.DebugLog = true;
	auto _ = prob.fitness(BestX);

	auto [Splines, J, S] = Problem.ParamToSpline(BestX);
	static bool DebugShowControlPoints = GConfig.Get<bool>("LinkageProblem", "DebugShowControlPoints");
	auto Linkage = Problem.LinkageSplineSample(Splines, Problem.Surface, 512, DebugShowControlPoints);
	Problem.LinkageConnectJoint(Linkage, J, S);
	return {Linkage, BestX};
}
double SurfaceLinkageProblem::SegmentCollisionForce(double Distance, double Threshold)
{
	if (Distance >= Threshold)
		return 0;

	return 1e4 * std::cos(M_PI * 0.5 * std::clamp(Distance / Threshold, 0., 1.));
}

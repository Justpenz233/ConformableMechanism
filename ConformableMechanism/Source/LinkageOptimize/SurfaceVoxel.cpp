//
// Created by MarvelLi on 2024/5/21.
//

#include "SurfaceVoxel.h"

#include "Fcl.h"

#include <bvh/v2/default_builder.h>
#include <bvh/v2/stack.h>
#include <igl/AABB.h>
#include <igl/fast_winding_number.h>
#include <igl/signed_distance.h>
#include <spdlog/stopwatch.h>

#include "SurfaceLinkageProblem.h"
#include "MeshFitting/MeshFittingDesigner.h"
#include "Misc/Config.h"
#include "pmp/surface_mesh.h"
#include "pmp/algorithms/geodesics.h"

#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>

bvh::v2::Vec<double,3> ToVec3(const FVector& v)
{
	return bvh::v2::Vec<double,3>(v.x(), v.y(), v.z());
}

void SurfaceVoxel::Build()
{
	ASSERTMSG(Surface != nullptr, "Surface is nullptr");
	Graph = Algorithm::GraphTheory::Graph<double>();

	spdlog::stopwatch sw;
	SurfaceMesh = Surface->GetSurfaceMesh();
	SurfaceMesh->TransformMesh(Surface->GetFTransform());


	// ----- Compute SDF for surface  -----
	igl::AABB<MatrixX3d, 3> tree;
	igl::FastWindingNumberBVH fwn_bvh;

	// ----- Construct the collision box-----
	{
		int BoxNum = SurfaceMesh->GetFaceNum();
		BoxScore.resize(BoxNum);
		CollisionBoxes.resize(BoxNum);

		for(int i = 0; i < BoxNum; i ++)
		{
			auto Face = SurfaceMesh->GetTriangle(i);
			auto V0 = SurfaceMesh->GetVertex(Face[0]);
			auto V1 = SurfaceMesh->GetVertex(Face[1]);
			auto V2 = SurfaceMesh->GetVertex(Face[2]);
			auto A = abs((V1 - V0).cross(V2 - V0).norm() * 0.5);
			BoxScore[i] = A;
			CollisionBoxes[i] = FBox{TArray<FVector>{V0, V1, V2}};
			Graph.AddNode(i, BoxScore[i]); // Create a node for current box, we will remove empty box later
		}
		CollisionBoxSumScores = std::accumulate(BoxScore.begin(), BoxScore.end(), 0.);
	}

	// Build Graph
	{
		pmp::SurfaceMesh Mesh;
		for (int i = 0; i < SurfaceMesh->GetVertexNum(); i ++)
			Mesh.add_vertex(SurfaceMesh->GetVertex(i));

		for (int i = 0; i < SurfaceMesh->GetFaceNum(); i ++)
		{
			auto Face = SurfaceMesh->GetTriangle(i);
			Mesh.add_triangle(pmp::Vertex(Face[0]), pmp::Vertex(Face[1]), pmp::Vertex(Face[2]));
		}

		for (auto Face : Mesh.faces())
		{
			int FId = Face.idx();
			for (auto Edge : Mesh.halfedges(Face))
				if(auto Next = Mesh.face(Mesh.opposite_halfedge(Edge)); Next.is_valid())
				{
					ASSERT(FId < SurfaceMesh->GetFaceNum());
					ASSERT(Next.idx() < SurfaceMesh->GetFaceNum());
					Graph.AddDirectionalEdge(FId, Next.idx());
				}
		}
	}

	ASSERT(Graph.ConnectedComponents().size() == 1);
	BuildBVH();
	LOG_INFO("Build surface voxel and graph cost {} s", sw);
}


TArray<int> SurfaceVoxel::IntersectCubes(const FVector& Start, const FVector& End) const
{
	auto ray = Ray{
		ToVec3(Start),						// Ray origin
		ToVec3((End - Start).normalized()), // Ray direction
		0.,
		(End - Start).norm() // Ray length
	};
	static double LinkageRadius =
		GConfig.Get<double>("JointMesh", "LinkageRadius") *
			GConfig.Get<double>("JointMesh", "Scale");
	static constexpr size_t			  stack_size = 4096;
	bvh::v2::GrowingStack<Bvh::Index> stack;
	TArray<int>						  IntersectedCubes;
	bvh.intersect<false, true>(ray, bvh.get_root().index, stack,
		[&](size_t begin, size_t end) {
			bool LeafHit = false;
			for (size_t i = begin; i < end; ++i)
			{
				auto CubeID = bvh.prim_ids[i];
				ASSERT(CubeID < CollisionBoxes.size());
				auto Hit = CollisionBoxes[CubeID].Intersect(Start, End);
				bool HasHit = false;
				for (auto HitT : Hit)
					HasHit |= (HitT > 0 && HitT < 1);
				if (HasHit)
					IntersectedCubes.push_back(CubeID);
				LeafHit |= HasHit;
			}
			return LeafHit;
		});

	std::ranges::sort(IntersectedCubes);
	IntersectedCubes.erase(std::ranges::unique(IntersectedCubes).begin(), IntersectedCubes.end());
	return IntersectedCubes;
}

TArray<int> SurfaceVoxel::IntersectBox(const FBox& Box, const FTransform& BoxTransform) const
{
	TArray<int> IntersectedCubes;
	auto ToLocal = BoxTransform.GetMatrix().inverse();
	for (int i = 0;i < CollisionBoxes.size();i ++)
	{
		FVector Center = BoxTransform.ToLocalSpace(CollisionBoxes[i].GetCenter());
		if (Box.Contain(Center))
			IntersectedCubes.push_back(i);
	}
	return IntersectedCubes;
}


TArray<int> SurfaceVoxel::IntersectBoundingVolume(const TFunction<bool(FVector)>& BV, const FTransform& BoxTransform) const
{
	TArray<int> IntersectedCubes;
	Matrix4d ToLocal = BoxTransform.GetMatrix().inverse();
	for (int i = 0;i < SurfaceMesh->GetFaceNum();i ++)
	{
		Vector3i Triangle = SurfaceMesh->GetTriangle(i);
		for(int j = 0; j < 3; j ++)
		{
			auto V = SurfaceMesh->GetVertex(Triangle[j]);
			if (BV((ToLocal * V.homogeneous()).head(3)))
			{
				IntersectedCubes.push_back(i);
				break;
			}
		}
	}
	return IntersectedCubes;
}

TArray<int> SurfaceVoxel::LinkageRuntimeCollision(
	const TArray<TArray<FVector>>& LinkageSample,
	const TArray<TArray<FTransform>>& Transform,
	const TArray<FTransform>& InitTransform,
	const TArray<TFunction<bool(FVector)>>& JointBV) const
{
	ASSERTMSG(LinkageSample.size() <= Transform.size(), "LinkageSample size not match Transform size; {} > {}", LinkageSample.size(), Transform.size());
	ASSERTMSG(LinkageSample.size() <= InitTransform.size(), "Transform size not match InitTransform size. {} instead of {}",
		LinkageSample.size(), InitTransform.size());

	int TimeStep = Transform[0].size();
	TArray<bool> CollisionDetected(Size(), false);
	TSet<int> InitCollide;
	TSet<int> DrivenCollide;
	for(int i = 0; i < JointBV.size(); i ++)
	{
		if(i == JointBV.size() - 1) continue;
		for(int j = 0; j < Transform[i].size(); j += 5)
		{
			auto Intersections = IntersectBoundingVolume(JointBV[i], Transform[i][j]);
			for (auto CubeID : Intersections)
			{
				CollisionDetected[CubeID] = true;
				if(j == 0) InitCollide.insert(CubeID);
				if (i == 0 && j == 0) DrivenCollide.insert(CubeID);
			}
		}
	}

	for (int i = 0; i < LinkageSample.size() - 1; i++)
	{
		for (int Time = 0; Time < TimeStep; Time++)
		{
			auto ToLocal = [&](int index, const FVector& Point) -> FVector {
				return (InitTransform[index].GetMatrix().inverse() * Point.homogeneous()).head(3);
			};
			for (int j = 0; j < LinkageSample[i].size() - 1; j++)
			{
				FVector Start, End;
				// if (i == LinkageSample.size() - 1) // Effector handle
				// {
				// 	Start = Transform[1][Time] * ToLocal(1, LinkageSample[i][j]);
				// 	End = Transform[1][Time] * ToLocal(1, LinkageSample[i][j + 1]);
				// }
				// else
				// {
					Start = Transform[i][Time] * ToLocal(i, LinkageSample[i][j]);
					End = Transform[i][Time] * ToLocal(i, LinkageSample[i][j + 1]);
				// }
				auto IntersectedCubes = IntersectCubes(Start, End);
				for (auto CubeID : IntersectedCubes)
				{
					if (Time == 0) InitCollide.insert(CubeID);
					CollisionDetected[CubeID] = true;
				}
			}
		}
	}
	auto ExpandedCollision = ExpandCollisionByGraph(CollisionDetected);
	TArray<int> Result(Size(), CollisionType::NoCollision);
	for (int i = 0; i < CollisionDetected.size(); i++)
	{
		if (CollisionDetected[i])
			Result[i] = CollisionType::BasicCollision;
		if (!CollisionDetected[i] && ExpandedCollision[i])
			Result[i] = CollisionType::Disconnected;
		// if (InitCollide.contains(i))
			// Result[i] = CollisionType::InitCollision;
	}
	static double WeakSturectureThreshold = GConfig.Get<double>("SurfaceGraph", "WeakStructureThreshold");

	auto FragilePart = DetectFragilePart(WeakSturectureThreshold, Result);

	for (auto ID : FragilePart)
		Result[ID] = CollisionType::WeakSturecture;

	for (auto ID: DrivenCollide)
		Result[ID] = CollisionType::NoCollision;

	ExpandCollisionByGraph(Result);

	return Result;
}

TArray<bool> SurfaceVoxel::ExpandCollisionByGraph(TArray<bool> CollisionDetected) const
{
	auto Result = Graph.ConnectedComponents([&](auto ID) {
		return !CollisionDetected[ID];
	});
	for (int i = 1; i < Result.size(); i++)
		for (const auto& ID : Result[i])
			CollisionDetected[ID] = true;

	return CollisionDetected;
}

void SurfaceVoxel::ExpandCollisionByGraph(TArray<int>& CollisionDetected) const
{
	auto Result = Graph.ConnectedComponents([&](auto ID) {
	return CollisionDetected[ID] == NoCollision;
	});

	for (int i = 1; i < Result.size(); i++)
		for (const auto& ID : Result[i])
			CollisionDetected[ID] = Disconnected;
}

int SurfaceVoxel::Size() const
{
	return CollisionBoxes.size();
}

SurfaceVoxel::CollisionScores SurfaceVoxel::CalculateScore(const TArray<int>& CollisionDetected) const
{
	ASSERTMSG(CollisionDetected.size() == CollisionBoxes.size(), "CollisionDetected size not match CollisionBoxes size");
	double BasicCollisionScore = 0;
	double DisconnectedScore = 0;
	double WeakSturectureScore = 0;
	for (int i = 0; i < CollisionDetected.size(); i++)
	{
		if (CollisionDetected[i] == CollisionType::BasicCollision)
			BasicCollisionScore += BoxScore[i];
		else if (CollisionDetected[i] == CollisionType::Disconnected)
			DisconnectedScore += BoxScore[i];
		else if (CollisionDetected[i] == CollisionType::WeakSturecture)
			WeakSturectureScore += BoxScore[i];
	}
	return {BasicCollisionScore, DisconnectedScore, WeakSturectureScore};
}

void SurfaceVoxel::DebugDraw(World* World)
{
	for (const auto & CollisionBoxe : CollisionBoxes)
	{
		FColor Color = RGB(0, 0, 0);
		World->DebugDrawCube(CollisionBoxe.GetCenter(), CollisionBoxe.GetSize(), Color, 1.f);
	}

	static bool bShowGraph = GConfig.Get<bool>("SurfaceGraph", "DebugShowGraph");
	if (bShowGraph)
	{
		for(int i = 0; i < CollisionBoxes.size(); i ++)
		{
			auto Node = Graph.GetNode(i);
			for (auto Edge : Node.OutEdge)
			{
				if(Edge->EndNodeID > i)
				{
					auto NextBox = CollisionBoxes[Edge->EndNodeID];
					World->DebugDrawLine(CollisionBoxes[i].GetCenter(), NextBox.GetCenter(), RGB(0, 255, 0), 1.);
				}
			}
		}
	}
}

void SurfaceVoxel::DebugDrawCollision(World* World, const TArray<int>& CollisionDetected) const
{
	static double LinkageRadius =
	GConfig.Get<double>("JointMesh", "LinkageRadius") *
		GConfig.Get<double>("JointMesh", "Scale");

	for (int i = 0;i < CollisionDetected.size();i ++)
	{
		auto Box = CollisionBoxes[i];
		Box.Min += FVector::Constant(LinkageRadius * 0.5);
		Box.Max -= FVector::Constant(LinkageRadius * 0.5);
		FColor Color;
		// Different type should use different color
		if (CollisionDetected[i] == NoCollision)
			Color = RGB(0, 0, 0);
		if (CollisionDetected[i] == BasicCollision)
			Color = RGB(255, 0, 0);
		if (CollisionDetected[i] == Disconnected)
			Color = RGB(0, 255, 0);
		if (CollisionDetected[i] == WeakSturecture)
			Color = RGB(0, 0, 255);
		if (CollisionDetected[i] == InitCollision)
			Color = RGB(255, 255, 0);
		World->DebugDrawPoint(Box.GetCenter(),  4.f, Color);
		// World->DebugDrawCube(Box.GetCenter(), Box.GetSize(), Color, 1.f);
	}
}

TArray<int> SurfaceVoxel::DetectFragilePart(double Threshold, const TArray<int>& CollisionState) const
{
	pmp::SurfaceMesh Mesh;
	for (int i = 0; i < SurfaceMesh->GetVertexNum(); i ++)
		Mesh.add_vertex(SurfaceMesh->GetVertex(i));
	for (int i = 0; i < SurfaceMesh->GetFaceNum(); i ++)
	{
		auto Face = SurfaceMesh->GetTriangle(i);
		Mesh.add_triangle(pmp::Vertex(Face[0]), pmp::Vertex(Face[1]), pmp::Vertex(Face[2]));
	}

	static double Tolerance = GConfig.Get<double>("SurfaceGraph", "Tolerance");
	// First remove the collision triangles
	TSet<int> CollideTriangles;
	for (int i = 0; i < CollisionState.size(); i++)
		if (CollisionState[i] != NoCollision)
			CollideTriangles.insert(i);

	auto Dijkstra = [&](const TArray<int>& Start) {
		TArray<pmp::Vertex> StartVertices;
		for (auto ID : Start)
		{
			if (ID < Mesh.vertices_size())
			StartVertices.emplace_back(ID);
		}
		pmp::geodesics(Mesh, StartVertices);
		auto distances = Mesh.get_vertex_property<float>("geodesic:distance");
		TArray<double> Result(Mesh.vertices_size());
		for (int i = 0; i < Mesh.vertices_size(); i++)
			Result[i] = distances[pmp::Vertex(i)];
		return Result;
	};

	TArray<int> F;
	for (int i = 0; i < SurfaceMesh->GetFaceNum(); i++)
	{
		if (CollideTriangles.contains(i)) continue;
		int NextCount = 0;
		for(auto Next: Graph.GetNode(i).OutEdge)
		{
			auto NextID = Next->EndNodeID;
			if (CollideTriangles.contains(NextID)) continue;
			NextCount ++;
		}
		if (NextCount < 3)
			F.push_back(i);
	}


	// T_h(D(f), threshold)
	TArray<double> Dis = Dijkstra(F);

	TArray<int> Th_F;
	for (int i = 0; i < Dis.size(); i++)
		if (Dis[i] > Threshold && !CollideTriangles.contains(i))
			Th_F.push_back(i);

	Dis = Dijkstra(Th_F);

	TSet<int>Tl_Th_F;
	for (int i = 0; i < Dis.size(); i++)
		if (Dis[i] <= Threshold + 0.01 && !CollideTriangles.contains(i))
			Tl_Th_F.insert(i);

	TArray<int> Result;
	for(int i = 0;i < SurfaceMesh->GetFaceNum();i ++)
		if (!CollideTriangles.contains(i) && !Tl_Th_F.contains(i))
			Result.push_back(i);
	return Result;
}



void SurfaceVoxel::BuildBVH()
{
	static double LinkageRadius =
		GConfig.Get<double>("JointMesh", "LinkageRadius") *
			GConfig.Get<double>("JointMesh", "Scale");

	bvh::v2::ThreadPool thread_pool;
	std::vector<BBox> bboxes(CollisionBoxes.size());
	std::vector<Vec3> centers(CollisionBoxes.size());
	for (size_t i = 0; i < CollisionBoxes.size(); i++)
	{
		auto& box = CollisionBoxes[i];
		box.Min -= Vector3d::Constant(LinkageRadius * 0.5);
		box.Max += Vector3d::Constant(LinkageRadius * 0.5);
		bboxes[i] = BBox(Vec3(box.Min.x(), box.Min.y(), box.Min.z()), Vec3(box.Max.x(), box.Max.y(), box.Max.z()));
		centers[i] = bboxes[i].get_center();
	}
	typename bvh::v2::DefaultBuilder<Node>::Config config;
	config.quality = bvh::v2::DefaultBuilder<Node>::Quality::High;
	bvh = bvh::v2::DefaultBuilder<Node>::build(thread_pool, bboxes, centers, config);
}

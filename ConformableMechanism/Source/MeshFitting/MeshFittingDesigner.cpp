//
// Created by MarvelLi on 2024/2/26.
//

#include "Core/CoreMinimal.h"
#include "MeshFittingDesigner.h"
#include "FKProblem.h"
#include "Algorithm/GeometryProcess.h"
#include "Mesh/MeshBoolean.h"
#include "Math/Random.h"
#include "Misc/Config.h"
#include "TargetPointsActor.h"
#include "SpacialMechanism/SurfaceJoint.h"
#include "pagmo/problem.hpp"
#include <pagmo/algorithms/cmaes.hpp>
#include "Math/Intersect.h"
#include "spdlog/stopwatch.h"

TArray<double> MeshFittingDesigner::RandomParameter()
{
	TArray<double> Result(8);
	for (int i = 0; i < 8; i++)
	{
		if (i & 1)
			Result[i] = Random::RandomInterval(0.1, 0.9);
		else
			Result[i] = Random::Random();
	}
	return Result;
}

template <>
struct fmt::formatter<pagmo::problem> : fmt::formatter<String> {
	template <typename FormatContext>
	auto format(const pagmo::problem& a, FormatContext& ctx) {
		std::ostringstream stream;
		stream << a;
		String Context = fmt::format("{}", stream.str());
		return fmt::formatter<String>::format(Context, ctx);
	}
};

#include <pagmo/algorithms/de1220.hpp>
#include <pagmo/algorithms/gaco.hpp>
#include <pagmo/batch_evaluators/thread_bfe.hpp>
#include <pagmo/island.hpp>

MeshFittingDesigner::Parameter MeshFittingDesigner::RandomValidParameter(String Type, ObjectPtr<ReferenceSurface> Surface, const TArray<ObjectPtr<TargetPointsActor>>& TargetPoints)
{
	TArray<FVector> Targets(TargetPoints.size());
	for (int i = 0; i < TargetPoints.size(); i++)
		Targets[i] = TargetPoints[i]->GetTranslation();

	static const int GenereationPerBatch = GConfig.Get<int>("DesignerConfig", "GenerationPerBatch");
	pagmo::problem	 prob{ FKProblem(Type, Surface, Targets) };
	LOG_INFO(prob);

	pagmo::cmaes cmaesInstance(GenereationPerBatch, -1, -1, -1, -1, 0.5, 1e-6, 1e-4, true);
	cmaesInstance.set_bfe(pagmo::bfe(pagmo::thread_bfe()));
	pagmo::algorithm algo{ cmaesInstance };
	algo.set_verbosity(1);
	pagmo::island Island{algo, prob, 100};
	while (true)
	{
		Island.evolve();
		Island.wait_check();

		auto F = Island.get_population().champion_f();
		auto X = Island.get_population().champion_x();
		if(F[0] < 1e2)
		{
			LOG_INFO("Find one: {}", X);
			return TArray<double>(X);
		}
	}
}

#include "Solver/RandomValidX.h"
TArray<double> MeshFittingDesigner::SolveJoint(String Type, ObjectPtr<ReferenceSurface> InSurface, const TArray<FVector>& InTargetPoints, int Generation, int PopulationSize)
{
	static const int EvolutionRound = GConfig.Get<int>("DesignerConfig", "EvolutionRound");
	pagmo::vector_double BestF;
	pagmo::vector_double BestX;

	pagmo::problem	 prob{ FKProblem(Type, InSurface, InTargetPoints) };
	LOG_INFO(prob);

	pagmo::cmaes cmaesInstance(Generation, -1, -1, -1, -1, 0.3, 1e-6, 1e-6, false, true);
	cmaesInstance.set_bfe(pagmo::bfe(pagmo::thread_bfe()));
	pagmo::algorithm algo{ cmaesInstance };
	algo.set_verbosity(1);
	static auto InitSamples = GConfig.Get<int>("DesignerConfig", "InitSamples");
	for (int i = 1; i <= EvolutionRound; i ++)
	{
		auto Population = RandomValidX(prob, [](double F) { return F < 1e3; }, PopulationSize, InitSamples);
		pagmo::island Island{algo, Population };
		Island.evolve();
		Island.wait();
		auto F = Island.get_population().champion_f();
		auto X = Island.get_population().champion_x();
		if(BestF.empty() || BestF[0] > F[0])
		{
			BestF = F;
			BestX = X;
		}
		LOG_INFO("----------------------------------------");
		LOG_INFO("Round : {} / {}", i, EvolutionRound);
		LOG_INFO("Type : {}", Type);
		LOG_INFO("Best F: {} ", BestF);
		LOG_INFO("Best X: {} ", BestX);
		LOG_INFO("----------------------------------------");
	}
	return BestX;
}

ObjectPtr<StaticMesh> MeshFittingDesigner:: CutSurface(SurfaceLinkageConfiguration& Config)
{
	static auto Tolerance = GConfig.Get<double>("JointMesh", "Tolerance");

	// Copy the surface mesh
	auto SurfaceMesh = NewObject<StaticMesh>(*Config.AttachedSurface->GetParametricMeshComponent()->GetMeshData());
	auto& JointActor = Config.JointActor;
	SurfaceMesh->TransformMesh(Config.AttachedSurface->GetFTransform().GetMatrix());
	int LinkageNum = Config.JointActor.size() - 2;
	Config.LinkageSweptVolume.resize(LinkageNum);
	TArray<ObjectPtr<StaticMesh>> JointMeshes(LinkageNum);
	TArray<bool> IsCollide(LinkageNum);

	auto Cylinder = BasicShapesLibrary::GenerateCylinder(0.125, 0.08);
	Cylinder->TransformMesh(JointActor[0]->GetTransformMatrix());
	Cylinder->SaveOBJ(Path::ProjectContentDir() / "Cylinder.obj");

	ParallelFor(LinkageNum, [&](int i) {
		// Here we assume the joint mesh is already create
		double PreTolerance = JointActor[i]->GetJointComponent()->DefaultTolerance;
		double PrebPrint = JointActor[i]->GetJointComponent()->bPrint;
		double PreRadius = JointActor[i]->GetJointComponent()->LinkageRadius;
		JointActor[i]->GetJointComponent()->DefaultTolerance = 0.;
		JointActor[i]->GetJointComponent()->LinkageRadius = PreRadius + Tolerance * 2;

		JointMeshes[i] = NewObject<StaticMesh>(*JointActor[i]->GetJointComponent()->CreateJointMeshWithTolerance(Tolerance, 2.* Tolerance));
		LOG_INFO("Start calc swept volume for joint {}", i);
		auto Mesh = JointMeshes[i];
		static int JointMeshSweptVolumeSteps = GConfig.Get<int>("DesignerConfig", "JointMeshSweptVolumeSteps");
		int Steps = JointMeshSweptVolumeSteps;
		int GridSize = 200;
		Config.LinkageSweptVolume[i] = Algorithm::GeometryProcess::SweptVolume(Mesh, Config.JointTransforms[i], Steps, GridSize)->GetThis<StaticMesh>();
		Config.LinkageSweptVolume[i]->OffsetVertex(Tolerance);
		LOG_INFO("End calc swept volume for joint {}", i);
		JointActor[i]->GetJointComponent()->DefaultTolerance = PreTolerance;
		JointActor[i]->GetJointComponent()->bPrint = PrebPrint;
		JointActor[i]->GetJointComponent()->LinkageRadius = PreRadius;
	});
	if(GConfig.Get<bool>("DEBUG", "ShowJointSweptVolume"))
	{
		for (int i = 0;i < LinkageNum;i ++)
			GWorld->SpawnActor<StaticMeshActor>("SweptVolume" + std::to_string(i), Config.LinkageSweptVolume[i]);
	}

	// Dig hole in surface
	for(int i = 0; i < Config.LinkageSweptVolume.size();i ++)
	{
		const auto& SweptVolume = Config.LinkageSweptVolume[i];
		{
			// SweptVolume->SaveOBJ(Path::ProjectContentDir() / ("SweptVolume" + std::to_string(i) + ".obj"));
			SurfaceMesh = MeshBoolean::MeshMinus(SurfaceMesh, SweptVolume);

			auto SubMeshes = Algorithm::GeometryProcess::DivideMeshIntoComponents(SurfaceMesh);
			if(SubMeshes.size() == 1) continue;
			// Return the largest volume submesh
			auto LargestSubMesh = *std::ranges::max_element(SubMeshes,
				[](const ObjectPtr<StaticMesh>& a, const ObjectPtr<StaticMesh>& b) {
				return a->CalcVolume() < b->CalcVolume();
			});
			SurfaceMesh = LargestSubMesh;
		}
	}
	TArray<bool> PreDigHole(Config.JointActor.size());
	for (int i = 0;i < Config.JointActor.size();i ++)
	{
		PreDigHole[i] = JointActor[i]->GetJointComponent()->bPrint;
		JointActor[i]->GetJointComponent()->bPrint = false;
	}

	// Last cut joint0 and socket3 bbox
	{
		int LastIndex = Config.JointActor.size() - 2;
		auto Joint0Mesh = JointActor[0]->GetJointComponent()->GeneratePinMeshWithTolerance(Tolerance, Tolerance * 2.);
		Joint0Mesh->TransformMesh(JointActor[0]->GetTransformMatrix());
		SurfaceMesh = MeshBoolean::MeshMinus(SurfaceMesh, Joint0Mesh);
		auto Socket3Mesh = JointActor[LastIndex]->GetJointComponent()->GenerateSocketMeshWithTolerance(Tolerance, Tolerance * 2.);
		Socket3Mesh->TransformMesh(JointActor[LastIndex]->GetTransformMatrix());
		SurfaceMesh = MeshBoolean::MeshMinus(SurfaceMesh, Socket3Mesh);
	}
	for (int i = 0;i < JointActor.size();i ++)
		JointActor[i]->GetJointComponent()->bPrint = PreDigHole[i];

	SurfaceMesh->TransformMesh(Config.AttachedSurface->GetTransform().inverse().matrix());

	return SurfaceMesh;
}

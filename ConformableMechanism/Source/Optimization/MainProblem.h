//
// Created by marvel on 22/7/24.
//

#pragma once
#include "EffectorProblem.h"
#include "JointParameterDefine.h"
#include "MechanismSequence.h"
#include "Optimization/ReferenceSurface.h"
#include "Core/CoreMinimal.h"
#include "LinkageOptimize/SurfaceVoxel.h"
#include "pagmo/types.hpp"
#include "MechanismMobility.h"
#include "MeshFitting/FKProblem.h"
#include "MeshFitting/MeshFittingDesigner.h"

#include <pagmo/algorithms/maco.hpp>
#include <pagmo/algorithms/moead_gen.hpp>
#include <pagmo/algorithms/nspso.hpp>
#include <pagmo/algorithms/pso_gen.hpp>

class MainProblem
{
public:
	MainProblem() = default;

	MainProblem(const String& InType, const ObjectPtr<ReferenceSurface>& InSurface,
		const TArray<FVector>& InTargetPoints, const String& InMobility, const ObjectPtr<SurfaceVoxel>& InVoxel, const TArray<double>& InJointParams = {});

	MainProblem(const String& InType, const ObjectPtr<ReferenceSurface>& InSurface,
		const TArray<FTransform>& InTargetTransform, const String& InMobility, const ObjectPtr<SurfaceVoxel>& InVoxel, const TArray<double>& InJointParams = {});

	int GetJointParameterNumber() const;

	void FixJointUV(int JointIndex, const FVector2& FixedUV);

	pagmo::vector_double RealParameter(const pagmo::vector_double& dv) const;

	[[nodiscard]] std::tuple<pagmo::vector_double, pagmo::vector_double>
	DivideParameter(const pagmo::vector_double& dv) const;

	[[nodiscard]] pagmo::vector_double fitness(const pagmo::vector_double& dv) const;

	[[nodiscard]] std::pair<SurfaceLinkageConfiguration, TArray<double>> CalculateConfig(const pagmo::vector_double& dv) const;

	[[nodiscard]] std::pair<pagmo::vector_double, pagmo::vector_double> GetCurveFittingBounds() const;
	[[nodiscard]] std::pair<pagmo::vector_double, pagmo::vector_double> GetLinkageGeometryBounds() const;
	[[nodiscard]] std::pair<pagmo::vector_double, pagmo::vector_double> get_bounds() const;

	// Random, Real
	std::pair<TArray<double>, TArray<double>> RandomJointParameter() const;
	[[nodiscard]] pagmo::vector_double::size_type get_nobj() const
	{
		if (bOptimizeLinkageOnly)
			return 1;
		else
			return Mobility == Transmobile ? 2 + bLinkageMultiTarget : 3 + bLinkageMultiTarget;
	}

	template <typename T> requires std::is_same_v<T, FVector> || std::is_same_v<T, FTransform>
	static void Solve(
		const String& InType, const ObjectPtr<ReferenceSurface>& InSurface,
		const TArray<T>& InTargetPoints, const String& InMobility);

	template <typename T> requires std::is_same_v<T, FVector> || std::is_same_v<T, FTransform>
	static void SolveLinkageOnly(
		const String& InType, const ObjectPtr<ReferenceSurface>& InSurface,
		const TArray<T>& InTargetPoints, const String& InMobility,
		const pagmo::vector_double& dv);

	pagmo::thread_safety get_thread_safety() const
	{
		return pagmo::thread_safety::constant;
	}

	bool bDebugLog = false;

protected:
	TMap<int, FVector2> FixedUVs;

	bool bOptimizeLinkageOnly = false;
	bool bLinkageMultiTarget = false;

	TArray<double> JointParams = {};

	String Type;
	SurfaceVoxel* Voxel;
	ObjectPtr<ReferenceSurface> Surface;
	TArray<FVector> TargetPoints;
	TArray<FTransform> TargetMotion;
	MechanismMobility Mobility;
	int JointNum = 0;
};

#include <pagmo/island.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/algorithms/cmaes.hpp>
#include <pagmo/algorithms/nsga2.hpp>
#include <pagmo/batch_evaluators/thread_bfe.hpp>

template <typename T> requires std::is_same_v<T, FVector> || std::is_same_v<T, FTransform>
void MainProblem::Solve(const String& InType,
	const ObjectPtr<ReferenceSurface>& InSurface,
	const TArray<T>& InTargetPoints,
	const String& InMobility)
{
	auto				  Voxels = NewObject<SurfaceVoxel>(InSurface.get());
	static TArray<String> FourBarType = { "DUESR", "DSERU", "DRESU", "DUERS", "DUEUU", "DREUS" };

	bool EnumerateType = GConfig.Get<bool>("Optimizer", "EnumerateType");
	int InitSamples = GConfig.Get<int>("Optimizer", "InitSamples");
	TArray<String> TypesCandidate;
	if (EnumerateType) TypesCandidate = FourBarType;
	else TypesCandidate.push_back(InType);

	constexpr bool bSkipInvalid = true;

	struct Result
	{
		String		   Type;
		TArray<double> F;
		TArray<double> X;
	};
	auto PrintResult = [&](TArray<Result>& Results) {
		LOG_INFO("Solve problem for type {}", TypesCandidate);
		LOG_INFO("Actor name : {0}", InSurface->GetName());
		LOG_INFO("Mobility: {0}", InMobility);
		LOG_INFO("Actor transform: {0}", InSurface->GetFTransform());

		LOG_INFO("---------------- Fitting first ----------------");
		std::ranges::sort(Results, [](const auto& A, const auto& B) {
			for (auto i = 0; i < A.F.size(); i++)
				if (A.F[i] != B.F[i])
					return A.F[i] < B.F[i];
			return false;
		});

		for (int i = 0; i < std::min(int(Results.size()), 500); i++)
			LOG_INFO("Type {0} F: {1} X: {2}", Results[i].Type, Results[i].F, Results[i].X);
	};

	TArray<Result> Results;
	for (const auto& Type : TypesCandidate)
	{
		MainProblem	   Problem(Type, InSurface, InTargetPoints, InMobility, Voxels);

		TArray<std::pair<double, TArray<double>>> InitParams(InitSamples);
		if (!EnumerateType && InitSamples > 0)
		{
			int					   ThreadNum = std::thread::hardware_concurrency();
			std::atomic<int>	   Count = 0;
			ParallelFor(ThreadNum, [&](int __) {
				while (true)
				{
					auto [Random, Params] = Problem.RandomJointParameter();
					FKProblemSolver Solver(Type, InSurface);
					auto			Fitness = Solver.Calc(Params);
					auto			JointConstrainScore = FKProblem::WeightedFitness(Fitness);
					if (JointConstrainScore >= 1e-4) continue;

					int Id = Count.fetch_add(1);
					if ((Id + 1) % 100 == 0)
						LOG_INFO("Already samples {0} for type {1}", Id, Type);
					if (Id >= InitSamples) break;
					SurfaceLinkageConfiguration Config;
					{
						Config.OriginalTime = Solver.TimeClipCount;
						Config.Type = Solver.Type;
						Config.EffectorParentIndex = Solver.EffectorParentIndex;
						Config.AttachedSurface = Solver.Surface;

						for (const auto& i : Solver.Record.JointInitTransforms)
							Config.InitTransform.push_back(i);

						Config.JointTransforms.resize(Solver.Record.JointTransforms[0].size());
						for (auto i : Solver.Record.JointTransforms)
							for (int j = 0; j < i.size(); j++)
								Config.JointTransforms[j].push_back(i[j]);
					}
					auto& ParentTransform = Config.JointTransforms[Config.EffectorParentIndex];
					double CurveFitness = EffectorProblem::SolveEffectorTransmobile(ParentTransform, InTargetPoints, Config.OriginalTime).first;
					InitParams[Id] = {CurveFitness, Random};
					if (std::ranges::any_of(Random, [](auto i) { return isnan(i); }))
					{
						LOG_ERROR("NAN in Random");
						LOG_ERROR("Random : {}", Random);
					}
				}
			}, 2);
			std::ranges::sort(InitParams);
		}

		pagmo::problem Prob{ Problem };

		LOG_INFO("Solver for type {}", Type);
		static const int  EvolutionRound = GConfig.Get<int>("Optimizer", "EvolutionRound");
		static const uint PopulationSize = GConfig.Get<int>("Optimizer", "Population");
		static const int  Generation = GConfig.Get<int>("Optimizer", "Generation");
		ASSERTMSG(PopulationSize % 4 == 0, "Population size should be multiple of 4 as the NSGA2 algorithm");
		pagmo::nsga2 nsga_instance(Generation);
		nsga_instance.set_bfe(pagmo::bfe(pagmo::thread_bfe()));
		pagmo::algorithm algo{ nsga_instance };
		algo.set_verbosity(1);
		pagmo::population Population{Problem};
		if(InitSamples > 0)
		{
			for (int i = 0;i < PopulationSize; i ++)
			{
				auto RandParam = Population.random_decision_vector();
				if (i < InitSamples)
				{
					auto InitParam = InitParams[i].second;
					// Replace with InitParams
					for(int j = 0;j < InitParam.size(); j ++)
						RandParam[j] = InitParam[j];
				}
				if (std::ranges::any_of(RandParam, [](auto i) { return isnan(i); }))
				{
					LOG_ERROR("NAN in RandParam");
					LOG_ERROR("RandParam : {}", RandParam);
				}
				Population.push_back(RandParam);
			}
		}
		else
			Population = pagmo::population{Problem, pagmo::bfe(pagmo::thread_bfe()), PopulationSize};

		pagmo::island	  Island(algo, Population);
		Island.evolve(1);
		Island.wait();
		auto F = Island.get_population().get_f();
		auto X = Island.get_population().get_x();

		// print all the f and x values of the final population
		for (int i = 0; i < Island.get_population().size(); i++)
		{
			auto f = Island.get_population().get_f()[i];
			auto x = Island.get_population().get_x()[i];
			Results.push_back({Type, f, Problem.RealParameter(x) });
		}
		PrintResult(Results);
	}
}

template <typename T> requires std::is_same_v<T, FVector> || std::is_same_v<T, FTransform>
void MainProblem::SolveLinkageOnly(
	const String& Type,
	const ObjectPtr<ReferenceSurface>& Surface,
	const TArray<T>& TargetPoints, const String& InMobility, const pagmo::vector_double& dv)
{
	auto Voxels = NewObject<SurfaceVoxel>(Surface.get());
	int JointParameterNum = GetParameterNum(Type);
	TArray<double> PCurveFitting(dv.begin(), dv.begin() + JointParameterNum);

	MainProblem	Problem(Type, Surface, TargetPoints, InMobility, Voxels, PCurveFitting);
	pagmo::problem Prob{ Problem };

	static const uint PopulationSize = GConfig.Get<int>("Optimizer", "Population");
	static const int  Generation = GConfig.Get<int>("Optimizer", "Generation");
	pagmo::cmaes cmaes_instance(Generation, -1, -1, -1,-1,0.5, 1e-3, 1e-3);
	cmaes_instance.set_bfe(pagmo::bfe(pagmo::thread_bfe()));
	pagmo::algorithm algo{ cmaes_instance };
	algo.set_verbosity(1);
	pagmo::population Population{Problem, pagmo::bfe(pagmo::thread_bfe()), PopulationSize};
	if(dv.size() == Problem.get_bounds().first.size() + JointParameterNum)
	{
		TArray<double> GivenParam(dv.begin() + JointParameterNum, dv.end());
		Population.set_xf(0, GivenParam, Problem.fitness(GivenParam));
	}
	pagmo::island Island(algo, Population);
	Island.evolve(1);
	Island.wait();

	struct Result
	{
		String		   Type;
		TArray<double> F;
		TArray<double> X;
	};
	auto PrintResult = [&](TArray<Result>& Results) {
		std::ranges::sort(Results, [](const auto& A, const auto& B) {
			for (auto i = 0; i < A.F.size(); i++)
				if (A.F[i] != B.F[i])
					return A.F[i] < B.F[i];
			return false;
		});

		for (int i = 0; i < std::min(int(Results.size()), 500); i++)
			LOG_INFO("Type {0} F: {1} X: {2}", Results[i].Type, Results[i].F, Results[i].X);
	};
	TArray<Result> Results;

	auto F = Island.get_population().get_f();
	auto X = Island.get_population().get_x();

	// print all the f and x values of the final population
	for (int i = 0; i < Island.get_population().size(); i++)
	{
		auto f = Island.get_population().get_f()[i];
		auto x = Island.get_population().get_x()[i];
		Results.push_back({Type, f, Problem.RealParameter(x) });
	}
	PrintResult(Results);
}
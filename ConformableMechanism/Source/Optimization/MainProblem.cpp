//
// Created by marvel on 22/7/24.
//

#include "MainProblem.h"

#include "MobilityProblem.h"
#include "LinkageOptimize/SurfaceLinkageProblem.h"
#include "MeshFitting/MeshFittingDesigner.h"
#include "MeshFitting/FKProblem.h"
#include "MechanismSequence.h"
#include "EffectorProblem.h"
#include "JointParameterDefine.h"
#include "Math/Random.h"

#include <pagmo/algorithms/cmaes.hpp>

MainProblem::MainProblem(const String& InType,
	const ObjectPtr<ReferenceSurface>& InSurface,
	const TArray<FVector>&			   InTargetPoints,
	const String& InMobility, const ObjectPtr<SurfaceVoxel>& InVoxel,
	const TArray<double>& InJointParams)
	: Type(InType), Surface(InSurface), TargetPoints(InTargetPoints), Voxel(InVoxel.get()), JointParams(InJointParams)
{
	JointNum = Type.length() - 1; // minus the E
	if (InMobility == "Intromobile")
		Mobility = Intromobile;
	else if (InMobility == "Extramobile")
		Mobility = Extramobile;
	else if (InMobility == "Transmobile")
		Mobility = Transmobile;
	if (!InJointParams.empty())
		bOptimizeLinkageOnly = true;
}
MainProblem::MainProblem(const String& InType,
	const ObjectPtr<ReferenceSurface>& InSurface,
	const TArray<FTransform>&		   InTargetTransform,
	const String&					   InMobility,
	const ObjectPtr<SurfaceVoxel>&	   InVoxel,
	const TArray<double>&			   InJointParams)
	: Type(InType), Surface(InSurface), TargetMotion(InTargetTransform), Voxel(InVoxel.get()), JointParams(InJointParams)
{
	JointNum = Type.length() - 1; // minus the E
	if (InMobility == "Intromobile")
		Mobility = Intromobile;
	else if (InMobility == "Extramobile")
		Mobility = Extramobile;
	else if (InMobility == "Transmobile")
		Mobility = Transmobile;
	if (!InJointParams.empty())
		bOptimizeLinkageOnly = true;
}

int MainProblem::GetJointParameterNumber() const
{
	return GetParameterNum(Type) - FixedUVs.size() * 2;
}

void MainProblem::FixJointUV(int JointIndex, const FVector2& FixedUV)
{
	FixedUVs[JointIndex] = FixedUV;
}

pagmo::vector_double MainProblem::RealParameter(const pagmo::vector_double& dv) const
{
	pagmo::vector_double Result = dv;

	// Add joint fixed UV
	{
		int Delta = 0;
		for(int i = 0;i < Type.size();i ++)
		{
			if(Type[i] == 'E') continue;
			if(FixedUVs.contains(i))
			{
				auto UV = FixedUVs.at(i);
				Result.insert(Result.begin() + Delta, UV.x());
				Result.insert(Result.begin() + Delta + 1, UV.y());
			}
			Delta += JointParameterMap(Type[i], i == 0);
		}
	}

	// Fix for solve linakge only
	if (bOptimizeLinkageOnly)
	{
		Result = JointParams;
		Result.insert(Result.end(), dv.begin(), dv.end());
	}

	if (std::ranges::any_of(Result, [](auto i) { return isnan(i); }))
	{
		LOG_ERROR("NAN in RealParameter");
		LOG_ERROR("dv : {}", dv);
		LOG_ERROR("Result : {}", Result);
		LOG_ERROR("FixedUVs : {}", FixedUVs.size());
	}

	return Result;
}

std::tuple<pagmo::vector_double, pagmo::vector_double> MainProblem::DivideParameter(const pagmo::vector_double& dv) const
{
	int JointParameterNum = GetParameterNum(Type);
		return {
			TArray<double>(dv.begin(), dv.begin() + JointParameterNum),
			TArray<double>(dv.begin() + JointParameterNum, dv.end())};
}
pagmo::vector_double MainProblem::fitness(const pagmo::vector_double& dv) const
{
	if (std::ranges::any_of(dv, [](auto i) { return isnan(i); }))
	{
		LOG_ERROR("NAN in fitness");
		LOG_ERROR("dv : {}", dv);
	}
	auto RealParams = RealParameter(dv);
	return CalculateConfig(RealParams).second;
}

std::pair<SurfaceLinkageConfiguration, TArray<double>> MainProblem::CalculateConfig(const pagmo::vector_double& dv) const
{
	auto [PCurveFitting, PLinkageGeometry] = DivideParameter(dv);

	double JointConstrainScore = 0.;
	FKProblemSolver Solver = FKProblemSolver(Type, Surface);
	{
		auto Fitness = Solver.Calc(PCurveFitting);
		JointConstrainScore = FKProblem::WeightedFitness(Fitness);
	}

	if(JointConstrainScore >= 1e-4)
		return {{}, TArray<double>(get_nobj(), JointConstrainScore + 1e8)};

	SurfaceLinkageConfiguration Config;
	{
		Config.Mobility = Mobility;
		Config.OriginalTime = Solver.TimeClipCount;
		Config.Type = Solver.Type;
		Config.EffectorParentIndex = Solver.EffectorParentIndex;
		Config.AttachedSurface = Solver.Surface;

		{
			int pBegin = 0;
			for(int i = 0; i < Type.size();i ++)
				if(Type[i] != 'E')
				{
					Config.UV.emplace_back(PCurveFitting[pBegin], PCurveFitting[pBegin + 1]);
					pBegin += JointParameterMap(Type[i], i == 0);
				}
		}

		for (const auto& i : Solver.Record.JointInitTransforms)
			Config.InitTransform.push_back(i);

		Config.JointTransforms.resize(Solver.Record.JointTransforms[0].size());
		for(auto i : Solver.Record.JointTransforms)
			for(int j = 0; j < i.size(); j ++)
				Config.JointTransforms[j].push_back(i[j]);

		auto FixType = Config.Type; //Remove 'E'
		FixType.erase(FixType.find('E'), 1);
		Config.JointActor.clear();
		for (int i = 0; i < FixType.size(); i ++)
		{
			Config.JointActor.emplace_back(NewObject<SurfaceJointActor>
			(FixType[i], Config.AttachedSurface, Config.UV[i], Config.InitTransform[i], i==0, i == FixType.size() - 1));
		}
		Config.JointActor[0]->GetJointComponent()->GetIKJoint()->SetIsRoot(true);
		for (int i = 0; i < FixType.size();i ++)
			Config.JointActor[i]->AddNextJoint(Config.JointActor[(i + 1) % FixType.size()]);
	}


	static int SplineSample = GConfig.Get<int>("LinkageProblem", "SplineSample");
	{
		SurfaceLinkageProblem Problem(Config);
		auto [LinkageSpline, JointPort, SocketPort] = Problem.ParamToSpline(PLinkageGeometry);
		static bool DebugShowControlPoints = GConfig.Get<bool>("LinkageProblem", "DebugShowControlPoints");
		Config.LinkagePath = SurfaceLinkageProblem::LinkageSplineSample(LinkageSpline, Surface.get(), SplineSample, DebugShowControlPoints);
		Problem.LinkageConnectJoint(Config.LinkagePath, JointPort, SocketPort);
	}

	double MobilityScore = 0.; int MobilityAllowTime = Config.OriginalTime;
	if (Mobility != Transmobile)
		std::tie(MobilityScore, MobilityAllowTime) = MobilityProblem::Fitness(Config, Mobility);
	Config.MobilityAllowTime = MobilityAllowTime;

	double CurveFittingFitness = 0.;
	Config.EffectorIndex = Config.Type.size() - 1;
	if(!TargetPoints.empty()) // Pos generation
	{
		FVector EffectorPos;
		if(Mobility != Transmobile)
		{
			std::tie(CurveFittingFitness, EffectorPos) =
			   EffectorProblem::SolveEffectorNonTransmobile(
				   Config.JointTransforms[Config.EffectorParentIndex],
				   TargetPoints, MobilityAllowTime);

			// Cut all transform sequence within EffectorTime
			for(auto& i : Config.JointTransforms)
				i.resize(MobilityAllowTime);
			Config.EffectorBestTime = MobilityAllowTime;
		}
		else
		{
			std::tie(CurveFittingFitness, EffectorPos) =
				EffectorProblem::SolveEffectorTransmobile(
					Config.JointTransforms[Config.EffectorParentIndex],
					TargetPoints, MobilityAllowTime);
			Config.EffectorBestTime = Config.OriginalTime;
		}
		FTransform EffectorInitTransform = Config.InitTransform[Config.EffectorParentIndex];
		EffectorInitTransform.AddTranslationLocal(EffectorPos);
		Config.InitTransform[Config.EffectorIndex] = EffectorInitTransform;
		Config.EffectorTarjectory = EffectorProblem::EffectorTrajectory(EffectorPos,
		Config.JointTransforms[Config.EffectorParentIndex], Config.EffectorBestTime);
	} // Motion Generation
	else
	{
		FTransform EffectorLocal;
		if(Mobility != Transmobile)
		{
			std::tie(CurveFittingFitness, EffectorLocal) =
			   EffectorProblem::SolveEffectorNonTransmobile(
				   Config.JointTransforms[Config.EffectorParentIndex],
				   TargetMotion, MobilityAllowTime);

			// Cut all transform sequence within EffectorTime
			for(auto& i : Config.JointTransforms)
				i.resize(MobilityAllowTime);
			Config.EffectorBestTime = MobilityAllowTime;
		}
		else
		{
			std::tie(CurveFittingFitness, EffectorLocal) =
				EffectorProblem::SolveEffectorTransmobile(
					Config.JointTransforms[Config.EffectorParentIndex],
					TargetMotion, MobilityAllowTime);
			Config.EffectorBestTime = Config.OriginalTime;
		}
		FTransform EffectorInitTransform = Config.InitTransform[Config.EffectorParentIndex];
		EffectorInitTransform = EffectorInitTransform * EffectorLocal;
		Config.InitTransform[Config.EffectorIndex] = EffectorInitTransform;
		Config.EffectorTarjectory = EffectorProblem::EffectorTrajectory(EffectorLocal.GetLocation(),
		Config.JointTransforms[Config.EffectorParentIndex], Config.EffectorBestTime);
	}

	{
		int EffectorIndex = Config.EffectorIndex;
		int EffectorParentIndex = Config.EffectorParentIndex;
		auto FixType = Config.Type; //Remove 'E'
		FixType.erase(FixType.find('E'), 1);
		Config.JointActor.clear();
		for (int i = 0; i < FixType.size(); i ++)
		{
			Config.JointActor.emplace_back(NewObject<SurfaceJointActor>
			(FixType[i], Config.AttachedSurface, Config.UV[i], Config.InitTransform[i], i==0, i == FixType.size() - 1));
		}
		//Effector
		Config.JointActor.emplace_back(NewObject<SurfaceJointActor>
		('E', Config.AttachedSurface, Config.UV[EffectorIndex], Config.InitTransform[EffectorIndex], false, false));

		Config.JointActor[0]->GetJointComponent()->GetIKJoint()->SetIsRoot(true);
		for (int i = 0; i < FixType.size();i ++)
			Config.JointActor[i]->AddNextJoint(Config.JointActor[(i + 1) % FixType.size()]);

		Config.JointActor[EffectorParentIndex]->AddNextJoint(Config.JointActor[EffectorIndex]);
	}

	TArray<double> LinkageGeometryFitness;
	if(Voxel != nullptr)
	{
		SurfaceLinkageProblem LinkageProblem(Config, Voxel, bDebugLog);
		LinkageProblem.MultiTarget = bLinkageMultiTarget;
		LinkageGeometryFitness = LinkageProblem.fitness(PLinkageGeometry);
	}
	if (bOptimizeLinkageOnly) return {Config, LinkageGeometryFitness};

	if(bDebugLog)
	{
		LOG_INFO("CurveFittingFitness: {0}", CurveFittingFitness);
	}
	if(Mobility != Transmobile)
	{
		// return { Config, { CurveFittingFitness + JointConstrainScore,
			// LinkageGeometryFitness < 1e6 ? MobilityScore : LinkageGeometryFitness, LinkageGeometryFitness }};
		if (bLinkageMultiTarget)
			return { Config, { CurveFittingFitness + JointConstrainScore, LinkageGeometryFitness[0], LinkageGeometryFitness[1], MobilityScore} };
		else
			return { Config, { CurveFittingFitness + JointConstrainScore, LinkageGeometryFitness[0], MobilityScore} };
		// return { Config, { CurveFittingFitness + JointConstrainScore, LinkageGeometryFitness, MobilityScore} };
	}
	else
	{
		Config.MobilityAllowTime = Config.OriginalTime;
		Config.EffectorBestTime = Config.OriginalTime;
		if (bLinkageMultiTarget)
			return { Config, { CurveFittingFitness + JointConstrainScore, LinkageGeometryFitness[0], LinkageGeometryFitness[1] }};
		return {Config, { CurveFittingFitness + JointConstrainScore, LinkageGeometryFitness[0] }};
	}

}

std::pair<pagmo::vector_double, pagmo::vector_double> MainProblem::GetCurveFittingBounds() const
{
	int					 JointNum = Type.length() - 1; // minus the E
	static auto			 MinU = GConfig.Get<double>("DesignerConfig", "MinU");
	static auto			 MaxU = GConfig.Get<double>("DesignerConfig", "MaxU");
	static auto			 MinV = GConfig.Get<double>("DesignerConfig", "MinV");
	static auto			 MaxV = GConfig.Get<double>("DesignerConfig", "MaxV");
	pagmo::vector_double Min;
	pagmo::vector_double Max;
	for (int i = 0;i < Type.size() ;i ++)
	{
		if(Type[i] != 'E')
		{
			if(!FixedUVs.contains(i))
			{
				Min.push_back(MinU); Min.push_back(MinV);
				Max.push_back(MaxU); Max.push_back(MaxV);
			}
			for (int j = 0; j < JointParameterMap(Type[i],i ==0) - 2; j++)
			{
				Min.push_back(0.);
				Max.push_back(1.);
			}
		}
	}
	return bOptimizeLinkageOnly ? std::pair{pagmo::vector_double{}, pagmo::vector_double{}} : std::pair{ Min, Max };
}

std::pair<pagmo::vector_double, pagmo::vector_double> MainProblem::GetLinkageGeometryBounds() const
{
	pagmo::vector_double L;
	pagmo::vector_double R;
	static int			 ControlPointNum = GConfig.Get<int>("LinkageProblem", "ControlPointNum");
	for (int i = 0; i < ControlPointNum; i++)
	{
		for (int j = 0; j < JointNum - 1; j++)
		{
			L.push_back(0); R.push_back(1.);
			L.push_back(0); R.push_back(1.);
		}
	}
	return { L, R };
}


std::pair<pagmo::vector_double, pagmo::vector_double> MainProblem::get_bounds() const
{
	auto [CurveFittingLower, CurveFittingUpper] = GetCurveFittingBounds();
	auto [LinkageGeometryLower, LinkageGeometryUpper] = GetLinkageGeometryBounds();

	pagmo::vector_double Lower(CurveFittingLower.size() + LinkageGeometryLower.size());
	pagmo::vector_double Upper(CurveFittingUpper.size() + LinkageGeometryUpper.size());
	for (int i = 0; i < CurveFittingLower.size(); i++)
	{
		Lower[i] = CurveFittingLower[i];
		Upper[i] = CurveFittingUpper[i];
	}
	for (int i = 0; i < LinkageGeometryLower.size(); i++)
	{
		Lower[i + CurveFittingLower.size()] = LinkageGeometryLower[i];
		Upper[i + CurveFittingLower.size()] = LinkageGeometryUpper[i];
	}
	return { Lower, Upper };
}

std::pair<TArray<double>, TArray<double>> MainProblem::RandomJointParameter() const
{
	static double MinU = GConfig.Get<double>("DesignerConfig", "MinU");
	static double MaxU = GConfig.Get<double>("DesignerConfig", "MaxU");
	static double MinV = GConfig.Get<double>("DesignerConfig", "MinV");
	static double MaxV = GConfig.Get<double>("DesignerConfig", "MaxV");

	// auto RandomSelect = []() {
	// 	auto p1 = Random::RandomInterval(0, 3);
	// 	auto p2 = Random::RandomInterval(0, 3);
	// 	while (p1 == p2)
	// 		p2 = Random::RandomInterval(0, 3);
	// 	return std::pair{p1, p2};
	// };
	//
	// TArray<TArray<double>> JointParams;
	// for(int i = 0; i < Type.size(); i ++)
	// {
	// 	if (Type[i] == 'E') continue;
	// 	TArray<double> temp;
	// 	for (int j = 0; j < JointParameterMap(Type[i], i == 0); ++j)
	// 		temp.push_back(Random::Random());
	// 	JointParams.push_back(temp);
	// }
	// {
	// 	// Fix min u and max u
	// 	auto [MinIndex, MaxIndex] = RandomSelect();
	// 	JointParams[MinIndex][0] = 0.1 + 0.2 * JointParams[MinIndex][0];
	// 	JointParams[MaxIndex][0] = 0.7 + 0.2 * JointParams[MaxIndex][0];
	//
	// 	// Fix min v and max v
	// 	std::tie(MinIndex, MaxIndex) = RandomSelect();
	// 	JointParams[MinIndex][1] = 0.1 + 0.2 * JointParams[MinIndex][1];
	// 	JointParams[MaxIndex][1] = 0.7 + 0.2 * JointParams[MaxIndex][1];
	// }
	// // flat the joint params
	// TArray<double> FlatParams;
	// for (const auto& i : JointParams)
	// 	for (const auto& j : i)
	// 		FlatParams.push_back(j);
	// return {FlatParams, FlatParams};


	TArray<double> RealParams;
	TArray<double> RandomParams;

	for(int i = 0; i < Type.size(); i ++)
	{
		if (Type[i] == 'E') continue;
		if (FixedUVs.contains(i))
		{
			RealParams.push_back(FixedUVs.at(i).x());
			RealParams.push_back(FixedUVs.at(i).y());
			for(int j = 2; j < JointParameterMap(Type[i], i == 0); ++j)
			{
				RealParams.push_back(Random::Random());
				RandomParams.push_back(RealParams.back());
			}
		}
		else
		for (int j = 0; j < JointParameterMap(Type[i], i == 0); ++j)
		{
			auto p = Random::Random();
			if (j == 0) // U
				p = MinU + (MaxU - MinU) * p;
			else if (j == 1) // V
				p = MinV + (MaxV - MinV) * p;

			if (i == 0)
				p = 0.7 + 0.2 * p;
			if (i == Type.size() - 1)
				p = 0.1 + 0.2 * p;

			RealParams.push_back(p);
			RandomParams.push_back(p);
		}
	}
	return {RandomParams, RealParams};
}




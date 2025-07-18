#pragma once
#include "Core/CoreMinimal.h"
#include <algorithm>
#include "Actors/ParametricMeshActor.h"
#include "SpacialLinkageDesign.h"
#include "CurveFitting/SurfaceLinkageDesgin.h"
#include "spdlog/stopwatch.h"
#include <iostream>

inline void ProbalityTest()
{
    TArray<String> Types = {
        "RRCS", "RRUS", "RRSC", "RRSU", "RPCS", "RPUS", 
    "RPSC", "RPSU", "RCRS", "RCPS", "RCCC", "RCCU", "RCUC", "RCUU",
     "RCSR", "RCSP", "RURS", "RUPS", "RUCC", "RUCU", "RUUC", 
     "RUUU", "RUSR", "RUSP", "RSRC", "RSRU", "RSPC", "RSPU",
      "RSCR", "RSCP", "RSUR", "RSUP"};

    std::map<String,int > Result;
    for(auto Type : Types)
    {
        int Samples = 1e3;
        std::atomic_int Count = 0;
        SpacialLinkageDesign Solver(Type, nullptr);
        Solver.LinkageMinLength = 0.4;
        
        spdlog::stopwatch sw;
        ParallelFor(Samples, [&Solver, &Count, Type](int i){
            bool Valid = false;
            SpacialLinkageSolver Designer(Type, Solver.GetRandomParamter());
            Designer.BreakWhenInValid = true;
            Designer.CalcEnergy(Valid);
            if(Valid) Count ++;
        });
        Result[Type] = int(Count);
        double Probality = double((int)Count) / double(Samples);
        LOG_INFO("-------------{}-------------", Type);
        LOG_INFO("Samples {} takes time {};", Samples, sw);
        LOG_INFO("Valid sample {} / Total sample {};  Probality : {:.3f}%", int(Count), Samples, Probality * 100.);
    }

    TArray<std::pair<String, int>> pairs;
    for (auto itr = Result.begin(); itr != Result.end(); ++itr)
        pairs.push_back(*itr);

    sort(pairs.begin(), pairs.end(), [=](std::pair<String, int>& a, std::pair<String, int>& b)
    {
        return a.second < b.second;
    });

    for(auto i : pairs)
    {
        std::cout << i.first << " ";
    }
    std::cout << std::endl;
    for(auto i : pairs)
    {
        std::cout << i.second << " ";
    }
}

inline String FindBestType(ObjectPtr<ParametricMeshActor> Surface, ObjectPtr<CurveActor> TargetCurve, int SamplesPerConfig = 1e3)
{
    TArray<String> Types = {
        "RRCS", "RRUS", "RRSC", "RRSU", "RPCS", "RPUS",
    "RPSC", "RPSU", "RCRS", "RCPS", "RCCC", "RCCU", "RCUC", "RCUU",
     "RCSR", "RCSP", "RURS", "RUPS", "RUCC", "RUCU", "RUUC",
     "RUUU", "RUSR", "RUSP", "RSRC", "RSRU", "RSPC", "RSPU",
      "RSCR", "RSCP", "RSUR", "RSUP"};
    LOG_INFO("Try {} samples each for {} configs.", SamplesPerConfig, Types.size());
    double BestEnergy = 1e10;
    String BestType;
    for(auto Type : Types) {
        auto Solver = SurfaceSLDesginer(TargetCurve, Surface, Type);
        TArray<double> Energy(SamplesPerConfig);
        double E = 0.;
        int ValidCount = 0.;
        double MinEnergy = 1e10;
    	spdlog::stopwatch sw;
        // Evaluate samples
        ParallelFor(SamplesPerConfig, [&](int i){
            bool Valid = false;
            SurfaceLinkageSolver Designer(TargetCurve, Surface, Type, Solver.RandomOneParameter());
            Designer.BreakWhenInValid = true;
            VectorXd CEnergy = Designer.CalcEnergy(Valid);
            if(Valid) Energy[i] = CEnergy[1];
            else Energy[i] = -1;
        });
        // Statistics
        for(int i = 0;i < SamplesPerConfig; i ++)
        {
            if(Energy[i] > 0) {
                E += Energy[i];
                ValidCount ++;
                MinEnergy = std::min(MinEnergy, Energy[i]);
            }
        }
        E /= ValidCount;
        if(MinEnergy < BestEnergy) {
            BestEnergy = MinEnergy;
            BestType = Type;
        }
        if(ValidCount == 0) {
            LOG_INFO("Type {} has no valid sample.", Type);
            continue;
        }
    	double EstimateTime = sw.elapsed().count() / ValidCount * 1e3;
        LOG_INFO("Type {} has average energy {:.3f} | Min energy : {:.3f} | Valid count : {} | Estimate time per 1k : {:.1f}s", Type, E, MinEnergy, ValidCount, EstimateTime);
    }
    LOG_INFO("Best energy is {:.3f}, Type is {}", BestEnergy, BestType);
    return BestType;
}


inline TArray<String> FindTopNBestType(int n, ObjectPtr<ParametricMeshActor> Surface, ObjectPtr<CurveActor> TargetCurve, int SamplesPerConfig = 1e3)
{
    TArray<String> Types = {
        "RRCS", "RRUS", "RRSC", "RRSU", "RPCS", "RPUS",
    "RPSC", "RPSU", "RCRS", "RCPS", "RCCC", "RCCU", "RCUC", "RCUU",
     "RCSR", "RCSP", "RURS", "RUPS", "RUCC", "RUCU", "RUUC",
     "RUUU", "RUSR", "RUSP", "RSRC", "RSRU", "RSPC", "RSPU",
      "RSCR", "RSCP", "RSUR", "RSUP"};
    LOG_INFO("Try {} samples each for {} configs.", SamplesPerConfig, Types.size());
	TArray<std::pair<String, double>> AllChoice;
    for(auto Type : Types)
    {
	    auto Solver = SurfaceSLDesginer(TargetCurve, Surface, Type);
    	TArray<double> Energy(SamplesPerConfig);
    	spdlog::stopwatch sw;
    	// Evaluate samples
    	ParallelFor(SamplesPerConfig, [&](int i){
			bool Valid = false;
			SurfaceLinkageSolver Designer(TargetCurve, Surface, Type, Solver.RandomOneParameter());
			Designer.BreakWhenInValid = true;
			VectorXd CEnergy = Designer.CalcEnergy(Valid);
			if(Valid) Energy[i] = CEnergy[1];
			else Energy[i] = 1e10;
		});
    	double MinEnergy = *std::min_element(Energy.begin(), Energy.end());
    	AllChoice.emplace_back(Type, MinEnergy);
    }
	std::sort(AllChoice.begin(), AllChoice.end(), [](const std::pair<String, double>& a, const std::pair<String, double>& b) {
		return a.second < b.second;
	});
	TArray<String> Result;
	for(int i = 0; i < n; i ++)
	{
		Result.push_back(AllChoice[i].first);
		LOG_INFO("Rank: {} | Type {} has min energy : {:.3f}", i, AllChoice[i].first, AllChoice[i].second);
	}
	return Result;
}
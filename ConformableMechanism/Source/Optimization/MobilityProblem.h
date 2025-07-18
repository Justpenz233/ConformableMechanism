//
// Created by marvel on 9/8/24.
//

#pragma once
#include "MeshFitting/MeshFittingDesigner.h"
#include "MainProblem.h"
#include "MechanismSequence.h"
#include "ReferenceSurface.h"
class MobilityProblem
{
public:

	/**
	 * Calculate the a mobility score for a given configuration, and the maximum allowed time can statify the mobility
	 * @param Config
	 * @param Mobility
	 * @return
	 */
	static std::pair<double, int> Fitness(const SurfaceLinkageConfiguration& Config, MechanismMobility Mobility)
	{
		auto Surface = Config.AttachedSurface.get();
		const auto& Linkage = Config.LinkagePath;

		int MaximumTime = Config.OriginalTime;
		static double MobilityDistanceTolerance =
			GConfig.Get<double>("LinkageProblem", "MobilityDistanceTolerance");

		auto CalcMobilityTime = [&]() {
			for(int t = 2; t < Config.OriginalTime; t ++)
			{
				for(int i = 0; i < Linkage.size(); i ++)
				{
					auto InVInitT = Config.InitTransform[i].Inverse();
					for(const auto& p : Linkage[i])
					{
						auto SignedDistance = Surface->SignedDistance(Config.JointTransforms[i][t] * InVInitT * p);
						if((SignedDistance < -MobilityDistanceTolerance && Mobility == Extramobile)
							|| (SignedDistance > MobilityDistanceTolerance && Mobility == Intromobile))
							return t - 1;
					}
				}
			}
			return Config.OriginalTime;
		};
		MaximumTime = CalcMobilityTime();
		return {- MaximumTime, MaximumTime};
		// return {Config.OriginalTime - MaximumTime, MaximumTime};
	}
};

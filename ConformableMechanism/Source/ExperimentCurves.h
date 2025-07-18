//
// Created by MarvelLi on 2024/9/2.
//
#pragma once

#include <CoreMinimal.h>
#include "tinysplinecxx.h"
#include "Misc/Config.h"
#define SampleCount GConfig.Get<int>("DesignerConfig", "MechanismEvaluationSteps")

inline TArray<FVector> ThreeControlPointSpline()
{
	tinyspline::BSpline spline = tinyspline::BSpline::interpolateCubicNatural(
	{
		0,0,0,
		0,1,1,
		1,1,1,
		0,0,0
	},3);

	// tinyspline::BSpline spline(4, 3, 3, tinyspline::BSpline::Clamped);
	// spline.setControlPoints({
	// 	0, 0, 0,
	// 	0, 1, 1,
	// 	1, 1, 1,
	// 	0, 0, 0
	// });
	auto Samples = spline.sample(SampleCount);
	TArray<FVector> Points;
	for (int i = 0; i < Samples.size(); i+=3)
		Points.emplace_back(Samples[i], Samples[i + 1], Samples[i + 2]);

	return Points;
}
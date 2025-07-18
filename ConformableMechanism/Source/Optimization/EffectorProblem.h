//
// Created by marvel on 10/8/24.
//

#pragma once
#include "MechanismSequence.h"
#include "Math/LinearAlgebra.h"

struct EffectorProblem
{
	static inline double EffectorEnergy(
		const FVector& EffectorPos,
		const TArray<FTransform>& ParentTransform,
		const TArray<FVector>& TargetPoints, int Time, int Del = 0)
	{
		int n = Time;
		int m = TargetPoints.size();
		TArray<FVector> EffectorTrajectory(ParentTransform.size());
		for (int t = 0; t < ParentTransform.size(); t++)
			EffectorTrajectory[t] = ParentTransform[t] * EffectorPos;

		auto Dist = [&](int i, int j) {
			i--; j--; j = (j + Del) % m;
			return (EffectorTrajectory[i] - TargetPoints[j]).norm();
		};

		TArray<TArray<double>> F(n + 1, TArray<double>(m + 1, 1e10));
		F[0][0] = 0.;
		for (int i = 1; i <= n; i++)
		{
			for (int j = 1; j <= m; j++)
			{
				double d1 = F[i - 1][j - 1] + Dist(i, j); // Pair i and j
				double d2 = F[i - 1][j]; // Pair i-1 and j
				double d3 = F[i][j - 1] + Dist(i, j); // Pair i and j
				F[i][j] = std::min({ d1, d2, d3 });
			}
		}
		return F[n][m];
	}


	static inline double EffectorEnergy(
	const FTransform& EffectorLocalTransform,
	const TArray<FTransform>& ParentTransform,
	const TArray<FTransform>& TargetTransform, int Time, int Del = 0)
	{
		int n = Time;
		int m = TargetTransform.size();
		TArray<FTransform> EPath(ParentTransform.size());
		for (int t = 0; t < ParentTransform.size(); t++)
			EPath[t] = ParentTransform[t] * EffectorLocalTransform;
		static double RotationWeight = 1.;
		static double TranslationWeight = 5.;
		auto Dist = [&](int i, int j) {
			i--; j--; j = (j + Del) % m;
			Matrix3d R0 = EPath[i].GetMatrix().block(0, 0, 3, 3);
			Matrix3d R1 = TargetTransform[j].GetMatrix().block(0, 0, 3, 3);
			Vector3d T0 = EPath[i].GetTranslation();
			Vector3d T1 = TargetTransform[j].GetTranslation();
			double RotationDist = (R0 - R1).norm() * RotationWeight;
			double TranslationDist = (T0 - T1).norm() * TranslationWeight;
			return RotationDist + TranslationDist;
		};

		TArray<TArray<double>> F(n + 1, TArray<double>(m + 1, 1e10));
		F[0][0] = 0.;
		for (int i = 1; i <= n; i++)
		{
			for (int j = 1; j <= m; j++)
			{
				double d1 = F[i - 1][j - 1] + Dist(i, j); // Pair i and j
				double d2 = F[i - 1][j]; // Pair i-1 and j
				double d3 = F[i][j - 1] + Dist(i, j); // Pair i and j
				F[i][j] = std::min({ d1, d2, d3 });
			}
		}

		return F[n][m];
	}

	static TArray<FVector> EffectorTrajectory(
			const FVector& EffectorPos, const TArray<FTransform>& ParentTransform, int Time)
	{
		TArray<FVector> Result;
		for (int t = 0; t < Time; t++)
			Result.push_back(ParentTransform[t] * EffectorPos);
		return Result;
	}

	/**
	 * Calculate the best effector position and corresponding energy and time
	 * @param ParentTransform Effector's parent transform
	 * @param TargetPoints Target points
	 * @param Time Time allowed
	 * @return Best effector position, energy and time
	 */
	static std::pair<double, FVector> SolveEffectorTransmobile(
	const TArray<FTransform>& ParentTransform,
	const TArray<FVector>& TargetPoints, int Time)
	{
		FVector	Effector = (ParentTransform[0].GetMatrix().inverse() * TargetPoints[0].homogeneous()).head(3);
		auto Energy = EffectorEnergy(Effector, ParentTransform, TargetPoints, Time, 0);
		return {Energy, Effector};
	}

	static std::pair<double, FTransform> SolveEffectorTransmobile(
	const TArray<FTransform>& ParentTransform,
	const TArray<FTransform>& TargetTransform, int Time)
	{
		FTransform	EffectorLocal = ParentTransform[0].Inverse() * TargetTransform[0];
		auto Energy = EffectorEnergy(EffectorLocal, ParentTransform, TargetTransform, Time, 0);
		return {Energy, EffectorLocal};
	}


	static std::tuple<double, FVector> SolveEffectorNonTransmobile(
	const TArray<FTransform>& ParentTransform,
	const TArray<FVector>& TargetPoints, int Time)
	{
		FVector	Effector = (ParentTransform[0].GetMatrix().inverse() * TargetPoints[0].homogeneous()).head(3);
		auto Energy = EffectorEnergy(Effector, ParentTransform, TargetPoints, Time);

		return {Energy, Effector};
	}

	static std::tuple<double, FTransform> SolveEffectorNonTransmobile(
	const TArray<FTransform>& ParentTransform,
	const TArray<FTransform>& TargetTransform, int Time)
	{
		FTransform	Effector = ParentTransform[0].Inverse() * TargetTransform[0];
		auto Energy = EffectorEnergy(Effector, ParentTransform, TargetTransform, Time);
		return {Energy, Effector};
	}


	static std::pair<double, FVector> SolveEffectorSpatialMotion(
	const TArray<FTransform>& ParentTransform,
	const TArray<FVector>& TargetPoints, int Time)
	{
		FVector	Effector = ParentTransform[0].GetLocation() + FVector{0.1, -0.1, 0.7};
		Effector = (ParentTransform[0].GetMatrix().inverse() * Effector.homogeneous()).head(3);
		TArray<FVector> EffectorTrajectory(ParentTransform.size());
		for (int t = 0; t < ParentTransform.size(); t++)
			EffectorTrajectory[t] = ParentTransform[t] * Effector;

		auto F = Algorithm::GeometryProcess::EstimatePointsOBB(EffectorTrajectory);
		double MinExtent = F.GetScale().minCoeff();
		return {-MinExtent, Effector};
	}

};

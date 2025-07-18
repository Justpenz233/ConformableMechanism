//
// Created by MarvelLi on 2024/2/26.
//

#pragma once
#include "Optimization/ReferenceSurface.h"
#include "Core/CoreMinimal.h"
#include "Mechanisms/SpatialJoints.h"
#include "Misc/Config.h"
#include "SpacialMechanism/ClosedChainMechSolver.h"

struct FKProblemSolver
{
	struct JointSimulationRecord
	{
		int TimeClipCount;
		String Type;
		TArray<double> Parameter;

		int EffectorParentJointIndex;
		FVector EffectorLocalLocation;  // best effector pos in parent joint space

		TArray<TArray<FTransform>> JointTransforms;
		TArray<FTransform> JointInitTransforms;
		TArray<double> SystemEnergys;
		TArray<double> Singularitys;
		TArray<double> ConstrainEnergys;
		TArray<double> JointDistance;

		// Driven ,Coupler, Coupler, Ground, Effector
		double LinkageLength[10];
		// Average linkage distance
		double LinkageLengthE;
		// Variance of linkage distance
		double LinkageLengthV = -1;

		void Clear();
		bool Valid() const;

		// This is the interface to calc the energys to one number
		// Energys should be in 5 dimension, represents for
		// CurveEnergy, SystemEnergy, Singularity, ConstrainEnergy, JointDistance
		// static double CalcSingleObjectiveEnergy(std::TArray<double>& Energys);

	};

	int TimeClipCount = GConfig.Get<int>("DesignerConfig", "MechanismEvaluationSteps");
	const double MechanismSolveTolerance = GConfig.Get<double>("DesignerConfig", "MechanismSolveTolerance");
	const double JointConstrainTolerance = GConfig.Get<double>("DesignerConfig", "JointConstrainTolerance");
	const double SingularityTolerance = GConfig.Get<double>("DesignerConfig", "SingularityTolerance");
	const double MinJointDistance = GConfig.Get<double>("DesignerConfig", "MinJointDistance");
	const double MinGroundXAngle = GConfig.Get<double>("DesignerConfig", "MinGroundXAngle");

	int EffectorParentIndex = -1;
	int EffectorIndex = -1;
	
	JointSimulationRecord Record;

	TArray<ObjectPtr<SpatialJoint>> Joints;

	// Init joints from parameter
	void InitFromParameter(const TArray<double>& P);

	// Solve the mechanism and record the system energy, singularity, constrain energy
	void SolveSystem();

	// Calc constrain for current frame
	void CalcConstrain();

	// Solve the best effector trajectory
	TArray<FVector> SolveEffectorTrajectory() const;

	TArray<double> MakeEngery();

	String Type;
	ObjectPtr<ClosedChainMechSolver> Solver;
	ObjectPtr<ReferenceSurface> Surface;
	TArray<FVector> TargetPoints;

	int JointNum = 0;

	bool bOptimizeSpatial = false;
	bool bOptimizeDev = false;

	// Implementation of the objective function
	TArray<double> Calc(TArray<double> dv);

	FKProblemSolver(
		String InType,
		const ObjectPtr<ReferenceSurface>& InSurface,
		bool InOptimizeSpatial = false);

	FKProblemSolver(
		String InType,
		const ObjectPtr<ReferenceSurface>& InSurface,
		const FVector& EffectorPos, bool InOptimizeSpatial = false);


};




struct FKProblem
{
	String Type;
	ObjectPtr<ReferenceSurface> Surface;
	TArray<FVector> TargetPoints;

	FKProblem() = default;

	FKProblem(String InType, const ObjectPtr<ReferenceSurface>& InSurface, const TArray<FVector>& InTargetPoints)
	: Type(std::move(InType)), Surface(InSurface), TargetPoints(InTargetPoints) {}

	FKProblem(const FKProblem& other)
	: Type(other.Type), Surface(other.Surface), TargetPoints(other.TargetPoints)
	{}

	std::pair<TArray<double>, TArray<double>> get_bounds() const;

	// System energy, constrain energy, joint distance energy, singularity, curve energy
	TArray<double> fitness(const TArray<double>& dv) const;

	static double WeightedFitness(const TArray<double>& Fitness);

	double get_nobj() const {return 1; }

	static TArray<double> RemapParameter(const TArray<double>& P);
};
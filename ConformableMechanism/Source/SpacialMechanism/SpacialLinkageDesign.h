#pragma once
#include "Core/CoreMinimal.h"
#include "Curve/Curve.h"
#include "Actors/CurveActor.h"
#include "Math/Random.h"
#include "Mechanisms/SpatialJoints.h"
#include "SpacialMechanism/ClosedChainMechSolver.h"

// We assume the first is driven joints, the last is the effector. And effector is child of joint2
// Frist driven joints transfrom is always Identity
struct SpacialLinkageConfiguration
{
    String Type;
    TArray<FTransform> InitTransform;
    TArray<FVector> EffectorTarjectory;

    void ReadFromFile(String FilePath);
};

typedef TArray<ObjectPtr<SpatialJoint>> JointArray;


class SpacialLinkageSolver
{
protected:
	// Set which effector parent joint
	int EffectorParentJointIndex = 1;
	FVector EffectorLocalPos;

    TArray<FVector> Trajectory;
    JointArray Joints;
    ObjectPtr<SpatialJoint> DrivenJoint;
    ObjectPtr<SpatialJoint> EffectorJoint;
	ObjectPtr<ClosedChainMechSolver> Solver;
    // Parameter should be (3Location + 3Euler)*3Joints
    virtual void InitFromParameter(String Types, const VectorXd& Parameter);

    // The min value in singularity matrix
    double CalcSingularity() const;

    // Calc current system energy and singularity energy
    double SolveMechanism();

	// 1. joint movement range constraint,
	// 2. joint distance constraint
	// 3. linkage length constraint
    double CalcConstrainEnergy() const;

public:
    virtual ~SpacialLinkageSolver() = default;

    bool BreakWhenInValid = false;
    ObjectPtr<CurveActor> TargetCurveActor;

    // System energy tolerance
    double SimulationTolerance = 1e-4;
    // Tolerance for singularity value during simulation
    double SingularityTolerance = 1e-2;

    double LinkageMinLength = 0.3;
    double JointMinDistance = 0.1;

    SpacialLinkageSolver(){}
    SpacialLinkageSolver(String Types, const VectorXd& Parameter);
    
    VectorXd CalcEnergy(bool& Valid);
    bool CheckLinkageCollide();
    SpacialLinkageConfiguration ExportConfiguration();

	// Calc best effector pos and corresponding trajectory
    ObjectPtr<Curve> GetEffectorTrajectory();

	// Solve a best effector pos for a given sequence of parent joints transform
	// Transform in time uniform order
	FVector SolveBestEffectorPos(const TArray<Matrix4d>& ParentTransform) const;
};
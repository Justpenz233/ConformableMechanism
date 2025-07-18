#include "SpacialLinkageDesign.h"
#include "Animation/IKJoint.h"
#include "Animation/Joints.h"
#include "CoreMinimal.h"
#include "Math/LinearAlgebra.h"
#include "Mechanisms/SpatialJoints.h"
#include "igl/segment_segment_intersect.h"
#include "Object/Object.h"
#include <cmath>


SpacialLinkageSolver::SpacialLinkageSolver(String Types, const VectorXd& Parameter)
{
    SpacialLinkageSolver::InitFromParameter(Types, Parameter);
}

void SpacialLinkageSolver::InitFromParameter(String Types, const VectorXd& Parameter)
{
	Joints.clear();
	//Driven Joint, Identity by default
	for(int JointIndex = 0; JointIndex < 4; JointIndex ++)
	{
		int Begin = JointIndex * 6;
		FVector Location = Parameter.segment(Begin, 3);
		FVector RotationEuler = Parameter.segment(Begin + 3, 3);
		FQuat Rotation = MMath::QuaternionFromEulerXYZ(RotationEuler);
		Joints.push_back(NewObject<SpatialJoint>(Types[JointIndex], FTransform{Location, Rotation}));
	}

	Joints[0]->SetIsRoot(true);
	DrivenJoint = Joints[0];

	Joints[0]->AddNextJoint(Joints[1]);
	Joints[1]->AddNextJoint(Joints[2]);
	Joints[2]->AddNextJoint(Joints[3]);
	Joints[3]->AddNextJoint(Joints[0]);

	Solver = NewObject<ClosedChainMechSolver>();
	Solver->AddJoints(Joints);
	Solver->Init();
}

double SpacialLinkageSolver::CalcSingularity() const
{
	return Solver->Singularity;
}

double SpacialLinkageSolver::SolveMechanism()
{
	return Solver->Solve();
}

bool SpacialLinkageSolver::CheckLinkageCollide()
{
    FVector Begin[5];
    FVector End[5];
    FVector Dir[5];
    for(int i = 0;i <= 3; i ++)
    {
        Begin[i] = Joints[i]->GlobalTransform.GetTranslation();
        End[i] = Joints[(i + 1)%4]->GlobalTransform.GetTranslation();
        Dir[i] = End[i] - Begin[i];
    }
    Begin[4] = Joints[1]->GlobalTransform.GetTranslation();
    End[4] = Joints[4]->GlobalTransform.GetTranslation();
    Dir[4] = End[4] - Begin[4];

    for(int i = 0; i <= 4;i ++)
    {
        for(int j = i + 1;j <= 4;j ++)
        {
            double t, u;
            if((Begin[i] - Begin[j]).norm() < 1e-6
             || (Begin[i] - End[j]).norm() < 1e-6
             || (End[i] - Begin[j]).norm() < 1e-6
             || (End[i] - End[j]).norm() < 1e-6 )
                continue;
            if(igl::segment_segment_intersect(Begin[i], Dir[i], Begin[j], Dir[j], t, u))
                return true;
        }
    }
    return false;
}

double SpacialLinkageSolver::CalcConstrainEnergy() const
{
    double Energy = 0.0;
    for(auto Joint : Joints)
    {
    	// Calc joint rotation constrain espacially for R U S

    	// Rotation movement clamp to [-pi, pi]
    	// This section can also handle opposite orientation(tangent and -tanget)
    	FVector JointRotation = Joint->GetAddRotationEuler();
    	FVector JointTranslation = Joint->GetAddTranslation();
    	for(int i = 0;i < 3;i ++)
    	{
    		JointRotation[i] = std::fmod(JointRotation[i], 2. * M_PI);
    		if(JointRotation[i] > M_PI)
				JointRotation[i] = 2. * M_PI - JointRotation[i];
			if(JointRotation[i] < -M_PI)
				JointRotation[i] = 2. * M_PI + JointRotation[i];
    	}

		for (int i = 0; i < 3; i++)
		{
			// Add rotation energy
			if (JointRotation[i] < Joint->AddRotationRange[0][i])
				Energy += std::pow(JointRotation[i] - Joint->AddRotationRange[0][i], 2.);
			if (JointRotation[i] > Joint->AddRotationRange[1][i])
				Energy += std::pow(JointRotation[i] - Joint->AddRotationRange[1][i], 2.);

			// Add translation energy
			// In translation, we donnot assume periodicity
			if (JointTranslation[i] < Joint->AddTranslationRange[0][i])
				Energy += std::pow(JointTranslation[i] - Joint->AddTranslationRange[0][i], 2.);
			if (JointTranslation[i] > Joint->AddTranslationRange[1][i])
				Energy += std::pow(JointTranslation[i] - Joint->AddTranslationRange[1][i], 2.);
		}
    }

	// Add collide energy, in smooth shape
    // if(CheckLinkageCollide())
    //     Energy += 10000.;

	// Calculate joint distance to avoid too close
	int JointNum = Joints.size();
    for(int i = 0;i < JointNum; i ++)
    {
        for(int j = i + 1; j < JointNum; j ++)
        {
            double Distance =
            (Joints[i]->GlobalTransform.GetTranslation() - Joints[j]->GlobalTransform.GetTranslation()).norm();
            if(Distance < JointMinDistance)
                Energy += (JointMinDistance - Distance);
        }
    }

	// Calculate linkage length to avoid too short
    for(int i = 0;i < JointNum; i ++) {
        double JointLength = (Joints[i]->InitTransform.GetTranslation() - Joints[(i + 1) % 4]->InitTransform.GetTranslation()).norm();
        if(JointLength < LinkageMinLength)
            Energy += (LinkageMinLength - JointLength);
    }
    return Energy;
}

VectorXd SpacialLinkageSolver::CalcEnergy(bool& Valid)
{
    VectorXd Energy(2);
    Energy << 0., 0.;
	// Then we run in detail, mainly for trajectory
	Valid = true;

	// Calc system energy and constrain energy
	double Delta = 0.2;
	for (int Iteration = 0; Iteration * Delta < 2 * M_PI; Iteration++)
	{
		FQuat DeltaRotation = MMath::QuaternionFromEulerXYZ({ 0, 0, Delta });
		DrivenJoint->GlobalTransform.AddRotationLocal(DeltaRotation);
		// System energy
		double SystemEnergy = SolveMechanism();
		// Constrain energy
		double ConstrainEnergy = CalcConstrainEnergy();

        Energy[0] += SystemEnergy + ConstrainEnergy;
        if(Energy[0] > SimulationTolerance || CalcSingularity() < SingularityTolerance)
            Valid = false;
		else
			Energy[0] = 0.0;

        if(BreakWhenInValid && !Valid)
            return Energy;
	}

	// Trajectory energy
    if (TargetCurveActor)
    {
        auto   CurrentCurve = GetEffectorTrajectory();
        double TrajectoryEnergy = TargetCurveActor->CalcSimilarity(CurrentCurve);
        Energy[1] += TrajectoryEnergy;
    }

	return Energy;
}

ObjectPtr<Curve> SpacialLinkageSolver::GetEffectorTrajectory()
{
	Trajectory.clear();
	// Then we run in detail, mainly for trajectory
	std::vector<Matrix4d> EffectorParentTransform;

	double Delta = 0.05;
	for (int Iteration = 0; Iteration * Delta < 2 * M_PI; Iteration ++)
	{
		FQuat DeltaRotation = MMath::QuaternionFromEulerXYZ({ 0, 0, Delta });
		DrivenJoint->AddRotation(DeltaRotation);
		SolveMechanism();
		EffectorParentTransform.push_back(Joints[EffectorParentJointIndex]->GlobalTransform.GetMatrix());
	}

	EffectorLocalPos = SolveBestEffectorPos(EffectorParentTransform);

	for(const auto& i : EffectorParentTransform)
	{
		Vector4d AffinePos;
		AffinePos << EffectorLocalPos, 1.;
		Vector4d Pos = i * AffinePos;
		Trajectory.emplace_back(Pos[0], Pos[1], Pos[2]);
	}

	auto Result = NewObject<Curve>();
	Result->SetCurveData(Trajectory);
	return Result;
}

FVector SpacialLinkageSolver::SolveBestEffectorPos(const std::vector<Matrix4d>& ParentTransform) const
{
	int		 TimeSlot = ParentTransform.size();
	MatrixXd A(4 * TimeSlot, 4);
	MatrixXd B(4 * TimeSlot, 1);

	// A * x = B
	// (4n,4)  * (4,1) = (4n , 1)
	double delT = 1. / (TimeSlot - 1);
	for (int i = 0; i < TimeSlot; i++)
	{
		A.block(i * 4, 0, 4, 4) = ParentTransform[i];

		double	 t = i * delT;
		FVector	 Pos = TargetCurveActor->Sample(t);
		Vector4d AffinePos;
		AffinePos << Pos, 1.;
		B.block(i * 4, 0, 4, 1) = AffinePos;
	}
	MatrixXd X = LinearAlgbera::LinearEquationSolver(A, B);
	FVector	 Result = X.block(0, 0, 3, 1);
	return Result;
}

SpacialLinkageConfiguration SpacialLinkageSolver::ExportConfiguration()
{
	SpacialLinkageConfiguration Result;

	for(auto i : Joints)
	{
		Result.Type += i->Type;
		Result.InitTransform.push_back(i->InitTransform);
	}

	if(Trajectory.empty())
	{
		GetEffectorTrajectory();
		Result.EffectorTarjectory = Trajectory;
	}

	FTransform EffectorTransform = Joints[EffectorParentJointIndex]->InitTransform;
	EffectorTransform.AddTranslationLocal(EffectorLocalPos);
	Result.Type += "E";
	Result.InitTransform.push_back(EffectorTransform);

	return Result;
}
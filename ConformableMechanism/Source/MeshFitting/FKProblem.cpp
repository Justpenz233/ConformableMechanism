//
// Created by MarvelLi on 2024/2/26.
//

#include "FKProblem.h"
#include "igl/signed_distance.h"
#include "igl/AABB.h"
#include "InitJointFromParameter.h"
#include "Math/Geometry.h"
#include "Math/Intersect.h"
#include "Math/Random.h"
#include "SpacialMechanism/SurfaceJoint.h"
#include "spdlog/stopwatch.h"
#include <pagmo/types.hpp>

FKProblemSolver::FKProblemSolver(String InType,
	const ObjectPtr<ReferenceSurface>& InSurface, bool InOptimizeSpatial)
	: Type(std::move(InType)), Surface(InSurface), bOptimizeSpatial(InOptimizeSpatial)
{
	Record.Type = Type;
	ASSERTMSG(Type.find('E') != String::npos, "Type should contain E");
	JointNum = Type.length() - 1;	 // minus the E
	EffectorIndex = Type.size() - 1; // Put effector at the end
	EffectorParentIndex = Type.find('E') - 1;
	Type.erase(Type.find('E'), 1);
	Type += "E";
}

FKProblemSolver::FKProblemSolver(String InType, const ObjectPtr<ReferenceSurface>& InSurface, const FVector& EffectorPos, bool InOptimizeSpatial)
	: Type(std::move(InType)), Surface(InSurface), bOptimizeSpatial(InOptimizeSpatial)
{
	Record.Type = Type;
	ASSERTMSG(Type.find('E') != String::npos, "Type should contain E");
	JointNum = Type.length() - 1;	 // minus the E
	EffectorIndex = Type.size() - 1; // Put effector at the end
	EffectorParentIndex = Type.find('E') - 1;
	Type.erase(Type.find('E'), 1);
	Type += "E";
	Record.EffectorLocalLocation = EffectorPos;
}

void FKProblemSolver::JointSimulationRecord::Clear()
{
	TimeClipCount = 0;
	JointTransforms.clear();
	SystemEnergys.clear();
	Singularitys.clear();
	ConstrainEnergys.clear();
	JointDistance.clear();
}

bool FKProblemSolver::JointSimulationRecord::Valid() const
{
	if (TimeClipCount <= 0)
		return false;
	if (EffectorParentJointIndex < 0)
		return false;
	if (JointTransforms.empty())
		return false;
	if (SystemEnergys.empty())
		return false;
	if (Singularitys.empty())
		return false;
	if (ConstrainEnergys.empty())
		return false;
	if (JointDistance.empty())
		return false;
	return true;
}

void FKProblemSolver::InitFromParameter(const pagmo::vector_double& P)
{
	if (bOptimizeSpatial)
		Joints = InitFromParametersSpatial(Type, Surface, P);
	else if(bOptimizeDev)
		Joints = InitFromParametersDev(Surface, P);
	else
		Joints = InitFromParameters(Type, Surface, P);

	// Optimize develpable mechanism
	// Joints = InitFromParametersDev(Surface, P);

	Record.JointInitTransforms.clear();
	Record.JointInitTransforms.resize(Type.length());
	for(int i = 0; i < JointNum; i ++)
		Record.JointInitTransforms[i] = Joints[i]->InitTransform;
	Solver = NewObject<ClosedChainMechSolver>();
	Solver->AddJoints(Joints);
	Solver->Init();

	double Distance = 100000.;
	for (int i = 0; i < JointNum; i++)
	{
		for (int j = i + 1; j < JointNum; j++)
		{
			Distance = std::min(Distance,
				(Joints[i]->InitTransform.GetTranslation() - Joints[j]->InitTransform.GetTranslation()).norm());
		}
	}
	Record.JointDistance.push_back(Distance);
}

void FKProblemSolver::SolveSystem()
{
	double loss = Solver->Solve();
	Record.Singularitys.push_back(Solver->Singularity);
	Record.SystemEnergys.push_back(loss);

	// Record joint transform, except effector(calc later)
	TArray<FTransform> CurrentJoints(JointNum);
	for (int i = 0; i < JointNum; i++)
		CurrentJoints[i] = Joints[i]->GlobalTransform;
	Record.JointTransforms.emplace_back(std::move(CurrentJoints));
}

void FKProblemSolver::CalcConstrain()
{
	// Constrain energy calcuation
	double ConstrainEnergy = 0.;
	for (int Index = 1; Index < JointNum; Index++)
	{
		auto Joint = Joints[Index];
		// Rotation movement clamp to [-pi, pi]
		// This section can also handle opposite orientation(tangent and -tanget)
		FVector JointRotation = Joint->GetAddRotationEuler();
		FVector JointTranslation = Joint->GetAddTranslation();


		for (int i = 0; i < 3; i++)
		{
			// Here we assume the joint orientation can be forward and backward, shoule process at future mesh generation
			JointRotation[i] = std::fmod(JointRotation[i], 2. * M_PI);
			if (JointRotation[i] > M_PI)
				JointRotation[i] = JointRotation[i] - M_PI;
			if (JointRotation[i] < -M_PI)
				JointRotation[i] = JointRotation[i] + M_PI;
		}

		for (int i = 0; i < 3; i++)
		{
			// Add rotation energy
			if (JointRotation[i] < Joint->AddRotationRange[0][i])
				ConstrainEnergy += std::pow(JointRotation[i] - Joint->AddRotationRange[0][i], 2.);
			if (JointRotation[i] > Joint->AddRotationRange[1][i])
				ConstrainEnergy += std::pow(JointRotation[i] - Joint->AddRotationRange[1][i], 2.);

			// Add translation energy
			// In translation, we donnot assume periodicity
			if (JointTranslation[i] < Joint->AddTranslationRange[0][i])
				ConstrainEnergy += std::pow(JointTranslation[i] - Joint->AddTranslationRange[0][i], 2.);
			if (JointTranslation[i] > Joint->AddTranslationRange[1][i])
				ConstrainEnergy += std::pow(JointTranslation[i] - Joint->AddTranslationRange[1][i], 2.);
		}
	}
	Record.ConstrainEnergys.emplace_back(ConstrainEnergy);

	// Calc joint distance energy
	// Min distance between two joints
	double Distance = 100000.;
	for (int i = 0; i < JointNum; i++)
	{
		for (int j = i + 1; j < JointNum; j++)
		{
			if(i == 0 && j == 1) continue;
			Distance = std::min(Distance,
				(Joints[i]->GlobalTransform.GetTranslation() - Joints[j]->GlobalTransform.GetTranslation()).norm());
		}
	}
	Record.JointDistance.push_back(Distance);
}

pagmo::vector_double FKProblemSolver::Calc(pagmo::vector_double dv)
{
	ASSERT(Type.length() >= 5);
	ASSERT(JointNum >= 4);
	ASSERT(EffectorParentIndex != -1);

	Record.Parameter = dv;

	// Can evaluate the energy by distance to boundary

	InitFromParameter(dv);

	{
		auto FixedType = Type;
		FixedType.erase(FixedType.find('E'), 1);
		for (int i = 1; i < Joints.size(); i ++)
		{
			if (Joints[i]->Type != 'R' && Joints[i]->Type != 'U') continue;

			auto Param = JointParameter(FixedType, dv, i);
			auto UV = FVector2{Param[0], Param[1]};
			auto SurfaceNormal = Surface->SampleNormal(UV[0], UV[1]);
			FVector RotationAxis;
			if (Joints[i]->Type == 'R') // Z axis
				RotationAxis = (Joints[i]->InitTransform.GetRotation() * FVector::UnitZ()).normalized();
			if (Joints[i]->Type == 'U') // X axis
			{
				RotationAxis = (Joints[i]->InitTransform.GetRotation() * FVector::UnitX()).normalized();
			}

			auto CosAngle = abs(RotationAxis.dot(SurfaceNormal));
			if (CosAngle > std::cos(M_PI / 6.))
			{
				Record.ConstrainEnergys.push_back(CosAngle - std::cos(M_PI / 12.));
			}
		}
	}

	{
		TArray<FTransform> InitT(JointNum);
		for (int i = 0; i < JointNum; i++)
			InitT[i] = Joints[i]->InitTransform;
		Record.JointTransforms.emplace_back(std::move(InitT));
	}
	Record.EffectorParentJointIndex = EffectorParentIndex;
	Record.TimeClipCount = TimeClipCount;

	double Delta = 2 * M_PI / TimeClipCount;
	// Simulation first
	for (int TimeIndex = 0; TimeIndex < TimeClipCount; TimeIndex++)
	{
		// Driven joint
		if (Joints[0]->Type == 'R' || Joints[0]->Type == 'D')
		{
			FQuat DeltaRotation = MMath::QuaternionFromEulerXYZ({ 0, 0, Delta });
			Joints[0]->GlobalTransform.AddRotationLocal(DeltaRotation);
		}
		SolveSystem();
		CalcConstrain();
	}
	Record.LinkageLengthE = std::accumulate(Record.LinkageLength, Record.LinkageLength + 5, 0.) / 5.;
	Record.LinkageLengthV = std::accumulate(Record.LinkageLength, Record.LinkageLength + 5, 0.,
								[&](double Sum, double Length) {
									return Sum + std::pow(Length - Record.LinkageLengthE, 2.);})/ 5.;
	return MakeEngery();
}

pagmo::vector_double FKProblemSolver::MakeEngery()
{
	pagmo::vector_double Result(6);

	Result[0] = 0.;
	// System energy = 0
	ASSERT(!Record.SystemEnergys.empty());
	Result[1] = *std::ranges::max_element(Record.SystemEnergys);

	// Constrain energy = 0
	ASSERT(!Record.ConstrainEnergys.empty());
	Result[2] = std::accumulate(Record.ConstrainEnergys.begin(), Record.ConstrainEnergys.end(), 0.);

	// Singularity energy
	ASSERT(!Record.Singularitys.empty());
	Result[3] = *std::ranges::min_element(Record.Singularitys);

	// max Joint distance energy
	ASSERT(!Record.JointDistance.empty());
	Result[4] = *std::ranges::min_element(Record.JointDistance);

	if(Result[0] < 0.) Result[0] = 0.;
	if(Result[1] < MechanismSolveTolerance) Result[1] = 0.;
	if(Result[2] < JointConstrainTolerance) Result[2] = 0.;
	if (Result[3] > SingularityTolerance) Result[3] = 0.;
	else Result[3] = SingularityTolerance - Result[3];

	return Result;
}


std::pair<pagmo::vector_double, pagmo::vector_double> FKProblem::get_bounds() const
{
	int JointNum = Type.length() - 1; // minus the E
	static auto MinU = GConfig.Get<double>("DesignerConfig", "MinU");
	static auto MaxU = GConfig.Get<double>("DesignerConfig", "MaxU");
	static auto MinV = GConfig.Get<double>("DesignerConfig", "MinV");
	static auto MaxV = GConfig.Get<double>("DesignerConfig", "MaxV");
	pagmo::vector_double Min;
	pagmo::vector_double Max;
	for (int i = 0; i < JointNum; i++)
	{
		Min.push_back(MinU);
		Min.push_back(MinV);
		Min.push_back(0);
		Max.push_back(MaxU);
		Max.push_back(MaxV);
		Max.push_back(1);
	}
	return { Min, Max };
}

pagmo::vector_double FKProblem::fitness(const pagmo::vector_double& dv) const
{
	FKProblemSolver Solver(Type, Surface);
	auto					 t = Solver.Calc(dv);
	ASSERT(t.size() >= 5);
	return {WeightedFitness(Solver.Calc(dv))};
}
double FKProblem::WeightedFitness(const pagmo::vector_double& Fitness)
{
	static const auto	ConstrainWeight = GConfig.Get<double>("FitnessWeights", "ConstrainWeight");
	static const auto MinJointDistance = GConfig.Get<double>("DesignerConfig", "MinJointDistance");

	double ConstrainEnergy = (Fitness[0] + Fitness[1] + Fitness[2] + Fitness[3] +
		(Fitness[4] < MinJointDistance ? 1 : 0)) * ConstrainWeight;
	return { ConstrainEnergy};

}

pagmo::vector_double FKProblem::RemapParameter(const pagmo::vector_double& P)
{
	pagmo::vector_double NewP(P.size());
	for(int i = 0;i < P.size();i ++)
	{
		auto pi = P[i];
		pi = std::fmod(pi, 1.);
		if(pi < 0) pi += 1.;

		if(i % 3 == 1) pi = std::clamp(pi, 0.05, 0.95); // V
		if(i % 3 == 2) pi = pi * M_PI * 2.; // Theta
		NewP[i] = pi;
	}
	return NewP;
}

TArray<FVector> FKProblemSolver::SolveEffectorTrajectory() const
{
	TArray<FVector> Pos;
	for (int i = 0; i < Record.TimeClipCount; i++)
	{
		Pos.emplace_back(Record.JointTransforms[i][EffectorParentIndex] * Record.EffectorLocalLocation);
	}
	return Pos;
}
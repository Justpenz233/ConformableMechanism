//
// Created by MarvelLi on 2024/1/31.
//

#pragma once

#include "Actors/ParametricMeshActor.h"
#include "Core/CoreMinimal.h"
#include "Math/LinearAlgebra.h"
#include "Mechanisms/SpatialJoints.h"

#include <Optimization/JointParameterDefine.h>

// With R and P joints set rotation angle
// X suite to tangent with Theta dir
// [u, v, theta] * 4
inline TArray<ObjectPtr<SpatialJoint>> InitFromParameters(
	const String& Type, const ObjectPtr<ParametricMeshActor>& Surface, const TArray<double>& P)
{
	ASSERT(P.size() == GetParameterNum(Type));
	TArray<ObjectPtr<SpatialJoint>> Joints;
	Joints.clear();

	auto SampleTangent = [Surface](double u, double v, double theta) {
		double u1 = u + std::sin(theta) * 1e-2;
		double v1 = v + std::cos(theta) * 1e-2;
		return (Surface->Sample(u1, v1) - Surface->Sample(u, v)).normalized();
	};

	auto SampleNormal = [Surface, &SampleTangent](double u, double v) {
		auto a = SampleTangent(u, v, 0);
		auto b = SampleTangent(u, v, M_PI_2);
		return a.cross(b).normalized();
	};

	// Driven Joint, Identity by default
	int Begin = 0;
	for (int i = 0;i < Type.size();i ++)
	{
		auto JointType = Type[i];
		if(JointType == 'E') continue;
		int JointParamSize = JointParameterMap(JointType, i == 0);

		double u = P[Begin + 0], v = P[Begin + 1];

		FVector Location = Surface->Sample(u, v);
		FQuat Rotation = FQuat::Identity();
		// if(JointType == 'R' && i == 0)
		// {
		// 	theta *= 2. * M_PI;
		// 	FQuat XToTangent = FQuat::FromTwoVectors(FVector{1., 0., 0.}, SampleTangent(u, v, theta));
		// 	FQuat ZToNormal = FQuat::FromTwoVectors(XToTangent * Eigen::Vector3d::UnitZ(), Surface->SampleNormal(u, v));
		// 	Rotation = ZToNormal * XToTangent;
		// }
		if(JointType == 'D' || JointType == 'S' || (JointType == 'R' && i == 0))
		{
			double theta = P[Begin + 2];
			theta *= 2. * M_PI;
			FQuat XToTangent = FQuat::FromTwoVectors(FVector{1., 0., 0.}, SampleTangent(u, v, theta));
			FQuat YToNormal = FQuat::FromTwoVectors(XToTangent * Eigen::Vector3d::UnitY(), Surface->SampleNormal(u, v));
			Rotation = YToNormal * XToTangent;
		}
		else if(JointType == 'R')
		{
			double theta = P[Begin + 2], omega = P[Begin + 3];
			omega = std::acos(1. - omega); theta *= 2. * M_PI;
			FVector ZAxis = {sin(omega) * cos(theta), sin(omega) * sin(theta), cos(omega)};
			Rotation = FQuat::FromTwoVectors(FVector::UnitZ(), ZAxis);
		}
		else if (JointType == 'U')
		{
			double theta = P[Begin + 2], omega = P[Begin + 3], gama = P[Begin + 4];
			omega *= 2. * M_PI;
			theta *= 2. * M_PI;
			gama  *= 2. * M_PI;
			Rotation = MMath::QuaternionFromEulerXYZ({omega, theta, gama});
		}
		// else if (JointType == 'U')
		// {
		// 	double theta = P[Begin + 2];
		// 	double omega = P[Begin + 3];
		// 	omega = std::acos(1. - omega);
		// 	theta *= 2. * M_PI;
		// 	FVector YAxis = {sin(omega) * cos(theta), sin(omega) * sin(theta), cos(omega)};
		// 	Rotation = FQuat::FromTwoVectors(FVector::UnitY(), YAxis);
		// }
		Joints.push_back(NewObject<SpatialJoint>(JointType, FTransform(Location, Rotation)));
		Begin += JointParamSize;
	}

	Joints[0]->SetIsRoot(true);
	for (int i = 0;i < Joints.size();i++)
		Joints[i]->AddNextJoint(Joints[(i + 1) % Joints.size()]);
	return Joints;
}

inline TArray<double> JointParameter(const String& Type, const TArray<double>& P, int JointIndex)
{
	int num = 0;
	for (int i = 0; i < JointIndex;i ++)
		num += JointParameterMap(Type[i], i == 0);
	return TArray<double>(P.begin() + num, P.begin() + num + JointParameterMap(Type[JointIndex], JointIndex == 0));
}

inline TArray<ObjectPtr<SpatialJoint>> InitFromParametersSpatial(const String& Type, const ObjectPtr<ParametricMeshActor>& Surface, const TArray<double>& P)
{
	ASSERT(P.size() == (Type.size() - 1) * 6);
	TArray<ObjectPtr<SpatialJoint>> Joints;
	Joints.clear();


	int Begin = 0;
	for (const char & JointType : Type)
	{
		if(JointType == 'E') continue;
		double x = P[Begin + 0], y = P[Begin + 1], z = P[Begin + 2];
		double xo = P[Begin + 3] * 2. * M_PI, yo = P[Begin + 4] * 2. * M_PI, zo = P[Begin + 5] * 2. * M_PI;
		Begin += 6;

		FVector Location{x, y, z};
		FQuat Rotation =
			Eigen::AngleAxisd(zo, Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(yo, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(xo, Eigen::Vector3d::UnitX());

		Joints.push_back(NewObject<SpatialJoint>(JointType, FTransform(Location, Rotation)));
	}

	Joints[0]->SetIsRoot(true);
	for (int i = 0;i < Joints.size();i++)
		Joints[i]->AddNextJoint(Joints[(i + 1) % Joints.size()]);
	return Joints;
}

inline TArray<ObjectPtr<SpatialJoint>> InitFromParametersDev(
	const ObjectPtr<ParametricMeshActor>& Surface, const TArray<double>& P)
{
	auto RullingLineDir = [Surface](double u, double v) -> FVector {
		return Surface->Sample(u, v + 1e-2) - Surface->Sample(u, v);
	};
	double StartV = P[0];
	static constexpr double HalfWidth = 0.04;
	TArray<ObjectPtr<SpatialJoint>> Joints;

	for (int i = 0; i < 4; i ++)
	{
		double U = P[i + 1];
		double V = StartV + i * HalfWidth * 2;

		auto Location = Surface->Sample(U, V);
		auto Rotation = FQuat::FromTwoVectors(FVector::UnitZ(), RullingLineDir(U, V));
		Joints.push_back(NewObject<SpatialJoint>('R', FTransform(Location, Rotation), i == 0));
	}
	Joints[0]->SetIsRoot(true);
	for (int i = 0;i < Joints.size();i++)
		Joints[i]->AddNextJoint(Joints[(i + 1) % Joints.size()]);
	return Joints;
}


// R, U, S have no limitation about rotation around X, so check X is enough
inline double GroundJointRotationAngleCos(
	const FTransform& ATransform,
	const FTransform& BTransform)
{
	auto AX = ATransform.GetRotation() * Eigen::Vector3d::UnitX();
	auto BX = BTransform.GetRotation() * Eigen::Vector3d::UnitX();
	return abs(AX.dot(BX));
}

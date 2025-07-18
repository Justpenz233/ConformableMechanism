//
// Created by MarvelLi on 2024/7/3.
//

#pragma once
#include "Math/Box.h"
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/shape/geometric_shapes.h>

// Signed distance between a cylinder and a box
inline double CylinderBoxDistance(double Length, double Radius, const FTransform& CylinderT,
	const FVector& BoxSize, const FTransform& BoxT)
{
	hpp::fcl::Cylinder Cylinder(Radius, Length);
	hpp::fcl::Transform3f CylinderTransform(CylinderT.GetRotationMatrix().block(0,0,3,3), CylinderT.GetLocation());
	hpp::fcl::Box Box(BoxSize);
	hpp::fcl::Transform3f BoxTransform(BoxT.GetRotationMatrix().block(0,0,3,3), BoxT.GetLocation());

	hpp::fcl::DistanceRequest distanceRequest(true, true, 0);
	hpp::fcl::DistanceResult distanceResult;
	hpp::fcl::distance(&Cylinder, CylinderTransform, &Box, BoxTransform, distanceRequest, distanceResult);

	return distanceResult.min_distance;
}

inline double CapsuleBoxDistance(double Length, double Radius, const FTransform& CapsuleT,
	const FVector& BoxSize, const FTransform& BoxT)
{
	hpp::fcl::Capsule Capsule(Radius, Length);
	hpp::fcl::Transform3f CapsuleTransform(CapsuleT.GetRotationMatrix().block(0,0,3,3), CapsuleT.GetLocation());
	hpp::fcl::Box Box(BoxSize);
	hpp::fcl::Transform3f BoxTransform(BoxT.GetRotationMatrix().block(0,0,3,3), BoxT.GetLocation());

	hpp::fcl::DistanceRequest distanceRequest(true, true, 0);
	hpp::fcl::DistanceResult distanceResult;
	hpp::fcl::distance(&Capsule, CapsuleTransform, &Box, BoxTransform, distanceRequest, distanceResult);

	return distanceResult.min_distance;
}

inline std::pair<hpp::fcl::Cylinder, hpp::fcl::Transform3f> ToCylinder(const FVector& CylinderTop, const FVector& CylinderBottom, double Radius)
{
	auto Length = (CylinderTop - CylinderBottom).norm();
	FTransform CylinderT = FTransform::Identity();
	CylinderT.SetTranslation((CylinderTop + CylinderBottom) * 0.5);
	CylinderT.SetRotation(FQuat::FromTwoVectors(FVector{0, 0, 1}, CylinderTop - CylinderBottom));
	hpp::fcl::Transform3f CylinderTransform(CylinderT.GetRotationMatrix().block(0,0,3,3), CylinderT.GetLocation());
	return {hpp::fcl::Cylinder(Radius, Length), CylinderTransform};
}

inline double CylinderBoxDistance(
	const FVector& CylinderTop, const FVector& CylinderBottom, double Radius,
	const FBox& Box)
{
	FTransform CylinderT = FTransform::Identity();
	CylinderT.SetTranslation((CylinderTop + CylinderBottom) * 0.5);
	CylinderT.SetRotation(FQuat::FromTwoVectors(FVector{0, 0, 1}, CylinderTop - CylinderBottom));

	FTransform BoxT = FTransform::Identity();
	BoxT.SetTranslation(Box.GetCenter());

	return CylinderBoxDistance((CylinderTop - CylinderBottom).norm(), Radius, CylinderT, Box.GetSize(), BoxT);
}

inline double CylinderCylinderDistance(const FVector& S0, const FVector& E0, const FVector& S1, const FVector& E1, double Radius)
{
	auto [Cylinder0, T0] = ToCylinder(S0, E0, Radius);
	auto [Cylinder1, T1] = ToCylinder(S1, E1, Radius);

	hpp::fcl::DistanceRequest distanceRequest(true, true, 0);
	hpp::fcl::DistanceResult distanceResult;
	hpp::fcl::distance(&Cylinder0, T0, &Cylinder1, T1, distanceRequest, distanceResult);
	return distanceResult.min_distance;
}

inline double CylinderCylinderCollide(const FVector& S0, const FVector& E0, const FVector& S1, const FVector& E1, double Radius)
{
	auto [Cylinder0, T0] = ToCylinder(S0, E0, Radius);
	auto [Cylinder1, T1] = ToCylinder(S1, E1, Radius);

	hpp::fcl::CollisionRequest request;
	hpp::fcl::CollisionResult result;
	hpp::fcl::collide(&Cylinder0, T0, &Cylinder1, T1, request, result);
	if (!result.isCollision())
		return -1.;
	else
		return abs(result.getContact(0).penetration_depth);
}
inline double CapsuleBoxDistance(
	const FVector& CapsuleTop, const FVector& CapsuleBottom, double Radius,
	const FBox& Box)
{
	FTransform CapsuleT = FTransform::Identity();
	CapsuleT.SetTranslation((CapsuleTop + CapsuleBottom) * 0.5);
	CapsuleT.SetRotation(FQuat::FromTwoVectors(FVector{0, 0, 1}, CapsuleTop - CapsuleBottom));

	FTransform BoxT = FTransform::Identity();
	BoxT.SetTranslation(Box.GetCenter());

	return CapsuleBoxDistance((CapsuleTop - CapsuleBottom).norm(), Radius, CapsuleT, Box.GetSize(), BoxT);
}

inline bool TriangleBoxCollision(
	const FVector& V0, const FVector& V1, const FVector& V2,
	const FBox& Box, const FTransform& BoxTransform )
{
	hpp::fcl::TriangleP Triangle(
		hpp::fcl::Vec3f(V0[0], V0[1], V0[2]),
		hpp::fcl::Vec3f(V1[0], V1[1], V1[2]),
		hpp::fcl::Vec3f(V2[0], V2[1], V2[2])
	);
	hpp::fcl::Box BoxShape(Box.GetSize());
	hpp::fcl::Transform3f BoxT(BoxTransform.GetRotationMatrix().block(0,0,3,3), BoxTransform.GetLocation());
	BoxT.translation() += Box.GetCenter();
	hpp::fcl::CollisionObject TriangleObject(std::make_shared<hpp::fcl::TriangleP>(Triangle), hpp::fcl::Transform3f::Identity());
	hpp::fcl::CollisionObject BoxObject(std::make_shared<hpp::fcl::Box>(BoxShape), BoxT);

	hpp::fcl::CollisionRequest request;
	hpp::fcl::CollisionResult result;
	hpp::fcl::collide(&TriangleObject, &BoxObject, request, result);

	return result.isCollision();
}
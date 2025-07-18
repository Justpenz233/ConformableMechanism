//
// Created by marvel on 9/8/24.
//

#pragma once
#include "Actors/ParametricMeshActor.h"
#include "Surface/OrientedSurfaceComponent.h"

MCLASS(ReferenceSurface)
class ReferenceSurface : public ParametricMeshActor
{
    REFLECTION_BODY(ReferenceSurface)
public:
	ReferenceSurface(const ObjectPtr<ParametricSurface>& SurfaceData, double Thickness = 0.05, const ObjectPtr<StaticMesh>& OrientedSurface = nullptr)
		: ParametricMeshActor(SurfaceData, Thickness)
	{
		if (OrientedSurface)
			OrientedComponent = AddComponent<OrientedSurfaceComponent>(OrientedSurface).get();
	}

	ReferenceSurface(const ObjectPtr<ParametricSurface>& SurfaceData, const ObjectPtr<StaticMesh>& DisplayMesh, const ObjectPtr<StaticMesh>& OrientedSurface = nullptr)
		: ParametricMeshActor(SurfaceData, DisplayMesh)
	{
		if (OrientedSurface)
			OrientedComponent = AddComponent<OrientedSurfaceComponent>(OrientedSurface).get();
	}

	ReferenceSurface(const ObjectPtr<StaticMesh>& InitMesh, ParametrizationMethod Method, const ObjectPtr<StaticMesh>& OrientedSurface = nullptr)
		: ParametricMeshActor(InitMesh, Method)
	{
		if (OrientedSurface)
			OrientedComponent = AddComponent<OrientedSurfaceComponent>(OrientedSurface).get();
	}

	ReferenceSurface(const ObjectPtr<StaticMesh>& DisplayMesh, const ObjectPtr<StaticMesh>& PMesh,
	ParametrizationMethod Method, const ObjectPtr<StaticMesh>& OrientedSurface = nullptr)
		: ParametricMeshActor(DisplayMesh, PMesh, Method)
	{
		if (OrientedSurface)
			OrientedComponent = AddComponent<OrientedSurfaceComponent>(OrientedSurface).get();
	}


	bool IsInside(const FVector& Point) const
	{
		ASSERT(OrientedComponent != nullptr);
		return OrientedComponent->Inside(GetFTransform().ToLocalSpace(Point));
	}

	/**
	 * Calculate the signed distance from the point to the surface
	 * @param Point The point to calculate
	 * @return The signed distance from the point to the surface, if the point is inside the surface, the distance will be negative
	 */
	double SignedDistance(const FVector& Point) const
	{
		ASSERT(OrientedComponent != nullptr);
		return OrientedComponent->SignedDistance(GetFTransform().ToLocalSpace(Point));
	}

	double Distance(const FVector& Point) const
	{
		ASSERT(OrientedComponent != nullptr);
		return OrientedComponent->Distance(GetFTransform().ToLocalSpace(Point));
	}

	void SetSurfaceMesh(const ObjectPtr<StaticMesh>& Mesh)
	{
		SurfaceMesh = Mesh;
	}

	// Get the mesh for collision detection
	ObjectPtr<StaticMesh> GetSurfaceMesh()
	{
		return SurfaceMesh ? SurfaceMesh : GetParametricMeshComponent()->GetZeroThicknessMesh();
	}

protected:
	ReferenceSurface() = default;
    OrientedSurfaceComponent* OrientedComponent;

	ObjectPtr<StaticMesh> SurfaceMesh = nullptr;

};
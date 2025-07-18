//
// Created by MarvelLi on 2024/10/17.
//

#pragma once

#include "LightEnv.h"
#include "Actors/CameraActor.h"
#include "Optimization/ReferenceSurface.h"
#include "Game/World.h"
#include "Materials/Material.h"
#include "MeshFitting/MeshFittingUI.h"

inline auto MotionGenerationExample()
{
	return [](World& World)
	{
		auto Camera = World.SpawnActor<CameraActor>("MainCamera");
		Camera->SetTranslation({-5, 0, 0});
		Camera->LookAt({0,0,0});

		DefaultLightEnv(World);

		auto RMesh = StaticMesh::LoadObj(Path::ProjectContentDir()/"Flower.obj");
		auto DMesh = Algorithm::GeometryProcess::SolidifyMeshEven(RMesh, 0.02);
		auto Surface = World.SpawnActor<ReferenceSurface>(
			"ReferenceSurface", DMesh, RMesh, BoxBorderConformal);
		Surface->GetParametricMeshComponent()->GetMeshData()->GetMaterial()->SetAlpha(0.5f);

		auto ZeroThickness = World.SpawnActor<StaticMeshActor>("ZeroThicknessMesh", RMesh);
		ZeroThickness->SetVisible(false);

		auto Axis = StaticMesh::LoadObj(Path::ProjectContentDir() / "Axis.obj")->Scale(0.2)->GetThis<StaticMesh>();
		TArray<ObjectPtr<StaticMeshActor>> TargetPoints;
		TargetPoints.push_back(World.SpawnActor<StaticMeshActor>("TargetPoints1", Axis));
		TargetPoints.push_back(World.SpawnActor<StaticMeshActor>("TargetPoints2", Axis));

		TargetPoints[1]->SetTranslation(FVector{-0.8, 0, 1.})->SetRotation(FVector{0, DegToRad(330), 0});
		TargetPoints[0]->SetTranslation(FVector{-0.15, -0.5, 1.1})->SetRotation(FVector{DegToRad(22), DegToRad(0), DegToRad(20)});
		TargetPoints.push_back(World.SpawnActor<StaticMeshActor>("TargetPoints3", Axis));
		TargetPoints[2]->SetTranslation(FVector{0.56, 0, 1.})->SetRotation(FVector{DegToRad(18), DegToRad(20), DegToRad(70)});

		TArray<FTransform> TargetPointSample;
		for (const auto& i : TargetPoints)
			TargetPointSample.push_back(i->GetFTransform());

		World.AddWidget<MeshFittingUI>(Surface, TargetPointSample);
	};
}

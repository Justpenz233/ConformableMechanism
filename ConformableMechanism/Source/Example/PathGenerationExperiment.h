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

inline auto PathGenerationExample()
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

		TArray<ObjectPtr<TargetPointsActor>> TargetPoints;
		TargetPoints.push_back(World.SpawnActor<TargetPointsActor>("TargetPoints1"));
		TargetPoints.push_back(World.SpawnActor<TargetPointsActor>("TargetPoints2"));
		TargetPoints.push_back(World.SpawnActor<TargetPointsActor>("TargetPoints3"));

		TargetPoints[0]->SetTranslation(FVector{0, -0.43, 1});
		TargetPoints[1]->SetTranslation(FVector{0.3, 0., 0.7});
		TargetPoints[2]->SetTranslation(FVector{0., 0.3, 0.7});


		// RSEUR
		// 0.6502709589647465, 0.6282168886620556, 0.7430210728934388, 0.36381900805623885, 0.49569553901001995, 0.40347577647275, 0.8642248601536586, 0.5153498908212903, 0.506349623454013, 0.35275068160836925, 0.3952458211677759, 0.3169246582799269, 0.1397814945594053, 0.05259153693906711, 0.5459390699887695, 0.5123256703463943, 0.5295455779352315, 0.4990459601932346, 0.559256572030057, 0.4432199901318653, 0.4985396618766895, 0.4682821690280271, 0.5010141572318725, 0.7090315921179384, 0.5267874922229296, 0.6361100468274054, 0.5667657034781767, 0.7208507944808554, 0.4371869517566465, 0.58117647306466, 0.44709947216778706, 0.42711506695351653, 0.28227445885322416

		TArray<FVector> TargetPointSample;
		for (const auto& i : TargetPoints)
			TargetPointSample.push_back(i->GetLocation());

		World.AddWidget<MeshFittingUI>(Surface, TargetPointSample);
	};
}

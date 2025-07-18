//
// Created by MarvelLi on 2024/11/18.
//

#pragma once
#include "Actors/LightActor.h"
#include "Game/World.h"

class AreaLightActor;
inline void DefaultLightEnv(World& world, double Length = 4.)
{
	auto TopLight = world.SpawnActor<AreaLightActor>("TopLight");
	TopLight->SetTranslation({0, 0, Length});
	TopLight->GetLightComponent()->SetSize(FVector2{Length, Length});
	TopLight->GetLightComponent()->SetIntensity(FVector{8, 8, 8});
	TopLight->GetLightComponent()->SetVisible(false);
}
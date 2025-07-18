//
// Created by MarvelLi on 2024/2/1.
//

#pragma once

#include "Core/CoreMinimal.h"
#include "Game/StaticMeshActor.h"
#include "Mesh/BasicShapesLibrary.h"

MCLASS(TargetPointsActor)
class TargetPointsActor : public StaticMeshActor
{
	REFLECTION_BODY(TargetPointsActor)

public:

	// @see https://zhuanlan.zhihu.com/p/457797561
	static inline TArray<FColor> ColorPalette { RGB(142, 207, 201),  RGB(255, 190, 122), RGB(250, 127, 111), RGB(130, 176, 210),
	RGB(190, 184, 220), RGB(231, 218, 210), RGB(153, 153, 153)};

	TargetPointsActor() = default;

	virtual void Init() override
	{
		GetStaticMeshComponent()->SetMeshData(BasicShapesLibrary::GenerateSphere(0.02));
		static int ColorIndex = 0;
		GetStaticMeshComponent()->SetColor(ColorPalette[ColorIndex++ % ColorPalette.size()]);
	}
};
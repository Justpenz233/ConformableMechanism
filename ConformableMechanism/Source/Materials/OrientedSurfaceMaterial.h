//
// Created by MarvelLi on 2024/8/14.
//

#pragma once
#include "Core/CoreMinimal.h"
#include "Render/material/disney_material.h"

inline ObjectPtr<Material> OrientedSurfaceMaterial()
{
    static ObjectPtr<Material> MaterialInstance = NewObject<Material>();
    MaterialInstance->SetBaseColor(FColor(0.8, 0.1, 0.1));
    MaterialInstance->SetAlpha(0.2);
	return MaterialInstance;
}

class chess_border_texture : public Rendering::disney_material
{
	virtual luisa::compute::Float3 sample_base_color(const Rendering::bxdf_context& context) const override
	{
		auto& uv = context.intersection.uv;
		auto x = luisa::compute::UInt(uv.x * 10.f) % 2;
		auto y = luisa::compute::UInt(uv.y * 10.f) % 2;
		return luisa::compute::ite(x == y, luisa::compute::make_float3(0.1f), luisa::compute::make_float3(0.9f));
	}
};

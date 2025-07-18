//
// Created by MarvelLi on 2023/12/31.
//
#pragma once
#include "CoreMinimal.h"
#include "Mesh/StaticMesh.h"
#include "Math/FTransform.h"
#include "Actors/ParametricMeshActor.h"

struct SurfaceLinkageConfig
{
    Vector2d StartUV;
    Vector2d EndUV;

    // Radius of the linkage
    double Radius = 0.1;
    FTransform Transform = FTransform::Identity();

    // Make sure SX < EX
    void CleanUp()
    {
        Vector2d Length = EndUV - StartUV;
        if(abs(Length.x()) > abs(Length.y())) {
            if(StartUV.x() > EndUV.x())
                std::swap(StartUV, EndUV);
        }
        else {
            if (StartUV.y() > EndUV.y())
                std::swap(StartUV, EndUV);
        }
    }
};

inline TArray<Vector2d> BestLinkageUVPath(ObjectPtr<ParametricMeshActor> Surface, SurfaceLinkageConfig Config)
{
	TArray<Vector2d> UVPath;
	double EU = Config.EndUV.x();
	double EV = Config.EndUV.y();
	double SU = Config.StartUV.x();
	double SV = Config.StartUV.y();

	if(EU - SU > EV - SV) {
		for(double U = SU; U < EU; U += 0.01) {
			double V = SV + (EV - SV) * (U - SU) / (EU - SU);
			UVPath.emplace_back(U, V);
		}
		UVPath.emplace_back(EU, EV);
	}
	else {
		for(double V = SV; V < EV; V += 0.01) {
			double U = SU + (EU - SU) * (V - SV) / (EV - SV);
			UVPath.emplace_back(U, V);
		}
		UVPath.emplace_back(EU, EV);
	}
	return UVPath;
}


// Create a curve cylinder
inline ObjectPtr<StaticMesh> CreateSurfaceLinkage(
    ObjectPtr<ParametricMeshActor> Surface, SurfaceLinkageConfig Config, bool bUseCubiod = false)
{
    Config.CleanUp();
    std::vector<FVector> Vertices;
    double EU = Config.EndUV.x();
    double EV = Config.EndUV.y();
    double SU = Config.StartUV.x();
    double SV = Config.StartUV.y();
    if(EU - SU > EV - SV) {
        for(double U = SU; U < EU; U += 0.01) {
            double V = SV + (EV - SV) * (U - SU) / (EU - SU);
            Vertices.push_back(Surface->Sample(U, V));
        }
        Vertices.push_back(Surface->Sample(EU, EV));
    }
    else {
        for(double V = SV; V < EV; V += 0.01) {
            double U = SU + (EU - SU) * (V - SV) / (EV - SV);
            Vertices.push_back(Surface->Sample(U, V));
        }
        Vertices.push_back(Surface->Sample(EU, EV));
    }
    auto Result = BasicShapesLibrary::GenerateCurveMesh(Vertices, Config.Radius, false, bUseCubiod);
    if(Config.Transform != FTransform::Identity())
        Result->TransformMesh(Config.Transform.GetMatrix());
    return Result;
}
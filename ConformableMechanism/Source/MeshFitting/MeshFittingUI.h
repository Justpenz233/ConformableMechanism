//
// Created by MarvelLi on 2024/2/1.
//
#pragma once
#include "MeshFittingDesigner.h"
#include "TargetPointsActor.h"
#include "Core/CoreMinimal.h"
#include "UIWidget.h"
#include "imgui.h"
#include "ImguiPlus.h"
#include "Animation/IKController.h"
#include "LinkageOptimize/SurfaceVoxel.h"
#include "Optimization/ReferenceSurface.h"

class MeshFittingUI: public UIWidget
{
public:
	std::string ParamAllInOne;
	std::string EffectorParam;
	WeakObjectPtr<IKController> Solver;

	TFunction<void(const SurfaceLinkageConfiguration& Config)> OnConfigShow;

	String Type = "RSERU";
	String Mobility = "Transmobile";
	float InputSpeed = 1.;

	ObjectPtr<ReferenceSurface> Surface;
	TArray<FVector> TargetCurve;
	TArray<FTransform> TargetTransform;

	SurfaceLinkageConfiguration CurrentConfig;

	MeshFittingUI(const ObjectPtr<ReferenceSurface>& InSurface, const TArray<FVector>& TargetPoints)
		:TargetCurve(TargetPoints), UIWidget("MeshFittingUI"), Surface(InSurface){}

	MeshFittingUI(const ObjectPtr<ReferenceSurface>& InSurface, const TArray<FTransform>& InTargetTransform)
		:TargetTransform(InTargetTransform), UIWidget("MeshFittingUI"), Surface(InSurface){}

	virtual void Draw() override
	{
		ImGui::SetNextWindowSize({0,0});
		ImGui::Begin("Mesh fitting design");

		ImGui::InputText("Type", &Type);

		if(ImGui::BeginCombo("Mobility", Mobility.c_str()))
		{
			if(ImGui::Selectable("Intromobile"))
				Mobility = "Intromobile";
			if (ImGui::Selectable("Extramobile"))
				Mobility = "Extramobile";
			if (ImGui::Selectable("Transmobile"))
				Mobility = "Transmobile";
			ImGui::EndCombo();
		}

		ImGui::InputText("Input param", &ParamAllInOne);
		ImGui::InputText("Effector param", &EffectorParam);
		if(ImGui::Button("Show Config")) {
			ShowParam(StringToVector(ParamAllInOne), StringToVector(EffectorParam));
		}
		if(ImGui::Button("Optimize Linkage"))
			OptimizeLinkageOnly();

		if (ImGui::Button("Optimize Effector"))
			OptimizeEffectorOnly();

		ImGui::TextColored({0.3, 0.8 ,0.1, 1.}, "------ Design ------");

		if (ImGui::Button("Joint Optimize"))
			JointOptimize();

		if(!Solver.expired())
		{
			if(ImGui::Button("Cut Original Surface"))
				CutSweptVolume(CurrentConfig);

			if(ImGui::DragFloat("Animate speed", &InputSpeed, 0.1, 1., 10.))
				Solver.lock()->SetSpeed(InputSpeed);
			if(ImGui::Button("Run"))
				Solver.lock()->Run();
			ImGui::SameLine();
			if(ImGui::Button("Pause"))
				Solver.lock()->Pause();
			ImGui::SameLine();
			if(ImGui::Button("Export"))
				ExportWorldSequence();
		}
		ImGui::End();
	}

	void JointOptimize() const;
	void OptimizeLinkageOnly() const;
	void OptimizeEffectorOnly() const;

	void ShowConfig(SurfaceLinkageConfiguration& Config);

	void ShowParam(const TArray<double>& ParamAllInOne, const TArray<double>& EffectorParam);

	void ExportWorldSequence() const;

	static void CutSweptVolume(SurfaceLinkageConfiguration& Config);

	static TArray<double> StringToVector(const std::string& Input);
};
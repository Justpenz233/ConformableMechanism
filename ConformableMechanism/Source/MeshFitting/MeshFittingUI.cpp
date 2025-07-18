//
// Created by MarvelLi on 2024/7/24.
//

#include "MeshFittingUI.h"
#include "ShowConfig.h"
#include "LinkageOptimize/EffectorLinkageProblem.h"
#include "Optimization/MainProblem.h"



void MeshFittingUI::JointOptimize() const
{
	if(!TargetCurve.empty())
		MainProblem::Solve(Type, Surface, TargetCurve, Mobility);
	else
		MainProblem::Solve(Type, Surface, TargetTransform, Mobility);
	// We should not show a config here, because we need to chose a params to show
}
void MeshFittingUI::OptimizeLinkageOnly() const
{
	if (!TargetCurve.empty())
		MainProblem::SolveLinkageOnly(Type, Surface, TargetCurve, Mobility, StringToVector(ParamAllInOne));
	else
		MainProblem::SolveLinkageOnly(Type, Surface, TargetTransform, Mobility, StringToVector(ParamAllInOne));
}

void MeshFittingUI::OptimizeEffectorOnly() const
{
	if(TargetCurve.empty() && TargetTransform.empty())
	{
		ImGui::NotifyError("Error: Target not set, cannot optimize effector");
		return;
	}
	if(ParamAllInOne.empty())
	{
		ImGui::NotifyError("Error: Param not set");
		return;
	}
	ObjectPtr<SurfaceVoxel> Voxel = NewObject<SurfaceVoxel>(Surface.get());
	auto EndPoint = TargetCurve.empty() ? TargetTransform[0].GetLocation() : TargetCurve[0];
	MainProblem Problem(Type, Surface, TargetCurve, Mobility, Voxel);
	auto [Config, score] = Problem.CalculateConfig(StringToVector(ParamAllInOne));
	if(score[0] > 1e4)
	{
		ImGui::NotifyError("Error: config not valid");
		return;
	}
	EffectorLinkageProblem::Solve(Config, Voxel.get(), EndPoint);
}

void MeshFittingUI::ShowConfig(SurfaceLinkageConfiguration& Config)
{
	MeshFittingDesigner::SpawnActor(Config); // Spawn actors
	MeshFittingDesigner::PostProcess(CurrentConfig); // Generate mesh for the linkage and joint
	Solver = ShowSurfaceConfiguration(CurrentConfig);
	if (OnConfigShow)
		OnConfigShow(CurrentConfig);
}

void MeshFittingUI::ShowParam(const TArray<double>& ParamAllInOne, const TArray<double>& EffectorParam)
{
	LOG_INFO("Show param: {0}", ParamAllInOne);
	if(!EffectorParam.empty()) LOG_INFO("Effector param {0}", EffectorParam);
	ObjectPtr<SurfaceVoxel> Voxel = NewObject<SurfaceVoxel>(Surface.get());
	if(!TargetCurve.empty())
	{
		MainProblem Problem(Type, Surface, TargetCurve, Mobility, Voxel);
		Problem.bDebugLog = true;
		auto [Config, score] = Problem.CalculateConfig(ParamAllInOne);
		if(score[0] > 1e4)
		{
			ImGui::NotifyError("Error: config not valid");
			return;
		}
		LOG_INFO("Score: {0}", score);
		CurrentConfig = Config;
	}
	else
	{
		MainProblem Problem(Type, Surface, TargetTransform, Mobility, Voxel);
		Problem.bDebugLog = true;
		auto [Config, score] = Problem.CalculateConfig(ParamAllInOne);
		// ASSERTMSG(score[0] < 1e4, "Wrong params!");
		LOG_INFO("Score: {0}", score);
		CurrentConfig = Config;
	}

	if(!EffectorParam.empty())
	{
		if(TargetCurve.empty() && TargetTransform.empty())
			ImGui::NotifyError("Error: Target not set, cannot show effector");

		auto EndPoint = TargetCurve.empty() ? TargetTransform[0].GetLocation() : TargetCurve[0];
		EffectorLinkageProblem EffectorProblem(&CurrentConfig, EndPoint, nullptr, {});
		auto EffectorGeometry = EffectorProblem.EffectorLinkageGeometry(EffectorParam);
		CurrentConfig.EffectorGeometry = EffectorGeometry;
	}
	ShowConfig(CurrentConfig);
}

void MeshFittingUI::ExportWorldSequence() const
{
	// Firs choose a base directory
	auto BaseDir = SelectFolderDialog("Export to", Path::ProjectDir());
	auto BaseDirPath = Path(BaseDir.result());
	if(!BaseDirPath.Existing())
	{
		ImGui::NotifyError("Export error: Directory not exists");
		return;
	}
	// Make two subdirectory
	auto MeshDir = BaseDirPath / "Model";
	auto AnimationDir = BaseDirPath / "Motion";
	auto StaticDir = BaseDirPath / "Static";
	if(!exists(MeshDir))
		create_directory(MeshDir);
	if (!exists(AnimationDir))
		create_directory(AnimationDir);
	if (!exists(StaticDir))
		create_directory(StaticDir);
	// Export model
	World->ExportSceneToObj(StaticDir, true);
	World->ExportSceneToObj(MeshDir, false);

	constexpr auto TotalFrame = 360;
	constexpr auto TotalTime = 3;
	// Export motion
	auto Sequence = Solver.lock()->GetActorSequence(TotalFrame, TotalTime, [TotalFrame, TotalTime]
		(Joint* Joint, double DeltaTime) {
		constexpr auto DeltaAngle = 2 * M_PI / (TotalFrame - 1);
		Joint->GlobalTransform.AddRotationLocal(MMath::QuaternionFromEulerXYZ( {0, 0, DeltaAngle}));
	});
	// Output surface pmt
	{
		std::ofstream OutFile(AnimationDir / (Surface->GetName() + ".pmt"));
		Matrix4d TransformMatrix = Surface->GetTransformMatrix();
		for (int t = 0;t < TotalFrame; t ++)
		{
			for(int i = 0;i < 4; i ++)
				for (int j = 0;j < 4; j ++)
					OutFile << TransformMatrix(i, j) << " ";
			OutFile << std::endl;
		}
		OutFile.close();
	}

	// Output Joint pmt
	for(const auto& [Joint, TransformSequence] : Sequence.JointTransforms)
	{
		std::ofstream OutFile(AnimationDir / (Joint->GetName() + ".pmt"));
		for (int t = 0; t < TransformSequence.size(); t++)
		{
			for(int i = 0;i < 4; i ++)
				for (int j = 0;j < 4; j ++)
					OutFile << TransformSequence[t].GetMatrix()(i, j) << " ";
			OutFile << std::endl;
		}
		OutFile.close();
	}

}
void MeshFittingUI::CutSweptVolume(SurfaceLinkageConfiguration& Config)
{
	ASSERTMSG(Config.AttachedSurface.get(), "Surface not set");
	auto NewMesh = MeshFittingDesigner::CutSurface(Config); // Cut the surface

	auto PreMaterial = Config.AttachedSurface->GetParametricMeshComponent()->GetMeshData()->GetMaterialAsset();
	Config.AttachedSurface->GetParametricMeshComponent()->SetMeshData(NewMesh);
	Config.AttachedSurface->GetParametricMeshComponent()->GetMeshData()->SetMaterial(PreMaterial);
}

TArray<double> MeshFittingUI::StringToVector(const std::string& Input)
{
	std::stringstream ss(Input);
	std::string token;
	double value;

	TArray<double> Result;
	while (std::getline(ss, token, ',')) {
		std::stringstream tokenStream(token);
		tokenStream >> value;
		Result.push_back(value);
	}
	return Result;
}


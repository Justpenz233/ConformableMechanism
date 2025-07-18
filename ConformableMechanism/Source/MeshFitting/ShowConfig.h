#pragma once
#include "Actors/CurveActor.h"
#include "CoreMinimal.h"
#include "SpacialMechanism/SpacialLinkageDesign.h"
#include "Animation/IKController.h"
#include "SpacialMechanism/SurfaceJoint.h"
#include "Game/World.h"
#include "Optimization/MechanismMobility.h"

inline FVector DrivenLinkageColor      = RGB(53, 59, 85);
inline FVector GroundLinkageColor      = Vector3d(0.8, 0.8, 0.8); // 1: Gray
inline FVector EffectorLinkageColor    = RGB(240, 210, 112); //  Orange
inline FVector EffectorTrajectoryColor = Vector3d(0.4, 0.9, 0.4);
inline FVector Linkage2Color           = RGB(240, 210, 112);//  7: Yellow
inline FVector Linkage3Color           = RGB(234, 120, 34);			//  1: Orange

inline TArray<FVector> ColorCandidate = {
	RGB(240, 210, 112), RGB(234, 120, 34)
};

inline ObjectPtr<IKController> ShowSurfaceConfiguration(const SurfaceLinkageConfiguration& Config)
{
	if (Config.JointTransforms.size() == 0) return nullptr;
    World& World = *GWorld;

	for(auto& i : Config.JointActor)
		World.AddActor(i);

	for(auto Actor:Config.JointActor)
		Actor->GetJointComponent()->Remesh();

    // Spawn solver and register joints
    // Set Solver hyper parameters
	auto& Joints = Config.JointActor;

	for(int i = 0;i < Joints.size() - 1; i ++)
	{
		if(Joints[i]->GetJointComponent()->IsRoot())
			Joints[i]->SetName("DrivenJoint");
		else
			Joints[i]->SetName("joint" + std::to_string(i + 1));
	}
	Joints.back()->SetName("Effector");

	Joints[0]->GetJointComponent()->SetColor(DrivenLinkageColor);
	for(int i = 1;i < Joints.size() - 2; i ++)
		Joints[i]->GetJointComponent()->SetColor(ColorCandidate[(i - 1) % ColorCandidate.size()]);
	Joints[Joints.size() - 2]->GetJointComponent()->SetColor(GroundLinkageColor);
	Joints.back()->GetJointComponent()->SetColor(EffectorLinkageColor);

	auto Solver = NewObject<ClosedChainMechSolver>();
	bool bClosed = true;
	if(Config.Mobility != Transmobile && Config.OriginalTime - Config.MobilityAllowTime > 2)
	{
		Solver->SetLimitDrivenAngle(Config.AnglePerTimeClip() * Config.MobilityAllowTime);
		bClosed = false;
	}
	LOG_INFO("Sequence time : {}", Config.EffectorBestTime);
    auto Controller = World.SpawnActor<IKController>("Controller", Solver);
    Controller->AddTranslationGlobal({0,0,100});
    Controller->AddJoints(Config.JointActor);
    auto EffectorCurve = World.SpawnActor<CurveActor>("EffectorCurve",Config.EffectorTarjectory, bClosed);
    EffectorCurve->GetCurveComponent()->SetRadius(0.01);
    EffectorCurve->GetCurveComponent()->SetColor(EffectorTrajectoryColor);

	// for (int i = 0;i < Config.LinkagePath[1].size() - 1; i ++)
	// {
		// GWorld->DebugDrawPoint(Config.LinkagePath[1][i] + FVector{0, 0, 0.01}, 2, RGB(255, 0, 0));
		// GWorld->DebugDrawLine(Config.LinkagePath[1][i], Config.LinkagePath[1][i + 1], Linkage2Color, 3);
	// }

    return Controller;
}
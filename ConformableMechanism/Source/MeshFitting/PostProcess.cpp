//
// Created by MarvelLi on 2024/4/16.
//
#include "MeshFittingDesigner.h"
#include "Game/StaticMeshActor.h"
#include "SpacialMechanism/SurfaceJoint.h"


void MeshFittingDesigner::SpawnActor(SurfaceLinkageConfiguration& Config)
{
	int EffectorIndex = Config.EffectorIndex;
	int EffectorParentIndex = Config.EffectorParentIndex;
	auto FixType = Config.Type; //Remove 'E'
	FixType.erase(FixType.find('E'), 1);
	Config.JointActor.clear();
	for (int i = 0; i < FixType.size(); i ++)
	{
		Config.JointActor.emplace_back(NewObject<SurfaceJointActor>
		(FixType[i], Config.AttachedSurface, Config.UV[i], Config.InitTransform[i], i==0, i == FixType.size() - 1));
	}
	//Effector
	Config.JointActor.emplace_back(NewObject<SurfaceJointActor>
	('E', Config.AttachedSurface, Config.UV[EffectorIndex], Config.InitTransform[EffectorIndex], false, false));

	Config.JointActor[0]->GetJointComponent()->GetIKJoint()->SetIsRoot(true);
	for (int i = 0; i < FixType.size();i ++)
		Config.JointActor[i]->AddNextJoint(Config.JointActor[(i + 1) % FixType.size()]);

	Config.JointActor[EffectorParentIndex]->AddNextJoint(Config.JointActor[EffectorIndex]);
	for (auto & i : Config.JointActor)
		i->GetJointComponent()->JointMesh = i->GetJointComponent()->GeneratePinMesh();

}

void MeshFittingDesigner::PostProcess(SurfaceLinkageConfiguration& Config)
{
	ASSERT(!Config.LinkagePath.empty());
	for (int i = 0;i < Config.LinkagePath.size();i ++)
		Config.JointActor[i]->GetJointComponent()->LinkagePath = Config.LinkagePath[i];
	Config.JointActor[Config.EffectorParentIndex]->GetJointComponent()->EffectorGeometry = Config.EffectorGeometry;
}
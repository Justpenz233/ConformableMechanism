#pragma once

#include "Animation/Joints.h"
#include "Core/CoreMinimal.h"
#include "Game/Actor.h"
#include "Actors/ParametricMeshActor.h"
#include "Animation/JointComponent.h"
#include "Misc/Config.h"
#include "SurfaceJointComponent.h"

MCLASS(SurfaceJointActor)
class SurfaceJointActor : public Actor
{
	REFLECTION_BODY(SurfaceJointActor)
protected:
    ObjectPtr<SurfaceJointComponent> JointComponent;
	SurfaceJointActor() = default;
public:
    SurfaceJointActor(char Type, const ObjectPtr<ParametricMeshActor>& InSurface, const Vector2d& InUV, const FTransform& InitTransform, bool InIsRoot, bool InIsGround)
        :Actor(InitTransform)
    {
        if (Type == 'R') JointComponent = AddComponent<SurfaceJointR>(InSurface, InUV, InitTransform, InIsRoot, InIsGround);
    	else if (Type == 'D') JointComponent = AddComponent<SurfaceJointD>(InSurface, InUV, InitTransform, InIsRoot, InIsGround);
		else if (Type == 'U') JointComponent = AddComponent<SurfaceJointU>(InSurface, InUV, InitTransform, InIsRoot, InIsGround);
		else if (Type == 'S') JointComponent = AddComponent<SurfaceJointS>(InSurface, InUV, InitTransform, InIsRoot, InIsGround);
        else if (Type == 'E') JointComponent = AddComponent<SurfaceEffectorJoint>(InSurface, InUV, InitTransform, InIsRoot);
		else ASSERTMSG(false, "Type {0} is not a valid Spatial joint", Type);
    }

    ObjectPtr<SurfaceJointComponent> GetJointComponent() { return JointComponent; }
    void AddNextJoint(const ObjectPtr<Joint>& NextJoint)
    {
        JointComponent->GetJoint()->AddNextJoint(NextJoint);
    }
    void AddNextJoint(const ObjectPtr<SurfaceJointComponent>& NextJoint)
    {
        JointComponent->GetJoint()->AddNextJoint(NextJoint->GetJoint());
        JointComponent->AddNextJoint(NextJoint);
    }
    void AddNextJoint(const ObjectPtr<SurfaceJointActor>& NextJoint)
    {
        JointComponent->GetJoint()->AddNextJoint(NextJoint->GetJointComponent()->GetJoint());
        JointComponent->AddNextJoint(NextJoint->GetJointComponent());
    }
	virtual void SetName(const String& InName) override
    {
	    Object::SetName(InName);
    	JointComponent->GetJoint()->SetName(InName);
    }
};
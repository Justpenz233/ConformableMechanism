//
// Created by MarvelLi on 2024/2/28.
//

#pragma once
#include "Core/CoreMinimal.h"
#include "Reflection/reflection/reflection.h"
#include "Animation/Joints.h"
#include "Game/Actor.h"
#include "Mesh/BasicShapesLibrary.h"
#include "Actors/ParametricMeshActor.h"
#include "Animation/JointComponent.h"
#include "Misc/Config.h"

/**
 * @brief This an abstract class, should be derived and complete generate mesh.
 */
MCLASS(SurfaceJointComponent)
class SurfaceJointComponent : public IKJointComponent
{
	REFLECTION_BODY(SurfaceJointComponent)
public:

	double ParameterScale = GConfig.Get<double>("JointMesh", "Scale");
	double LinkageRadius = GConfig.Get<double>("JointMesh", "LinkageRadius") * ParameterScale;
    double DefaultTolerance = GConfig.Get<double>("JointMesh", "Tolerance"); // Eplison for manufraction

    WeakObjectPtr<ParametricMeshActor> Surface;
    [[deprecated]] Vector2d UV;

	bool bPrint = GConfig.Get<bool>("JointMesh", "Print");
	bool bRender = GConfig.Get<bool>("JointMesh", "Render");

	// Use which port
	[[deprecated]] int SocketPortIndex = 0;

	[[deprecated]] int JointPortIndex = 0;

	MPROPERTY()
	double JointPortLength = GConfig.Get<double>("JointMesh", "SocketPortLength")* ParameterScale;

	MPROPERTY()
	double SocketPortLength = JointPortLength;

	virtual void PostEdit(Reflection::FieldAccessor& Field) override;

	SurfaceJointComponent() = default;

    SurfaceJointComponent(
    	const ObjectPtr<ParametricMeshActor>& InSurface, const Vector2d& InUV, EDOF3D InRoationDOF, EDOF3D InTranslationDOF, const FTransform& InInitTransform, bool InIsRoot = false, bool InIsGround = false)
    :IKJointComponent(InRoationDOF, InTranslationDOF, InInitTransform, InIsRoot),
        UV(InUV), Surface(InSurface), bIsGround(InIsGround)
	{ }


	ObjectPtr<StaticMesh> GeneratePinMesh() const { return bIsGround ? GenerateSecondMesh() : GenerateFirstMesh();}
	ObjectPtr<StaticMesh> GenerateSocketMesh() const { return bIsGround ? GenerateFirstMesh() : GenerateSecondMesh();}

	ObjectPtr<StaticMesh> GeneratePinMeshWithTolerance(double Tolerance, double offset = 0.) const { return bIsGround ? GenerateSecondMeshWithTolerance(Tolerance, offset) : GenerateFirstMeshWithTolerance(Tolerance, offset);}
	ObjectPtr<StaticMesh> GenerateSocketMeshWithTolerance(double Tolerance, double offset = 0.) const { return bIsGround ? GenerateFirstMeshWithTolerance(Tolerance, offset) : GenerateSecondMeshWithTolerance(Tolerance, offset);}

	TArray<FVector> GetPinPortInMeshLocation() { return bIsGround ? GetSecondPortInMeshLocation() : GetFirstPortInMeshLocation();}
	TArray<FVector> GetSocketPortInMeshLocation() { return bIsGround ? GetFirstPortInMeshLocation() : GetSecondPortInMeshLocation();}

	FVector GetPinPortInMeshWorldLocation(int Index) { return bIsGround ? GetSecondPortInMeshWorldLocation(Index) : GetFirstPortInMeshWorldLocation(Index);}
	FVector GetSocketPortInMeshWorldLocation(int Index) { return bIsGround ? GetFirstPortInMeshWorldLocation(Index) : GetSecondPortInMeshWorldLocation(Index);}

	FVector GetPinPortWorldLocation(int Index) { return bIsGround ? GetSecondPortWorldLocation(Index) : GetFirstPortWorldLocation(Index);}
	FVector GetSocketPortWorldLocation(int Index) { return bIsGround ? GetFirstPortWorldLocation(Index) : GetSecondPortWorldLocation(Index);}

    bool IsRoot() const;

	// Create Joint mesh connect to next joint's socket
	virtual ObjectPtr<StaticMesh> CreateJointMesh() override;
	virtual ObjectPtr<StaticMesh> CreateJointMeshWithTolerance(double Tolerance, double offset = 0.);

	// Bound box of the joint, not include the linkage
	virtual FBox GetJointBoundingBox() const { return {}; }
	virtual TFunction<bool(FVector)> GetJointBoundingVolume() const { return {}; }

	int PinPortNum() { return GetFirstPortLocation().size(); }
	int SocketPortNum() { return GetSecondPortLocation().size(); }

    // A joint is a JointMesh + Next joint socket mesh + linkage connect each
    virtual ObjectPtr<StaticMesh> GenerateFirstMesh() const;
    virtual ObjectPtr<StaticMesh> GenerateSecondMesh() const;

	virtual ObjectPtr<StaticMesh> GenerateFirstMeshWithTolerance(double Tolerance, double offset) const { return nullptr; }
	virtual ObjectPtr<StaticMesh> GenerateSecondMeshWithTolerance(double Tolerance, double offset) const { return nullptr; }

    // Port: via port the linkage connect to next joint
    // Return the port location start location, in world space.
    virtual TArray<FVector> GetFirstPortInMeshLocation() { return {FVector::Zero()}; }
    virtual TArray<FVector> GetSecondPortInMeshLocation() { return {FVector::Zero()}; }

	virtual TArray<FVector> GetFirstPortLocation();
	virtual TArray<FVector> GetSecondPortLocation();

    FVector GetFirstPortWorldLocation(int Index) { return GetOwner()->GetFTransform() * GetFirstPortLocation()[Index]; }
    FVector GetSecondPortWorldLocation(int Index) { return GetOwner()->GetFTransform() * GetSecondPortLocation()[Index]; }
	FVector GetFirstPortInMeshWorldLocation(int Index) { return GetOwner()->GetFTransform() * GetFirstPortInMeshLocation()[Index]; }
	FVector GetSecondPortInMeshWorldLocation(int Index) { return GetOwner()->GetFTransform() * GetSecondPortInMeshLocation()[Index]; }

    ObjectPtr<StaticMesh> GenerateFirstMeshWithPort(int PortIndex);
    ObjectPtr<StaticMesh> GenerateSecondMeshWithPort(int PortIndex);

    ObjectPtr<StaticMesh> GenerateFirstMeshWithPort(const TArray<int>& PortIndexs);
    ObjectPtr<StaticMesh> GenerateSecondMeshWithPort(TArray<int> PortIndexs);

    /**
     *  Generate linkage mesh from world PortLocation to world Target location
     *  The linkage will be shortest in UV space. (We not consider the world space for simplicity)
     */
	ObjectPtr<StaticMesh> GenerateLinkageTo(int JointPortIndex, int SocketPortIndex, const ObjectPtr<SurfaceJointComponent>& NextJoint);
	ObjectPtr<StaticMesh> GenerateLinkageToChild(int ChildIndex);

	// Genereate a linkage mesh to connect all the next joints, without joint mesh only linkage
	ObjectPtr<StaticMesh> GenerateLinkageToChildsForCollisionTest(bool bAssumePort);

	TArray<FVector> LinkagePath;
	TArray<FVector> EffectorGeometry;
	ObjectPtr<StaticMesh> GenerateLinkageToEffector(const ObjectPtr<SurfaceJointComponent>& NextJoint) const;

    void AddNextJoint(const ObjectPtr<SurfaceJointComponent>& NextJoint) { NextJointComponents.push_back(NextJoint); NextJoint->ParentJointComponents = Cast<SurfaceJointComponent>(GetThis());}

	// Joint mesh  in LOCAL space
	ObjectPtr<StaticMesh> JointMesh;

protected:
	bool bIsGround = false;
	TArray<WeakObjectPtr<SurfaceJointComponent>> NextJointComponents;
	WeakObjectPtr<SurfaceJointComponent> ParentJointComponents;
	// MeshData = MeshUnion(LinkageMesh, JointMesh)
};


class SurfaceJointR : public SurfaceJointComponent
{
public:
	double OuterRadius = GConfig.Get<double>("RevoluteJoint", "OuterRadius") * ParameterScale;
	double InnerRadius = GConfig.Get<double>("RevoluteJoint", "InnerRadius") * ParameterScale;

	double SocketHeight = GConfig.Get<double>("RevoluteJoint", "SocketHeight") * ParameterScale;
	double JointHeight = GConfig.Get<double>("RevoluteJoint", "JointHeight") * ParameterScale;

	SurfaceJointR(const ObjectPtr<ParametricMeshActor>& InSurface, const Vector2d& InUV, const FTransform& InInitTransform, bool InIsRoot = false, bool InIsGround = false)
	 : SurfaceJointComponent(InSurface, InUV, EDOF3D::FreeZ, EDOF3D::FreeNone, InInitTransform, InIsRoot, InIsGround)
	{ }
	static ObjectPtr<StaticMesh> GenerateFirstMesh(double OuterRadius, double InnerRadius, double SocketHeight, double JointHeight, bool bPrint, bool bRender);
	static ObjectPtr<StaticMesh> GenerateSecondMesh(double OuterRadius, double InnerRadius, double Height, bool bPrint, bool bRender);
	virtual ObjectPtr<StaticMesh> GenerateFirstMesh() const override {
		return GenerateFirstMesh(OuterRadius, InnerRadius, SocketHeight, JointHeight, bPrint, bRender);
	}

	virtual ObjectPtr<StaticMesh> GenerateSecondMesh() const override {
		return GenerateSecondMesh(OuterRadius, InnerRadius, JointHeight, bPrint, bRender);
	}

	virtual ObjectPtr<StaticMesh> GenerateFirstMeshWithTolerance(double Tolerance, double offset) const override {
		return GenerateFirstMesh(OuterRadius + offset, InnerRadius - Tolerance, SocketHeight + Tolerance + offset, JointHeight + 2. * Tolerance - offset, bPrint, bRender);
	}

	virtual ObjectPtr<StaticMesh> GenerateSecondMeshWithTolerance(double Tolerance, double offset) const override {
		return GenerateSecondMesh(OuterRadius + offset, InnerRadius + Tolerance, JointHeight - 2. * Tolerance + offset, bPrint, bRender);
	}

	TFunction<bool(FVector)> GetJointBoundingVolume() const override;
	virtual FBox GetJointBoundingBox() const override;

	virtual TArray<FVector> GetFirstPortInMeshLocation() override;
	virtual TArray<FVector> GetSecondPortInMeshLocation() override;
	virtual TArray<FVector> GetFirstPortLocation() override;
};



class SurfaceJointD : public SurfaceJointComponent
{
public:
    double OuterRadius = GConfig.Get<double>("DrivenRJoint", "OuterRadius") * ParameterScale;
    double InnerRadius = GConfig.Get<double>("DrivenRJoint", "InnerRadius") * ParameterScale;

    double SocketHeight = GConfig.Get<double>("DrivenRJoint", "SocketHeight") * ParameterScale;
    double JointHeight = GConfig.Get<double>("DrivenRJoint", "JointHeight") * ParameterScale;

	double CapRadius = GConfig.Get<double>("DrivenRJoint", "CapRadius") * ParameterScale;

	double BarRadius = GConfig.Get<double>("DrivenRJoint", "BarRadius") * ParameterScale;
	double BarLength = GConfig.Get<double>("DrivenRJoint", "BarLength") * ParameterScale;

    SurfaceJointD(const ObjectPtr<ParametricMeshActor>& InSurface, const Vector2d& InUV, const FTransform& InInitTransform, bool InIsRoot = false, bool InIsGround = false)
     : SurfaceJointComponent(InSurface, InUV, EDOF3D::FreeZ, EDOF3D::FreeNone, InInitTransform, true, InIsGround)
    { }
	static ObjectPtr<StaticMesh> GenerateSecondMesh(
		double OuterRadius, double InnerRadius, double SocketHeight, double JointHeight, double CapRadius, double BarRadius, double BarLength,
		bool bPrint, bool bRender);
	static ObjectPtr<StaticMesh> GenerateJointMesh(double InnerRadius, double Height, double BarRaidus, double BarLength, bool bPrint, bool bRender);
	virtual ObjectPtr<StaticMesh> GenerateSecondMesh() const override {
		return GenerateSecondMesh(OuterRadius, InnerRadius, SocketHeight, JointHeight, CapRadius, BarRadius, BarLength, bPrint, bRender);
	}

	virtual ObjectPtr<StaticMesh> GenerateFirstMesh() const override {
		return GenerateJointMesh(InnerRadius, JointHeight, BarRadius, BarLength, bPrint, bRender);
	}

	virtual ObjectPtr<StaticMesh> GenerateSecondMeshWithTolerance(double Tolerance, double offset) const override {
		return GenerateSecondMesh(
			OuterRadius + offset, InnerRadius + Tolerance,
			SocketHeight + Tolerance, JointHeight + 2. * Tolerance, CapRadius + Tolerance,
			BarRadius + Tolerance, BarLength, bPrint, bRender);
	}

	virtual ObjectPtr<StaticMesh> GenerateFirstMeshWithTolerance(double Tolerance, double offset) const override {
		return GenerateJointMesh(InnerRadius - Tolerance, JointHeight - 2. * Tolerance, BarRadius - Tolerance + offset, BarLength - Tolerance + offset, bPrint, bRender);
	}
	virtual FBox GetJointBoundingBox() const override;
	TFunction<bool(FVector)> GetJointBoundingVolume() const override;
	virtual TArray<FVector> GetFirstPortInMeshLocation() override;
	virtual TArray<FVector> GetSecondPortInMeshLocation() override;
};

class SurfaceJointU : public SurfaceJointComponent
{
public:
    double Diameter = GConfig.Get<double>("UniversalJoint", "Diameter") * ParameterScale;
    double SocketThickness = GConfig.Get<double>("UniversalJoint", "SocketThickness") * ParameterScale;
    double JointRadius = GConfig.Get<double>("UniversalJoint", "JointRadius") * ParameterScale;
    double SupporterRadius = GConfig.Get<double>("UniversalJoint", "SupporterRadius") * ParameterScale;

    SurfaceJointU(const ObjectPtr<ParametricMeshActor>& InSurface, const Vector2d& InUV, const FTransform& InInitTransform, bool InIsRoot = false, bool InIsGround = false)
     : SurfaceJointComponent(InSurface, InUV, EDOF3D::FreeXZ, EDOF3D::FreeNone, InInitTransform, InIsRoot, InIsGround)
    { }

	virtual FBox GetJointBoundingBox() const override;
	TFunction<bool(FVector)> GetJointBoundingVolume() const override;

    // Diameter in Y,Z  Thickness in Z. Cylinder raidus setup for supporter
    static ObjectPtr<StaticMesh> GenerateSecondMesh(double Diameter, double JointRadius, double Thickness, double CylinderRadius, bool bPrint, bool bRender);
    static ObjectPtr<StaticMesh> GenerateJointMesh(double Radius, double SupporterRadius, bool bPrint, bool bRender);

    virtual ObjectPtr<StaticMesh> GenerateSecondMesh() const override {
        return GenerateSecondMesh(Diameter, JointRadius, SocketThickness, SupporterRadius, bPrint, bRender);
    }
    virtual ObjectPtr<StaticMesh> GenerateFirstMesh() const override {
        return GenerateJointMesh(JointRadius, SupporterRadius, bPrint, bRender);
    }
	virtual ObjectPtr<StaticMesh> GenerateSecondMeshWithTolerance(double Tolerance, double offset) const override {
    	return GenerateSecondMesh(Diameter + offset, JointRadius + Tolerance, SocketThickness, SupporterRadius - Tolerance, bPrint, bRender);
	}
	virtual ObjectPtr<StaticMesh> GenerateFirstMeshWithTolerance(double Tolerance, double offset) const override {
        return GenerateJointMesh(JointRadius - Tolerance, SupporterRadius + Tolerance, bPrint, bRender);
	}
    virtual TArray<FVector> GetFirstPortInMeshLocation() override;
    virtual TArray<FVector> GetSecondPortInMeshLocation() override;
};

class SurfaceJointS : public SurfaceJointComponent
{
public:
    double OuterRadius = GConfig.Get<double>("SphericalJoint", "OuterRadius") * ParameterScale;
    double InnerRadius = GConfig.Get<double>("SphericalJoint", "InnerRadius") * ParameterScale;
    SurfaceJointS(const ObjectPtr<ParametricMeshActor>& InSurface, const Vector2d& InUV, const FTransform& InInitTransform, bool InIsRoot = false, bool InIsGround = false)
     : SurfaceJointComponent(InSurface, InUV, EDOF3D::FreeXYZ, EDOF3D::FreeNone, InInitTransform, InIsRoot, InIsGround)
    { }

	virtual FBox GetJointBoundingBox() const override;
	TFunction<bool(FVector)> GetJointBoundingVolume() const override;

    // Will cut 0.5 Radius, result in [-60,60] deg
    static ObjectPtr<StaticMesh> GenerateSecondMesh(double OuterRadius, double InnerRadius, bool bPrint, bool bRender);
    static ObjectPtr<StaticMesh> GenerateJointMesh(double Radius, bool bPrint, bool bRender);

    virtual ObjectPtr<StaticMesh> GenerateSecondMesh() const override {
        return GenerateSecondMesh(OuterRadius, InnerRadius, bPrint, bRender);
    }
    virtual ObjectPtr<StaticMesh> GenerateFirstMesh() const override {
        return GenerateJointMesh(InnerRadius, bPrint, bRender);
    }
	virtual ObjectPtr<StaticMesh> GenerateSecondMeshWithTolerance(double Tolerance, double offset) const override {
        return GenerateSecondMesh(OuterRadius + offset, InnerRadius + Tolerance + offset, bPrint, bRender);
    }
	virtual ObjectPtr<StaticMesh> GenerateFirstMeshWithTolerance(double Tolerance, double offset) const override {
        return GenerateJointMesh(InnerRadius - Tolerance + offset, bPrint, bRender);
    }
    virtual TArray<FVector> GetFirstPortInMeshLocation() override;
    virtual TArray<FVector> GetSecondPortInMeshLocation() override;
};

class SurfaceEffectorJoint : public SurfaceJointComponent
{
public:
    double Radius = GConfig.Get<double>("EffectorJoint", "Radius") * ParameterScale;
    SurfaceEffectorJoint(const ObjectPtr<ParametricMeshActor>& InSurface, const Vector2d& InUV, const FTransform& InInitTransform, bool InIsRoot = false)
    : SurfaceJointComponent(InSurface, InUV, FreeNone, FreeNone, InInitTransform, false, false){ }

    virtual ObjectPtr<StaticMesh> GenerateSecondMesh() const override {
        return BasicShapesLibrary::GenerateSphere(Radius, bPrint | bRender ? 128: 32);
    }
    virtual ObjectPtr<StaticMesh> GenerateFirstMesh() const override {
        return BasicShapesLibrary::GenerateSphere(Radius, bPrint | bRender? 128: 32);
    }
	virtual ObjectPtr<StaticMesh> GenerateSecondMeshWithTolerance(double Tolerance, double offset) const override {
        return BasicShapesLibrary::GenerateSphere(Radius, bPrint | bRender? 128: 32);
    }
	virtual ObjectPtr<StaticMesh> GenerateFirstMeshWithTolerance(double Tolerance, double offset) const override {
        return BasicShapesLibrary::GenerateSphere(Radius, bPrint | bRender? 128: 32);
    }

    virtual TArray<FVector> GetFirstPortInMeshLocation() override {return {FVector::Zero()};};
    virtual TArray<FVector> GetSecondPortInMeshLocation() override  {return {FVector::Zero()};};

};

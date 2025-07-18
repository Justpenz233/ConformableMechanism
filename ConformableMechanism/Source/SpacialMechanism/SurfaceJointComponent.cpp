//
// Created by MarvelLi on 2024/2/28.
//

#include "SurfaceJointComponent.h"
#include "Math/LinearAlgebra.h"
#include "Mesh/MeshBoolean.h"

void SurfaceJointComponent::PostEdit(Reflection::FieldAccessor& Field)
{
	IKJointComponent::PostEdit(Field);
	if( Field.GetName() == NAME(SocketPortIndex)  || Field.GetName() == NAME(SocketPortLength) )
	{
		ParentJointComponents.lock()->MarkAsDirty(DIRTY_REMESH);
	}
	if (Field.GetName() == NAME(JointPortIndex) || Field.GetName() == NAME(JointPortLength) || Field.GetName() == NAME(TransitLinkageLength))
	{
		MarkAsDirty(DIRTY_REMESH);
	}
}

bool SurfaceJointComponent::IsRoot() const
{
    return JointPtr->IsRootJoint();
}

ObjectPtr<StaticMesh> SurfaceJointComponent::GenerateFirstMesh() const
{
	return nullptr;
}

ObjectPtr<StaticMesh> SurfaceJointComponent::GenerateSecondMesh() const
{
	return nullptr;
}

TArray<FVector> SurfaceJointComponent::GetFirstPortLocation()
{
    TArray<FVector> Result = GetFirstPortInMeshLocation();
    for(FVector& InMesh : Result)
    {
        InMesh = InMesh + InMesh.normalized() * JointPortLength;
    }
    return Result;
}

TArray<FVector> SurfaceJointComponent::GetSecondPortLocation()
{
    TArray<FVector> Result = GetSecondPortInMeshLocation();
    for(FVector& InMesh : Result)
    {
        InMesh = InMesh + InMesh.normalized() * SocketPortLength;
    }
    return Result;
}

ObjectPtr<StaticMesh> GenerateOneHatMesh(const FVector& A, const FVector& B, double Radius, bool bTop, int RingSample = 32)
{
	double Length = (B - A).norm();
	auto   Mesh = BasicShapesLibrary::GenerateOneHatCapsule( Radius, Length, bTop, 32, RingSample);

	// Compute the translation matrix
	Vector3d   center = 0.5f * (A + B);
	Vector3d   TArray = (B - A).normalized();
	FQuat	   Rotation = FQuat::FromTwoVectors(Vector3d{ 0., 0., 1. }, TArray);
	FTransform Transform = FTransform::Identity();
	Transform.SetRotation(Rotation);
	Transform.SetTranslation(center);

	// Transform the cylinder to its target position and orientation
	Mesh->TransformMesh(Transform.GetMatrix());
	return Mesh;
}

ObjectPtr<StaticMesh> GenerateCapsuleMesh(const FVector& A, const FVector& B, double Radius, bool HighSample)
{
	double Length = (B - A).norm();
	auto   Mesh = BasicShapesLibrary::GenerateCapsule(Radius, Length, HighSample ? 128 : 32, HighSample ? 256 : 32);

	// Compute the translation matrix
	Vector3d   center = 0.5f * (A + B);
	Vector3d   TArray = (B - A).normalized();
	FQuat	   Rotation = FQuat::FromTwoVectors(Vector3d{ 0., 0., 1. }, TArray);
	FTransform Transform = FTransform::Identity();
	Transform.SetRotation(Rotation);
	Transform.SetTranslation(center);

	// Transform the cylinder to its target position and orientation
	Mesh->TransformMesh(Transform.GetMatrix());
	return Mesh;
}

ObjectPtr<StaticMesh> SurfaceJointComponent::GenerateFirstMeshWithPort(int PortIndex)
{
    ASSERTMSG(PortIndex < GetFirstPortInMeshLocation().size() && PortIndex >= 0, "PortIndex out of range");
    auto JointMesh = GenerateFirstMesh();
    FVector PortLocation = GetFirstPortInMeshLocation()[PortIndex];
    FVector PortEnd = GetFirstPortLocation()[PortIndex];
    auto PortMesh = GenerateOneHatMesh(PortLocation, PortEnd, LinkageRadius, true, bPrint | bRender ? 256 : 32);
    return MeshBoolean::MeshUnion(JointMesh, PortMesh);
}

ObjectPtr<StaticMesh> SurfaceJointComponent::GenerateFirstMeshWithPort(const TArray<int>& PortIndexs)
{
    auto PortLocations = GetFirstPortInMeshLocation();
    auto JointMesh = GenerateFirstMesh();

    for (int PortIndex : PortIndexs)
    {
        ASSERTMSG(PortIndex < PortLocations.size() && PortIndex >= 0, "PortIndex out of range");
        FVector PortLocation = GetFirstPortInMeshLocation()[PortIndex];
        FVector PortEnd = GetFirstPortLocation()[PortIndex];

        auto PortMesh = GenerateOneHatMesh(PortLocation, PortEnd, LinkageRadius, true, bPrint | bRender ? 256 : 32);
        JointMesh = MeshBoolean::MeshUnion(JointMesh, PortMesh);
    }
    return JointMesh;
}

ObjectPtr<StaticMesh> SurfaceJointComponent::GenerateSecondMeshWithPort(int PortIndex)
{
    ASSERTMSG(PortIndex < GetSecondPortInMeshLocation().size() && PortIndex >= 0, "PortIndex out of range");
    FVector PortLocation = GetSecondPortInMeshLocation()[PortIndex];
    FVector PortEnd = GetSecondPortLocation()[PortIndex];
    auto SocketMesh = GenerateSecondMesh();
	auto PortMesh = GenerateOneHatMesh(PortLocation, PortEnd, LinkageRadius, true, bPrint | bRender ? 256 : 32);
    return MeshBoolean::MeshUnion(SocketMesh, PortMesh);
}

ObjectPtr<StaticMesh> SurfaceJointComponent::GenerateSecondMeshWithPort(TArray<int> PortIndexs)
{
    auto PortLocations = GetSecondPortInMeshLocation();
	auto SocketMesh = GenerateSecondMesh();

    for (int PortIndex : PortIndexs)
    {
        ASSERTMSG(PortIndex < PortLocations.size() && PortIndex >= 0, "PortIndex out of range");
        FVector PortLocation = PortLocations[PortIndex];
        FVector PortEnd = GetSecondPortLocation()[PortIndex];

    	auto PortMesh = GenerateOneHatMesh(PortLocation, PortEnd, LinkageRadius, true, bPrint | bRender ? 256 : 32);
        SocketMesh = MeshBoolean::MeshUnion(SocketMesh, PortMesh);
    }
    return SocketMesh;
}

ObjectPtr<StaticMesh> SurfaceJointComponent::GenerateLinkageTo(int JointPortIndex, int SocketPortIndex, const ObjectPtr<SurfaceJointComponent>& NextJoint)
{
	FVector ThisLocation = GetOwner()->GetTranslation();
	FVector NextLocation = NextJoint->GetOwner()->GetTranslation();
	if(JointPortIndex == -1 && SocketPortIndex == -1) // For determine best index
	{
		if(LinkagePath.empty() && NextJointComponents.size() > 1)
			LinkagePath = Surface.lock()->GeodicShortestPath(ThisLocation, NextLocation);
		auto LinkageMesh = BasicShapesLibrary::GenerateCurveMesh(LinkagePath, LinkageRadius, false, false, bRender | bPrint ? 256 : 32);
		LinkageMesh->TransformMesh(GetOwner()->GetFTransform().Inverse().GetMatrix());
		return LinkageMesh;
	}
	// Make mesh in Global Space and move to joint local space
	// Linage start UV and end UV, used for genreate linkage from surface
	FVector PortLocation = GetPinPortWorldLocation(JointPortIndex);
	FVector TargetLocation = NextJoint->GetSocketPortWorldLocation(SocketPortIndex);

	if(LinkagePath.empty())
		LinkagePath = Surface.lock()->GeodicShortestPath(PortLocation, TargetLocation);

	auto LinkageMesh = BasicShapesLibrary::GenerateCurveMesh(LinkagePath, LinkageRadius, false, false, bRender | bPrint ? 256 : 32, LinkagePath.size());

	static bool bUseSurfaceLinkage = GConfig.Get<bool>("JointMesh", "SurfaceLinkage");
	if(bUseSurfaceLinkage)
	{
		auto SurfaceWorldMesh = NewObject<StaticMesh>(*Surface.lock()->GetParametricMeshComponent()->GetMeshData());
		SurfaceWorldMesh->TransformMesh(Surface.lock()->GetFTransform().GetMatrix());
		LinkageMesh = MeshBoolean::MeshIntersect(LinkageMesh, SurfaceWorldMesh);
	}

	// Move to joint local space
	LinkageMesh->TransformMesh(GetOwner()->GetFTransform().Inverse().GetMatrix());
	return LinkageMesh;
}

ObjectPtr<StaticMesh> SurfaceJointComponent::GenerateLinkageToEffector(const ObjectPtr<SurfaceJointComponent>& NextJoint) const
{
	ASSERTMSG(!LinkagePath.empty(), "Effector joint should not be first child joint");
	static double Scale = GConfig.Get<double>("JointMesh", "Scale");
	static double LinkageBaseRadius = GConfig.Get<double>("JointMesh", "LinkageRadius") * Scale;
	static double LinkageRadius = GConfig.Get<double>("EffectorJoint", "LinkageRadius") * Scale;
	static double EffectorJointRadius = GConfig.Get<double>("EffectorJoint", "Radius") * Scale;
	static double Tolerance = GConfig.Get<double>("EffectorJoint", "Tolerance");
	static double SlotHeight = GConfig.Get<double>("EffectorJoint", "SlotHeight") * Scale;
	static double SlotSize = LinkageRadius * 1.3;

	FVector NextLocation = NextJoint->GetOwner()->GetTranslation();
	FVector BestPos = LinkagePath[LinkagePath.size() * 0.5];
	auto SurfacePtr = Surface.lock();
	auto EffectorJoint = BasicShapesLibrary::GenerateSphere(EffectorJointRadius, bPrint | bRender ? 128 : 32);
	EffectorJoint->Translate(NextLocation);
	if(bPrint)
	{
		auto GenerateCubeAlign = [](const FVector& ZDir, const FVector& Size, const FVector& CenterPos) {
			auto Cube = BasicShapesLibrary::GenerateCuboid(Size);
			auto Rotation = FQuat::FromTwoVectors(FVector{0, 0, 1}, ZDir);
			Cube->Rotate(Rotation);
			Cube->Translate(CenterPos);
			return Cube;
		};

		auto UV = SurfacePtr->Projection(BestPos);
		auto Start = SurfacePtr->Sample(UV);
		auto Normal = Surface.lock()->SampleNormal(UV.x(), UV.y());
		// Base


		ObjectPtr<StaticMesh> LinkageMesh;
		if (EffectorGeometry.empty())
		{
			FVector BaseStart = Start;
			FVector BaseEnd = Start + Normal * 0.1;
			auto Base0 = BasicShapesLibrary::GenerateCylinder(BaseStart, BaseEnd, LinkageBaseRadius, bPrint | bRender ? 256 : 32);
			auto Cut = GenerateCubeAlign(Normal, {SlotSize + Tolerance, SlotSize + Tolerance, SlotHeight + Tolerance}, BaseEnd);
			Base0 = MeshBoolean::MeshMinus(Base0, Cut);

			FVector Turn = Start + Normal * 0.2;
			auto Turn0 = BasicShapesLibrary::GenerateCylinder(BaseEnd, Turn, LinkageRadius, bPrint | bRender ? 256 : 32);
			auto Slot = GenerateCubeAlign(Normal, {SlotSize - Tolerance, SlotSize - Tolerance, SlotHeight - Tolerance}, BaseEnd);
			Turn0 = MeshBoolean::MeshConnect(Turn0, Slot);
			LinkageMesh = BasicShapesLibrary::GenerateCylinder(Turn, NextLocation, LinkageRadius, bPrint | bRender ? 256 : 32);
			auto Sphere = BasicShapesLibrary::GenerateSphere(LinkageRadius , bPrint | bRender ? 128 : 32); Sphere->Translate(Turn);
			LinkageMesh = MeshBoolean::MeshUnion(LinkageMesh, Sphere);
			LinkageMesh = MeshBoolean::MeshUnion(LinkageMesh, Turn0);
			LinkageMesh = MeshBoolean::MeshConnect(LinkageMesh, Base0);
			LinkageMesh = MeshBoolean::MeshUnion(LinkageMesh, EffectorJoint);
		}
		else
		{
			int TurnIndex = 0;
			{
				double Distance = 0;
				for(int i = 1;i < EffectorGeometry.size();i ++)
				{
					Distance += (EffectorGeometry[i] - EffectorGeometry[i - 1]).norm();
					if(Distance > 0.2)
					{
						TurnIndex = i;
						break;
					}
				}
			}
			TArray<FVector> BaseCurve(EffectorGeometry.begin(), EffectorGeometry.begin() + TurnIndex + 1);
			TArray<FVector> TurnCurve(EffectorGeometry.begin() + TurnIndex, EffectorGeometry.end());
			auto Base = BasicShapesLibrary::GenerateCurveMesh(BaseCurve, LinkageBaseRadius, false, false, bRender | bPrint ? 256 : 32, BaseCurve.size());
			auto Turn = BasicShapesLibrary::GenerateCurveMesh(TurnCurve, LinkageRadius, false, false, bRender | bPrint ? 256 : 32, TurnCurve.size());
			Turn = MeshBoolean::MeshUnion(Turn, EffectorJoint);
			LinkageMesh = MeshBoolean::MeshConnect(Base, Turn);
		}

		LinkageMesh->TransformMesh(GetOwner()->GetFTransform().Inverse().GetMatrix());
		return LinkageMesh;
	}
	else
	{
		if(EffectorGeometry.empty())
		{
			auto LinkageMesh = GenerateCapsuleMesh(BestPos, NextLocation, LinkageRadius, bPrint | bRender);
			LinkageMesh = MeshBoolean::MeshUnion(LinkageMesh, EffectorJoint);
			LinkageMesh->TransformMesh(GetOwner()->GetFTransform().Inverse().GetMatrix());
			return LinkageMesh;
		}
		else
		{
			auto LinkageMesh = BasicShapesLibrary::GenerateCurveMesh(EffectorGeometry, LinkageRadius, false, false, bRender | bPrint ? 256 : 32, EffectorGeometry.size());
			LinkageMesh = MeshBoolean::MeshUnion(LinkageMesh, EffectorJoint);
			LinkageMesh->TransformMesh(GetOwner()->GetFTransform().Inverse().GetMatrix());
			return LinkageMesh;
		}
	}
	return nullptr;
}

ObjectPtr<StaticMesh> SurfaceJointComponent::GenerateLinkageToChild(int ChildIndex)
{
	auto NextSpacialJoint = NextJointComponents[ChildIndex].lock();
	if(NextSpacialJoint->IsA<SurfaceEffectorJoint>())
	{
		return GenerateLinkageToEffector(NextSpacialJoint);
	}
	else
		return GenerateLinkageTo(JointPortIndex, NextSpacialJoint->SocketPortIndex, NextSpacialJoint);
}

ObjectPtr<StaticMesh> SurfaceJointComponent::GenerateLinkageToChildsForCollisionTest(bool bAssumePort)
{
	auto LinkageMesh = NewObject<StaticMesh>();
	for(const auto& child : NextJointComponents)
	{
		if(auto NextSpacialJoint = child.lock())
		{
			if(NextSpacialJoint->IsA<SurfaceEffectorJoint>())
			{
				LinkageMesh = MeshBoolean::MeshConnect(LinkageMesh, GenerateLinkageToEffector(NextSpacialJoint));
			}
			else
			{
				if(!bAssumePort)
					LinkageMesh = MeshBoolean::MeshConnect(LinkageMesh, GenerateLinkageTo(-1, -1, NextSpacialJoint));
				else
					LinkageMesh = MeshBoolean::MeshConnect(LinkageMesh, GenerateLinkageTo(JointPortIndex, NextSpacialJoint->SocketPortIndex, NextSpacialJoint));
			}
		}
	}
	return LinkageMesh;
}

ObjectPtr<StaticMesh> SurfaceJointComponent::CreateJointMesh()
{
	static bool EffectorMesh = GConfig.Get<bool>("JointMesh", "EffectorMesh");
	if(DefaultTolerance != 0.)
		return CreateJointMeshWithTolerance(DefaultTolerance);

	if (!NextJointComponents.empty())
	{
		auto ThisMesh = GeneratePinMesh();
		for (int ChildIndex = 0; ChildIndex < NextJointComponents.size(); ChildIndex++)
		{
			if (auto NextSpacialJoint = NextJointComponents[ChildIndex].lock())
			{
				if(NextSpacialJoint->IsA<SurfaceEffectorJoint>() && !EffectorMesh) continue;

				// ADD Next Socket
				auto	 NextSocketMesh = NextSpacialJoint->GenerateSocketMesh();
				Matrix4d ToLocal = (GetOwner()->GetTransform().inverse() * NextSpacialJoint->GetOwner()->GetTransform()).matrix();
				NextSocketMesh->TransformMesh(ToLocal);
				if (NextSpacialJoint->IsRoot())
				{
					ThisMesh = MeshBoolean::MeshConnect(ThisMesh, NextSocketMesh);
				}
				else if (NextSpacialJoint->IsA<SurfaceEffectorJoint>())
				{
					auto LinkageToNext = GenerateLinkageToChild(ChildIndex);
					ThisMesh = MeshBoolean::MeshUnion(ThisMesh, LinkageToNext);
				}
				else
				{
					auto LinkageToNext = GenerateLinkageToChild(ChildIndex);
					if(bPrint) LinkageToNext = MeshBoolean::MeshMinus(LinkageToNext, GenerateSocketMesh());
					ThisMesh = MeshBoolean::MeshUnion(ThisMesh, LinkageToNext);
					ThisMesh = MeshBoolean::MeshUnion(ThisMesh, NextSocketMesh);
				}
			}
		}
		return ThisMesh->Clean()->GetThis<StaticMesh>();
	}
	else if (IsRoot()) // Single Joint, Show joint and socket, for debug
	{
		auto SocketMesh = GenerateSocketMesh();
		auto JointMesh = GeneratePinMesh();
		return MeshBoolean::MeshConnect(SocketMesh, JointMesh);
	}
	return nullptr;
}

ObjectPtr<StaticMesh> SurfaceJointComponent::CreateJointMeshWithTolerance(double Tolerance, double offset)
{
	static bool EffectorMesh = GConfig.Get<bool>("JointMesh", "EffectorMesh");
	if (!NextJointComponents.empty())
	{
		auto ThisMesh = GeneratePinMeshWithTolerance(Tolerance, offset);
		for (int ChildIndex = 0; ChildIndex < NextJointComponents.size(); ChildIndex++)
		{
			if (auto NextSpacialJoint = NextJointComponents[ChildIndex].lock())
			{
				if(NextSpacialJoint->IsA<SurfaceEffectorJoint>() && !EffectorMesh) continue;

				// ADD Next Socket
				auto	 NextSocketMesh = NextSpacialJoint->GenerateSocketMeshWithTolerance(Tolerance, offset);
				Matrix4d ToLocal = (GetOwner()->GetTransform().inverse() * NextSpacialJoint->GetOwner()->GetTransform()).matrix();
				NextSocketMesh->TransformMesh(ToLocal);
				if (NextSpacialJoint->IsRoot())
				{
					ThisMesh = MeshBoolean::MeshConnect(ThisMesh, NextSocketMesh);
				}
				else if (NextSpacialJoint->IsA<SurfaceEffectorJoint>())
				{
					auto LinkageToNext = GenerateLinkageToChild(ChildIndex);
					ThisMesh = MeshBoolean::MeshConnect(ThisMesh, LinkageToNext);
				}
				else
				{
					auto LinkageToNext = GenerateLinkageToChild(ChildIndex);
					if(bPrint) LinkageToNext = MeshBoolean::MeshMinus(LinkageToNext, GenerateSocketMeshWithTolerance(-Tolerance - 0.0001, offset));
					ThisMesh = MeshBoolean::MeshUnion(ThisMesh, LinkageToNext);
					ThisMesh = MeshBoolean::MeshUnion(ThisMesh, NextSocketMesh);
				}
			}
		}
		return ThisMesh->Clean()->GetThis<StaticMesh>();
	}
	else if(IsRoot())
	{
		auto SocketMesh = GenerateSocketMesh();
		auto JointMesh = GeneratePinMesh();
		return MeshBoolean::MeshConnect(SocketMesh, JointMesh);
	}
	return nullptr;
}

ObjectPtr<StaticMesh> SurfaceJointR::GenerateFirstMesh(double OuterRadius, double InnerRadius, double SocketHeight, double JointHeight, bool bPrint, bool bRender)
{
	auto ResultMesh = BasicShapesLibrary::GenerateCylinder(SocketHeight, OuterRadius, bRender | bPrint ? 256 : 32);
	auto JointMesh = BasicShapesLibrary::GenerateHollowCylinder(OuterRadius * 2, InnerRadius, JointHeight, bRender | bPrint ? 256 : 32);
	auto Result = MeshBoolean::MeshMinus(ResultMesh, JointMesh);
	if(bPrint)
	{
		auto FanRadius0 = (OuterRadius - InnerRadius) / 3 + InnerRadius;
		auto Fan0 = BasicShapesLibrary::GenerateCylinder( JointHeight * 4, FanRadius0);
		auto FanRadius1 = (OuterRadius - InnerRadius) * 2 / 3 + InnerRadius;
		auto Fan1 = BasicShapesLibrary::GenerateFan(FanRadius1, JointHeight * 3, -0.1-M_PI_4/2, M_PI_4/2+0.1);
		auto Hole = MeshBoolean::MeshMinus(Fan1, Fan0);
		Result = MeshBoolean::MeshMinus(Result, Hole);
		for (int i = 0;i < 3;i ++)
		{
			Hole->RotateEuler({0, 0, M_PI_2});
			Result = MeshBoolean::MeshMinus(Result, Hole);
		}
	}
	return Result;
}

ObjectPtr<StaticMesh> SurfaceJointR::GenerateSecondMesh(double OuterRadius, double InnerRadius, double Height, bool bPrint, bool bRender)
{
	auto Base = BasicShapesLibrary::GenerateHollowCylinder(OuterRadius, InnerRadius, Height, bRender | bPrint ? 256 : 32);
	if(bPrint)
	{
		auto Cut1 = BasicShapesLibrary::GenerateCuboid({OuterRadius * 4, InnerRadius * 0.5, Height * 0.6});
		Cut1->RotateEuler(FVector{0, 0, M_PI_4});
		auto Cut2 = BasicShapesLibrary::GenerateCuboid({ InnerRadius * 0.5, OuterRadius * 4, Height * 0.6});
		Cut2->RotateEuler(FVector{0, 0,M_PI_4});
		Base = MeshBoolean::MeshMinus(Base, Cut1);
		Base = MeshBoolean::MeshMinus(Base, Cut2);
	}
	return Base;
}

TFunction<bool(FVector)> SurfaceJointR::GetJointBoundingVolume() const
{
	return [&](FVector Pos) {
		if (Pos.z() > 0.5 * SocketHeight || Pos.z() < 0.5 * -SocketHeight)
			return false;
		return Pos.x() * Pos.x() + Pos.y() * Pos.y() <= OuterRadius * OuterRadius;
	};
}
FBox SurfaceJointR::GetJointBoundingBox() const
{
	return Math::FBox(FVector{OuterRadius * 2, OuterRadius * 2, SocketHeight});
}

TArray<FVector> SurfaceJointR::GetSecondPortInMeshLocation()
{
	double			RadiusSample = (InnerRadius + OuterRadius) * 0.5;
	constexpr int	SampleU = 64;
	TArray<FVector> Result;
	for (int UIndex = 0; UIndex < SampleU; UIndex++)
	{
		double u = ((double)UIndex) / (double)(SampleU);
		Result.emplace_back(sin(u * 2. * M_PI) * RadiusSample, cos(u * 2. * M_PI) * RadiusSample, 0.);
	}
	return Result;
}

TArray<FVector> SurfaceJointR::GetFirstPortInMeshLocation()
{
	double Z = SocketHeight * 0.5 * 0.95;
    constexpr int SampleU = 16;
	constexpr int SampleV = 16;
	TArray<FVector> Result;
	Result.emplace_back(0, 0, Z);
	Result.emplace_back(0, 0, -Z);

	for(int UIndex = 0; UIndex < SampleU; UIndex ++)
	{
		double u = ((double)UIndex) / (double)(SampleU);
		for (int VIndex = 1; VIndex < SampleV; VIndex++)
		{
			double v = ((double)VIndex) / (double)(SampleV - 1);
			Result.emplace_back(sin(u * 2. * M_PI) * OuterRadius * v * 0.7, cos(u * 2. * M_PI) * OuterRadius * v * 0.7, Z);
			Result.emplace_back(sin(u * 2. * M_PI) * OuterRadius * v * 0.7, cos(u * 2. * M_PI) * OuterRadius * v * 0.7, -Z);
		}
	}
	return Result;
}

TArray<FVector> SurfaceJointR::GetFirstPortLocation()
{
	auto Result = GetFirstPortInMeshLocation();
	for (auto& i : Result)
		i.z() += i.z() > 0 ? JointPortLength : -JointPortLength;
	return Result;
}



ObjectPtr<StaticMesh> SurfaceJointD::GenerateSecondMesh(
	double OuterRadius, double InnerRadius,
	double SocketHeight, double JointHeight, double CapRadius,
	double BarRadius, double BarLength,
	bool bPrint, bool bRender)
{
	auto ResultMesh = BasicShapesLibrary::GenerateCylinder(SocketHeight, OuterRadius, bRender | bPrint ? 256 : 32);
	auto JointMesh = BasicShapesLibrary::GenerateCylinder(JointHeight, InnerRadius, bRender | bPrint ? 256 : 32);
	auto CapCut = BasicShapesLibrary::GenerateCylinder(JointHeight * 4, CapRadius, bRender | bPrint ? 256 : 32);
	CapCut->Translate(FVector{0, 0, JointHeight * 0.5});
	auto Result = MeshBoolean::MeshMinus(ResultMesh, JointMesh);
	Result = MeshBoolean::MeshMinus(Result, CapCut);
	auto BarCut = BasicShapesLibrary::GenerateCylinder(BarRadius * 2., BarLength, bRender | bPrint ? 256 : 32);
	return MeshBoolean::MeshMinus(Result, BarCut);
}

ObjectPtr<StaticMesh> SurfaceJointD::GenerateJointMesh(double InnerRadius, double Height, double BarRaidus, double BarLength, bool bPrint, bool bRender)
{
	auto Base = BasicShapesLibrary::GenerateCylinder(Height, InnerRadius, bRender | bPrint ? 256 : 32);
	if (bPrint)
	{
		auto Bar = BasicShapesLibrary::GenerateCylinder(BarLength, BarRaidus, bRender | bPrint ? 256 : 32);
		Bar->RotateEuler(FVector{ 0, M_PI_2, 0 });
		auto Bar2 = BasicShapesLibrary::GenerateCylinder(BarLength, BarRaidus, bRender | bPrint ? 256 : 32);
		Bar2->RotateEuler(FVector{ M_PI_2, 0, 0 });
		Base = MeshBoolean::MeshUnion(Base, Bar);
		Base = MeshBoolean::MeshUnion(Base, Bar2);
	}
	return Base;
}

FBox SurfaceJointD::GetJointBoundingBox() const
{
	return Math::FBox(FVector{OuterRadius * 2, OuterRadius * 2, SocketHeight});
}
TFunction<bool(FVector)> SurfaceJointD::GetJointBoundingVolume() const
{
	return [&](FVector Pos) {
		bool CollideSocket = false;
		bool CollideJoint = false;
		if(Pos.z() <= 0.5 * SocketHeight && Pos.z() >= 0.5 * -SocketHeight)
			CollideSocket = Pos.x() * Pos.x() + Pos.y() * Pos.y() <= OuterRadius * OuterRadius;
		if(Pos.z() <= BarRadius && Pos.z() >= -BarRadius)
			CollideJoint = Pos.x() * Pos.x() + Pos.y() * Pos.y() <= BarLength * BarLength * 0.25;
		return CollideSocket | CollideJoint;
	};
}

TArray<FVector> SurfaceJointD::GetFirstPortInMeshLocation()
{
    return{
    	FVector(0, 0, JointHeight * 0.4),
		FVector(0, 0, -JointHeight * 0.4),
	};
}

TArray<FVector> SurfaceJointD::GetSecondPortInMeshLocation()
{
    return {
    	FVector((InnerRadius + OuterRadius) * 0.5, 0, 0),
		FVector(-(InnerRadius + OuterRadius) * 0.5, 0, 0),
    };
}

FBox SurfaceJointU::GetJointBoundingBox() const
{
	return Math::FBox(FVector{JointRadius * 2, Diameter, Diameter});
}
TFunction<bool(FVector)> SurfaceJointU::GetJointBoundingVolume() const
{
	return [&](FVector Pos) {
		bool CollideSocket = false;
		bool CollideJoint = false;
		if(Pos.z() <= 0.5 * SocketThickness && Pos.z() >= 0.5 * -SocketThickness)
			CollideSocket = Pos.x() * Pos.x() + Pos.y() * Pos.y() <= 0.25 * Diameter * Diameter;
		if(Pos.norm() < JointRadius)
			CollideJoint = true;

		return CollideSocket | CollideJoint;
	};
}

ObjectPtr<StaticMesh> SurfaceJointU::GenerateSecondMesh(double Diameter, double JointRadius, double Thickness, double CylinderRadius, bool bPrint, bool bRender)
{
	auto Cube = BasicShapesLibrary::GenerateCylinder( Thickness, Diameter * 0.5, bRender | bPrint ? 200 : 32);
	Cube->RotateEuler({0, M_PI_2, 0});
	auto JointBall = BasicShapesLibrary::GenerateSphere(JointRadius, bRender | bPrint ? 128 : 32);
	Cube = MeshBoolean::MeshMinus(Cube, JointBall);

	if(bPrint)
	{
		auto Cut = BasicShapesLibrary::GenerateCuboid({Thickness * 0.5,  Diameter * 4, Diameter * 0.5});
		Cube = MeshBoolean::MeshMinus(Cube, Cut);
	}

	auto SupporterCylinder = BasicShapesLibrary::GenerateCylinder(Diameter * 0.95, CylinderRadius, bRender | bPrint ? 200 : 32);
	Cube = MeshBoolean::MeshUnion(Cube, SupporterCylinder);

	auto CylinderCut = BasicShapesLibrary::GenerateCylinder(JointRadius, CylinderRadius * 1.2);
	return MeshBoolean::MeshMinus(Cube, CylinderCut);
}

ObjectPtr<StaticMesh> SurfaceJointU::GenerateJointMesh(double Radius, double SupporterRadius, bool bPrint, bool bRender)
{
	auto SphereMesh = BasicShapesLibrary::GenerateSphere(Radius, bRender | bPrint ? 128 : 32);
	auto CutCylinder = BasicShapesLibrary::GenerateHollowCylinder(2*Radius, 0.5*Radius, SupporterRadius * 2., bRender | bPrint ? 128 : 32);
	CutCylinder->RotateEuler({0, -0.5*M_PI, 0});
	SphereMesh = MeshBoolean::MeshMinus(SphereMesh, CutCylinder);
	return MeshBoolean::MeshMinus(SphereMesh, CutCylinder);
}

TArray<FVector> SurfaceJointU::GetFirstPortInMeshLocation()
{
	static double MaxLength = sqrt(Math::Pow2(JointRadius) - Math::Pow2(LinkageRadius));
    return { FVector(MaxLength * 0.9, 0, 0), FVector(-MaxLength * 0.9, 0, 0) };
}

TArray<FVector> SurfaceJointU::GetSecondPortInMeshLocation()
{
	double			RadiusSample = (JointRadius + Diameter * 0.5) * 0.5;
	constexpr int	SampleU = 64;
	TArray<FVector> Result;
	for (int UIndex = 0; UIndex < SampleU; UIndex++)
	{
		double u = ((double)UIndex) / (double)(SampleU);
		Result.emplace_back(0., RadiusSample * sin(u * 2. * M_PI), RadiusSample * cos(u * 2. * M_PI));
	}
	return Result;
}

FBox SurfaceJointS::GetJointBoundingBox() const
{
	return Math::FBox(FVector{OuterRadius * 2, OuterRadius * 2, OuterRadius * 2});
}
TFunction<bool(FVector)> SurfaceJointS::GetJointBoundingVolume() const
{
	return [&](const FVector& Pos) {
		return Pos.norm() <= OuterRadius;
	};
}

ObjectPtr<StaticMesh> SurfaceJointS::GenerateSecondMesh(double OuterRadius, double InnerRadius, bool bPrint, bool bRender)
{
	auto OuterSphere = BasicShapesLibrary::GenerateSphere(OuterRadius, bRender | bPrint ? 128 : 32);
	auto InnerSphere = BasicShapesLibrary::GenerateSphere(InnerRadius, bRender | bPrint ? 128 : 32);
	auto CutCube = BasicShapesLibrary::GenerateCuboid({OuterRadius*2, OuterRadius*6, OuterRadius*6});
	CutCube->Translate({1.5 * OuterRadius, 0, 0});
	OuterSphere = MeshBoolean::MeshMinus(OuterSphere, InnerSphere);
	OuterSphere = MeshBoolean::MeshMinus(OuterSphere, CutCube);
	if( bPrint)
	{
		CutCube = BasicShapesLibrary::GenerateCuboid({InnerRadius * 0.7, OuterRadius*4, InnerRadius * 0.7});
		OuterSphere = MeshBoolean::MeshMinus(OuterSphere, CutCube);
	}
	return OuterSphere;
}

ObjectPtr<StaticMesh> SurfaceJointS::GenerateJointMesh(double Radius, bool bPrint, bool bRender)
{
	return BasicShapesLibrary::GenerateSphere(Radius, bRender | bPrint ? 128 : 32);
}

TArray<FVector> SurfaceJointS::GetFirstPortInMeshLocation()
{
	return {FVector(InnerRadius * 0.7, 0, 0)};
}

TArray<FVector> SurfaceJointS::GetSecondPortInMeshLocation()
{
	return {
		FVector(-(InnerRadius + OuterRadius) * 0.5, 0, 0), // X
		// FVector(0, 0, OuterRadius * 0.9), // Z Up
		// FVector(0, 0, -OuterRadius * 0.9), // Z Down
		FVector(0, 0, -(InnerRadius + OuterRadius) * 0.5),
		FVector(0, 0, (InnerRadius + OuterRadius) * 0.5),
	};
}
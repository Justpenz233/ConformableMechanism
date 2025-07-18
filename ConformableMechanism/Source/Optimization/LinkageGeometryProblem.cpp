//
// Created by marvel on 11/8/24.
//

#include "LinkageGeometryProblem.h"

#include "LinkageOptimize/SurfaceLinkageProblem.h"
#include "igl/segment_segment_intersect.h"

void LinkageGeometryProblem::GetLinkageGeometry(const TArray<double>& Params, MechanismSequence& Sequence)
{
	ASSERT(Params.size() % 2 == 0);
	ASSERT(!Sequence.Joints.empty());

	auto Surface = Sequence.RSurface;
	int LinkageNum = Sequence.JointNumWithoutEffector() - 1;
	int ControlPointNum = Params.size() / 2 / LinkageNum;
	auto& Joints = Sequence.Joints;

	// Calculate the joint pin and socket uv first
	for (auto& Joint : Sequence.Joints)
	{
		ASSERT(Joint.GetComponent() != nullptr);
		if(Joint.Type == 'E') [[unlikely]] continue;
		if(Joint.PinUV.empty())
		{
			auto JointPortNum = Joint.GetComponent()->GetPinPortInMeshLocation().size();
			for (int j = 0; j < JointPortNum; j++)
			{
				auto	 JointPortInMesh = Joint.GetComponent()->GetPinPortInMeshLocation()[j];
				FVector	 JointPortInMeshWorld = Joint.InitTransform * JointPortInMesh;
				FVector2 UV = Surface->ProjectionThickness(JointPortInMeshWorld);
				Joint.PinInMeshUV.push_back(UV);

				FVector JointPortWorld = Joint.GetComponent()->GetPinPortWorldLocation(j);
				UV = Surface->ProjectionThickness(JointPortWorld);
				Joint.PinUV.push_back(UV);
			}
		}
		if(Joint.SocketUV.empty())
		{
			auto SocketPortNum = Joint.GetComponent()->GetSocketPortInMeshLocation().size();
			for (int j = 0; j < SocketPortNum; j++)
			{
				auto	 SocketInMesh = Joint.GetComponent()->GetSocketPortInMeshLocation()[j];
				FVector	 SocketInMeshWorld = Joint.InitTransform * SocketInMesh;
				FVector2 UV = Surface->ProjectionThickness(SocketInMeshWorld);
				Joint.SocketInMeshUV.push_back(UV);

				auto SocketPortWorld = Joint.GetComponent()->GetSocketPortWorldLocation(j);
				UV = Surface->ProjectionThickness(SocketPortWorld);
				Joint.SocketUV.push_back(UV);
			}
		}
	}

	//Calculate the linkage geometry
	for(int i = 0; i < Sequence.Joints.size() - 1; i ++)
	{
		auto& Joint = Sequence.Joints[i];
		ASSERT(!Joint.TransformSequence.empty());
		ASSERT(!(Joint.InitTransform == FTransform::Identity()));
		if(Joint.Type == 'E') [[unlikely]] continue;
		auto& LinkageGeometry = Joint.LinkageSpline;
		LinkageGeometry = tinyspline::BSpline(2 + ControlPointNum, 2);
		TArray<FVector2> ControlPoint(2 + ControlPointNum);

		for (int j = 0; j < ControlPointNum; j ++)
		{
			ControlPoint[j + 1] = {
				Params[j * 2 + i * ControlPointNum * 2],
				Params[j * 2 + i * ControlPointNum * 2 + 1]};

			// TODO Here should remap or check valid of V
		}
		auto MinDistance = std::numeric_limits<double>::max();
		for(int j = 0; j < Joint.PinUV.size(); j++)
		{
			if (auto Distance = (ControlPoint[1] - Joint.PinUV[j]).norm(); Distance < MinDistance)
			{
				MinDistance = Distance;
				Joint.Pin = j;
			}
		}
		MinDistance = std::numeric_limits<double>::max();
		for (int j = 0; j < Joints[i + 1].SocketUV.size(); j ++)
		{
			if (auto Distance = (ControlPoint[ControlPointNum] -
				Joints[i + 1].SocketUV[j]).norm(); Distance < MinDistance)
			{
				MinDistance = Distance;
				Joints[i + 1].Socket = j;
			}
		}
		int JointIndex = Joint.Pin; int SocketIndex = Joints[i + 1].Socket;
		ControlPoint[0] = Joint.PinUV[JointIndex];
		ControlPoint[ControlPointNum + 1] = Joints[i + 1].SocketUV[SocketIndex];
		TArray<double> ControlPointData(ControlPoint.size() * 2);
		std::memcpy(ControlPointData.data(), ControlPoint.data(), ControlPoint.size() * 2 * sizeof(double));
		LinkageGeometry.setControlPoints(ControlPointData);
	}

	//Sample linkage spline
	static int SplineSample = GConfig.Get<int>("LinkageProblem", "SplineSample");
	for (auto& Joint : Sequence.Joints)
		Joint.LinkageSample = SurfaceLinkageProblem::LinkageSplineSample(Joint.LinkageSpline, Surface.get(), SplineSample);


	// Effector linkage
	{
		auto& Effector = Joints[Sequence.EffectorIndex];
		auto& ParentSample = Joints[Sequence.EffectorParentIndex].LinkageSample;
		int S = ParentSample.size() * 0.5;
		Effector.LinkageSample = {ParentSample[S], Effector.InitTransform.GetTranslation()};
	}
}

double LinkageGeometryProblem::CalcLinkageProblemScore(MechanismSequence& Sequence)
{
	auto& Joints = Sequence.Joints;

	static auto MeshScale = GConfig.Get<double>("JointMesh", "Scale");
	static auto Tolerance = GConfig.Get<double>("JointMesh", "Tolerance") * MeshScale;
	static auto LinkageRadius = GConfig.Get<double>("JointMesh", "LinkageRadius") * MeshScale;
	static auto LinkageMinDistance = LinkageRadius * 2. + Tolerance;

	double SegSegMinDistance = std::numeric_limits<double>::max();

	int SelfLinkageIntersection = 0;
	static bool PostProcessLinkage = GConfig.Get<bool>("LinkageProblem", "PostProcessLinkage");
	if(!PostProcessLinkage)
	{
		for (const auto& Joint : Joints)
		{
			if( Joint.Type == 'E' ) continue;
			for(int k = 0; k < Joint.LinkageSample.size() - 2; k ++)
			{
				const auto& Linkage = Joint.LinkageSample;
				for(int l = k + 2; l < Linkage.size() - 2;l ++)
				{
					auto Start0 = Linkage[k]; auto End0 = Linkage[k + 1];
					auto Start1 = Linkage[l]; auto End1 = Linkage[l + 1];
					double t, u;
					if(igl::segment_segment_intersect(Start0, End0 - Start0, Start1, End1 - Start1, t, u))
						SelfLinkageIntersection ++;
				}
			}
		}
	}


	return {};
}
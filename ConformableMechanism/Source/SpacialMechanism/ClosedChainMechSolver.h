//
// Created by MarvelLi on 2023/12/28.
//

#pragma once
#include "Animation/IKSolver.h"

struct ClosedChainMechFunctor : IKBaseFunctor<double>
{
	ClosedChainMechFunctor(int ParameterDimension)
		: IKBaseFunctor<double>(ParameterDimension, 12), ParameterDimension(ParameterDimension) {}
	int							  ParameterDimension;
	std::vector<ObjectPtr<Joint>> Joints;

	/// \brief calc loss of full body ik
	/// \param x Parameter of full body
	/// \param fvec Loss vector for each dimention, shoule be 4 * 3 of a transform matrix
	/// \return 0 if successful
	int operator()(const Eigen::VectorXd& x, Eigen::VectorXd& fvec) const;
	int df(const VectorXd& x, MatrixXd& fjac) const;
};

class ClosedChainMechSolver : public IKSolver
{
private:
	// Calc the min value of singularity matrix, should call solve first
	double CalcSingularity(const VectorXd& Parameter) const;

	bool bLoop = true;
	double MaxiumDrivenAngle = 0.;
	double CurrentDrivenAngle = 0.;
	double DrivenDir = 1.;

public:
	double SimulationEps = 1e-4;
	double Singularity;
	double Loss;

	virtual double Solve() override;

	virtual void Drive(ObjectPtr<Joint> Root, double DeltaTime) override;

	void SetLimitDrivenAngle(double MaxiumAngle);
};
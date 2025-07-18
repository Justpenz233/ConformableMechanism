//
// Created by marvel on 23/7/24.
//

#pragma once
#include "Algorithm/GeometryProcess.h"
#include "Core/CoreMinimal.h"
#include "Surface/ParametricSurface.h"

class HemiSphereSurface : public ParametricSurface
{
public:
	HemiSphereSurface() : ParametricSurface(false) {}

	virtual FVector Sample(double u, double v) const override
	{
		u = std::clamp(u, 0., 1.);
		v = std::clamp(v, 0., 1.);
		u = 2. * u - 1.; v = 2. * v - 1.;
		return {u, v, std::sqrt(2. - u*u - v*v)};
	}
};


class HalfCylinder : public ParametricSurface
{
protected:
	double Length;
	double Radius;
public:
	HalfCylinder(double InRadius = 1., double InLength = 1.) : ParametricSurface(false), Radius(InRadius), Length(InLength) {}
	virtual FVector Sample(double u, double v) const override
	{
		v = v * 2. - 1.;
		u = u * M_PI;
		return {v * Length * 0.5, cos(u) * Radius, sin(u) * Radius};
	}

	virtual FVector SampleThickness(double u, double v, double Thickness) const override
	{
		v = v * 2. - 1.;
		u = u * M_PI;
		return {v * Length * 0.5, cos(u) * (Radius + Thickness), sin(u) * (Radius + Thickness)};
	}

	virtual FVector2 Projection(const FVector& Pos) const override
	{
		return {std::atan2(Pos[2], Pos[1]) / M_PI, Pos[0] / Length + 0.5};
	}
};

class HalfCatenoid : public ParametricSurface
{
protected:
	double c;
	double h;

public:
	HalfCatenoid(double InC = 1., double InH = 1.): ParametricSurface(false), c(InC), h(InH) {}
	virtual FVector Sample(double u, double v) const override
	{
		u = u * M_PI;
		v = v * 2.0 - 1.0;

		return {v * h * 0.5, c * cosh(v/c) * cos(u) * 0.5, c * cosh(v/c) * sin(u) * 0.5};
	}

	virtual FVector SampleThickness(double u, double v, double Thickness) const override
	{
		u = u * M_PI;
		v = v * 2.0 - 1.0;

		return {v * h * 0.5, c * (cosh(v/c) + Thickness) * cos(u) * 0.5, c * (cosh(v/c) + Thickness) * sin(u) * 0.5};
	}
};



class HalfHyperboloid : public ParametricSurface
{
protected:
	double c;
	double h;

public:
	HalfHyperboloid(double InRadius = 1., double InHeight = 1.): ParametricSurface(false), c(InRadius), h(InHeight) {}
	virtual FVector Sample(double u, double v) const override
	{
		u = u * M_PI;
		v = v * 2.0 - 1.0;

		return {v * h * 0.5, c * cosh(v) * cos(u), c * cosh(v) * sin(u)};
	}

	virtual FVector SampleThickness(double u, double v, double Thickness) const override
	{
		u = u * M_PI;
		v = v * 2.0 - 1.0;

		return {v * h * 0.5, c * (cosh(v) + Thickness) * cos(u), c * (cosh(v) + Thickness) * sin(u)};
	}
};

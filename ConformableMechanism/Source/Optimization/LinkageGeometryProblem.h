//
// Created by marvel on 11/8/24.
//

#pragma once
#include "MechanismSequence.h"
#include "Core/CoreMinimal.h"

/**
 * This problem should define linkages' geometry and return as segments
 * Should also calculate the collision constrains and collision score
 */
struct LinkageGeometryProblem
{
	/**
	 * Calculate the linkage geometry and fill into the sequence
	 * @param Params The parameters define linkage geometry
	 */
	static void GetLinkageGeometry(const TArray<double>& Params, MechanismSequence& Sequence);

	/**
	 * Calculate the linkage collision score
	 * @param Sequence The sequence to calculate
	 * @return The collision score
	 */
	static double CalcLinkageProblemScore(MechanismSequence& Sequence);
};

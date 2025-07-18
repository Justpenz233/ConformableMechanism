//
// Created by MarvelLi on 2024/6/19.
//

#pragma once

#include "spdlog/stopwatch.h"

#include <pagmo/problem.hpp>
#include <pagmo/algorithms/cmaes.hpp>
#include <pagmo/algorithms/gaco.hpp>
#include <pagmo/batch_evaluators/thread_bfe.hpp>
#include <pagmo/island.hpp>

/**
 * Random valid X for a given constraint problem, in population size.
 * When MaxSamples is -1, will random until find an all valid X population.
 * @param Problem The problem to solve
 * @param IfValid The valid function to check if X is valid by given a fitness value
 * @param PopulationSize The population size
 * @param MaxSamples The max samples to find valid X
 * @return The population with valid X
 */
inline pagmo::population RandomValidX(pagmo::problem& Problem, const std::function<bool(double)>& IfValid, int PopulationSize, int MaxSamples = -1)
{
	LOG_INFO("Try to find random valid X for problem: {} with population size: {} and max samples: {}", Problem.get_name(), PopulationSize, MaxSamples);
	spdlog::stopwatch Watch;

	pagmo::population Population(Problem);
	TArray<TArray<double>> PopulationX(PopulationSize); TArray<TArray<double>> PopulationF(PopulationSize);
	std::atomic_int ValidCount = 0;
	std::atomic_int SampleCount = 0;
	ParallelFor(std::thread::hardware_concurrency(), [&](int i) {
		while(true)
		{
			if(MaxSamples != -1 && SampleCount >= MaxSamples)
				return;
			if (ValidCount >= PopulationSize)
				return;
			SampleCount += 1;
			auto X = Population.random_decision_vector();
			auto Fitness = Problem.fitness(X);
			if (IfValid(Fitness[0]))
			{
				auto Index = ValidCount.fetch_add(1);
				if (Index >= PopulationSize) return;
				PopulationX[Index] = X;
				PopulationF[Index] = Fitness;
			}
		}
	});

	// Now we fill the population with random population if not enough valid X
	if (ValidCount < PopulationSize)
	{
		LOG_INFO("Not enough valid X, found {}, fill with {} random X", int(ValidCount), int(PopulationSize - ValidCount));
		int IndexBase = ValidCount;
		ParallelFor(PopulationSize - IndexBase, [&](int i) {
			auto X = Population.random_decision_vector();
			auto Fitness = Problem.fitness(X);
			PopulationX[IndexBase + i] = X;
			PopulationF[IndexBase + i] = Fitness;
			ASSERT(IndexBase + i < PopulationSize);
		});
	}

	ASSERT(Population.size() == 0);
	for (int i = 0; i < PopulationSize; i++)
	{
		Population.push_back(PopulationX[i], PopulationF[i]);
	}
	ASSERT(Population.size() == PopulationSize);
	LOG_INFO("Find random valid X with valid count: {} by {} samples in {}s", int(ValidCount), int(SampleCount), Watch);
	return Population;
}
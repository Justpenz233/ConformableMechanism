//
// Created by MarvelLi on 2024/9/10.
//
#pragma once
#include "ContainerTypes.h"



inline int JointParameterMap(char Type, bool Root = false)
{
	static TMap<char, int> JointParameterMap = {
		{'D', 1 + 2}, {'R', 2 + 2}, {'U', 2 + 3}, {'S', 2 + 1}, {'E', 0}
	};

	return Type == 'R' && Root ? 3 : JointParameterMap[Type];
}

inline int GetParameterNum(const String& Type)
{
	int Sum = 0;
	for (int i = 0; i < Type.size(); i++)
		Sum += JointParameterMap(Type[i], i == 0);
	return Sum;
}

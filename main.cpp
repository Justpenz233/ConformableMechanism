#include "Editor.h"
#include "Example/PathGenerationExperiment.h"
#include "Example/MotionGenerationExperiment.h"
// Project DIR,use as the root of the project
#ifndef PROJECT_DIR
#define PROJECT_DIR ""
#endif

int main(int argc, char *argv[])
{
	std::cout << "Bin Path: " << argv[0] << std::endl;
	std::cout << "Project Dir: " << PROJECT_DIR << std::endl;
    GEditor.Init(argv[0], PROJECT_DIR);
	// GEditor.LoadWorld(MotionGenerationExample());
	GEditor.LoadWorld(PathGenerationExample());
    GEditor.Start();
}

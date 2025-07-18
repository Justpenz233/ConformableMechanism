#include "Editor.h"
#include "Example/PathGenerationExperiment.h"
#include "Example/MotionGenerationExperiment.h"
// Project DIR,use as the root of the project
#ifndef PROJECT_DIR
#define PROJECT_DIR ""
#endif

int main(int argc, char *argv[])
{
    GEditor.Init(argv[0], PROJECT_DIR);
	// GEditor.LoadWorld(MotionGenerationExample());
	GEditor.LoadWorld(PathGenerationExample());
    GEditor.Start();
}

#include "Scene.h"
#include <map>

#include "ExplicitSimulation.h"
#include "Scene1.h"
#include "SingleStep.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Demo Scene", creator<Scene1>()},
    {"Single Step", creator<SingleStep>()},
    {"Explicit Simulation", creator<ExplicitSimulation>()}
    // add more Scene types here
};

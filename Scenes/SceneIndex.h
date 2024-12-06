#include "Scene.h"
#include <map>

#include "Scene1.h"
#include "RigidSceneSingleStep.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Demo Scene", creator<Scene1>()},
    {"Rigid Single Step", creator<RigidSceneSingleStep>()},
    // add more Scene types here
};

#include "Scene.h"
#include <map>

#include "Scene1.h"
#include "RigidSceneSingleStep.h"
#include "RigidScene2.h"

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
    {"Rigid 2", creator<RigidScene2>()},
    // add more Scene types here
};

#include "Scene.h"
#include <map>

#include "Scene1.h"
#include "RigidSceneSingleStep.h"
#include "RigidScene2.h"
#include "RigidScene3.h"
#include "RigidScene4.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Demo Scene", creator<Scene1>()},
    {"Single Step", creator<RigidSceneSingleStep>()},
    {"Spinning", creator<RigidScene2>()},
    {"Collision", creator<RigidScene3>()},
    {"Complex", creator<RigidScene4>()},
    // add more Scene types here
};

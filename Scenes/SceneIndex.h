#include "Scene.h"
#include <map>

#include "Scene1.h"
#include "SceneSingleStep.h"
#include "SceneEuler.h"
#include "SceneMidpoint.h"
#include "SceneComplex.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Demo Scene", creator<Scene1>()},
    {"Scene Single Step", creator<SceneSingleStep>()},
    {"Scene Euler", creator<SceneEuler>()},
    {"Scene Midpoint", creator<SceneMidpoint>()},
    {"Scene Complex", creator<SceneComplex>()},
    // add more Scene types here
};

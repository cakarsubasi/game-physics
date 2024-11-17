#include "Scene.h"

class Scene1 : public Scene
{
    public:
    // This only exists as a sanity check to ensure the renderer isn't broken
    virtual auto onDraw(Renderer &renderer) -> void override {
        renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
    }
};
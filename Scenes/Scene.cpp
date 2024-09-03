#include "Scene.h"
#include <imgui.h>

using namespace glm;
void Scene::init()
{
    resetGrid();
}

void Scene::onDraw(Renderer &renderer)
{
    if (grid.width > 0)
        renderer.drawImage(grid.data, grid.height, grid.width, 0, 1);
}

void Scene::onGUI()
{
    if (ImGui::Button("Reset"))
    {
        resetGrid();
    }
    ImGui::SliderInt("Draw Radius", &drawRadius, 1, 10);
    ImGui::SliderFloat("Alpha", &alpha, 0, 1);
    if (ImGui::SliderInt("Resolution", (int *)&resolution, 10, 1000))
    {
        resetGrid();
    }
    ImGui::SliderFloat("dt", &dt, 0, 10, "%.3f", ImGuiSliderFlags_Logarithmic);
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && !ImGui::GetIO().WantCaptureMouse)
    {
        ImVec2 mousePos = ImGui::GetMousePos();
        ImVec2 windowSize = ImGui::GetIO().DisplaySize;
        ImVec2 normalizedMousePos = ImVec2(mousePos.x / (float)windowSize.x, mousePos.y / (float)windowSize.y);
        float x = normalizedMousePos.x * grid.width;
        float y = normalizedMousePos.y * grid.height;
        for (int i = -drawRadius; i < drawRadius; i++)
            for (int j = -drawRadius; j < drawRadius; j++)
            {
                int px = x + 0.5 + i;
                int py = y + 0.5 + j;
                if (px < 0 || px >= grid.width || py < 0 || py >= grid.height)
                    continue;
                if (i * i + j * j < 25)
                    grid(py, px) = 1;
            }
    }
}

/*
 * This function initializes the grid with random values between 0 and 1.
 * The border of the grid is set to 0.
 */
void Scene::randomInit(size_t width, size_t height)
{
    grid = Grid(height, width);
    for (size_t i = 0; i < width * height; i++)
    {
        grid.data[i] = rand() / (float)RAND_MAX;
    }
    for (int x = 0; x < width; x++)
    {
        grid(0, x) = 0;
        grid(height - 1, x) = 0;
    }
    for (int y = 0; y < height; y++)
    {
        grid(y, 0) = 0;
        grid(y, width - 1) = 0;
    }
}

void Scene::resetGrid()
{
    randomInit(resolution * Renderer::camera.aspectRatio(), resolution);
}

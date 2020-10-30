#include "bounding_volume_hierarchy.h"
#include "disable_all_warnings.h"
#include "draw.h"
#include "image.h"
#include "ray_tracing.h"
#include "screen.h"
#include "trackball.h"
#include "window.h"
#include "lighting.h"
#include "ray_debug.h"
#include "post_processing.h"
// Disable compiler warnings in third-party code (which we cannot change).
DISABLE_WARNINGS_PUSH()
#include <glm/gtc/type_ptr.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
#include <imgui.h>
DISABLE_WARNINGS_POP()
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <type_traits>
#ifdef USE_OPENMP
#include <omp.h>
#endif

const int BVH_DEPTH = 25;              // Max BVH tree depth.
const int BVH_MIN_NODE_TRIANGLES = 10; // Min number of triangles required to for BVH node to attempt to spawn child nodes.
const int BVH_BINS = 5;                // Number of bins to use for BVH SAH.
const int SAMPLING_FACTOR = 1;         // Super-sampling factor (1=>1x super-sampling, 2=>2x super-sampling, 3=>4x super-sampling, etc).

// This is the main application. The code in here does not need to be modified.
constexpr glm::ivec2 windowResolution { 800, 800 };
const std::filesystem::path dataPath { DATA_DIR };
const std::filesystem::path outputPath { OUTPUT_DIR };

enum class ViewMode {
    Rasterization = 0,
    RayTracing = 1
};

static glm::vec3 getFinalColor(const Scene& scene, BoundingVolumeHierarchy& bvh, Ray &ray)
{
    HitInfo hitInfo;
    if (bvh.intersect(ray, hitInfo)) {
        return recursiveRayTrace(ray, hitInfo, scene, bvh, 0);
    } else {
        // Draw a red debug ray if the ray missed.
        drawRay(ray, glm::vec3(1.0f, 0.0f, 0.0f));
        return glm::vec3(0.0f);
    }
}

static void setOpenGLMatrices(const Trackball& camera);
static void renderOpenGL(const Scene& scene, const Trackball& camera, int selectedLight);

// This is the main rendering function. You are free to change this function in any way (including the function signature).
static void renderRayTracing(const Scene& scene, const Trackball& camera, BoundingVolumeHierarchy& bvh, Screen& screen, int sample_factor)
{
    // Compute necessary constants.
    int sample_multiplier = std::pow(2, sample_factor - 1);
    int sample_width = windowResolution.x * sample_multiplier;
    int sample_height = windowResolution.y * sample_multiplier;
    glm::vec3 sampling_coefficient = {
        sample_multiplier * sample_multiplier,
        sample_multiplier * sample_multiplier,
        sample_multiplier * sample_multiplier};

    // Create suitably sized 2D vectors to house sample colour and depth values.
    std::vector<std::vector<glm::vec3>> sample_array;
    sample_array.resize(sample_height);
    std::vector<std::vector<float>> sample_depths;
    sample_depths.resize(sample_height);

    // Populate array of sample values.
    #ifdef USE_OPENMP
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < sample_height; y++) {
        for (int x = 0; x != sample_width; x++) {
            // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
            const glm::vec2 normalizedPixelPos {
                float(x) / sample_width * 2.0f - 1.0f,
                float(y) / sample_height * 2.0f - 1.0f
            };
            Ray cameraRay = camera.generateRay(normalizedPixelPos);

            // Store sample value and sample depth.
            sample_array[y].push_back(getFinalColor(scene, bvh, cameraRay));
            sample_depths[y].push_back(cameraRay.t * glm::length(cameraRay.direction));
        }
    }

    // Create a suitably sized 2D vector to house pixel values before post-processing.
    std::vector<std::vector<glm::vec3>> pixel_array;
    pixel_array.resize(windowResolution.y);
    std::vector<std::vector<float>> pixel_depths;
    pixel_depths.resize(windowResolution.y);

    // Sample pixel array.
    #ifdef USE_OPENMP
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++)
    {
        for (int x = 0; x != windowResolution.x; x++)
        {
            // Sample 'boxes' of pixels starting from the top-left corner
            glm::vec3 pixel_value = {0.0f, 0.0f, 0.0f};
            float depth_value = 0.0f;
            int corner_x = x * sample_multiplier;
            int corner_y = y * sample_multiplier;
            for (int y_step = 0; y_step < sample_multiplier; y_step++)
            {
                for (int x_step = 0; x_step < sample_multiplier; x_step++)
                {
                    pixel_value += sample_array[corner_y + y_step][corner_x + x_step];
                    depth_value += sample_depths[corner_y + y_step][corner_x + x_step];
                }
            }
            pixel_array[y].push_back(pixel_value / sampling_coefficient);
            pixel_depths[y].push_back(depth_value / (sample_multiplier * sample_multiplier));
        }
    }

    // Run through post-processing pipeline and render.
    postProcessingPipeline(screen, windowResolution, pixel_array, pixel_depths);
}

/**Generate debug rays according to the chosen sample factor.
 * 
 * Parameters:
 * camera: The viewport Trackball object
 * pixel_center: Non-normalised pixel coordinates of the pixel whose sample rays are to be visualised (Assumes (0,0) origin at bottom-left corner)
 * sample_factor: Governs super-sampling factor. See documentation of SAMPLING_FACTOR for details.
 * 
 * Returns: A vector of debug rays, to be rendered in rasterization mode.
 */
std::optional<std::vector<Ray>> generateDebugRays(const Trackball &camera, const glm::vec2 &pixel_center, int sample_factor)
{
    // Compute necessary constants.
    int sample_multiplier = std::pow(2, sample_factor - 1);
    float atomic_step = 1.0f / sample_multiplier;

    // Create a vector to store the debug rays.
    std::optional<std::vector<Ray>> debug_rays {std::vector<Ray>{}};

    // 'Walk' in horizontal and vertical portions to emulate positions of super-sampled rays.
    for (int y_step = -(sample_multiplier - 1); y_step < sample_multiplier; y_step++)
    {
        if (y_step % 2 == 0 && sample_factor != 1) {continue;} // Remove unnecessary center rays when appropriate.
        for (int x_step = -(sample_multiplier - 1); x_step < sample_multiplier; x_step++)
        {
            if (x_step % 2 == 0 && sample_factor != 1) {continue;} // Remove unnecessary center rays when appropriate.

            // Compute sample position and generate ray.
            float sample_x = pixel_center.x + (x_step * atomic_step);
            float sample_y = pixel_center.y + (y_step * atomic_step);
            const glm::vec2 normalizedSamplePos = {
                sample_x / windowResolution.x * 2.0f - 1.0f,
                sample_y / windowResolution.y * 2.0f - 1.0f
            };
            const Ray sample_ray = camera.generateRay(normalizedSamplePos);
            debug_rays.value().push_back(sample_ray);
        }
    }
    return debug_rays;
}

int main(int argc, char** argv)
{
    Trackball::printHelp();
    std::cout << "\n Press the [R] key on your keyboard to create a ray towards the mouse cursor" << std::endl
              << std::endl;

    Window window { "Final Project - Part 2", windowResolution, OpenGLVersion::GL2 };
    Screen screen { windowResolution };
    Trackball camera { &window, glm::radians(50.0f), 3.0f };
    camera.setCamera(glm::vec3(0.0f, 0.0f, 0.0f), glm::radians(glm::vec3(20.0f, 20.0f, 0.0f)), 3.0f);

    SceneType sceneType { SceneType::SingleTriangle };
    std::optional<std::vector<Ray>> optDebugRays;
    Scene scene = loadScene(sceneType, dataPath);
    BoundingVolumeHierarchy bvh {&scene, BVH_DEPTH, BVH_MIN_NODE_TRIANGLES, BVH_BINS};

    int bvhDebugLevel = 0;
    bool debugBVH { false };
    ViewMode viewMode { ViewMode::Rasterization };

    window.registerKeyCallback([&](int key, int /* scancode */, int action, int /* mods */) {
        if (action == GLFW_PRESS) {
            switch (key) {
            case GLFW_KEY_R: {
                // Shoot a ray. Produce a ray from camera to the far plane.
                const auto tmp = window.getCursorPos();
                optDebugRays = generateDebugRays(camera, tmp, SAMPLING_FACTOR);
                viewMode = ViewMode::Rasterization;
            } break;
            case GLFW_KEY_ESCAPE: {
                window.close();
            } break;
            };
        }
    });

    int selectedLight { 0 };
    while (!window.shouldClose()) {
        window.updateInput();

        // === Setup the UI ===
        ImGui::Begin("Final Project - Part 2");
        {
            constexpr std::array items { "SingleTriangle", "Cube", "Cornell Box (with mirror)", "Cornell Box (spherical light and mirror)", "Monkey", "Dragon", /* "AABBs",*/ "Spheres", /*"Mixed",*/ "Custom" };
            if (ImGui::Combo("Scenes", reinterpret_cast<int*>(&sceneType), items.data(), int(items.size()))) {
                optDebugRays.reset();
                scene = loadScene(sceneType, dataPath);
                bvh = BoundingVolumeHierarchy(&scene, BVH_MIN_NODE_TRIANGLES, BVH_BINS);
                if (optDebugRays) {
                    for (Ray &ray : optDebugRays.value())
                    {
                        HitInfo dummy {};
                        bvh.intersect(ray, dummy);
                    }
                }
            }
        }
        {
            constexpr std::array items { "Rasterization", "Ray Traced" };
            ImGui::Combo("View mode", reinterpret_cast<int*>(&viewMode), items.data(), int(items.size()));
        }
        if (ImGui::Button("Render to file")) {
            {
                using clock = std::chrono::high_resolution_clock;
                const auto start = clock::now();
                renderRayTracing(scene, camera, bvh, screen, SAMPLING_FACTOR);
                const auto end = clock::now();
                std::cout << "Time to render image: " << std::chrono::duration<float, std::milli>(end - start).count() << " milliseconds" << std::endl;
            }
            screen.writeBitmapToFile(outputPath / "render.bmp");
        }
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Debugging");
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw BVH", &debugBVH);
            if (debugBVH)
                ImGui::SliderInt("BVH Level", &bvhDebugLevel, 1, bvh.numLevels());
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Lights");
        if (!scene.pointLights.empty() || !scene.sphericalLight.empty()) {
            {
                std::vector<std::string> options;
                for (size_t i = 0; i < scene.pointLights.size(); i++) {
                    options.push_back("Point Light " + std::to_string(i + 1));
                }
                for (size_t i = 0; i < scene.sphericalLight.size(); i++) {
                    options.push_back("Spherical Light " + std::to_string(i + 1));
                }

                std::vector<const char*> optionsPointers;
                std::transform(std::begin(options), std::end(options), std::back_inserter(optionsPointers),
                    [](const auto& str) { return str.c_str(); });

                ImGui::Combo("Selected light", &selectedLight, optionsPointers.data(), static_cast<int>(optionsPointers.size()));
            }

            {
                const auto showLightOptions = [](auto& light) {
                    ImGui::DragFloat3("Light position", glm::value_ptr(light.position), 0.01f, -3.0f, 3.0f);
                    ImGui::ColorEdit3("Light color", glm::value_ptr(light.color));
                    if constexpr (std::is_same_v<std::decay_t<decltype(light)>, SphericalLight>) {
                        ImGui::DragFloat("Light radius", &light.radius, 0.01f, 0.01f, 0.5f);
                    }
                };
                if (selectedLight < static_cast<int>(scene.pointLights.size())) {
                    // Draw a big yellow sphere and then the small light sphere on top.
                    showLightOptions(scene.pointLights[selectedLight]);
                } else {
                    // Draw a big yellow sphere and then the smaller light sphere on top.
                    showLightOptions(scene.sphericalLight[selectedLight - scene.pointLights.size()]);
                }
            }
        }

        if (ImGui::Button("Add point light")) {
            scene.pointLights.push_back(PointLight { glm::vec3(0.0f), glm::vec3(1.0f) });
            selectedLight = int(scene.pointLights.size() - 1);
        }
        if (ImGui::Button("Add spherical light")) {
            scene.sphericalLight.push_back(SphericalLight { glm::vec3(0.0f), 0.1f, glm::vec3(1.0f) });
            selectedLight = int(scene.pointLights.size() + scene.sphericalLight.size() - 1);
        }
        if (ImGui::Button("Remove selected light")) {
            if (selectedLight < static_cast<int>(scene.pointLights.size())) {
                scene.pointLights.erase(std::begin(scene.pointLights) + selectedLight);
            } else {
                scene.sphericalLight.erase(std::begin(scene.sphericalLight) + (selectedLight - scene.pointLights.size()));
            }
            selectedLight = 0;
        }

        // Clear screen.
        glClearDepth(1.0f);
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw either using OpenGL (rasterization) or the ray tracing function.
        switch (viewMode) {
        case ViewMode::Rasterization: {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            renderOpenGL(scene, camera, selectedLight);
            if (optDebugRays) {
                // Call getFinalColor for the debug ray. Ignore the result but tell the function that it should
                // draw the rays instead.
                enableDrawRay = true;
                for (Ray &ray : optDebugRays.value())
                {
                    (void)getFinalColor(scene, bvh, ray);
                }
                enableDrawRay = false;
            }
            glPopAttrib();
        } break;
        case ViewMode::RayTracing: {
            screen.clear(glm::vec3(0.0f));
            renderRayTracing(scene, camera, bvh, screen, SAMPLING_FACTOR);
            screen.setPixel(0, 0, glm::vec3(1.0f));
            screen.draw(); // Takes the image generated using ray tracing and outputs it to the screen using OpenGL.
        } break;
        default:
            break;
        };

        if (debugBVH) {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            setOpenGLMatrices(camera);
            glDisable(GL_LIGHTING);
            glEnable(GL_DEPTH_TEST);

            // Enable alpha blending. More info at:
            // https://learnopengl.com/Advanced-OpenGL/Blending
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            bvh.debugDraw(bvhDebugLevel);
            glPopAttrib();
        }

        ImGui::End();
        window.swapBuffers();
    }

    return 0; // execution never reaches this point
}

static void setOpenGLMatrices(const Trackball& camera)
{
    // Load view matrix.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    const glm::mat4 viewMatrix = camera.viewMatrix();
    glMultMatrixf(glm::value_ptr(viewMatrix));

    // Load projection matrix.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const glm::mat4 projectionMatrix = camera.projectionMatrix();
    glMultMatrixf(glm::value_ptr(projectionMatrix));
}

static void renderOpenGL(const Scene& scene, const Trackball& camera, int selectedLight)
{
    // Normals will be normalized in the graphics pipeline.
    glEnable(GL_NORMALIZE);
    // Activate rendering modes.
    glEnable(GL_DEPTH_TEST);
    // Draw front and back facing triangles filled.
    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_FILL);
    // Interpolate vertex colors over the triangles.
    glShadeModel(GL_SMOOTH);
    setOpenGLMatrices(camera);

    glDisable(GL_LIGHTING);
    // Render point lights as very small dots
    for (const auto& light : scene.pointLights)
        drawSphere(light.position, 0.01f, light.color);
    for (const auto& light : scene.sphericalLight)
        drawSphere(light.position, light.radius, light.color);

    if (!scene.pointLights.empty() || !scene.sphericalLight.empty()) {
        if (selectedLight < static_cast<int>(scene.pointLights.size())) {
            // Draw a big yellow sphere and then the small light sphere on top.
            const auto& light = scene.pointLights[selectedLight];
            drawSphere(light.position, 0.05f, glm::vec3(1, 1, 0));
            glDisable(GL_DEPTH_TEST);
            drawSphere(light.position, 0.01f, light.color);
            glEnable(GL_DEPTH_TEST);
        } else {
            // Draw a big yellow sphere and then the smaller light sphere on top.
            const auto& light = scene.sphericalLight[selectedLight - scene.pointLights.size()];
            drawSphere(light.position, light.radius + 0.01f, glm::vec3(1, 1, 0));
            glDisable(GL_DEPTH_TEST);
            drawSphere(light.position, light.radius, light.color);
            glEnable(GL_DEPTH_TEST);
        }
    }

    // Activate the light in the legacy OpenGL mode.
    glEnable(GL_LIGHTING);

    int i = 0;
    const auto enableLight = [&](const auto& light) {
        glEnable(GL_LIGHT0 + i);
        const glm::vec4 position4 { light.position, 1 };
        glLightfv(GL_LIGHT0 + i, GL_POSITION, glm::value_ptr(position4));
        const glm::vec4 color4 { glm::clamp(light.color, 0.0f, 1.0f), 1.0f };
        const glm::vec4 zero4 { 0.0f, 0.0f, 0.0f, 1.0f };
        glLightfv(GL_LIGHT0 + i, GL_AMBIENT, glm::value_ptr(zero4));
        glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, glm::value_ptr(color4));
        glLightfv(GL_LIGHT0 + i, GL_SPECULAR, glm::value_ptr(zero4));
        // NOTE: quadratic attenuation doesn't work like you think it would in legacy OpenGL.
        // The distance is not in world space but in NDC space!
        glLightf(GL_LIGHT0 + i, GL_CONSTANT_ATTENUATION, 1.0f);
        glLightf(GL_LIGHT0 + i, GL_LINEAR_ATTENUATION, 0.0f);
        glLightf(GL_LIGHT0 + i, GL_QUADRATIC_ATTENUATION, 0.0f);
        i++;
    };
    for (const auto& light : scene.pointLights)
        enableLight(light);
    for (const auto& light : scene.sphericalLight)
        enableLight(light);

    // Draw the scene and the ray (if any).
    drawScene(scene);

    // Draw a colored sphere at the location at which the trackball is looking/rotating around.
    glDisable(GL_LIGHTING);
    drawSphere(camera.lookAt(), 0.01f, glm::vec3(0.2f, 0.2f, 1.0f));
}

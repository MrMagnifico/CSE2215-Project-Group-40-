#pragma once
#include "screen.h"
DISABLE_WARNINGS_PUSH()
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
DISABLE_WARNINGS_POP()

// Handle the entire post-processing pipeline and render final image
void postProcessingPipeline(Screen &screen, const glm::ivec2 &windowResolution, std::vector<std::vector<glm::vec3>> &pixel_array);

// Create a grid of blurred pixels using box blur, blurring pixels whose 'intensity' meets or exceeds threshold.
// Uses all directly neighbouring pixels to create blur.
// Set threshold to 0.0f to blur all pixels in pixel_array.
std::vector<std::vector<glm::vec3>> boxBlur(const glm::ivec2 &windowResolution, std::vector<std::vector<glm::vec3>> &pixel_array,
                                            float threshold, int filter_size);

#pragma once
#include "screen.h"
DISABLE_WARNINGS_PUSH()
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
DISABLE_WARNINGS_POP()

// Handle the entire post-processing pipeline and render final image.
void postProcessingPipeline(Screen &screen, const glm::ivec2 &windowResolution, std::vector<std::vector<glm::vec3>> &pixel_array);

/**Create a grid of blurred pixels using box blur, blurring pixels whose 'intensity' meets or exceeds threshold.
 * 
 * Parameters:
 * windowResolution: Resolution of the viewport.
 * pixel_array: Array of pixels to compute blur map of.
 * threshold: Cutoff value that determines minimum intensity of pixel to include in the blur map.
 * filter_size: Dictates number of neighbouring pixels to use for computing blur.
 * 
 * Returns: a 2D std::vector of filtered pixels with box blur applied.
 */
std::vector<std::vector<glm::vec3>> boxBlur(const glm::ivec2 &windowResolution, std::vector<std::vector<glm::vec3>> &pixel_array,
                                            float threshold, int filter_size);

/**Apply a bloom filter by computing a grid of blurred pixels and adding their values to the pixel array.
 * 
 * Parameters:
 * windowResolution: Resolution of the viewport.
 * pixel_array: Array of pixels to compute bloom filter of.
 * threshold: Cutoff value that determines minimum intensity of pixel to include in the internal box blur map.
 * blur_filter_size: Dictates number of neighbouring pixels to use for computing internal box blur.
 */
void bloomFilter(const glm::ivec2 &windowResolution, std::vector<std::vector<glm::vec3>> &pixel_array, float threshold, int blur_filter_size);

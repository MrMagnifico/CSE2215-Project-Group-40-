#pragma once
#include "screen.h"
DISABLE_WARNINGS_PUSH()
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/constants.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <random>

struct DepthBuffer
{
    int width;
    int height;
    float sample_mean;
    float std_dev;
    std::vector<std::vector<float>> depth_values;
};

// Handle the entire post-processing pipeline and render final image.
void postProcessingPipeline(Screen &screen, const glm::ivec2 &windowResolution,
                            std::vector<std::vector<glm::vec3>> &pixel_array, std::vector<std::vector<float>> &depth_array);

// Return true if the given float is practically equal to the max float value.
bool isMax(float num);

/**Compute the probability density function of a Gaussian (normal) distribution at the given value.
 * 
 * Parameters:
 * mean: Mean to use for the Gaussian (normal) distribution.
 * std: Standard deviation to use for the Gaussian (normal) distribution.
 * value: The value to compute the probability density function of.
 */
float gaussianPdf(float mean, float std, float value);

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
 * pixel_array: Array of pixels to compute bloom filter for.
 * threshold: Cutoff value that determines minimum intensity of pixel to include in the internal box blur map.
 * blur_filter_size: Dictates number of neighbouring pixels to use for computing internal box blur.
 */
void bloomFilter(const glm::ivec2 &windowResolution, std::vector<std::vector<glm::vec3>> &pixel_array, float threshold, int blur_filter_size);

/**Process a 2D vector of depth values and return a DepthBuffer struct.
 * 
 * Parameters:
 * depth_array: 2D vector of depth values.
 * 
 * Returns: A DepthBuffer struct.
 */
DepthBuffer processDepthBuffer(std::vector<std::vector<float>> &depth_array, const glm::ivec2 &windowResolution);

/**Apply a depth of field filter by averaging pixels in the center of the viewport and blurring based on a Gaussian distribution of depth values.
 * 
 * Parameters:
 * windowResolution: Resolution of the viewport.
 * pixel_array: Array of pixels to compute depth of field for.
 * depth_buffer: 2D vector of depth values per-pixel. Used to compute blur blend.
 * blur_filter_size: Dictates number of neighbouring pixels to use for computing internal box blur.
 */
void depthOfFieldFilter(const glm::ivec2 &windowResolution, std::vector<std::vector<glm::vec3>> &pixel_array, DepthBuffer &depth_buffer, int blur_filter_size);

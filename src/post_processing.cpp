#include "post_processing.h"
#ifdef USE_OPENMP
#include <omp.h>
#endif

const bool POST_PROCESS = false; // Enable/disable post-processing.

// Bloom filter parameters.
const float BLOOM_THRESHOLD = 0.95f; // Threshold to use for filtering 'bright' pixels.
const int BLOOM_BLUR_SIZE = 3;       // Size to use for the box filter used for blurring.

void postProcessingPipeline(Screen &screen, const glm::ivec2 &windowResolution, std::vector<std::vector<glm::vec3>> &pixel_array)
{
    if (POST_PROCESS)
    {
        bloomFilter(windowResolution, pixel_array, BLOOM_THRESHOLD, BLOOM_BLUR_SIZE);
    }

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    // Render final pixel values to screen.
    for (int y = 0; y < windowResolution.y; y++)
    {
        for (int x = 0; x < windowResolution.x; x++)
        {
            screen.setPixel(x, y, pixel_array[y][x]);
        }
    }
}

std::vector<std::vector<glm::vec3>> boxBlur(const glm::ivec2 &windowResolution, std::vector<std::vector<glm::vec3>> &pixel_array,
                                            float threshold, int filter_size)
{
    // Create suitably sized 2D vector to house filtered pixel values.
    std::vector<std::vector<glm::vec3>> filtered_pixels;
    filtered_pixels.resize(windowResolution.y);

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    // Populate filtered_pixels with pixel values that match or exceed the threshold.
    for (int y = 0; y < windowResolution.y; y++)
    {
        for (int x = 0; x < windowResolution.x; x++)
        {
            float pixel_intensity = glm::length(pixel_array[y][x]);
            glm::vec3 filtered_value = pixel_intensity >= threshold ? pixel_array[y][x] : glm::vec3{0.0f};
            filtered_pixels[y].push_back(filtered_value);
        }
    }

    // Create suitably sized 2D vector to house blurred pixel values.
    std::vector<std::vector<glm::vec3>> blur_pixels;
    blur_pixels.resize(windowResolution.y);

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    // Process blur for every pixel in filtered_pixels.
    for (int y = 0; y < windowResolution.y; y++)
    {
        for (int x = 0; x < windowResolution.x; x++)
        {
            // Compute blur.
            glm::vec3 pixel_sum{0.0f};
            for (int filter_y = -filter_size; filter_y <= filter_size; filter_y++)
            {
                for (int filter_x = -filter_size; filter_x <= filter_size; filter_x++)
                {
                    int neighbour_y = y + filter_y;
                    int neighbour_x = x + filter_x;
                    if (!(neighbour_y < 0) && (neighbour_y < windowResolution.y) && !(neighbour_x < 0) && (neighbour_x < windowResolution.x))
                    {
                        pixel_sum += filtered_pixels[neighbour_y][neighbour_x];
                    }
                }
            }
            pixel_sum /= (2 * filter_size + 1) * (2 * filter_size + 1);
            blur_pixels[y].push_back(pixel_sum);
        }
    }
    return blur_pixels;
}

void bloomFilter(const glm::ivec2 &windowResolution, std::vector<std::vector<glm::vec3>> &pixel_array, float threshold, int blur_filter_size)
{
    std::vector<std::vector<glm::vec3>> blurred_pixels = boxBlur(windowResolution, pixel_array, threshold, blur_filter_size);

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    // Add values of blurred pixels to un-processed pixels.
    for (int y = 0; y < windowResolution.y; y++)
    {
        for (int x = 0; x < windowResolution.x; x++)
        {
            pixel_array[y][x] += blurred_pixels[y][x];
        }
    }
}

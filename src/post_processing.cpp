#include "post_processing.h"

const bool POST_PROCESS = true; // Enable/disable post-processing.

void postProcessingPipeline(Screen &screen, const glm::ivec2 &windowResolution, std::vector<std::vector<glm::vec3>> &pixel_array)
{
    if (POST_PROCESS)
    {
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
    // Create suitably sized 2D vector to house blurred pixel values.
    std::vector<std::vector<glm::vec3>> blur_array;
    blur_array.resize(windowResolution.y);

    // Process blur for every pixel.
    for (int y = 0; y < windowResolution.y; y++)
    {
        for (int x = 0; x < windowResolution.x; x++)
        {
            // Compute blur if pixel intensity exceeds threshold.   
            float pixel_intensity = glm::length(pixel_array[y][x]);
            if (pixel_intensity > threshold)
            {
                glm::vec3 pixel_sum = {0.0f, 0.0f, 0.0f};
                int valid_samples = 0;
                for (int filter_y = -filter_size; filter_y <= filter_size; filter_y++)
                {
                    for (int filter_x = -filter_size; filter_x <= filter_size; filter_x++)
                    {
                        int neighbour_y = y + filter_y;
                        int neighbour_x = x + filter_x;
                        if (!(neighbour_y < 0) && !(neighbour_x < 0))
                        {
                            pixel_sum += pixel_array[neighbour_y][neighbour_x];
                            valid_samples++;
                        }
                    }
                }
                pixel_sum /= valid_samples;
                blur_array[y][x] = pixel_sum;
            }
        }
    }
}

#include "post_processing.h"
#ifdef USE_OPENMP
#include <omp.h>
#endif

const bool POST_PROCESS = false; // Enable/disable post-processing.

// Bloom filter parameters.
const float BLOOM_THRESHOLD = 0.95f; // Threshold to use for filtering 'bright' pixels.
const int BLOOM_BLUR_SIZE = 3;       // Size to use for the box filter used for blurring.

// Depth of field filter parameters.
const int DOF_SAMPLE_SIZE = 1; // Determines no. of pixels to use for estimating focal point depth value (1=> 4 pixels, 2=> 16 pixels, 3=> 64, etc).
const int DOF_BLUR_SIZE = 3;   // Size to use for the box filter used for blurring.
const int DOF_BLUR_SKEW = 3;   // Skews the results of the Gaussian PDF used for computing blur blend. Set to 1 for no skew. Increase for more aggressive blur.

void postProcessingPipeline(Screen &screen, const glm::ivec2 &windowResolution,
                            std::vector<std::vector<glm::vec3>> &pixel_array, std::vector<std::vector<float>> &depth_array)
{
    if (POST_PROCESS)
    {
        DepthBuffer depth_buffer = processDepthBuffer(depth_array, windowResolution);
        depthOfFieldFilter(windowResolution, pixel_array, depth_buffer, DOF_BLUR_SIZE);
        bloomFilter(windowResolution, pixel_array, BLOOM_THRESHOLD, BLOOM_BLUR_SIZE);
    }

    // Render final pixel values to screen.
    #ifdef USE_OPENMP
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++)
    {
        for (int x = 0; x < windowResolution.x; x++)
        {
            screen.setPixel(x, y, pixel_array[y][x]);
        }
    }
}

bool isMax(float num) {return num > 3.4e38f;}

float gaussianPdf(float mean, float std, float value)
{
    float lhs = 1.0f / (std * glm::root_two_pi<float>());
    float rhs_exp = -0.5f * std::pow((value - mean) / std, 2.0f);
    return lhs * std::exp(rhs_exp);
}

std::vector<std::vector<glm::vec3>> boxBlur(const glm::ivec2 &windowResolution, std::vector<std::vector<glm::vec3>> &pixel_array,
                                            float threshold, int filter_size)
{
    // Create suitably sized 2D vector to house filtered pixel values.
    std::vector<std::vector<glm::vec3>> filtered_pixels;
    filtered_pixels.resize(windowResolution.y);

    // Populate filtered_pixels with pixel values that match or exceed the threshold.
    #ifdef USE_OPENMP
    #pragma omp parallel for schedule(guided)
    #endif
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

    // Process blur for every pixel in filtered_pixels.
    #ifdef USE_OPENMP
    #pragma omp parallel for schedule(guided)
    #endif
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

    // Add values of blurred pixels to un-processed pixels.
    #ifdef USE_OPENMP
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++)
    {
        for (int x = 0; x < windowResolution.x; x++)
        {
            pixel_array[y][x] += blurred_pixels[y][x];
        }
    }
}

DepthBuffer processDepthBuffer(std::vector<std::vector<float>> &depth_array, const glm::ivec2 &windowResolution)
{   
    // Compute necessary constants.
    int sample_edge_length = std::pow(2, DOF_SAMPLE_SIZE);
    int initial_y = (windowResolution.y  - sample_edge_length) / 2;
    int initial_x = (windowResolution.x  - sample_edge_length) / 2;

    // Compute average depth value of pixels in 'selection box'
    float sample_average = 0.0f;
    #ifdef USE_OPENMP
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y_offset = 0; y_offset < sample_edge_length; y_offset++)
    {
        int current_y = initial_y + y_offset;
        for (int x_offset = 0; x_offset < sample_edge_length; x_offset++)
        {
            int current_x = initial_x + x_offset;
            sample_average += depth_array[current_y][current_x];
        }
    }
    sample_average /= (sample_edge_length * sample_edge_length);

    // Compute standard deviation of all valid values (i.e: not float::max()) in depth buffer relative to the mean of the 'focal point' computed earlier.
    int valid_samples = 0;
    #ifdef USE_OPENMP
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++)
    {
        for (int x = 0; x < windowResolution.x; x++)
        {
            float depth_value = depth_array[y][x];
            if (!isMax(depth_value)) {valid_samples++;}
        }
    }
    float std_dev = 0.0f;
    #ifdef USE_OPENMP
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++)
    {
        for (int x = 0; x < windowResolution.x; x++)
        {
            float depth_value = depth_array[y][x];
            if (!isMax(depth_value)) {std_dev += depth_value / valid_samples;}
        }
    }

    return DepthBuffer{
        windowResolution.x,
        windowResolution.y,
        sample_average,
        std_dev,
        depth_array
    };
}

void depthOfFieldFilter(const glm::ivec2 &windowResolution, std::vector<std::vector<glm::vec3>> &pixel_array, DepthBuffer &depth_buffer, int blur_filter_size)
{
    // Blur all viewport pixels.
    std::vector<std::vector<glm::vec3>> blurred_pixels = boxBlur(windowResolution, pixel_array, 0.0f, blur_filter_size);

    // Compute a skewed standard deviation for more or less aggressive blurring and compute probability value for 'focal point'.
    float skewed_std = depth_buffer.std_dev * (1.0f / DOF_BLUR_SKEW);
    float focal_prob = gaussianPdf(depth_buffer.sample_mean, skewed_std, depth_buffer.sample_mean);

    // Blend blurred and unblurred values based on Gaussian distribution.
    #ifdef USE_OPENMP
    #pragma omp parallel for schedule(guided)
    #endif
    for (int y = 0; y < windowResolution.y; y++)
    {
        for (int x = 0; x < windowResolution.x; x++)
        {
            float sample_probability = gaussianPdf(depth_buffer.sample_mean, skewed_std, depth_buffer.depth_values[y][x]);
            float sharpness_factor =  focal_prob > 0.0f ? (sample_probability / focal_prob) : 0.0f;
            pixel_array[y][x] =  (pixel_array[y][x] * sharpness_factor) + (blurred_pixels[y][x] * (1.0f - sharpness_factor));
        }
    }
}

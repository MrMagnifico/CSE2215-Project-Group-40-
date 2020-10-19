#include "lighting.h"
#include "disable_all_warnings.h"
// Suppress warnings in third-party code.
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
#include <glm/gtc/constants.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>
#include <limits>

const int RECURSION_LIMIT = 5;      // Defines the maximal level of recursion to use.
const float RAY_STEP = 1.0e-3f;     // Defines the 'step' to take in a ray's direction to prevent erroneous intersections.
const int SPHERE_SAMPLE_LIMIT = 10; // Defines the maximal number of samples to compute when sampling a spherical light source.

glm::vec3 phongDiffuseOnly(const HitInfo &hitInfo, const glm::vec3 &vertexPos, const glm::vec3 &lightPos)
{
    glm::vec3 normalized = glm::normalize(hitInfo.normal);
    glm::vec3 L = glm::normalize(lightPos - vertexPos);
    return hitInfo.material.kd * glm::max(glm::dot(normalized, L), 0.f);
}

glm::vec3 phongSpecularOnly(const HitInfo &hitInfo, const glm::vec3 &vertexPos, const glm::vec3 &lightPos, const glm::vec3 &cameraPos)
{
    glm::vec3 normalized = glm::normalize(hitInfo.normal);
    glm::vec3 L = glm::normalize(lightPos - vertexPos);
    glm::vec3 V = glm::normalize(cameraPos - vertexPos);
    glm::vec3 R = glm::normalize(2 * glm::dot(L, normalized) * normalized - L);
    return hitInfo.material.ks * glm::pow(glm::max(glm::dot(R, V), 0.f), hitInfo.material.shininess);
}

std::vector<glm::vec3> randomPointOnSphere(const SphericalLight &sphere, const int &amount)
{
    // Use fibonacci sequence to calculate a vector of unifromly distributed points on the sphere.
    std::vector<glm::vec3> uniformPoints;
    float phi = glm::pi<float>() * (3.0f - sqrt(5.0f));

    for (int i = 0; i < amount; i++)
    {
        float y = 1.0f - (i / float(amount - 1)) * 2.0f;
        float radius = sqrt(1.0f - pow(y, 2));
        float theta = phi * i;
        float x = cos(theta) * radius;
        float z = sin(theta) * radius;

        // Multiply by the sphere of the radius to scale to any given sphere.
        x *= sphere.radius;
        y *= sphere.radius;
        z *= sphere.radius;
        glm::vec3 randPoint = {x, y, z};
        // Add the sphere position to move to any given spheres position
        randPoint += sphere.position;
        uniformPoints.push_back(randPoint);
    }
    return uniformPoints;
}

bool checkRightHalf(const SphericalLight &sphere, const glm::vec3 &point, const float &distanceOfRay)
{
    glm::vec3 centerToP = sphere.position - point;
    float lengthOfCenterToP = sqrt(pow(centerToP.x, 2) + pow(centerToP.y, 2) + pow(centerToP.z, 2));
    float c = pow(lengthOfCenterToP, 2) + pow(sphere.radius, 2);
    c = sqrt(c);
    if (c <= distanceOfRay)
        return true;
    return false;
}

glm::vec3 softShadow(const HitInfo &hitInfo, const Ray &ray, const SphericalLight &light, const BoundingVolumeHierarchy &bvh)
{
    int valid_samples = 0;
    glm::vec3 final_lighting = {0.0f, 0.0f, 0.0f};
    std::vector<glm::vec3> sample_points = randomPointOnSphere(light, SPHERE_SAMPLE_LIMIT);
    for (glm::vec3 point_position : sample_points)
    {
        glm::vec3 intersection_point = ray.origin + (ray.t * ray.direction);
        float intersection_to_point_dist = glm::distance(point_position, intersection_point);
        if (!checkRightHalf(light, point_position, intersection_to_point_dist)) {continue;}

        PointLight point_sample = {point_position, light.color};
        if (!shadowRay(ray, point_sample, bvh))
        {
            valid_samples++;
            final_lighting += (phongDiffuseOnly(hitInfo, intersection_point, point_position) + phongSpecularOnly(hitInfo, intersection_point, point_position, ray.origin));
        };
    }
    if (valid_samples == 0) {return final_lighting;}
    return final_lighting * (1.0f / float(valid_samples));
}

glm::vec3 lightRay(const Ray &ray, const HitInfo &hitInfo, const Scene &scene, const BoundingVolumeHierarchy &bvh)
{
    // Calculate the point of intersection.
    glm::vec3 p = ray.origin + ray.t * ray.direction;

    // Draw a blue debug ray if the ray hit.
    drawRay(ray, glm::vec3{0.0f, 0.0f, 1.0f});

    // Draw a green debug ray for the normal if the ray hit.
    debugRay(p, hitInfo.normal, 1.0f, glm::vec3{0.0f, 1.0f, 0.0f});

    // Calculate the lighting considering each point light source.
    glm::vec3 lighting = glm::vec3{0.0f};
    for (const PointLight &light : scene.pointLights)
    {
        if (!shadowRay(ray, light, bvh))
        {
            lighting = lighting + light.color * (phongDiffuseOnly(hitInfo, p, light.position) + phongSpecularOnly(hitInfo, p, light.position, ray.origin));
        }
    }
    for (const SphericalLight &light : scene.sphericalLight)
    {
        lighting = lighting + light.color * softShadow(hitInfo, ray, light, bvh);
    }
    return lighting;
}

bool shadowRay(const Ray &ray, const PointLight &light, const BoundingVolumeHierarchy &bvh)
{
    // Calculate the point of intersection and the shadow ray direction.
    glm::vec3 p = ray.origin + ray.t * ray.direction;
    glm::vec3 ray_direction = glm::normalize(light.position - p);

    // Construct shadow ray
    Ray shadowRay = {p + (RAY_STEP * ray_direction), ray_direction, glm::distance(p, light.position)};

    // Draw a red debug ray if the shadow ray hits another source.
    HitInfo hitInfo;
    if (bvh.intersect(shadowRay, hitInfo))
    {
        drawRay(shadowRay, glm::vec3{1.0f, 0.0f, 0.0f});
        return true;
    }

    // Draw a yellow debug ray if there is no intersection with another object
    drawRay(shadowRay, glm::vec3{1.0f, 1.0f, 0.0f});
    return false;
}

glm::vec3 recursiveRayTrace(const Ray &intersectionRay, const HitInfo &hitInfo, const Scene &scene,
                            const BoundingVolumeHierarchy &bvh, int rayLevel)
{
    if (rayLevel < RECURSION_LIMIT) // Control the recursion level.
    {
        if (hitInfo.material.ks != glm::vec3{0.0f}) // Check if current intersection surface is specular.
        {
            // Construct the mirror reflection ray.
            glm::vec3 reverse_incidence_direction = glm::normalize(glm::vec3{-intersectionRay.direction.x, -intersectionRay.direction.y, -intersectionRay.direction.z}); // TODO: Normalisation could potentially be removed for optimisation.
            glm::vec3 normalised_normal = glm::normalize(hitInfo.normal);
            glm::vec3 reflection_ray_direction = (2.0f * glm::dot(reverse_incidence_direction, normalised_normal) * normalised_normal) - reverse_incidence_direction;
            Ray reflection_ray = {
                glm::vec3{intersectionRay.origin + (intersectionRay.t * intersectionRay.direction) + (RAY_STEP * reflection_ray_direction)}, // Slight offset in the reflection ray direction to prevent intersection with intersectionRay's intersection point.
                reflection_ray_direction,
                std::numeric_limits<float>::max()};

            // Compute new ray lighting data and recursively add lighting data of subsequent mirror reflection rays.
            HitInfo newRayInfo;
            if (bvh.intersect(reflection_ray, newRayInfo))
            {
                return lightRay(intersectionRay, hitInfo, scene, bvh) + (hitInfo.material.ks * recursiveRayTrace(reflection_ray, newRayInfo, scene, bvh, ++rayLevel));
            }
        }
        return lightRay(intersectionRay, hitInfo, scene, bvh);
    }
    return glm::vec3{0.0f};
}

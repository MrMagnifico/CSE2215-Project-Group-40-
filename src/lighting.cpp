#include "lighting.h"
#include "disable_all_warnings.h"
// Suppress warnings in third-party code.
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>
#include <limits>

const int RECURSION_LIMIT = 5; //Defines the maximal level of recursion to use
const float RECURSIVE_RAY_STEP = 1.0e-3f;

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

glm::vec3 shadeRay(const Ray &ray, const HitInfo &hitInfo, const Scene &scene)
{
    // Draw a blue debug ray if the ray hit.
    drawRay(ray, glm::vec3(0.0f, 0.0f, 1.0f));

    // Draw a green normal.
    glm::vec3 p = ray.origin + ray.t * ray.direction;
    intersectionNormal(p, hitInfo.normal, 1.0f);

    // Calculate the lighting considering each point light source.
    glm::vec3 lighting = glm::vec3{0.0f};
    for (PointLight light : scene.pointLights)
    {
        lighting = lighting + light.color * (phongDiffuseOnly(hitInfo, p, light.position) + phongSpecularOnly(hitInfo, p, light.position, ray.origin));
    }
    return lighting;
}

glm::vec3 recursiveRayTrace(const Ray &intersectionRay, const HitInfo &hitInfo, const Scene &scene,
                            const BoundingVolumeHierarchy &bvh, int rayLevel)
{
    if (rayLevel < RECURSION_LIMIT) //Control recursion level
    {   
        if (hitInfo.material.ks != glm::vec3{0.0f}) //Check if current intersection surface is specular
        {
            //Construct the mirror reflection ray
            glm::vec3 reverse_incidence_direction = glm::normalize(glm::vec3{-intersectionRay.direction.x, -intersectionRay.direction.y, -intersectionRay.direction.z}); //TODO:Normalisation could potentially be removed for optimisation
            glm::vec3 normalised_normal = glm::normalize(hitInfo.normal);
            glm::vec3 reflection_ray_direction = (2.0f * glm::dot(reverse_incidence_direction, normalised_normal) * normalised_normal) - reverse_incidence_direction;
            Ray reflection_ray = {
                glm::vec3{intersectionRay.origin + (intersectionRay.t * intersectionRay.direction) + (RECURSIVE_RAY_STEP * reflection_ray_direction)}, //Slight offset in reflection ray direction to prevent intersection with intersectionRay's intersection point
                reflection_ray_direction,
                std::numeric_limits<float>::max()};

            //Compute new ray lighting data and recursively add lighting data of subsequent mirror reflection rays
            HitInfo newRayInfo;
            if (bvh.intersect(reflection_ray, newRayInfo))
            {   
                return shadeRay(intersectionRay, hitInfo, scene) + (hitInfo.material.ks * recursiveRayTrace(reflection_ray, newRayInfo, scene, bvh, ++rayLevel));
            }
        }
        return shadeRay(intersectionRay, hitInfo, scene);
    }
    return glm::vec3{0.0f};
}

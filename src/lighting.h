#pragma once
#include "disable_all_warnings.h"
#include "ray_tracing.h"
#include "ray_debug.h"
#include "bounding_volume_hierarchy.h"
DISABLE_WARNINGS_PUSH()
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()

glm::vec3 phongDiffuseOnly(const HitInfo &hitInfo, const glm::vec3 &vertexPos, const glm::vec3 &lightPos);
glm::vec3 phongSpecularOnly(const HitInfo &hitInfo, const glm::vec3 &vertexPos, const glm::vec3 &lightPos, const glm::vec3 &cameraPos);

//Returns a vector representing the RGB values computed by the Phong reflection model.
glm::vec3 shadeRay(const Ray &ray, const HitInfo &hitInfo, const Scene &scene);

//Recursively computes the appropriate RGB value of a given ray.
glm::vec3 recursiveRayTrace(const Ray &intersection_ray, const HitInfo &hitInfo, const Scene &scene,
                            const BoundingVolumeHierarchy &bvh, int rayLevel);

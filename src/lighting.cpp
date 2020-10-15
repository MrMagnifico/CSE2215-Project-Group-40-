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

glm::vec3 phongDiffuseOnly(const HitInfo& hitInfo, const glm::vec3& vertexPos, const glm::vec3& lightPos)
{
    glm::vec3 normalized = glm::normalize(hitInfo.normal);
    glm::vec3 L = glm::normalize(lightPos - vertexPos);
    return hitInfo.material.kd * glm::max(glm::dot(normalized, L), 0.f);
}

glm::vec3 phongSpecularOnly(const HitInfo& hitInfo, const glm::vec3& vertexPos, const glm::vec3& lightPos, const glm::vec3& cameraPos)
{
    glm::vec3 normalized = glm::normalize(hitInfo.normal);
    glm::vec3 L = glm::normalize(lightPos - vertexPos);
    glm::vec3 V = glm::normalize(cameraPos - vertexPos);
    glm::vec3 R = glm::normalize(2 * glm::dot(L, normalized) * normalized - L);
    return hitInfo.material.ks * glm::pow(glm::max(glm::dot(R, V), 0.f), hitInfo.material.shininess);
}
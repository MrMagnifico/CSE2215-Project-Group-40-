#pragma once
#include "disable_all_warnings.h"
#include "ray_tracing.h"
DISABLE_WARNINGS_PUSH()
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()

glm::vec3 phongDiffuseOnly(const HitInfo& hitInfo, const glm::vec3& vertexPos, const glm::vec3& lightPos);
glm::vec3 phongSpecularOnly(const HitInfo& hitInfo, const glm::vec3& vertexPos, const glm::vec3& lightPos, const glm::vec3& cameraPos);

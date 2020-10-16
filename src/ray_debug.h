#pragma once
#include "disable_all_warnings.h"
#include "ray.h"
#include "draw.h"
DISABLE_WARNINGS_PUSH()
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()

Ray debugRay(glm::vec3 origin, glm::vec3 direction, float t, glm::vec3 colour);

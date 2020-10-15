#include "ray_debug.h"
#include "disable_all_warnings.h"
#include "draw.h"
// Suppress warnings in third-party code.
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>

void intersectionNormal(glm::vec3 origin, glm::vec3 direction, float t)
{
	Ray ray;
	ray.origin = origin;
	ray.direction = direction;
	ray.t = t;
	glm::vec3 colour{ 1.0f };
	drawRay(ray, colour);
}
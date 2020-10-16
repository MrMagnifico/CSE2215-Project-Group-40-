#include "ray_debug.h"
#include "disable_all_warnings.h"
// Suppress warnings in third-party code.
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>

Ray debugRay(glm::vec3 origin, glm::vec3 direction, float t, glm::vec3 colour)
{
	Ray ray;
	ray.origin = origin;
	ray.direction = direction;
	ray.t = t;
	drawRay(ray, colour);
	return ray;
}
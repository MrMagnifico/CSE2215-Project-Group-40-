#include "ray_tracing.h"
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

bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p)
{
    glm::vec3 vector;

    // compute the sign of edge 0
    vector = glm::cross(v1 - v0, p - v0);
    float sign0 = glm::dot(n, vector);

    // compute the sign of edge 1
    vector = glm::cross(v2 - v1, p - v1);
    float sign1 = glm::dot(n, vector);

    // compute the sign of edge 2
    vector = glm::cross(v0 - v2, p - v2);
    float sign2 = glm::dot(n, vector);

    // check if all the signs match
    if ((sign0 >= 0 && sign1 >= 0 && sign2 >= 0) || (sign0 <= 0 && sign1 <= 0 && sign2 <= 0)) {
        return true;
    }

    return false;
}

bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    // compute the dot product
    float dot = glm::dot(plane.normal, ray.direction);
    if (dot == 0.0f) {
        return false;
    }

    // compute the ray's t
    float t = (plane.D - glm::dot(plane.normal, ray.origin)) / dot;
    if (t < 0) {
        return false;
    }

    // compute the intersection point
    glm::vec3 p = ray.origin + t * ray.direction;

    // compute the distance of the old and current t
    float old = glm::distance(ray.origin, ray.origin + ray.t * ray.direction);
    float current = glm::distance(ray.origin, p);
    if (old < current) {
        return false;
    }

    // update the ray's t
    ray.t = t;

    return true;
}

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    Plane plane;

    // compute the plane's n
    plane.normal = glm::cross(v1 - v0, v2 - v0);

    // compute the plane's d
    plane.D = glm::dot(plane.normal, v0);

    return plane;
}

/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray, HitInfo& hitInfo)
{
    Plane plane = trianglePlane(v0, v1, v2);

    float tOld = ray.t;

    if (!intersectRayWithPlane(plane, ray)) {
        return false;
    }

    glm::vec3 p = ray.origin + ray.t * ray.direction;

    if (!pointInTriangle(v0, v1, v2, plane.normal, p)) {
        ray.t = tOld;
        return false;
    }
    return true;
}

/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo)
{
    glm::vec3 origin = ray.origin - sphere.center;
    float A = glm::pow(ray.direction.x, 2) + glm::pow(ray.direction.y, 2) + glm::pow(ray.direction.z, 2);
    float B = 2 * (ray.direction.x * origin.x + ray.direction.y * origin.y + ray.direction.z * origin.z);
    float C = glm::pow(origin.x, 2) + glm::pow(origin.y, 2) + glm::pow(origin.z, 2) - glm::pow(sphere.radius, 2);
    float discriminant = glm::pow(B, 2) - 4 * A * C;

    if (discriminant < 0) {
        return false;
    }

    float t0 = (-B + glm::sqrt(discriminant)) / 2 * A;
    float t1 = (-B - glm::sqrt(discriminant)) / 2 * A;

    ray.t = glm::min(t0, t1);

    return true;
}

/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{
    float txmin = (box.lower.x - ray.origin.x) / ray.direction.x;
    float txmax = (box.upper.x - ray.origin.x) / ray.direction.x;

    float tymin = (box.lower.y - ray.origin.y) / ray.direction.y;
    float tymax = (box.upper.y - ray.origin.y) / ray.direction.y;

    float tzmin = (box.lower.z - ray.origin.z) / ray.direction.z;
    float tzmax = (box.upper.z - ray.origin.z) / ray.direction.z;

    float tinx = glm::min(txmin, txmax);
    float tiny = glm::min(tymin, tymax);
    float tinz = glm::min(tzmin, tzmax);

    float toutx = glm::max(txmin, txmax);
    float touty = glm::max(tymin, tymax);
    float toutz = glm::max(tzmin, tzmax);

    float tin = glm::max(glm::max(tinx, tiny), tinz);
    float tout = glm::min(glm::min(toutx, touty), toutz);

    if (tin > tout || tout < 0) {
        return false;
    }

    ray.t = tin;

    return true;
}
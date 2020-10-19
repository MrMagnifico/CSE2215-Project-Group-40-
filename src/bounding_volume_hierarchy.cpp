#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <glm/vector_relational.hpp>
#include <limits>

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene, int maxLevel, int binNum)
    : m_pScene(pScene)
{
    max_level = maxLevel;
    bin_num = binNum;
    root_node = constructNode(*pScene, 0);
}

// Use this function to visualize your BVH. This can be useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDraw(int level)
{
    // Draw the AABB as a transparent green box.
    //AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawShape(aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);

    // Draw the AABB as a (white) wireframe box.
    AxisAlignedBox aabb { glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawAABB(aabb, DrawMode::Wireframe);
    drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1);
}

int BoundingVolumeHierarchy::numLevels() const
{
    return 5;
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
{
    bool hit = false;
    // Intersect with all triangles of all meshes.
    for (const auto& mesh : m_pScene->meshes) {
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
            if (intersectRayWithTriangle(v0.p, v1.p, v2.p, ray, hitInfo)) {
                hitInfo.material = mesh.material;
                hit = true;
            }
        }
    }
    // Intersect with spheres.
    for (const auto& sphere : m_pScene->spheres)
        hit |= intersectRayWithShape(sphere, ray, hitInfo);
    return hit;
}

BVHNode BoundingVolumeHierarchy::constructNode(Scene &scene, int axis_selector, int current_level)
{
    // Define and alternate axis to split on.
    glm::vec3 comparison_axis = (axis_selector % 2 == 0) ? glm::vec3{1.0f, 0.0f, 0.0f} : glm::vec3{0.0f, 1.0f, 0.0f};
    glm::vec3 other_axis = (axis_selector % 2 == 1) ? glm::vec3{1.0f, 0.0f, 0.0f} : glm::vec3{0.0f, 1.0f, 0.0f};

    // Compute borders of the axes.
    std::pair<glm::vec3, glm::vec3> comparison_limits = {
        glm::vec3{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()} * comparison_axis,
        glm::vec3{std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min()} * comparison_axis};
    std::pair<glm::vec3, glm::vec3> other_limits = {
        glm::vec3{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()} * other_axis,
        glm::vec3{std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min()} * other_axis};
    for (Mesh &mesh : scene.meshes)
    {
        for (Triangle &triangle : mesh.triangles)
        {
            for (int counter = 0; counter < 3; counter++)
            {
                glm::vec3 vertex_pos = mesh.vertices[triangle[counter]].p;
                //TODO: finish border computation
            }
        }
    } 

    if (current_level < --max_level) // Control hierarchy depth.
    {

    }

    /////////////////////////////////////////////////////////

    // Create leaf node from given scene vertices.
    std::vector<Vertex> bvh_vertices;
    for (Mesh &mesh : scene.meshes)
    {
        for (Triangle &triangle : mesh.triangles)
        {
            bvh_vertices.push_back(mesh.vertices[triangle[0]]);
            bvh_vertices.push_back(mesh.vertices[triangle[1]]);
            bvh_vertices.push_back(mesh.vertices[triangle[2]]);
        }
    } 
}

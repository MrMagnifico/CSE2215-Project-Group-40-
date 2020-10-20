#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <glm/vector_relational.hpp>
#include <limits>

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene, int max_level, int bin_num)
    : m_pScene(pScene)
{
    maxLevel = max_level;
    binNum = binNum;

    // Construct indices for all triangles and construct parent node.
    std::vector<std::pair<int, std::vector<int>>> all_mesh_triangle_indices;
    for (int mesh_index = 0; mesh_index < pScene->meshes.size(); mesh_index++)
    {
        Mesh& curr_mesh = pScene->meshes[mesh_index];
        std::pair<int, std::vector<int>> mesh_triangle_pairs = {mesh_index, std::vector<int>{}};
        for (int triangle_index = 0; triangle_index < curr_mesh.triangles.size(); triangle_index++)
        {
            mesh_triangle_pairs.second.push_back(triangle_index);
        }
        all_mesh_triangle_indices.push_back(mesh_triangle_pairs);
    }
    constructNode(*pScene, all_mesh_triangle_indices, 0, 0, 1);
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

int BoundingVolumeHierarchy::constructNode(Scene &scene, std::vector<std::pair<int, std::vector<int>>> meshTriangleIndices,
                                           int node_index, int axis_selector, int current_level)
{
    // Define and alternate axis to split on.
    int comparison_axis_selector = current_level % 2;

    // Compute borders of the axes.
    std::vector<std::pair<float, float>> limits = {
        std::pair<float, float>{std::numeric_limits<float>::max(), std::numeric_limits<float>::min()},
        std::pair<float, float>{std::numeric_limits<float>::max(), std::numeric_limits<float>::min()},
        std::pair<float, float>{std::numeric_limits<float>::max(), std::numeric_limits<float>::min()}};
    for (std::pair<int, std::vector<int>> &coordinate_pairs : meshTriangleIndices)
    {
        Mesh& current_mesh = scene.meshes[coordinate_pairs.first];
        for (int triangle_index : coordinate_pairs.second)
        {
            Triangle current_triangle = current_mesh.triangles[triangle_index];
            std::vector<Vertex> vertices = {
                current_mesh.vertices[current_triangle[0]],
                current_mesh.vertices[current_triangle[1]],
                current_mesh.vertices[current_triangle[2]]};
            for (Vertex vertex : vertices)
            {
                glm::vec3 vertex_pos = vertex.p;
                if (vertex_pos.x < limits[0].first) {limits[0].first = vertex_pos.x;}
                if (vertex_pos.x > limits[0].second) {limits[0].second = vertex_pos.x;}
                if (vertex_pos.y < limits[1].first) {limits[1].first = vertex_pos.y;}
                if (vertex_pos.y > limits[1].second) {limits[1].second = vertex_pos.y;}
                if (vertex_pos.z < limits[2].first) {limits[2].first = vertex_pos.z;}
                if (vertex_pos.z > limits[2].second) {limits[2].second = vertex_pos.z;}
            }
        }
    }

    // Construct inner node if applicable
    if (current_level < maxLevel)
    {
        //TODO: Actually do BVH splitting
    }

    // Construct leaf node if maxLevel reached
    AxisAlignedBox node_box = {
        glm::vec3{limits[0].first, limits[1].first, limits[2].first},
        glm::vec3{limits[0].second, limits[1].second, limits[2].second}};
    BVHNode constructed_node = {node_box, true, maxLevel, std::pair<int, int>{}, meshTriangleIndices};
    nodeVector.push_back(constructed_node);
    return (nodeVector.size() - 1);
}

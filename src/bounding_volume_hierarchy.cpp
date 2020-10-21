#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <glm/vector_relational.hpp>
#include <limits>
#include <iostream>

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene *pScene, int max_level, int bin_num)
    : m_pScene(pScene)
{
    maxLevel = max_level;
    binNum = bin_num;

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
    constructNode(*pScene, all_mesh_triangle_indices, 1);
}

// Use this function to visualize your BVH. This can be useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDraw(int level)
{
    for (BVHNode node : nodeVector)
    {
        if (node.level <= level)
        {
            drawAABB(node.boundingBox, DrawMode::Wireframe);
        }
    }
}

int BoundingVolumeHierarchy::numLevels() const
{
    int max_level = 1;
    for (BVHNode node : nodeVector)
    {
        if (node.level > max_level)
        {
            max_level = node.level;
        }
    }
    return max_level;
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

int BoundingVolumeHierarchy::constructNode(Scene &scene, std::vector<std::pair<int, std::vector<int>>> &meshTriangleIndices, int current_level)
{
    std::vector<std::pair<float, float>> limits = computeBoundingBoxLimits(scene, meshTriangleIndices);
    int new_node_index = nodeVector.size();

    if (current_level < maxLevel)
    {
        // Construct inner node if applicable.
        std::pair<std::vector<std::pair<int, std::vector<int>>>, std::vector<std::pair<int, std::vector<int>>>> child_triangle_meshes
        = computeOptimalSplit(scene, meshTriangleIndices, limits, current_level);

        AxisAlignedBox node_box = {
            glm::vec3{limits[0].first, limits[1].first, limits[2].first},
            glm::vec3{limits[0].second, limits[1].second, limits[2].second}};
        BVHNode constructed_node = {
            node_box,
            false,
            current_level,
            std::pair<int, int>{constructNode(scene, child_triangle_meshes.first, current_level + 1), constructNode(scene, child_triangle_meshes.second, current_level + 1)},
            std::vector<std::pair<int, std::vector<int>>>{}};
        nodeVector.push_back(constructed_node);
    } else {
        // Construct leaf node if maxLevel reached.
        AxisAlignedBox node_box = {
        glm::vec3{limits[0].first, limits[1].first, limits[2].first},
        glm::vec3{limits[0].second, limits[1].second, limits[2].second}};
        BVHNode constructed_node = {node_box, true, current_level, std::pair<int, int>{}, meshTriangleIndices};
        nodeVector.push_back(constructed_node);
    }
    return new_node_index;
}

std::vector<std::pair<float, float>> BoundingVolumeHierarchy::computeBoundingBoxLimits(Scene &scene,
                                                                                       std::vector<std::pair<int, std::vector<int>>> &meshTriangleIndices)
{
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
            for (Vertex &vertex : vertices)
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
    return limits;
}

std::pair<std::vector<std::pair<int, std::vector<int>>>, std::vector<std::pair<int, std::vector<int>>>>
BoundingVolumeHierarchy::computeOptimalSplit(Scene &scene, std::vector<std::pair<int, std::vector<int>>> meshTriangleIndices,
                                             std::vector<std::pair<float, float>> limits, int current_level)
{
    // Define and alternate axis to split on.
    int comparison_axis = current_level % 3;
    int secondary_axis = (current_level + 1) % 3;
        
    // Compute atomic bin area (smallest area permissible by binNum).
    float comparison_edge_length = limits[comparison_axis].second - limits[comparison_axis].first;
    float secondary_edge_length = limits[secondary_axis].second - limits[secondary_axis].first;
    float atomic_bin_area = (comparison_edge_length * secondary_edge_length) / binNum;

    float min_bin_cost = std::numeric_limits<float>::max();
    std::pair<std::vector<std::pair<int, std::vector<int>>>, std::vector<std::pair<int, std::vector<int>>>> left_right_split_indices;
    for (int bin = 1; bin < binNum; bin++)
    {
        float lhs_area = atomic_bin_area * bin;
        float rhs_area = atomic_bin_area * (binNum - bin);
        float split_boundary = limits[comparison_axis].first + (comparison_edge_length * (bin / binNum)); // 'Walk' an appropriate distance from the comparison axis' min. coordinate.

        int left_triangle_count = 0;
        int right_triangle_count = 0;

        // Create vectors to store LHS and RHS mesh-triangle indices.
        std::vector<std::pair<int, std::vector<int>>> lhs_indices;
        std::vector<std::pair<int, std::vector<int>>> rhs_indices;

        // Compute which triangles belong in which side of the bin.
        for (std::pair<int, std::vector<int>> &coordinate_pair : meshTriangleIndices)
        {
            // Create an empty vector for the current mesh for each corresponding split vector.
            Mesh &current_mesh = scene.meshes[coordinate_pair.first];
            std::pair<int, std::vector<int>> lhs_mesh_indices = {coordinate_pair.first, std::vector<int>{}};
            std::pair<int, std::vector<int>> rhs_mesh_indices = {coordinate_pair.first, std::vector<int>{}};
            
            // Populate the triangle index vector for the current mesh appropriately for each split vector.
            for (int triangle_index : coordinate_pair.second)
            {
                Triangle &current_triangle = current_mesh.triangles[triangle_index];
                int position_info = checkTriangleBorderSide(scene, current_mesh, current_triangle, comparison_axis, split_boundary);

                switch (position_info) {
                    case 0:
                        left_triangle_count++;
                        lhs_mesh_indices.second.push_back(triangle_index);
                        break;
                    case 1:
                        right_triangle_count++;
                        rhs_mesh_indices.second.push_back(triangle_index);
                        break;
                    case 2:
                        left_triangle_count++;
                        lhs_mesh_indices.second.push_back(triangle_index);
                        right_triangle_count++;
                        rhs_mesh_indices.second.push_back(triangle_index);
                        break;
                }
            }

            // Add indices of this mesh's triangles to mesh-triangle indices vectors.
            lhs_indices.push_back(lhs_mesh_indices);
            rhs_indices.push_back(rhs_mesh_indices);
        }

        // Assess cost function and utilise or dispose of bin appropriately.
        float cost = (traversalCost) + (lhs_area * left_triangle_count * triangleIntersectionCost) + (rhs_area * right_triangle_count * triangleIntersectionCost);
        if (cost < min_bin_cost)
        {
            min_bin_cost = cost;
            left_right_split_indices.first = lhs_indices;
            left_right_split_indices.second = rhs_indices;
        }
        std::cout << "Bin: " << bin << " - Cost: " << cost << std::endl; //FOR DEBUGGING
    }

    return left_right_split_indices;
}

int BoundingVolumeHierarchy::checkTriangleBorderSide(Scene &scene, Mesh &mesh, Triangle &triangle, int comparison_axis, float split_boundary)
{
    std::vector<Vertex> vertices = {mesh.vertices[triangle[0]], mesh.vertices[triangle[1]], mesh.vertices[triangle[2]]};
    bool isLeft = true;
    bool isRight = true;

    for (Vertex &vertex : vertices)
    {
        if (vertex.p[comparison_axis] <= split_boundary) {isRight = false;}
        if (vertex.p[comparison_axis] >= split_boundary) {isLeft = false;}
    }
    if (!isLeft && !isRight) {return 2;}
    else if (isRight) {return 1;}
    else {return 0;}
}

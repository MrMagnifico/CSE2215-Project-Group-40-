#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <glm/vector_relational.hpp>
#include <limits>
#include <iostream> 

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene *pScene, int max_level, int min_triangles, int sah_bins)
    : m_pScene(pScene)
{
    maxLevel = max_level;
    minTriangles = min_triangles;
    sahBins = sah_bins;

    // Construct indices for all triangles and construct parent node.
    std::vector<std::pair<int, std::vector<int>>> all_mesh_triangle_indices;
    for (int mesh_index = 0; mesh_index < pScene->meshes.size(); mesh_index++)
    {
        Mesh &curr_mesh = pScene->meshes[mesh_index];
        std::pair<int, std::vector<int>> mesh_triangle_pairs = {mesh_index, std::vector<int>{}};
        for (int triangle_index = 0; triangle_index < curr_mesh.triangles.size(); triangle_index++)
        {
            mesh_triangle_pairs.second.push_back(triangle_index);
        }
        all_mesh_triangle_indices.push_back(mesh_triangle_pairs);
    }
    constructNode(*pScene, all_mesh_triangle_indices, 1);
}

void BoundingVolumeHierarchy::debugDraw(int level)
{
    for (BVHNode &node : nodeVector)
    {
        if (node.level == level) {drawAABB(node.boundingBox, DrawMode::Wireframe);}
    }
}

int BoundingVolumeHierarchy::numLevels()
{
    int max_level = 1;
    for (BVHNode &node : nodeVector)
    {
        if (node.level > max_level) {max_level = node.level;}
    }
    return max_level;
}

bool  BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) {
    return intersectRecurseMethod(ray, hitInfo, nodeVector[0]);
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
bool BoundingVolumeHierarchy::intersectRecurseMethod(Ray& ray, HitInfo& hitInfo, BVHNode& node)
{
    bool hit = false; 
    if (!intersectRayWithShape(node.boundingBox, ray)) {
        return hit; 
    }

    if (node.isLeaf) {
        for (const std::pair<int, std::vector<int>> x : node.triangleChildrenIndices) {
            std::vector<Mesh> mesh = m_pScene->meshes; 
            for (const auto& tri : mesh[x.first].triangles) {
                const auto v0 = mesh[x.first].vertices[tri[0]];
                const auto v1 = mesh[x.first].vertices[tri[1]];
                const auto v2 = mesh[x.first].vertices[tri[2]];
                if (intersectRayWithTriangle(v0.p, v1.p, v2.p, ray, hitInfo)) {
                    hitInfo.material = mesh[x.first].material;
                    std::cout << hit;
                    hit = true;
                }
            }
            for (const auto& sphere : m_pScene->spheres) {
                hit |= intersectRayWithShape(sphere, ray, hitInfo);
                return hit;
            }
               // return hit; 
            //for (const auto& tri : m_pScene->meshes
            //const auto v0 = x.second[0];
            //const auto v1 = x.second[1];
            //const auto v2 = x.second[2];
            //
            //if (intersectRayWithTriangle(x.second[0]., v1.p, v2.p, ray, hitInfo)) {
            //    std::cout << "hi";
            //    hitInfo.material = mesh.material;
            //    hit = true;
            //}
        }    
        return hit; 
    }

    if (!node.isLeaf) {
        int left = node.nodeChildrenIndices.first;       
        int right = node.nodeChildrenIndices.second;
       // std::cout << node.nodeChildrenIndices.first;
        ray.t = std::numeric_limits<float>::max();
        return (intersectRecurseMethod(ray, hitInfo, nodeVector[left]) || intersectRecurseMethod(ray, hitInfo, nodeVector[right]));
    }         

    return hit;
}

int BoundingVolumeHierarchy::constructNode(Scene &scene, std::vector<std::pair<int, std::vector<int>>> &meshTriangleIndices, int current_level)
{
    std::vector<std::pair<float, float>> limits = computeBoundingBoxLimits(scene, meshTriangleIndices);
    int num_triangles = countTriangles(meshTriangleIndices);
    AxisAlignedBox node_box = {
            glm::vec3{limits[0].first, limits[1].first, limits[2].first},
            glm::vec3{limits[0].second, limits[1].second, limits[2].second}};
    int new_node_index = nodeVector.size();
    BVHNode constructed_node;

    if (current_level < maxLevel && num_triangles >= minTriangles)
    {
        // Construct inner node if applicable.
        std::pair<std::vector<std::pair<int, std::vector<int>>>, std::vector<std::pair<int, std::vector<int>>>> child_triangle_meshes
        = computeOptimalSplit(scene, meshTriangleIndices, limits, current_level);
        constructed_node = {
            node_box,
            false,
            current_level,
            std::pair<int, int>{},
            std::vector<std::pair<int, std::vector<int>>>{}};
        nodeVector.push_back(std::move(constructed_node));
        nodeVector[new_node_index].nodeChildrenIndices = { constructNode(scene, child_triangle_meshes.first, current_level + 1), constructNode(scene, child_triangle_meshes.second, current_level + 1) };
    } else {
        // Construct leaf node if maxLevel reached or number of triangles is below minimum.
        constructed_node = {node_box, true, current_level, std::pair<int, int>{}, meshTriangleIndices};
        nodeVector.push_back(std::move(constructed_node));
    }
    std::cout << "---Node with index " << new_node_index << std::endl;
    std::cout << "Left child index: " << constructed_node.nodeChildrenIndices.first << std::endl;
    std::cout << "Right child index: " << constructed_node.nodeChildrenIndices.second << std::endl;
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
        Mesh &current_mesh = scene.meshes[coordinate_pairs.first];
        for (int triangle_index : coordinate_pairs.second)
        {
            Triangle &current_triangle = current_mesh.triangles[triangle_index];
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
    std::cout << limits[0].first << limits[0].second << std::endl;
    return limits;
}

std::pair<std::vector<std::pair<int, std::vector<int>>>, std::vector<std::pair<int, std::vector<int>>>>
BoundingVolumeHierarchy::computeOptimalSplit(Scene &scene, std::vector<std::pair<int, std::vector<int>>> &meshTriangleIndices,
                                             std::vector<std::pair<float, float>> &limits, int current_level)
{
    // Define and alternate axis to split on.
    int comparison_axis = current_level % 3;
    int area_axis = (current_level + 1) % 3;

    std::pair<std::vector<std::pair<int, std::vector<int>>>, std::vector<std::pair<int, std::vector<int>>>> left_right_split_indices;
    float cost = std::numeric_limits<float>::max();

    for (int bin = 1; bin < sahBins; bin++)
    {
        // Define split boundary and create vectors to store LHS and RHS mesh-triangle indices.
        float split_boundary = ((limits[comparison_axis].first * bin) + (limits[comparison_axis].second * (sahBins - bin))) / sahBins; // This. This fucker right here. Fuck him.
        std::vector<std::pair<int, std::vector<int>>> lhs_indices;
        std::vector<std::pair<int, std::vector<int>>> rhs_indices;

        // Compute which triangles belong on which side of the bin.
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

                switch (position_info)
                {
                case 0:
                    lhs_mesh_indices.second.push_back(triangle_index);
                    break;
                case 1:
                    rhs_mesh_indices.second.push_back(triangle_index);
                    break;
                case 2:
                    lhs_mesh_indices.second.push_back(triangle_index);
                    rhs_mesh_indices.second.push_back(triangle_index);
                    break;
                }
            }
            // Add indices of this mesh's triangles to mesh-triangle indices vectors.
            lhs_indices.push_back(lhs_mesh_indices);
            rhs_indices.push_back(rhs_mesh_indices);
        }
        // Compute surface area of splits' bounding boxes.
        std::vector<std::pair<float, float>> lhs_bounding_box = computeBoundingBoxLimits(scene, lhs_indices);
        std::vector<std::pair<float, float>> rhs_bounding_box = computeBoundingBoxLimits(scene, rhs_indices);
        float lhs_area = (std::abs(lhs_bounding_box[comparison_axis].second - lhs_bounding_box[comparison_axis].first)
        * std::abs(lhs_bounding_box[area_axis].second - lhs_bounding_box[area_axis].first));
        float rhs_area = (std::abs(rhs_bounding_box[comparison_axis].second - rhs_bounding_box[comparison_axis].first)
        * std::abs(rhs_bounding_box[area_axis].second - rhs_bounding_box[area_axis].first));

        // Compute cost function and keep or discard current bin.
        float candidate_cost = float(lhs_area*countTriangles(lhs_indices)) + float(rhs_area*countTriangles(rhs_indices));
        if (candidate_cost < cost)
        {
            cost = candidate_cost;
            left_right_split_indices.first = lhs_indices;
            left_right_split_indices.second = rhs_indices;
        }
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

int BoundingVolumeHierarchy::countTriangles(std::vector<std::pair<int, std::vector<int>>> &meshTriangleIndices)
{
    int triangle_count = 0;
    for (std::pair<int, std::vector<int>> &coordinate_pair : meshTriangleIndices) {triangle_count += coordinate_pair.second.size();}
    return triangle_count;
}

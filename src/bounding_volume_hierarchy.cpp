#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <glm/geometric.hpp>
#include <glm/vector_relational.hpp>
#include <cmath>
#include <limits>

const bool BARYCENTRIC_INTERPOLATION = false;

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene *pScene, int min_triangles, int sah_bins)
    : m_pScene(pScene)
{
    // Construct indices for all triangles.
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

    // Use number of triangles as heuristic to determine a max depth and assign values to other variables.
    maxLevel = std::ceil(std::log2(countTriangles(all_mesh_triangle_indices) + 1)) ;
    minTriangles = min_triangles;
    sahBins = sah_bins;

    // Construct parent node.
    constructNode(all_mesh_triangle_indices, 1);
}

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
    constructNode(all_mesh_triangle_indices, 1);
}

void BoundingVolumeHierarchy::debugDraw(int level)
{
    for (BVHNode &node : nodeVector)
    {
        if (node.level == level)
        {
            if (node.isLeaf) {drawAABB(node.boundingBox, DrawMode::Wireframe, glm::vec3{1.0f, 1.0f, 0.0f});} // Draw yellow box if leaf node.
            else {drawAABB(node.boundingBox, DrawMode::Wireframe, glm::vec3{1.0f, 0.0f, 0.0f});}             // Draw red box if inner node.
        }
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

void BoundingVolumeHierarchy::barycentricInterpolation(std::vector<Vertex> vertices, glm::vec3 p, HitInfo& hitInfo)
{
    Vertex v0 = vertices[0];
    Vertex v1 = vertices[1];
    Vertex v2 = vertices[2];
    
    float area = triangleArea(v0.p, v1.p, v2.p);
    float areaAlpha = triangleArea(p, v1.p, v2.p);
    float areaBeta = triangleArea(p, v0.p, v2.p);
    float areaGamma = triangleArea(p, v0.p, v1.p);

    float alpha = areaAlpha / area;
    float beta = areaBeta / area;
    float gamma = 1 - alpha - beta;

    hitInfo.normal = alpha * v0.n + beta * v1.n + gamma * v2.n;
}

float BoundingVolumeHierarchy::triangleArea(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2) {
    float a = glm::distance(v0, v1);
    float b = glm::distance(v1, v2);
    float c = glm::distance(v2, v0);
    float s = (a + b + c) / 2;
    return glm::sqrt(s * (s - a) * (s - b) * (s - c));
}

bool BoundingVolumeHierarchy::intersect(Ray &ray, HitInfo &hitInfo) {return bvhIntersect(ray, hitInfo, nodeVector[0]);}

bool BoundingVolumeHierarchy::bvhIntersect(Ray& ray, HitInfo& hitInfo, BVHNode& node)
{
    // Check for intersection with current node and reset 't' value set by the node intersection check when appropriate.
    float old_t = ray.t;
    if (!intersectRayWithShape(node.boundingBox, ray)) {return false;}
    ray.t = old_t;

    // Recurse if node is not a leaf.
    if (!node.isLeaf)
    {
        int left_child_index = node.nodeChildrenIndices.first;       
        int right_child_index = node.nodeChildrenIndices.second;
        bool left_side_intersect = bvhIntersect(ray, hitInfo, nodeVector[left_child_index]);
        bool right_side_intersect = bvhIntersect(ray, hitInfo, nodeVector[right_child_index]);
        return (left_side_intersect || right_side_intersect);
    }

    // Check for intersection with leaf node's triangles' vertices if node is a leaf.
    bool hit = false;
    for (const std::pair<int, std::vector<int>> &mesh_triangle_indices : node.triangleChildrenIndices)
    {
        Mesh &current_mesh = m_pScene->meshes[mesh_triangle_indices.first];
        for (int current_triangle_index : mesh_triangle_indices.second)
        {
            Triangle &current_triangle = current_mesh.triangles[current_triangle_index];
            std::vector<Vertex> vertices = {
                current_mesh.vertices[current_triangle[0]],
                current_mesh.vertices[current_triangle[1]],
                current_mesh.vertices[current_triangle[2]]};
            if (intersectRayWithTriangle(vertices[0].p, vertices[1].p, vertices[2].p, ray, hitInfo))
            {
                if (BARYCENTRIC_INTERPOLATION) {barycentricInterpolation(vertices, ray.origin + ray.t * ray.direction, hitInfo);} // ACTIVATES INTERPOLATION
                hitInfo.material = current_mesh.material;
                hit = true;
            }
        }
    }
    return hit;
}

int BoundingVolumeHierarchy::constructNode(std::vector<std::pair<int, std::vector<int>>> &meshTriangleIndices, int current_level)
{
    std::vector<std::pair<float, float>> limits = computeBoundingBoxLimits(meshTriangleIndices);
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
        = computeOptimalSplit(meshTriangleIndices, limits, current_level);
        constructed_node = {
            node_box,
            false,
            current_level,
            std::pair<int, int>{},
            std::vector<std::pair<int, std::vector<int>>>{}};
        nodeVector.push_back(constructed_node);
        nodeVector[new_node_index].nodeChildrenIndices = {constructNode(child_triangle_meshes.first, current_level + 1), constructNode(child_triangle_meshes.second, current_level + 1)};
    } else {
        // Construct leaf node if maxLevel reached or number of triangles is below minimum.
        constructed_node = {node_box, true, current_level, std::pair<int, int>{}, meshTriangleIndices};
        nodeVector.push_back(constructed_node);
    }
    return new_node_index;
}

std::vector<std::pair<float, float>> BoundingVolumeHierarchy::computeBoundingBoxLimits(std::vector<std::pair<int, std::vector<int>>> &meshTriangleIndices)
{
    std::vector<std::pair<float, float>> limits = {
        std::pair<float, float>{std::numeric_limits<float>::max(), std::numeric_limits<float>::min()},
        std::pair<float, float>{std::numeric_limits<float>::max(), std::numeric_limits<float>::min()},
        std::pair<float, float>{std::numeric_limits<float>::max(), std::numeric_limits<float>::min()}};
    for (std::pair<int, std::vector<int>> &coordinate_pairs : meshTriangleIndices)
    {
        Mesh &current_mesh = m_pScene->meshes[coordinate_pairs.first];
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
    return limits;
}

std::pair<std::vector<std::pair<int, std::vector<int>>>, std::vector<std::pair<int, std::vector<int>>>>
BoundingVolumeHierarchy::computeOptimalSplit(std::vector<std::pair<int, std::vector<int>>> &meshTriangleIndices,
                                             std::vector<std::pair<float, float>> &limits, int current_level)
{
    // Define the axis to split on.
    int comparison_axis = determineLongestAxis(limits);

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
            Mesh &current_mesh = m_pScene->meshes[coordinate_pair.first];
            std::pair<int, std::vector<int>> lhs_mesh_indices = {coordinate_pair.first, std::vector<int>{}};
            std::pair<int, std::vector<int>> rhs_mesh_indices = {coordinate_pair.first, std::vector<int>{}};

            // Populate the triangle index vector for the current mesh appropriately for each split vector.
            for (int triangle_index : coordinate_pair.second)
            {
                Triangle &current_triangle = current_mesh.triangles[triangle_index];
                int position_info = checkTriangleBorderSide(current_mesh, current_triangle, comparison_axis, split_boundary);

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
        // Compute total surface area of the splits' bounding boxes.
        std::vector<std::pair<float, float>> lhs_bounding_box = computeBoundingBoxLimits(lhs_indices);
        std::vector<std::pair<float, float>> rhs_bounding_box = computeBoundingBoxLimits(rhs_indices);
        float lhs_area = computeTotalSurfaceArea(lhs_bounding_box);
        float rhs_area = computeTotalSurfaceArea(rhs_bounding_box);

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

int BoundingVolumeHierarchy::checkTriangleBorderSide(Mesh &mesh, Triangle &triangle, int comparison_axis, float split_boundary)
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

int BoundingVolumeHierarchy::determineLongestAxis(std::vector<std::pair<float, float>> &limits)
{
    float x_length = std::abs(limits[0].second - limits[0].first);
    float y_length = std::abs(limits[1].second - limits[1].first);
    float z_length = std::abs(limits[2].second - limits[2].first);
    float lengths[] = {x_length, y_length, z_length};

    int max_axis = -1;
    float max_length = std::numeric_limits<float>::min();
    for (int counter = 0; counter < 3; counter++)
    {
        if (lengths[counter] > max_length)
        {
            max_length = lengths[counter];
            max_axis = counter;
        }
    }
    return max_axis;
}

float BoundingVolumeHierarchy::computeTotalSurfaceArea(std::vector<std::pair<float, float>> &limits)
{
    float x_length = std::abs(limits[0].second - limits[0].first);
    float y_length = std::abs(limits[1].second - limits[1].first);
    float z_length = std::abs(limits[2].second - limits[2].first);
    return (2.0f*x_length*y_length) + (2.0f*x_length*z_length) + (2.0f*y_length*z_length);
}

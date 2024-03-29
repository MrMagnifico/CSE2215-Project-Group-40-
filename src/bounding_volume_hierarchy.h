#pragma once
#include "ray_tracing.h"
#include "scene.h"

struct BVHNode
{
    AxisAlignedBox boundingBox;
    bool isLeaf;
    int level;
    std::pair<int, int> nodeChildrenIndices;                               // First value is the left child index, second value is the right child index, in case of a non-leaf node, empty otherwise
    std::vector<std::pair<int, std::vector<int>>> triangleChildrenIndices; // First value is the mesh index, second value is the triangle indices, in case of a leaf node, empty otherwise.
};

class BoundingVolumeHierarchy
{
public:
    /**Construct a BVH tree (starting at level 1) from the given Scene.
     * 
     * Parameters:
     * pScene: Pointer to the scene to be rendered.
     * min_triangles: Minimum number of triangles required for a node to attempt a split.
     * sah_bins: Number of bins to use for SAH (must be >=2).
     * 
     * Returns: A BVH tree to use for accelerating ray intersections.
     */
    BoundingVolumeHierarchy(Scene *pScene, int min_triangles, int sah_bins);

    /**Construct a BVH tree (starting at level 1) from the given Scene.
     * Uses the number of triangles in the scene as a heuristic to determine the max depth of the tree.
     * 
     * Parameters:
     * pScene: Pointer to the scene to be rendered.
     * max_level: The maximum level to which the tree should grow to.
     * min_triangles: Minimum number of triangles required for a node to attempt a split.
     * sah_bins: Number of bins to use for SAH (must be >=2).
     * 
     * Returns: A BVH tree to use for accelerating ray intersections.
     */
    BoundingVolumeHierarchy(Scene *pScene, int max_level, int min_triangles, int sah_bins);

    void debugDraw(int level);
    int numLevels();
    void barycentricInterpolation(std::vector<Vertex> vertices, glm::vec3 p, HitInfo& hitInfo);
    float triangleArea(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2);

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray &ray, HitInfo &hitInfo);

    std::vector<BVHNode> nodeVector; // Stores all of the BVH tree's nodes.

private:
    Scene *m_pScene;

    int maxLevel;     // Max level the tree should grow to.
    int minTriangles; // Minimum number of triangles required for a node to split itself and not become a leaf node.
    int sahBins;      // Number of bins to use for SAH.

    /**Traverse the BVH tree recursively starting at the given node and return whether a hit occurs at any point down the tree,
     * modifying hitInfo as appropriate.
     * 
     * Parameters:
     * scene: The scene to be rendered.
     * ray: The ray whose intersection info (or lack thereof) must be computed.
     * hitInfo: The HitInfo struct to be used to store information about the ray's intersection (if any).
     * node: The node in the BVH tree to start computations at (this should be nodeVector[0] on the initial call).
     * 
     * Returns: A boolean indicating whether an intersection occurs in the tree at or below the given node.
     */
    bool bvhIntersect(Ray &ray, HitInfo &hitInfo, BVHNode &node);

    /**Construct a BVH node from the data in the given scene and return its index in the std::vector of nodes.
    * 
    * Parameters:
    * scene: The scene to be rendered.
    * meshTriangleIndices: A mapping of the indices of the meshes and subsequent triangle that the node should analyse and split or store.
    * current_level: Indicates the level of the tree that the node to be created will lie on (should be incremented on each call).
    * 
    * Returns: The index of the created node in the node vector.
    */
    int constructNode(std::vector<std::pair<int, std::vector<int>>> &meshTriangleIndices, int current_level);

    // Compute borders of the bounding box.
    std::vector<std::pair<float, float>> computeBoundingBoxLimits(std::vector<std::pair<int, std::vector<int>>> &meshTriangleIndices);

    /**Compute triangles that belong in each split and return a std::pair of std::vector<std::pair<int, std::vector<int>>,
     * with the int in each pair representing the index of a mesh in the scene and the std::vector<int> representing the indices of the
     * triangles belonging to that mesh
     */
    std::pair<std::vector<std::pair<int, std::vector<int>>>, std::vector<std::pair<int, std::vector<int>>>>
    computeOptimalSplit(std::vector<std::pair<int, std::vector<int>>> &meshTriangleIndices,
                        std::vector<std::pair<float, float>> &limits, int current_level);

    /**Check if a triangle is wholly on the left, wholly on the right or split between the boundary defined by comparison_axis and split_boundary
     * 
     * Parameters:
     * scene: The scene to be rendered.
     * comparison_axis: Defines which of the three axes to compare on.
     * triangle: The triangle whose orientation is to be computed.
     * split_boundary: The value to use as the border for the given axis.
     * 
     * Returns:
     * 0 - If the triangle is wholly on the left (negative side).
     * 1 - If the triangle is wholly on the right (positive side).
     * 2 - If the triangle's vertices are split across the border.
     */
    int checkTriangleBorderSide(Mesh &mesh, Triangle &triangle, int comparison_axis, float split_boundary);

    // Count the number of triangles contained in meshTriangleIndices.
    int countTriangles(std::vector<std::pair<int, std::vector<int>>> &meshTriangleIndices);

    // Determine the axis with the greatest length in the box defined by limits.
    int determineLongestAxis(std::vector<std::pair<float, float>> &limits);

    // Compute the total surface area of the box defined by limits.
    float computeTotalSurfaceArea(std::vector<std::pair<float, float>> &limits);
};

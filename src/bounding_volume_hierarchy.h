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
     * max_level: The maximum level to which the tree should grow to.
     * bin_num: The number of bins to use when computing the SAH (must be >=2).
     * 
     * Returns: A BVH tree to use for accelerating ray intersections.
     */
    BoundingVolumeHierarchy(Scene *pScene, int max_level, int bin_num);
    
    void debugDraw(int level);
    int numLevels() const;

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray &ray, HitInfo &hitInfo) const;

private:
    Scene *m_pScene;
    std::vector<BVHNode> nodeVector; // Stores all of the BVH tree's nodes.
    int binNum;                      // Number of bins to use for SAH
    int maxLevel;                    // Max level of the tree

    static const int traversalCost = 1;
    static const int triangleIntersectionCost = 1;

    /**Construct a BVH node from the data in the given scene and return its index in the std::vector of nodes.
    * 
    * Parameters:
    * scene: The scene to be rendered.
    * meshTriangleIndices: A mapping of the indices of the meshes and subsequent triangle that the node should analyse and split or store.
    * axis_selector: Used to decide whether to split on the X or Y axes (should be incremented for each function call).
    * current_level: Indicates the level of the tree that the node to be created will lie on.
    * 
    * Returns: The index of the created node in the node vector.
    */
    int constructNode(Scene &scene, std::vector<std::pair<int, std::vector<int>>> &meshTriangleIndices, int current_level);

    // Compute borders of the bounding box.
    std::vector<std::pair<float, float>> computeBoundingBoxLimits(Scene &scene, std::vector<std::pair<int, std::vector<int>>> &meshTriangleIndices);

    // Compute the cost function of each possible bin and return std::pair defining the meshes and triangles contained in the left and right split.
    std::pair<std::vector<std::pair<int, std::vector<int>>>, std::vector<std::pair<int, std::vector<int>>>> computeOptimalSplit(Scene &scene, std::vector<std::pair<int, std::vector<int>>> meshTriangleIndices,
                                                                                                                                std::vector<std::pair<float, float>> limits, int current_level);

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
    int checkTriangleBorderSide(Scene &scene, Mesh &mesh, Triangle &triangle, int comparison_axis, float split_boundary);
};

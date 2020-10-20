#pragma once
#include "ray_tracing.h"
#include "scene.h"

struct BVHNode {
    AxisAlignedBox boundingBox;
    bool isLeaf;
    int level;
    std::pair<int, int> nodeChildrenIndices; // First value is the left child index, second value is the right child index, in case of a non-leaf node, empty otherwise
    std::vector<std::pair<int, std::vector<int>>> triangleChildrenIndices; // First value is the mesh index, second value is the triangle indices, in case of a leaf node, empty otherwise.
};

class BoundingVolumeHierarchy {
    public:
        // Constructs a BVH tree (starting at level 1) from the given Scene.
        BoundingVolumeHierarchy(Scene* pScene, int maxLevel, int bin_num);

        // Use this function to visualize your BVH. This can be useful for debugging.
        void debugDraw(int level);
        int numLevels() const;

        // Return true if something is hit, returns false otherwise.
        // Only find hits if they are closer than t stored in the ray and the intersection
        // is on the correct side of the origin (the new t >= 0).
        bool intersect(Ray& ray, HitInfo& hitInfo) const;

        /**Construct a BVH node from the data in the given scene and return its index in the std::vector of nodes.
         * 
         * Parameters:
         * scene: The scene to be rendered.
         * meshTriangleIndices: A mapping of the indices of the meshes and subsequent triangle that the node should analyse and split or store.
         * node_index: The index that the node to be generated will take (Should be equal to the size of the node vector on each call).
         * axis_selector: Used to decide whether to split on the X or Y axes (should be incremented for each function call).
         * current_level: Indicates the level of the tree that the node to be created will lie on.
         * 
         * Returns: the index of the node to be created in the node vector.
         */
        int constructNode(Scene &scene, std::vector<std::pair<int, std::vector<int>>> meshTriangleIndices, int node_index, int axis_selector, int current_level);

    private:
        Scene* m_pScene;
        std::vector<BVHNode> nodeVector; // Stores all of the BVH tree's nodes.
        int binNum; // Number of bins to use for SAH
        int maxLevel; // Max level of the tree

        const int traversalCost = 1;
        const int triangleIntersectionCost = 1;
};

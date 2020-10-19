#pragma once
#include "ray_tracing.h"
#include "scene.h"

struct BVHNode {
    AxisAlignedBox boundingBox;
    bool isLeaf;
    std::vector<BVHNode> nodeChildren;
    std::vector<Vertex> vertexChildren;
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

        // Constructs a BVH node from the data in the given scene.
        BVHNode constructNode(Scene &scene, int axis_selector, int current_level);

    private:
        Scene* m_pScene;
        BVHNode root_node;
        int max_level;
        int bin_num;

        const int traversal_cost = 1;
        const int triangle_intersection_cost = 1;
};

#ifndef FRONTIER_EXPLORATION_PATH_PLANNING_HPP
#define FRONTIER_EXPLORATION_PATH_PLANNING_HPP

#include <octomap/octomap.h>
#include <vector>
#include <unordered_map>

struct Node {
    int x, y, z;
    bool operator==(const Node& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    bool operator<(const Node& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
};

namespace std {
    template <>
    struct hash<Node> {
        size_t operator()(const Node& node) const {
            return ((hash<int>()(node.x)
                   ^ (hash<int>()(node.y) << 1)) >> 1)
                   ^ (hash<int>()(node.z) << 1);
        }
    };
}

#endif
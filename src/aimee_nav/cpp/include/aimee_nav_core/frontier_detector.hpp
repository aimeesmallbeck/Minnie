#pragma once

#include <vector>
#include <unordered_set>
#include <cstdint>
#include "aimee_nav_core/grid_map.hpp"

namespace aimee_nav_core {

/**
 * @brief Frontier cluster metadata.
 */
struct FrontierCluster {
    float cx = 0.0f;       // centroid world x
    float cy = 0.0f;       // centroid world y
    float size = 0.0f;     // number of cells
    float min_x = 0.0f;    // bbox
    float max_x = 0.0f;
    float min_y = 0.0f;
    float max_y = 0.0f;
};

/**
 * @brief Incremental frontier detector for occupancy grids.
 *
 * Maintains a set of frontier cells (free cells adjacent to unknown)
 * incrementally as the map is updated. Clusters frontiers using
 * union-find for efficiency.
 */
class FrontierDetector {
public:
    FrontierDetector();

    /** Reset and scan entire map for initial frontiers. */
    void initialize(const GridMap& map);

    /** Notify detector that a cell changed. Call after map update. */
    void on_cell_changed(int gx, int gy);

    /** Batch process changed cells (more efficient than individual calls). */
    void on_cells_changed(const std::vector<std::pair<int, int>>& cells);

    /** Get current frontier clusters with at least min_size cells. */
    std::vector<FrontierCluster> get_clusters(int min_size_cells) const;

    /** Clear all state. */
    void clear();

private:
    const GridMap* map_ = nullptr;
    int width_ = 0;
    int height_ = 0;

    // Frontier flag per cell
    std::vector<bool> is_frontier_;

    // Union-find for clustering
    mutable std::vector<int> uf_parent_;
    mutable std::vector<int> uf_size_;

    inline int index(int gx, int gy) const { return gy * width_ + gx; }
    bool is_valid(int gx, int gy) const {
        return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
    }

    bool check_is_frontier(int gx, int gy) const;
    int uf_find(int i) const;
    void uf_union(int a, int b) const;

    void rebuild_clusters(std::vector<FrontierCluster>& out, int min_size) const;
};

} // namespace aimee_nav_core

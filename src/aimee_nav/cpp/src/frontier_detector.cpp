#include "aimee_nav_core/frontier_detector.hpp"
#include <algorithm>

namespace aimee_nav_core {

FrontierDetector::FrontierDetector() {}

void FrontierDetector::clear() {
    map_ = nullptr;
    width_ = 0;
    height_ = 0;
    is_frontier_.clear();
    uf_parent_.clear();
    uf_size_.clear();
}

void FrontierDetector::initialize(const GridMap& map) {
    clear();
    map_ = &map;
    width_ = map.width_cells();
    height_ = map.height_cells();
    int n = width_ * height_;
    is_frontier_.assign(n, false);
    uf_parent_.resize(n);
    uf_size_.resize(n);

    for (int gy = 1; gy < height_ - 1; ++gy) {
        for (int gx = 1; gx < width_ - 1; ++gx) {
            if (check_is_frontier(gx, gy)) {
                is_frontier_[index(gx, gy)] = true;
            }
        }
    }
}

bool FrontierDetector::check_is_frontier(int gx, int gy) const {
    if (!is_valid(gx, gy)) return false;
    if (map_->cell(gx, gy) != 0) return false;  // must be free

    // Check 4-neighbors for unknown
    const int dx[4] = {-1, 1, 0, 0};
    const int dy[4] = {0, 0, -1, 1};
    for (int i = 0; i < 4; ++i) {
        int nx = gx + dx[i];
        int ny = gy + dy[i];
        if (is_valid(nx, ny) && map_->cell(nx, ny) == -1) {
            return true;
        }
    }
    return false;
}

void FrontierDetector::on_cell_changed(int gx, int gy) {
    if (!map_ || !is_valid(gx, gy)) return;

    int idx = index(gx, gy);
    bool was_frontier = is_frontier_[idx];
    bool now_frontier = check_is_frontier(gx, gy);

    if (was_frontier != now_frontier) {
        is_frontier_[idx] = now_frontier;
    }

    // Re-check 4-neighbors since they may have gained/lost an unknown neighbor
    const int dx[4] = {-1, 1, 0, 0};
    const int dy[4] = {0, 0, -1, 1};
    for (int i = 0; i < 4; ++i) {
        int nx = gx + dx[i];
        int ny = gy + dy[i];
        if (!is_valid(nx, ny)) continue;
        int nidx = index(nx, ny);
        bool n_was = is_frontier_[nidx];
        bool n_now = check_is_frontier(nx, ny);
        if (n_was != n_now) {
            is_frontier_[nidx] = n_now;
        }
    }
}

void FrontierDetector::on_cells_changed(const std::vector<std::pair<int, int>>& cells) {
    if (!map_) return;
    // Deduplicate
    std::unordered_set<int> changed;
    changed.reserve(cells.size() * 5);
    for (const auto& [gx, gy] : cells) {
        if (!is_valid(gx, gy)) continue;
        changed.insert(index(gx, gy));
        const int dx[4] = {-1, 1, 0, 0};
        const int dy[4] = {0, 0, -1, 1};
        for (int i = 0; i < 4; ++i) {
            int nx = gx + dx[i];
            int ny = gy + dy[i];
            if (is_valid(nx, ny)) {
                changed.insert(index(nx, ny));
            }
        }
    }
    for (int idx : changed) {
        int gx = idx % width_;
        int gy = idx / width_;
        is_frontier_[idx] = check_is_frontier(gx, gy);
    }
}

int FrontierDetector::uf_find(int i) const {
    if (uf_parent_[i] != i) {
        uf_parent_[i] = uf_find(uf_parent_[i]);
    }
    return uf_parent_[i];
}

void FrontierDetector::uf_union(int a, int b) const {
    int ra = uf_find(a);
    int rb = uf_find(b);
    if (ra == rb) return;
    if (uf_size_[ra] < uf_size_[rb]) {
        std::swap(ra, rb);
    }
    uf_parent_[rb] = ra;
    uf_size_[ra] += uf_size_[rb];
}

void FrontierDetector::rebuild_clusters(std::vector<FrontierCluster>& out, int min_size) const {
    if (!map_ || width_ == 0 || height_ == 0) return;

    int n = width_ * height_;
    for (int i = 0; i < n; ++i) {
        uf_parent_[i] = i;
        uf_size_[i] = 1;
    }

    // Union adjacent frontier cells (8-connected)
    const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    for (int gy = 0; gy < height_; ++gy) {
        for (int gx = 0; gx < width_; ++gx) {
            int idx = index(gx, gy);
            if (!is_frontier_[idx]) continue;
            for (int d = 0; d < 8; ++d) {
                int nx = gx + dx[d];
                int ny = gy + dy[d];
                if (!is_valid(nx, ny)) continue;
                int nidx = index(nx, ny);
                if (is_frontier_[nidx]) {
                    uf_union(idx, nidx);
                }
            }
        }
    }

    // Collect clusters
    std::unordered_map<int, std::vector<std::pair<int, int>>> clusters;
    for (int gy = 0; gy < height_; ++gy) {
        for (int gx = 0; gx < width_; ++gx) {
            int idx = index(gx, gy);
            if (!is_frontier_[idx]) continue;
            int root = uf_find(idx);
            clusters[root].emplace_back(gx, gy);
        }
    }

    float res = map_->resolution_m();
    float ox = map_->origin_x();
    float oy = map_->origin_y();

    for (const auto& [root, cells] : clusters) {
        if (static_cast<int>(cells.size()) < min_size) continue;

        float sx = 0.0f, sy = 0.0f;
        float min_x = 1e9f, max_x = -1e9f;
        float min_y = 1e9f, max_y = -1e9f;

        for (const auto& [gx, gy] : cells) {
            float wx = ox + (gx + 0.5f) * res;
            float wy = oy + (gy + 0.5f) * res;
            sx += wx;
            sy += wy;
            min_x = std::min(min_x, wx);
            max_x = std::max(max_x, wx);
            min_y = std::min(min_y, wy);
            max_y = std::max(max_y, wy);
        }

        FrontierCluster fc;
        fc.cx = sx / cells.size();
        fc.cy = sy / cells.size();
        fc.size = static_cast<float>(cells.size());
        fc.min_x = min_x;
        fc.max_x = max_x;
        fc.min_y = min_y;
        fc.max_y = max_y;
        out.push_back(fc);
    }
}

std::vector<FrontierCluster> FrontierDetector::get_clusters(int min_size_cells) const {
    std::vector<FrontierCluster> result;
    rebuild_clusters(result, min_size_cells);
    return result;
}

} // namespace aimee_nav_core

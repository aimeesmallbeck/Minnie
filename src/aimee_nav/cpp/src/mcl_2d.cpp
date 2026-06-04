#include "aimee_nav_core/mcl_2d.hpp"
#include "aimee_nav_core/math_utils.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <unordered_set>

namespace aimee_nav_core {

MCL2D::MCL2D() : rng_(std::random_device{}()) {}

void MCL2D::set_motion_noise(float a1, float a2, float a3, float a4) {
    alpha1_ = a1; alpha2_ = a2; alpha3_ = a3; alpha4_ = a4;
}

void MCL2D::set_min_max_particles(int min_p, int max_p) {
    min_particles_ = std::max(50, min_p);
    max_particles_ = std::max(min_particles_, max_p);
}

void MCL2D::set_kld_epsilon(float eps) {
    kld_epsilon_ = eps;
}

void MCL2D::global_localization(const GridMap& map, int max_particles) {
    map_ = &map;
    max_particles_ = max_particles;
    particles_.clear();
    particles_.reserve(max_particles_);

    // Collect free cell coordinates
    std::vector<std::pair<int, int>> free_cells;
    free_cells.reserve(4096);
    int w = map.width_cells();
    int h = map.height_cells();
    for (int gy = 0; gy < h; ++gy) {
        for (int gx = 0; gx < w; ++gx) {
            if (map.cell(gx, gy) == 0) {  // free cell
                free_cells.emplace_back(gx, gy);
            }
        }
    }

    if (free_cells.empty()) {
        // No free cells known — sample uniformly over whole map
        std::uniform_real_distribution<float> ux(map.origin_x(),
                                                  map.origin_x() + map.width_m());
        std::uniform_real_distribution<float> uy(map.origin_y(),
                                                  map.origin_y() + map.height_m());
        std::uniform_real_distribution<float> ut(-M_PI, M_PI);
        for (int i = 0; i < max_particles_; ++i) {
            particles_.push_back({ux(rng_), uy(rng_), ut(rng_), 1.0f});
        }
        return;
    }

    // Sample uniformly from free cells
    std::uniform_int_distribution<size_t> cell_dist(0, free_cells.size() - 1);
    std::uniform_real_distribution<float> ut(-M_PI, M_PI);
    float res = map.resolution_m();
    float ox = map.origin_x();
    float oy = map.origin_y();

    for (int i = 0; i < max_particles_; ++i) {
        auto [gx, gy] = free_cells[cell_dist(rng_)];
        float wx = ox + (gx + 0.5f) * res;
        float wy = oy + (gy + 0.5f) * res;
        particles_.push_back({wx, wy, ut(rng_), 1.0f / max_particles_});
    }
}

void MCL2D::set_initial_pose(float x, float y, float theta,
                             float xy_var, float theta_var,
                             int num_particles) {
    particles_.clear();
    particles_.reserve(num_particles);
    std::normal_distribution<float> nx(x, std::sqrt(xy_var));
    std::normal_distribution<float> ny(y, std::sqrt(xy_var));
    std::normal_distribution<float> nt(theta, std::sqrt(theta_var));
    for (int i = 0; i < num_particles; ++i) {
        particles_.push_back({nx(rng_), ny(rng_), normalize_angle(nt(rng_)),
                              1.0f / num_particles});
    }
}

float MCL2D::sample_normal(float stddev) {
    return noise_dist_(rng_) * stddev;
}

void MCL2D::predict(float v, float w, float dt) {
    if (particles_.empty() || dt <= 0.0f) return;

    for (auto& p : particles_) {
        // Sample motion noise
        float v_hat = v + sample_normal(alpha1_ * std::abs(v) + alpha2_ * std::abs(w));
        float w_hat = w + sample_normal(alpha3_ * std::abs(v) + alpha4_ * std::abs(w));
        float gamma_hat = sample_normal(alpha1_ * std::abs(v) + alpha2_ * std::abs(w));

        if (std::abs(w_hat) < 1e-3f) {
            // Straight line
            p.x += v_hat * dt * std::cos(p.theta);
            p.y += v_hat * dt * std::sin(p.theta);
        } else {
            // Circular arc
            float r = v_hat / w_hat;
            p.x += -r * std::sin(p.theta) + r * std::sin(p.theta + w_hat * dt);
            p.y +=  r * std::cos(p.theta) - r * std::cos(p.theta + w_hat * dt);
            p.theta += w_hat * dt;
        }
        p.theta = normalize_angle(p.theta + gamma_hat * dt);
    }
}

bool MCL2D::update(const std::vector<float>& ranges,
                   float angle_min, float angle_increment,
                   float range_min, float range_max) {
    if (!map_ || particles_.empty()) return false;

    // Precompute scan endpoints in robot frame
    scan_xs_.clear();
    scan_ys_.clear();
    for (size_t i = 0; i < ranges.size(); ++i) {
        float r = ranges[i];
        if (std::isnan(r) || std::isinf(r) || r < range_min || r > range_max) continue;
        float a = angle_min + static_cast<float>(i) * angle_increment;
        scan_xs_.push_back(r * std::cos(a));
        scan_ys_.push_back(r * std::sin(a));
    }

    if (scan_xs_.empty()) return false;

    // Score each particle
    float max_weight = 0.0f;
    for (auto& p : particles_) {
        p.weight = score_particle(p);
        if (p.weight > max_weight) max_weight = p.weight;
    }

    if (max_weight <= 0.0f) {
        // All particles scored zero — sensor failure or bad map
        return false;
    }

    normalize_weights();

    // KLD-adaptive resample
    resample();

    return true;
}

float MCL2D::score_particle(const Particle& p) const {
    float score = 0.0f;
    float c = std::cos(p.theta);
    float s = std::sin(p.theta);
    for (size_t i = 0; i < scan_xs_.size(); ++i) {
        float wx = p.x + c * scan_xs_[i] - s * scan_ys_[i];
        float wy = p.y + s * scan_xs_[i] + c * scan_ys_[i];
        int gx, gy;
        if (map_->world_to_grid(wx, wy, gx, gy)) {
            int8_t cell = map_->cell(gx, gy);
            if (cell >= 50) {
                score += 1.0f;
            } else if (cell >= 0) {
                score += 0.3f;
            }
            // unknown (-1) gives 0
        }
    }
    return score;
}

void MCL2D::normalize_weights() {
    float sum = 0.0f;
    for (const auto& p : particles_) sum += p.weight;
    if (sum > 0.0f) {
        for (auto& p : particles_) p.weight /= sum;
    } else {
        float uniform = 1.0f / particles_.size();
        for (auto& p : particles_) p.weight = uniform;
    }
}

int MCL2D::compute_desired_particle_count() const {
    // KLD-sampling: bin particles and count occupied bins
    // Bin size = resolution x resolution x 10 degrees
    if (map_ == nullptr || particles_.size() < 100) return min_particles_;

    float res = map_->resolution_m();
    float angle_bin = 10.0f * M_PI / 180.0f;
    float ox = map_->origin_x();
    float oy = map_->origin_y();

    std::unordered_set<uint64_t> bins;
    bins.reserve(particles_.size() * 2);

    for (const auto& p : particles_) {
        int bx = static_cast<int>((p.x - ox) / res);
        int by = static_cast<int>((p.y - oy) / res);
        int bt = static_cast<int>(p.theta / angle_bin);
        uint64_t key = ((static_cast<uint64_t>(bx) & 0xFFFF) << 32)
                     | ((static_cast<uint64_t>(by) & 0xFFFF) << 16)
                     | (static_cast<uint64_t>(bt) & 0xFFFF);
        bins.insert(key);
    }

    int k = static_cast<int>(bins.size());
    if (k <= 1) return min_particles_;

    // Simplified KLD: N = (k - 1) / (2 * epsilon)
    float desired = static_cast<float>(k - 1) / (2.0f * kld_epsilon_);
    int n = static_cast<int>(desired);
    return std::clamp(n, min_particles_, max_particles_);
}

void MCL2D::resample() {
    // Effective sample size
    float sum_sq = 0.0f;
    for (const auto& p : particles_) sum_sq += p.weight * p.weight;
    float n_eff = (sum_sq > 0.0f) ? 1.0f / sum_sq : 0.0f;

    int desired_n = compute_desired_particle_count();

    // Only resample if effective sample size is low OR we need more particles
    if (n_eff > particles_.size() * 0.5f &&
        static_cast<int>(particles_.size()) >= desired_n) {
        return;
    }

    // Low-variance resampler
    resampled_.clear();
    resampled_.reserve(desired_n);

    float step = 1.0f / desired_n;
    std::uniform_real_distribution<float> u_dist(0.0f, step);
    float r = u_dist(rng_);
    float c = particles_[0].weight;
    size_t i = 0;

    for (int m = 0; m < desired_n; ++m) {
        float U = r + m * step;
        while (U > c && i + 1 < particles_.size()) {
            ++i;
            c += particles_[i].weight;
        }
        resampled_.push_back(particles_[i]);
        resampled_.back().weight = 1.0f / desired_n;
    }

    particles_.swap(resampled_);
}

std::array<float, 3> MCL2D::get_pose() const {
    if (particles_.empty()) return {0.0f, 0.0f, 0.0f};

    // Weighted mean for x and y
    float wx = 0.0f, wy = 0.0f;
    for (const auto& p : particles_) {
        wx += p.weight * p.x;
        wy += p.weight * p.y;
    }

    // Circular mean for theta
    float sin_sum = 0.0f, cos_sum = 0.0f;
    for (const auto& p : particles_) {
        sin_sum += p.weight * std::sin(p.theta);
        cos_sum += p.weight * std::cos(p.theta);
    }
    float wtheta = std::atan2(sin_sum, cos_sum);

    return {wx, wy, wtheta};
}

std::array<float, 9> MCL2D::get_covariance() const {
    std::array<float, 9> P = {};
    if (particles_.size() < 2) return P;

    auto [mx, my, mtheta] = get_pose();

    float cxx = 0.0f, cyy = 0.0f, ctt = 0.0f;
    float cxy = 0.0f, cxt = 0.0f, cyt = 0.0f;

    for (const auto& p : particles_) {
        float dx = p.x - mx;
        float dy = p.y - my;
        float dt = normalize_angle(p.theta - mtheta);
        cxx += dx * dx;
        cyy += dy * dy;
        ctt += dt * dt;
        cxy += dx * dy;
        cxt += dx * dt;
        cyt += dy * dt;
    }

    float n = static_cast<float>(particles_.size());
    P[0] = cxx / n;  P[1] = cxy / n;  P[2] = cxt / n;
    P[3] = cxy / n;  P[4] = cyy / n;  P[5] = cyt / n;
    P[6] = cxt / n;  P[7] = cyt / n;  P[8] = ctt / n;
    return P;
}

bool MCL2D::is_converged(float pos_tol, float angle_tol) const {
    if (particles_.size() < 10) return false;

    auto [mx, my, mtheta] = get_pose();

    // Count particles within tolerance of mean
    int inside = 0;
    for (const auto& p : particles_) {
        float d = std::hypot(p.x - mx, p.y - my);
        float da = std::abs(normalize_angle(p.theta - mtheta));
        if (d < pos_tol && da < angle_tol) {
            ++inside;
        }
    }

    float ratio = static_cast<float>(inside) / static_cast<float>(particles_.size());
    return ratio > 0.85f;
}

} // namespace aimee_nav_core

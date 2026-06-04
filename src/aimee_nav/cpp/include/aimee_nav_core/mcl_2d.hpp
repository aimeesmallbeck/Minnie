#pragma once

#include <vector>
#include <array>
#include <random>
#include <memory>
#include "aimee_nav_core/grid_map.hpp"

namespace aimee_nav_core {

/**
 * @brief Lightweight 2D Monte Carlo Localization (MCL).
 *
 * Adaptive KLD-sampling particle filter for global localization
 * and pose tracking on an occupancy grid.
 */
struct Particle {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    float weight = 0.0f;
};

class MCL2D {
public:
    MCL2D();

    /** Initialize particles uniformly over free space in the map. */
    void global_localization(const GridMap& map, int max_particles = 2000);

    /** Initialize particles around a prior pose with Gaussian noise. */
    void set_initial_pose(float x, float y, float theta,
                          float xy_variance, float theta_variance,
                          int num_particles = 500);

    /** Motion model prediction. */
    void predict(float v, float w, float dt);

    /**
     * Sensor model update.
     * @return true if update succeeded (enough valid scan points).
     */
    bool update(const std::vector<float>& ranges,
                float angle_min, float angle_increment,
                float range_min, float range_max);

    /** Get estimated pose [x, y, theta] and covariance [9]. */
    std::array<float, 3> get_pose() const;
    std::array<float, 9> get_covariance() const;

    /** Check if particle distribution has converged. */
    bool is_converged(float position_tolerance_m = 0.3f,
                      float angle_tolerance_rad = 0.3f) const;

    /** Access particle cloud (for visualization). */
    const std::vector<Particle>& particles() const { return particles_; }

    /** Configuration. */
    void set_motion_noise(float alpha1, float alpha2, float alpha3, float alpha4);
    void set_min_max_particles(int min_p, int max_p);
    void set_kld_epsilon(float eps);

private:
    const GridMap* map_ = nullptr;
    std::vector<Particle> particles_;
    std::vector<Particle> resampled_;

    // Precomputed scan endpoints in robot frame
    std::vector<float> scan_xs_;
    std::vector<float> scan_ys_;

    // Random generator
    std::mt19937 rng_;
    std::normal_distribution<float> noise_dist_{0.0f, 1.0f};

    // Parameters
    int min_particles_ = 250;
    int max_particles_ = 2000;
    float kld_epsilon_ = 0.05f;
    float z_hit_ = 0.9f;
    float z_rand_ = 0.1f;
    float sigma_hit_ = 0.2f;

    // Motion model noise parameters (odometry model)
    float alpha1_ = 0.2f;  // rotation noise from rotation
    float alpha2_ = 0.2f;  // rotation noise from translation
    float alpha3_ = 0.2f;  // translation noise from translation
    float alpha4_ = 0.2f;  // translation noise from rotation

    void resample();
    void normalize_weights();
    float score_particle(const Particle& p) const;
    int compute_desired_particle_count() const;
    float sample_normal(float stddev);
};

} // namespace aimee_nav_core

/**
 * @file logging.h
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 11-06-2024
 * @copyright Copyright (c) 2024
 */

#ifndef SLAM_UTILITY_LOGGING_H_
#define SLAM_UTILITY_LOGGING_H_

#include <iomanip>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include <sophus/se3.hpp>

namespace slam_utility {

struct TrackingTimeLog {
    double timestamp = 0.0;
    double feature_extraction = 0.0;
    double stereo_matching = 0.0;
    double create_frame = 0.0;
    double track_motion = 0.0;
    double track_keyframe = 0.0;
    double track_map = 0.0;
    double update_motion = 0.0;
    double post_processing = 0.0;

    /**
     * @brief Set the Zero object
     *
     */
    void setZero() {
        timestamp = 0.0;
        feature_extraction = 0.0;
        stereo_matching = 0.0;
        create_frame = 0.0;
        track_motion = 0.0;
        track_keyframe = 0.0;
        track_map = 0.0;
        update_motion = 0.0;
        post_processing = 0.0;
    }

    friend std::ostream &operator<<(std::ostream &os,
                                    const TrackingTimeLog &l) {
        os << std::setprecision(6);
        os << l.timestamp << " " << l.feature_extraction << " "
           << l.stereo_matching << " " << l.create_frame << " "
           << l.track_motion << " " << l.track_keyframe << " " << l.track_map
           << " " << l.update_motion << " " << l.post_processing;
        return os;
    }

    static std::string header() {
        return "# timestamp feature_extraction stereo_matching "
               "create_frame track_motion track_keyframe track_map "
               "update_motion post_processing";
    }
};

struct MappingTimeLog {
    double timestamp = 0.0;
    double local_ba = 0.0;
    int num_fixed_kfs = 0;
    int num_opt_kfs = 0;
    int num_points = 0;
    int num_edges = 0;

    /**
     * @brief Set the Zero object
     *
     */
    void setZero() {
        timestamp = 0.0;
        local_ba = 0.0;
    }

    friend std::ostream &operator<<(std::ostream &os, const MappingTimeLog &l) {
        os << std::setprecision(6);
        os << l.timestamp << " " << l.local_ba << " " << l.num_fixed_kfs << " "
           << l.num_opt_kfs << " " << l.num_points;
        //    << l.num_edges;
        return os;
    }

    static std::string header() {
        return "# timestamp local_ba num_fixed_kfs num_opt_kfs num_points "
               "num_edges";
    }
};

/**
 * @brief Create a Disturbed S E3 object
 *
 * @param translation_noise
 * @param rotation_noise
 * @return Sophus::SE3d
 */
inline Sophus::SE3f createDisturbedSE3(float translation_noise,
                                       float rotation_noise) {
    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> dist_trans(0.0f, translation_noise);
    std::normal_distribution<float> dist_rot(0.0f, rotation_noise);

    // Add noise to translation
    Eigen::Vector3f disturbed_translation(dist_trans(gen), dist_trans(gen),
                                          dist_trans(gen));

    // Add noise to rotation (perturbation in the form of a small rotation
    // vector)
    Eigen::Vector3f rotation_noise_vec(dist_rot(gen), dist_rot(gen),
                                       dist_rot(gen));
    Eigen::Matrix3f disturbed_rotation =
        Sophus::SO3f::exp(rotation_noise_vec).matrix();

    // Return the disturbed SE3 transformation
    return Sophus::SE3f(Eigen::Quaternionf(disturbed_rotation),
                        disturbed_translation);
}

} // namespace slam_utility

#endif
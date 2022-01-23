#pragma once

#include <Eigen/Dense>

#include <ByteTrack/Rect.h>

namespace byte_track
{
class KalmanFilter
{
public:
    using DetectBox = Xyah<float>;
    using DetectBoxes = Eigen::Matrix<float, -1, 4, Eigen::RowMajor>;

    using StateMean = Eigen::Matrix<float, 1, 8, Eigen::RowMajor>;
    using StateCov = Eigen::Matrix<float, 8, 8, Eigen::RowMajor>;

    using StateHMean = Eigen::Matrix<float, 1, 4, Eigen::RowMajor>;
    using StateHCov = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;

    using State = std::pair<StateMean, StateCov>;
    using StateH = std::pair<StateHMean, StateHCov>;

    KalmanFilter(const float& std_weight_position = 1. / 20,
                 const float& std_weight_velocity = 1. / 160);
    State initiate(const Xyah<float>& measurement);
    void predict(StateMean& mean, StateCov& covariance);
    StateH project(const StateMean& mean, const StateCov& covariance);
    State update(const StateMean& mean,
        const StateCov& covariance,
        const Xyah<float>& measurement);

    Eigen::Matrix<float, 1, -1> calcGatingDistance(
        const StateMean& mean,
        const StateCov& covariance,
        const std::vector<Xyah<float>>& measurements,
        bool only_position = false);

private:
    Eigen::Matrix<float, 8, 8, Eigen::RowMajor> motion_mat_;
    Eigen::Matrix<float, 4, 8, Eigen::RowMajor> update_mat_;
    float std_weight_position_;
    float std_weight_velocity_;
};
}
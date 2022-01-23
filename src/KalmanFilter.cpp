#include <ByteTrack/KalmanFilter.h>

byte_track::KalmanFilter::KalmanFilter(const float& std_weight_position,
                                       const float& std_weight_velocity) :
    motion_mat_(Eigen::MatrixXf::Identity(8, 8)),
    update_mat_(Eigen::MatrixXf::Identity(4, 8)),
    std_weight_position_(std_weight_position),
    std_weight_velocity_(std_weight_velocity)
{
    constexpr size_t ndim = 4;
    constexpr double dt = 1.;
    for (size_t i = 0; i < ndim; i++)
    {
        motion_mat_(i, ndim + i) = dt;
    }
}

byte_track::KalmanFilter::State byte_track::KalmanFilter::initiate(const DetectBox &measurement)
{
    DetectBox mean_pos = measurement;
    DetectBox mean_vel;
    for (size_t i = 0; i < 4; i++)
        mean_vel(i) = 0;

    StateMean mean;
    for (size_t i = 0; i < 8; i++)
    {
        if (i < 4)
            mean(i) = mean_pos(i);
        else
            mean(i) = mean_vel(i - 4);
    }

    StateMean std;
    std(0) = 2 * std_weight_position_ * measurement[3];
    std(1) = 2 * std_weight_position_ * measurement[3];
    std(2) = 1e-2;
    std(3) = 2 * std_weight_position_ * measurement[3];
    std(4) = 10 * std_weight_velocity_ * measurement[3];
    std(5) = 10 * std_weight_velocity_ * measurement[3];
    std(6) = 1e-5;
    std(7) = 10 * std_weight_velocity_ * measurement[3];

    StateMean tmp = std.array().square();
    StateCov var = tmp.asDiagonal();
    return std::make_pair(mean, var);
}

void byte_track::KalmanFilter::predict(StateMean &mean, StateCov &covariance)
{
    //revise the data;
    DetectBox std_pos;
    std_pos << std_weight_position_ * mean(3),
        std_weight_position_ * mean(3),
        1e-2,
        std_weight_position_ * mean(3);
    DetectBox std_vel;
    std_vel << std_weight_velocity_ * mean(3),
        std_weight_velocity_ * mean(3),
        1e-5,
        std_weight_velocity_ * mean(3);
    StateMean tmp;
    tmp.block<1, 4>(0, 0) = std_pos;
    tmp.block<1, 4>(0, 4) = std_vel;
    tmp = tmp.array().square();
    StateCov motion_cov = tmp.asDiagonal();
    StateMean mean1 = this->motion_mat_ * mean.transpose();
    StateCov covariance1 = this->motion_mat_ * covariance * (motion_mat_.transpose());
    covariance1 += motion_cov;

    mean = mean1;
    covariance = covariance1;
}

byte_track::KalmanFilter::StateH byte_track::KalmanFilter::project(const StateMean &mean, const StateCov &covariance)
{
    DetectBox std;
    std << std_weight_position_ * mean(3), std_weight_position_ * mean(3),
        1e-1, std_weight_position_ * mean(3);
    StateHMean mean1 = update_mat_ * mean.transpose();
    StateHCov covariance1 = update_mat_ * covariance * (update_mat_.transpose());
    Eigen::Matrix<float, 4, 4> diag = std.asDiagonal();
    diag = diag.array().square().matrix();
    covariance1 += diag;
    //    covariance1.diagonal() << diag;
    return std::make_pair(mean1, covariance1);
}

byte_track::KalmanFilter::State byte_track::KalmanFilter::update(
    const StateMean &mean,
    const StateCov &covariance,
    const DetectBox &measurement)
{
    StateH pa = project(mean, covariance);
    StateHMean projected_mean = pa.first;
    StateHCov projected_cov = pa.second;

    //chol_factor, lower =
    //scipy.linalg.cho_factor(projected_cov, lower=True, check_finite=False)
    //kalmain_gain =
    //scipy.linalg.cho_solve((cho_factor, lower),
    //np.dot(covariance, self._upadte_mat.T).T,
    //check_finite=False).T
    Eigen::Matrix<float, 4, 8> B = (covariance * (update_mat_.transpose())).transpose();
    Eigen::Matrix<float, 8, 4> kalman_gain = (projected_cov.llt().solve(B)).transpose(); // eg.8x4
    Eigen::Matrix<float, 1, 4> innovation = measurement - projected_mean;                //eg.1x4
    auto tmp = innovation * (kalman_gain.transpose());
    StateMean new_mean = (mean.array() + tmp.array()).matrix();
    StateCov new_covariance = covariance - kalman_gain * projected_cov * (kalman_gain.transpose());
    return std::make_pair(new_mean, new_covariance);
}

Eigen::Matrix<float, 1, -1> byte_track::KalmanFilter::calcGatingDistance(
    const StateMean &mean,
    const StateCov &covariance,
    const std::vector<DetectBox> &measurements,
    bool only_position)
{
    StateH pa = this->project(mean, covariance);
    if (only_position)
    {
        printf("not implement!");
        exit(0);
    }
    StateHMean mean1 = pa.first;
    StateHCov covariance1 = pa.second;

    //    Eigen::Matrix<float, -1, 4, Eigen::RowMajor> d(size, 4);
    DetectBoxes d(measurements.size(), 4);
    int pos = 0;
    for (DetectBox box : measurements)
    {
        d.row(pos++) = box - mean1;
    }
    Eigen::Matrix<float, -1, -1, Eigen::RowMajor> factor = covariance1.llt().matrixL();
    Eigen::Matrix<float, -1, -1> z = factor.triangularView<Eigen::Lower>().solve<Eigen::OnTheRight>(d).transpose();
    auto zz = ((z.array()) * (z.array())).matrix();
    auto square_maha = zz.colwise().sum();
    return square_maha;
}
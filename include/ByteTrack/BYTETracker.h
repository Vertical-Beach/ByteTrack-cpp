#pragma once

#include <ByteTrack/STrack.h>
#include <ByteTrack/lapjv.h>
#include <ByteTrack/Object.h>

namespace byte_track
{
class BYTETracker
{
public:
    BYTETracker(const int& frame_rate = 30,
                const int& track_buffer = 30,
                const float& track_thresh = 0.5,
                const float& high_thresh = 0.6,
                const float& match_thresh = 0.8);
    ~BYTETracker();

    std::vector<STrack> update(const std::vector<Object>& objects);
    cv::Scalar getColor(int idx);

private:
    std::vector<STrack*> jointStracks(std::vector<STrack*> &tlista, std::vector<STrack> &tlistb);
    std::vector<STrack> jointStracks(std::vector<STrack> &tlista, std::vector<STrack> &tlistb);

    std::vector<STrack> subStracks(std::vector<STrack> &tlista, std::vector<STrack> &tlistb);
    void removeDuplicateStracks(std::vector<STrack> &resa, std::vector<STrack> &resb, std::vector<STrack> &stracksa, std::vector<STrack> &stracksb);

    void linearAssignment(std::vector<std::vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
                          std::vector<std::vector<int> > &matches, std::vector<int> &unmatched_a, std::vector<int> &unmatched_b);
    std::vector<std::vector<float> > calcIouDistance(std::vector<STrack*> &atracks, std::vector<STrack> &btracks, int &dist_size, int &dist_size_size);
    std::vector<std::vector<float> > calcIouDistance(std::vector<STrack> &atracks, std::vector<STrack> &btracks);
    std::vector<std::vector<float> > getIous(std::vector<byte_track::Tlbr<float>> &atlbrs, std::vector<byte_track::Tlbr<float>> &btlbrs);

    double execLapjv(const std::vector<std::vector<float> > &cost, std::vector<int> &rowsol, std::vector<int> &colsol,
        bool extend_cost = false, float cost_limit = LONG_MAX, bool return_cost = true);

private:
    const float track_thresh_;
    const float high_thresh_;
    const float match_thresh_;
    const int max_time_lost_;

    int frame_id_;

    std::vector<STrack> tracked_stracks_;
    std::vector<STrack> lost_stracks_;
    std::vector<STrack> removed_stracks_;
    KalmanFilter kalman_filter_;
};
}
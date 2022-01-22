#pragma once

#include <opencv2/opencv.hpp>

#include <ByteTrack/KalmanFilter.h>

namespace byte_track
{
enum TrackState { New = 0, Tracked, Lost, Removed };

class STrack
{
public:
    STrack(cv::Rect2f _rect, float score);
    ~STrack();

    void static multi_predict(std::vector<STrack*> &stracks, KalmanFilter &kalman_filter);

    void static_tlwh();
    void static_tlbr();

    void mark_lost();
    void mark_removed();
    int next_id();
    int end_frame();
    
    void activate(KalmanFilter &kalman_filter, int frame_id);
    void re_activate(STrack &new_track, int frame_id, bool new_id = false);
    void update(STrack &new_track, int frame_id);

public:
    bool is_activated;
    int track_id;
    int state;

    cv::Rect2f rect;
    std::vector<float> tlbr;

    int frame_id;
    int tracklet_len;
    int start_frame;

    KAL_MEAN mean;
    KAL_COVA covariance;
    float score;

private:
    KalmanFilter kalman_filter_;
};

std::vector<float> tlwh_to_xyah(const std::vector<float>& tlwh);
std::vector<float> tlwh_to_xyah(const cv::Rect2f& tlwh);
cv::Rect2f tlbr_to_tlwh(const std::vector<float> &tlbr);

}
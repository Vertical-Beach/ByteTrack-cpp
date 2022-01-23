#pragma once

#include <ByteTrack/Rect.h>
#include <ByteTrack/KalmanFilter.h>

namespace byte_track
{
enum TrackState { New = 0, Tracked, Lost, Removed };

class STrack
{
public:
    STrack(const Rect<float>& _rect, const float& _score);
    ~STrack();

    void static multi_predict(std::vector<STrack*> &stracks, KalmanFilter &kalman_filter);

    void updateRect();

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

    Rect<float> rect;
 
    int frame_id;
    int tracklet_len;
    int start_frame;

    KalmanFilter::StateMean mean;
    KalmanFilter::StateCov covariance;
    float score;

private:
    KalmanFilter kalman_filter_;
};
}
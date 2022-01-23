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

    void static multiPredict(std::vector<STrack*> &stracks, KalmanFilter &kalman_filter);

    void updateRect();

    void markAsLost();
    void markAsRemoved();
    int getNextId();
    int getEndFrame();
    
    void activate(KalmanFilter &kalman_filter, int frame_id);
    void reActivate(STrack &new_track, int frame_id, bool new_id = false);
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
#include <ByteTrack/STrack.h>

byte_track::STrack::STrack(const Rect<float>& _rect, const float& _score) :
    is_activated(false),
    track_id(0),
    state(TrackState::New),
    rect(_rect),
    frame_id(0),
    tracklet_len(0),
    start_frame(0),
    mean(),
    covariance(),
    score(_score),
    kalman_filter_()
{
}

byte_track::STrack::~STrack()
{
}

void byte_track::STrack::activate(KalmanFilter &kalman_filter, int frame_id)
{
    kalman_filter_ = kalman_filter;
    this->track_id = this->next_id();

    const auto xyah = rect.getXyah();
    DETECTBOX xyah_box;
    xyah_box[0] = xyah[0];
    xyah_box[1] = xyah[1];
    xyah_box[2] = xyah[2];
    xyah_box[3] = xyah[3];
    auto mc = kalman_filter_.initiate(xyah_box);
    this->mean = mc.first;
    this->covariance = mc.second;

    updateRect();

    this->tracklet_len = 0;
    this->state = TrackState::Tracked;
    if (frame_id == 1)
    {
        this->is_activated = true;
    }
    //this->is_activated = true;
    this->frame_id = frame_id;
    this->start_frame = frame_id;
}

void byte_track::STrack::re_activate(STrack &new_track, int frame_id, bool new_id)
{
    const auto xyah = new_track.rect.getXyah();
    DETECTBOX xyah_box;
    xyah_box[0] = xyah[0];
    xyah_box[1] = xyah[1];
    xyah_box[2] = xyah[2];
    xyah_box[3] = xyah[3];
    auto mc = kalman_filter_.update(this->mean, this->covariance, xyah_box);
    this->mean = mc.first;
    this->covariance = mc.second;

    updateRect();

    this->tracklet_len = 0;
    this->state = TrackState::Tracked;
    this->is_activated = true;
    this->frame_id = frame_id;
    this->score = new_track.score;
    if (new_id)
        this->track_id = next_id();
}

void byte_track::STrack::update(STrack &new_track, int frame_id)
{
    this->frame_id = frame_id;
    this->tracklet_len++;

    const auto xyah = new_track.rect.getXyah();
    DETECTBOX xyah_box;
    xyah_box[0] = xyah[0];
    xyah_box[1] = xyah[1];
    xyah_box[2] = xyah[2];
    xyah_box[3] = xyah[3];

    auto mc = kalman_filter_.update(this->mean, this->covariance, xyah_box);
    this->mean = mc.first;
    this->covariance = mc.second;

    updateRect();

    this->state = TrackState::Tracked;
    this->is_activated = true;

    this->score = new_track.score;
}

void byte_track::STrack::updateRect()
{
    rect.width() = mean[2] * mean[3];
    rect.height() = mean[3];
    rect.x() = mean[0] - rect.width() / 2;
    rect.y() = mean[1] - rect.height() / 2;
}

void byte_track::STrack::mark_lost()
{
    state = TrackState::Lost;
}

void byte_track::STrack::mark_removed()
{
    state = TrackState::Removed;
}

int byte_track::STrack::next_id()
{
    static int _count = 0;
    _count++;
    return _count;
}

int byte_track::STrack::end_frame()
{
    return this->frame_id;
}

void byte_track::STrack::multi_predict(std::vector<STrack*> &stracks, KalmanFilter &kalman_filter)
{
    for (size_t i = 0; i < stracks.size(); i++)
    {
        if (stracks[i]->state != TrackState::Tracked)
        {
            stracks[i]->mean[7] = 0;
        }
        kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
    }
}

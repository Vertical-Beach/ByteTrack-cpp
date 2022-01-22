#include <ByteTrack/STrack.h>

byte_track::STrack::STrack(cv::Rect2f _rect, float score)
{
    is_activated = false;
    track_id = 0;
    state = TrackState::New;

    rect = _rect;

    frame_id = 0;
    tracklet_len = 0;
    this->score = score;
    start_frame = 0;
}

byte_track::STrack::~STrack()
{
}

void byte_track::STrack::activate(KalmanFilter &kalman_filter, int frame_id)
{
    kalman_filter_ = kalman_filter;
    this->track_id = this->next_id();

    std::vector<float> xyah = tlwh_to_xyah(rect);
    DETECTBOX xyah_box;
    xyah_box[0] = xyah[0];
    xyah_box[1] = xyah[1];
    xyah_box[2] = xyah[2];
    xyah_box[3] = xyah[3];
    auto mc = kalman_filter_.initiate(xyah_box);
    this->mean = mc.first;
    this->covariance = mc.second;

    static_tlwh();

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
    std::vector<float> xyah = tlwh_to_xyah(new_track.rect);
    DETECTBOX xyah_box;
    xyah_box[0] = xyah[0];
    xyah_box[1] = xyah[1];
    xyah_box[2] = xyah[2];
    xyah_box[3] = xyah[3];
    auto mc = kalman_filter_.update(this->mean, this->covariance, xyah_box);
    this->mean = mc.first;
    this->covariance = mc.second;

    static_tlwh();

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

    std::vector<float> xyah = tlwh_to_xyah(new_track.rect);
    DETECTBOX xyah_box;
    xyah_box[0] = xyah[0];
    xyah_box[1] = xyah[1];
    xyah_box[2] = xyah[2];
    xyah_box[3] = xyah[3];

    auto mc = kalman_filter_.update(this->mean, this->covariance, xyah_box);
    this->mean = mc.first;
    this->covariance = mc.second;

    static_tlwh();

    this->state = TrackState::Tracked;
    this->is_activated = true;

    this->score = new_track.score;
}

void byte_track::STrack::static_tlwh()
{
    rect.width = mean[2] * mean[3];
    rect.height = mean[3];
    rect.x = mean[0] - rect.width / 2;
    rect.y = mean[1] - rect.height / 2;
}

std::vector<float> byte_track::STrack::getTlbr() const
{
    std::vector<float> tlbr(4);
    tlbr[0] = rect.x;
    tlbr[1] = rect.y;
    tlbr[2] = rect.x + rect.width;
    tlbr[3] = rect.y + rect.height;
    return tlbr;
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

std::vector<float> byte_track::tlwh_to_xyah(const std::vector<float>& tlwh)
{
    std::vector<float> xyah(4);
    xyah[0] = tlwh[0] + tlwh[2] / 2;
    xyah[1] = tlwh[1] + tlwh[3] / 2;
    xyah[2] = tlwh[2] / tlwh[3];
    xyah[3] = tlwh[3];
    return xyah;
}

std::vector<float> byte_track::tlwh_to_xyah(const cv::Rect2f& tlwh)
{
    std::vector<float> xyah(4);
    xyah[0] = tlwh.x + tlwh.width / 2;
    xyah[1] = tlwh.y + tlwh.height / 2;
    xyah[2] = tlwh.width / tlwh.height;
    xyah[3] = tlwh.height;
    return xyah;
}

cv::Rect2f byte_track::tlbr_to_tlwh(const std::vector<float> &tlbr)
{
    cv::Rect2f rect;
    rect.x = tlbr[0];
    rect.y = tlbr[1];
    rect.width = tlbr[2] - tlbr[0];
    rect.height = tlbr[3] - tlbr[1];
    return rect;
}

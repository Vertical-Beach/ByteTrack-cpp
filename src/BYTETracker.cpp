#include <ByteTrack/BYTETracker.h>

byte_track::BYTETracker::BYTETracker(const int& frame_rate,
                                     const int& track_buffer,
                                     const float& track_thresh,
                                     const float& high_thresh,
                                     const float& match_thresh) :
    track_thresh_(track_thresh),
    high_thresh_(high_thresh),
    match_thresh_(match_thresh),
    max_time_lost_(static_cast<int>(frame_rate / 30.0 * track_buffer)),
    frame_id_(0)
{
}

byte_track::BYTETracker::~BYTETracker()
{
}

std::vector<byte_track::STrack> byte_track::BYTETracker::update(const std::vector<Object>& objects)
{

    ////////////////// Step 1: Get detections //////////////////
    frame_id_++;
    std::vector<STrack> activated_stracks;
    std::vector<STrack> refind_stracks;
    std::vector<STrack> removed_stracks;
    std::vector<STrack> lost_stracks;
    std::vector<STrack> detections;
    std::vector<STrack> detections_low;

    std::vector<STrack> detections_cp;
    std::vector<STrack> tracked_stracks_swap;
    std::vector<STrack> resa, resb;
    std::vector<STrack> output_stracks;

    std::vector<STrack*> unconfirmed;
    std::vector<STrack*> tracked_stracks;
    std::vector<STrack*> strack_pool;
    std::vector<STrack*> r_tracked_stracks;

    if (objects.size() > 0)
    {
        for (size_t i = 0; i < objects.size(); i++)
        {
            Tlbr<float> tlbr_;
            tlbr_[0] = objects[i].rect.x;
            tlbr_[1] = objects[i].rect.y;
            tlbr_[2] = objects[i].rect.x + objects[i].rect.width;
            tlbr_[3] = objects[i].rect.y + objects[i].rect.height;

            float score = objects[i].prob;

            STrack strack(generate_rect_by_tlbr(tlbr_), score);
            if (score >= track_thresh_)
            {
                detections.push_back(strack);
            }
            else
            {
                detections_low.push_back(strack);
            }
            
        }
    }

    // Add newly detected tracklets to tracked_stracks
    for (size_t i = 0; i < tracked_stracks_.size(); i++)
    {
        if (!tracked_stracks_[i].is_activated)
            unconfirmed.push_back(&tracked_stracks_[i]);
        else
            tracked_stracks.push_back(&tracked_stracks_[i]);
    }

    ////////////////// Step 2: First association, with IoU //////////////////
    strack_pool = joint_stracks(tracked_stracks, lost_stracks_);
    STrack::multi_predict(strack_pool, kalman_filter_);

    std::vector<std::vector<float> > dists;
    int dist_size = 0, dist_size_size = 0;
    dists = iou_distance(strack_pool, detections, dist_size, dist_size_size);

    std::vector<std::vector<int> > matches;
    std::vector<int> u_track, u_detection;
    linear_assignment(dists, dist_size, dist_size_size, match_thresh_, matches, u_track, u_detection);

    for (size_t i = 0; i < matches.size(); i++)
    {
        STrack *track = strack_pool[matches[i][0]];
        STrack *det = &detections[matches[i][1]];
        if (track->state == TrackState::Tracked)
        {
            track->update(*det, frame_id_);
            activated_stracks.push_back(*track);
        }
        else
        {
            track->re_activate(*det, frame_id_, false);
            refind_stracks.push_back(*track);
        }
    }

    ////////////////// Step 3: Second association, using low score dets //////////////////
    for (size_t i = 0; i < u_detection.size(); i++)
    {
        detections_cp.push_back(detections[u_detection[i]]);
    }
    detections.clear();
    detections.assign(detections_low.begin(), detections_low.end());
    
    for (size_t i = 0; i < u_track.size(); i++)
    {
        if (strack_pool[u_track[i]]->state == TrackState::Tracked)
        {
            r_tracked_stracks.push_back(strack_pool[u_track[i]]);
        }
    }

    dists.clear();
    dists = iou_distance(r_tracked_stracks, detections, dist_size, dist_size_size);

    matches.clear();
    u_track.clear();
    u_detection.clear();
    linear_assignment(dists, dist_size, dist_size_size, 0.5, matches, u_track, u_detection);

    for (size_t i = 0; i < matches.size(); i++)
    {
        STrack *track = r_tracked_stracks[matches[i][0]];
        STrack *det = &detections[matches[i][1]];
        if (track->state == TrackState::Tracked)
        {
            track->update(*det, frame_id_);
            activated_stracks.push_back(*track);
        }
        else
        {
            track->re_activate(*det, frame_id_, false);
            refind_stracks.push_back(*track);
        }
    }

    for (size_t i = 0; i < u_track.size(); i++)
    {
        STrack *track = r_tracked_stracks[u_track[i]];
        if (track->state != TrackState::Lost)
        {
            track->mark_lost();
            lost_stracks.push_back(*track);
        }
    }

    // Deal with unconfirmed tracks, usually tracks with only one beginning frame
    detections.clear();
    detections.assign(detections_cp.begin(), detections_cp.end());

    dists.clear();
    dists = iou_distance(unconfirmed, detections, dist_size, dist_size_size);

    matches.clear();
    std::vector<int> u_unconfirmed;
    u_detection.clear();
    linear_assignment(dists, dist_size, dist_size_size, 0.7, matches, u_unconfirmed, u_detection);

    for (size_t i = 0; i < matches.size(); i++)
    {
        unconfirmed[matches[i][0]]->update(detections[matches[i][1]], frame_id_);
        activated_stracks.push_back(*unconfirmed[matches[i][0]]);
    }

    for (size_t i = 0; i < u_unconfirmed.size(); i++)
    {
        STrack *track = unconfirmed[u_unconfirmed[i]];
        track->mark_removed();
        removed_stracks.push_back(*track);
    }

    ////////////////// Step 4: Init new stracks //////////////////
    for (size_t i = 0; i < u_detection.size(); i++)
    {
        STrack *track = &detections[u_detection[i]];
        if (track->score < high_thresh_)
            continue;
        track->activate(kalman_filter_, frame_id_);
        activated_stracks.push_back(*track);
    }

    ////////////////// Step 5: Update state //////////////////
    for (size_t i = 0; i < lost_stracks_.size(); i++)
    {
        if (frame_id_ - lost_stracks_[i].end_frame() > max_time_lost_)
        {
            lost_stracks_[i].mark_removed();
            removed_stracks.push_back(lost_stracks_[i]);
        }
    }
    
    for (size_t i = 0; i < tracked_stracks_.size(); i++)
    {
        if (tracked_stracks_[i].state == TrackState::Tracked)
        {
            tracked_stracks_swap.push_back(tracked_stracks_[i]);
        }
    }
    tracked_stracks_.clear();
    tracked_stracks_.assign(tracked_stracks_swap.begin(), tracked_stracks_swap.end());

    tracked_stracks_ = joint_stracks(tracked_stracks_, activated_stracks);
    tracked_stracks_ = joint_stracks(tracked_stracks_, refind_stracks);

    lost_stracks_ = sub_stracks(lost_stracks_, tracked_stracks_);
    for (size_t i = 0; i < lost_stracks.size(); i++)
    {
        lost_stracks_.push_back(lost_stracks[i]);
    }

    lost_stracks_ = sub_stracks(lost_stracks_, removed_stracks_);
    for (size_t i = 0; i < removed_stracks.size(); i++)
    {
        removed_stracks_.push_back(removed_stracks[i]);
    }
    
    remove_duplicate_stracks(resa, resb, tracked_stracks_, lost_stracks_);

    tracked_stracks_.clear();
    tracked_stracks_.assign(resa.begin(), resa.end());
    lost_stracks_.clear();
    lost_stracks_.assign(resb.begin(), resb.end());
    
    for (size_t i = 0; i < tracked_stracks_.size(); i++)
    {
        if (tracked_stracks_[i].is_activated)
        {
            output_stracks.push_back(tracked_stracks_[i]);
        }
    }
    return output_stracks;
}
std::vector<byte_track::STrack*> byte_track::BYTETracker::joint_stracks(std::vector<STrack*> &tlista, std::vector<STrack> &tlistb)
{
    std::map<int, int> exists;
    std::vector<STrack*> res;
    for (size_t i = 0; i < tlista.size(); i++)
    {
        exists.insert(std::pair<int, int>(tlista[i]->track_id, 1));
        res.push_back(tlista[i]);
    }
    for (size_t i = 0; i < tlistb.size(); i++)
    {
        int tid = tlistb[i].track_id;
        if (!exists[tid] || exists.count(tid) == 0)
        {
            exists[tid] = 1;
            res.push_back(&tlistb[i]);
        }
    }
    return res;
}

std::vector<byte_track::STrack> byte_track::BYTETracker::joint_stracks(std::vector<STrack> &tlista, std::vector<STrack> &tlistb)
{
    std::map<int, int> exists;
    std::vector<STrack> res;
    for (size_t i = 0; i < tlista.size(); i++)
    {
        exists.insert(std::pair<int, int>(tlista[i].track_id, 1));
        res.push_back(tlista[i]);
    }
    for (size_t i = 0; i < tlistb.size(); i++)
    {
        int tid = tlistb[i].track_id;
        if (!exists[tid] || exists.count(tid) == 0)
        {
            exists[tid] = 1;
            res.push_back(tlistb[i]);
        }
    }
    return res;
}

std::vector<byte_track::STrack> byte_track::BYTETracker::sub_stracks(std::vector<STrack> &tlista, std::vector<STrack> &tlistb)
{
    std::map<int, STrack> stracks;
    for (size_t i = 0; i < tlista.size(); i++)
    {
        stracks.insert(std::pair<int, STrack>(tlista[i].track_id, tlista[i]));
    }
    for (size_t i = 0; i < tlistb.size(); i++)
    {
        int tid = tlistb[i].track_id;
        if (stracks.count(tid) != 0)
        {
            stracks.erase(tid);
        }
    }

    std::vector<STrack> res;
    std::map<int, STrack>::iterator  it;
    for (it = stracks.begin(); it != stracks.end(); ++it)
    {
        res.push_back(it->second);
    }

    return res;
}

void byte_track::BYTETracker::remove_duplicate_stracks(std::vector<STrack> &resa, std::vector<STrack> &resb, std::vector<STrack> &stracksa, std::vector<STrack> &stracksb)
{
    std::vector<std::vector<float> > pdist = iou_distance(stracksa, stracksb);
    std::vector<std::pair<int, int> > pairs;
    for (size_t i = 0; i < pdist.size(); i++)
    {
        for (size_t j = 0; j < pdist[i].size(); j++)
        {
            if (pdist[i][j] < 0.15)
            {
                pairs.push_back(std::pair<int, int>(i, j));
            }
        }
    }

    std::vector<int> dupa, dupb;
    for (size_t i = 0; i < pairs.size(); i++)
    {
        int timep = stracksa[pairs[i].first].frame_id - stracksa[pairs[i].first].start_frame;
        int timeq = stracksb[pairs[i].second].frame_id - stracksb[pairs[i].second].start_frame;
        if (timep > timeq)
            dupb.push_back(pairs[i].second);
        else
            dupa.push_back(pairs[i].first);
    }

    for (size_t i = 0; i < stracksa.size(); i++)
    {
        std::vector<int>::iterator iter = std::find(dupa.begin(), dupa.end(), i);
        if (iter == dupa.end())
        {
            resa.push_back(stracksa[i]);
        }
    }

    for (size_t i = 0; i < stracksb.size(); i++)
    {
        std::vector<int>::iterator iter = std::find(dupb.begin(), dupb.end(), i);
        if (iter == dupb.end())
        {
            resb.push_back(stracksb[i]);
        }
    }
}

void byte_track::BYTETracker::linear_assignment(std::vector<std::vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
    std::vector<std::vector<int> > &matches, std::vector<int> &unmatched_a, std::vector<int> &unmatched_b)
{
    if (cost_matrix.size() == 0)
    {
        for (int i = 0; i < cost_matrix_size; i++)
        {
            unmatched_a.push_back(i);
        }
        for (int i = 0; i < cost_matrix_size_size; i++)
        {
            unmatched_b.push_back(i);
        }
        return;
    }

    std::vector<int> rowsol; std::vector<int> colsol;
    lapjv(cost_matrix, rowsol, colsol, true, thresh);
    for (size_t i = 0; i < rowsol.size(); i++)
    {
        if (rowsol[i] >= 0)
        {
            std::vector<int> match;
            match.push_back(i);
            match.push_back(rowsol[i]);
            matches.push_back(match);
        }
        else
        {
            unmatched_a.push_back(i);
        }
    }

    for (size_t i = 0; i < colsol.size(); i++)
    {
        if (colsol[i] < 0)
        {
            unmatched_b.push_back(i);
        }
    }
}

std::vector<std::vector<float> > byte_track::BYTETracker::ious(std::vector<byte_track::Tlbr<float>> &atlbrs, std::vector<byte_track::Tlbr<float>> &btlbrs)
{
    std::vector<std::vector<float> > ious;
    if (atlbrs.size()*btlbrs.size() == 0)
        return ious;

    ious.resize(atlbrs.size());
    for (size_t i = 0; i < ious.size(); i++)
    {
        ious[i].resize(btlbrs.size());
    }

    //bbox_ious
    for (size_t k = 0; k < btlbrs.size(); k++)
    {
        std::vector<float> ious_tmp;
        float box_area = (btlbrs[k][2] - btlbrs[k][0] + 1)*(btlbrs[k][3] - btlbrs[k][1] + 1);
        for (size_t n = 0; n < atlbrs.size(); n++)
        {
            float iw = std::min(atlbrs[n][2], btlbrs[k][2]) - std::max(atlbrs[n][0], btlbrs[k][0]) + 1;
            if (iw > 0)
            {
                float ih = std::min(atlbrs[n][3], btlbrs[k][3]) - std::max(atlbrs[n][1], btlbrs[k][1]) + 1;
                if(ih > 0)
                {
                    float ua = (atlbrs[n][2] - atlbrs[n][0] + 1)*(atlbrs[n][3] - atlbrs[n][1] + 1) + box_area - iw * ih;
                    ious[n][k] = iw * ih / ua;
                }
                else
                {
                    ious[n][k] = 0.0;
                }
            }
            else
            {
                ious[n][k] = 0.0;
            }
        }
    }

    return ious;
}

std::vector<std::vector<float> > byte_track::BYTETracker::iou_distance(std::vector<STrack*> &atracks, std::vector<STrack> &btracks, int &dist_size, int &dist_size_size)
{
    std::vector<std::vector<float> > cost_matrix;
    if (atracks.size() * btracks.size() == 0)
    {
        dist_size = atracks.size();
        dist_size_size = btracks.size();
        return cost_matrix;
    }
    std::vector<byte_track::Tlbr<float>> atlbrs, btlbrs;
    for (size_t i = 0; i < atracks.size(); i++)
    {
        atlbrs.push_back(atracks[i]->rect.getTlbr());
    }
    for (size_t i = 0; i < btracks.size(); i++)
    {
        btlbrs.push_back(btracks[i].rect.getTlbr());
    }

    dist_size = atracks.size();
    dist_size_size = btracks.size();

    std::vector<std::vector<float> > _ious = ious(atlbrs, btlbrs);
    
    for (size_t i = 0; i < _ious.size();i++)
    {
        std::vector<float> _iou;
        for (size_t j = 0; j < _ious[i].size(); j++)
        {
            _iou.push_back(1 - _ious[i][j]);
        }
        cost_matrix.push_back(_iou);
    }

    return cost_matrix;
}

std::vector<std::vector<float> > byte_track::BYTETracker::iou_distance(std::vector<STrack> &atracks, std::vector<STrack> &btracks)
{
    std::vector<byte_track::Tlbr<float>> atlbrs, btlbrs;
    for (size_t i = 0; i < atracks.size(); i++)
    {
        atlbrs.push_back(atracks[i].rect.getTlbr());
    }
    for (size_t i = 0; i < btracks.size(); i++)
    {
        btlbrs.push_back(btracks[i].rect.getTlbr());
    }

    std::vector<std::vector<float> > _ious = ious(atlbrs, btlbrs);
    std::vector<std::vector<float> > cost_matrix;
    for (size_t i = 0; i < _ious.size(); i++)
    {
        std::vector<float> _iou;
        for (size_t j = 0; j < _ious[i].size(); j++)
        {
            _iou.push_back(1 - _ious[i][j]);
        }
        cost_matrix.push_back(_iou);
    }

    return cost_matrix;
}

double byte_track::BYTETracker::lapjv(const std::vector<std::vector<float> > &cost, std::vector<int> &rowsol, std::vector<int> &colsol,
    bool extend_cost, float cost_limit, bool return_cost)
{
    std::vector<std::vector<float> > cost_c;
    cost_c.assign(cost.begin(), cost.end());

    std::vector<std::vector<float> > cost_c_extended;

    int n_rows = cost.size();
    int n_cols = cost[0].size();
    rowsol.resize(n_rows);
    colsol.resize(n_cols);

    int n = 0;
    if (n_rows == n_cols)
    {
        n = n_rows;
    }
    else
    {
        if (!extend_cost)
        {
            throw std::runtime_error("The `extend_cost` variable should set True");
        }
    }
        
    if (extend_cost || cost_limit < LONG_MAX)
    {
        n = n_rows + n_cols;
        cost_c_extended.resize(n);
        for (size_t i = 0; i < cost_c_extended.size(); i++)
            cost_c_extended[i].resize(n);

        if (cost_limit < LONG_MAX)
        {
            for (size_t i = 0; i < cost_c_extended.size(); i++)
            {
                for (size_t j = 0; j < cost_c_extended[i].size(); j++)
                {
                    cost_c_extended[i][j] = cost_limit / 2.0;
                }
            }
        }
        else
        {
            float cost_max = -1;
            for (size_t i = 0; i < cost_c.size(); i++)
            {
                for (size_t j = 0; j < cost_c[i].size(); j++)
                {
                    if (cost_c[i][j] > cost_max)
                        cost_max = cost_c[i][j];
                }
            }
            for (size_t i = 0; i < cost_c_extended.size(); i++)
            {
                for (size_t j = 0; j < cost_c_extended[i].size(); j++)
                {
                    cost_c_extended[i][j] = cost_max + 1;
                }
            }
        }

        for (size_t i = n_rows; i < cost_c_extended.size(); i++)
        {
            for (size_t j = n_cols; j < cost_c_extended[i].size(); j++)
            {
                cost_c_extended[i][j] = 0;
            }
        }
        for (int i = 0; i < n_rows; i++)
        {
            for (int j = 0; j < n_cols; j++)
            {
                cost_c_extended[i][j] = cost_c[i][j];
            }
        }

        cost_c.clear();
        cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
    }

    double **cost_ptr;
    cost_ptr = new double *[sizeof(double *) * n];
    for (int i = 0; i < n; i++)
        cost_ptr[i] = new double[sizeof(double) * n];

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            cost_ptr[i][j] = cost_c[i][j];
        }
    }

    int* x_c = new int[sizeof(int) * n];
    int *y_c = new int[sizeof(int) * n];

    int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
    if (ret != 0)
    {
        throw std::runtime_error("The result of lapjv_internal() is invalid.");
    }

    double opt = 0.0;

    if (n != n_rows)
    {
        for (int i = 0; i < n; i++)
        {
            if (x_c[i] >= n_cols)
                x_c[i] = -1;
            if (y_c[i] >= n_rows)
                y_c[i] = -1;
        }
        for (int i = 0; i < n_rows; i++)
        {
            rowsol[i] = x_c[i];
        }
        for (int i = 0; i < n_cols; i++)
        {
            colsol[i] = y_c[i];
        }

        if (return_cost)
        {
            for (size_t i = 0; i < rowsol.size(); i++)
            {
                if (rowsol[i] != -1)
                {
                    opt += cost_ptr[i][rowsol[i]];
                }
            }
        }
    }
    else if (return_cost)
    {
        for (size_t i = 0; i < rowsol.size(); i++)
        {
            opt += cost_ptr[i][rowsol[i]];
        }
    }

    for (int i = 0; i < n; i++)
    {
        delete[]cost_ptr[i];
    }
    delete[]cost_ptr;
    delete[]x_c;
    delete[]y_c;

    return opt;
}

cv::Scalar byte_track::BYTETracker::get_color(int idx)
{
    idx += 3;
    return cv::Scalar(37 * idx % 255, 17 * idx % 255, 29 * idx % 255);
}
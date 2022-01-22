#include <ByteTrack/Object.h>

byte_track::Object::Object(const cv::Rect2f &_rect,
                           const int &_label,
                           const float &_prob) : rect(_rect), label(_label), prob(_prob)
{
}
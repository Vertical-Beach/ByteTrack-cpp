#pragma once

#include <array>

namespace byte_track
{
template<typename T>
using Tlwh = std::array<T, 4>;

template<typename T>
using Tlbr = std::array<T, 4>;

template<typename T>
using Xyah = std::array<T, 4>;

template<typename T>
class Rect
{
    public:
    Tlwh<T> tlwh;

    Rect() = default;
    Rect(const T &x, const T &y, const T &width, const T &height);

    ~Rect();

    const T &x() const;
    const T &y() const;
    const T &width() const;
    const T &height() const;

    T &x();
    T &y();
    T &width();
    T &height();

    const T &tl_x() const;
    const T &tl_y() const;
    T br_x() const;
    T br_y() const;

    Tlbr<T> getTlbr() const;
    Xyah<T> getXyah() const;
};

template<typename T>
Rect<T> generate_rect_by_tlbr(const Tlbr<T>& tlbr);

template<typename T>
Rect<T> generate_rect_by_xyah(const Xyah<T>& xyah);

}
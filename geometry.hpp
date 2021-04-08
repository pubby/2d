#ifndef i2d_GEOMETRY_HPP
#define i2d_GEOMETRY_HPP

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <functional>
#include <iterator>
#include <utility>
#include <vector>

#include "units.hpp"

namespace i2d {

constexpr coord_t left_n(coord_t c, int_t n)  
    { return coord_t{ c.x - n, c.y }; }
constexpr coord_t right_n(coord_t c, int_t n)
    { return coord_t{ c.x + n, c.y }; }
constexpr coord_t up_n(coord_t c, int_t n)
    { return coord_t{ c.x, c.y - n }; }
constexpr coord_t down_n(coord_t c, int_t n)
    { return coord_t{ c.x, c.y + n }; }

constexpr coord_t left1(coord_t c)  { return left_n(c, 1); }
constexpr coord_t right1(coord_t c) { return right_n(c, 1); }
constexpr coord_t up1(coord_t c)    { return up_n(c, 1); }
constexpr coord_t down1(coord_t c)  { return down_n(c, 1); }

namespace impl {
    template<typename T>
    constexpr auto sqr(T t) { return t * t; }

    inline int_t gcd(int_t a, int_t b)
    {
        while (b != 0)
        {
            a %= b;
            std::swap(a, b);
        }
        return a;
    }
} // namespace impl

template<typename T = int_t>
constexpr T dot_product(coord_t c1, coord_t c2)
{
    return (static_cast<T>(c1.x) * static_cast<T>(c2.x)
            + static_cast<T>(c1.y) * static_cast<T>(c2.y));
}

constexpr int_t area(dimen_t d) { return d.w * d.h; }
constexpr int_t area(rect_t r) { return area(r.d); }

// Given a 5x3 rect_t:
//   -----
// | xxxxx |
// | x   x |
// | xxxxx |
//   -----
//   perimeter: number of | and - characters (16)
//   inner_perimeter: number of x characters (12)
constexpr int_t perimeter(dimen_t d) { return 2 * d.w + 2 * d.h; }
constexpr int_t perimeter(rect_t r) { return perimeter(r.d); }
constexpr int_t inner_perimeter(dimen_t d) { return 2*(d.w-1) + 2*(d.h-1); }
constexpr int_t inner_perimeter(rect_t r) { return inner_perimeter(r.d); }

inline int_t c_dist(coord_t c1, coord_t c2) // chess distance
{
    return std::max(std::abs(c1.x - c2.x), std::abs(c1.y - c2.y));
}

inline int_t m_dist(coord_t c1, coord_t c2) // manhattan distance
{
    return std::abs(c1.x - c2.x) + std::abs(c1.y - c2.y);
}

inline double e_dist(coord_t c1, coord_t c2) // euclidian distance
{
    return std::sqrt(impl::sqr(c1.x - c2.x) + impl::sqr(c1.y - c2.y));
}

// Reduces the fraction representind direction
inline coord_t simplify_dir(coord_t direction)
{
    int_t const gcd_value = impl::gcd(direct_tion.x, direct_tion.y);
    components([&direct_tion, gcd_value](auto c)
               { direct_tion[c] /= gcd_value; });
    return direction;
}

// Returns in range [-pi, pi].
inline double dir_to_rad(coord_t dir)
{
    return std::atan2(-dir.y, dir.x);
}

inline coord_t rad_to_dir(double rad, int_t length)
{
    return
    {
        (int_t)std::round(std::cos(rad) * length),
        (int_t)std::round(-std::sin(rad) * length),
    };
}

constexpr rect_t to_rect(dimen_t dim) { return { {0,0}, dim }; }
constexpr coord_t to_coord(dimen_t dim) { return { dim.w, dim.h }; }

constexpr bool in_bounds(coord_t crd, rect_t r)
{
    return (crd.x >= r.c.x && crd.y >= r.c.y
            && crd.x < r.ex() && crd.y < r.ey());
}

constexpr bool in_bounds(coord_t crd, dimen_t dim)
{
    return in_bounds(crd, to_rect(dim));
}

constexpr bool in_bounds(rect_t sub, rect_t super)
{
    return (sub.c.x >= super.c.x
            && sub.c.y >= super.c.y
            && sub.ex() <= super.ex()
            && sub.ey() <= super.ey());
}

constexpr bool in_bounds(rect_t sub, dimen_t dim)
{
    return in_bounds(sub, to_rect(dim));
}

constexpr bool in_bounds(dimen_t sub, dimen_t super)
{
    return in_bounds(to_rect(sub), to_rect(super));
}

constexpr bool overlapping(rect_t r1, rect_t r2)
{
    return (r1.c.x < r2.ex() && r1.ex() > r2.c.x
            && r1.c.y < r2.ey() && r1.ey() > r2.c.y);
}

// Minimum bounding box that contains 2 coords
inline rect_t rect_from_2_coords(coord_t c1, coord_t c2)
{
    using std::swap;
    if(c1.x > c2.x)
        swap(c1.x, c2.x);
    if(c1.y > c2.y)
        swap(c1.y, c2.y);
    return { c1, { c2.x - c1.x + 1, c2.y - c1.y + 1 } };
}

template<typename It>
rect_t rect_from_n_coords(It begin, It end)
{
    coord_t c = *begin;
    coord_t e = *begin;
    for(It it = begin; it != end; ++it)
    {
        c.x = std::min(c.x, it->x);
        c.y = std::min(c.y, it->y);
        e.x = std::max(e.x, it->x);
        e.y = std::max(e.y, it->y);
    }
    return { c, { e.x - c.x + 1, e.y - c.y + 1 } };
}

inline rect_t grow_rect_to_contain(rect_t r, coord_t to_hold)
{
    if(area(r) == 0)
        return {to_hold, {1,1}};

    std::array<coord_t, 5> crds =
    {{
        to_hold,
        r.nw(),
        r.ne(),
        r.sw(),
        r.se(),
    }};
    return rect_from_n_coord_ts(crds.begin(), crds.end());
}

inline rect_t grow_rect_to_contain(rect_t r1, rect_t r2)
{
    if(area(r1) == 0)
        return r2;
    if(area(r2) == 0)
        return r1;

    std::array<coord_t, 8> crds =
    {{
        r1.nw(), r1.ne(), r1.sw(), r1.se(),
        r2.nw(), r2.ne(), r2.sw(), r2.se(),
    }};
    return rect_from_n_coord_ts(crds.begin(), crds.end());
}

inline coord_t crop(coord_t crd, rect_t super)
{
    crd.x = std::min(std::max(crd.x, super.c.x), super.rx());
    crd.y = std::min(std::max(crd.y, super.c.y), super.ry());
    return crd;
}

inline dimen_t crop(dimen_t too_big, dimen_t crop_boundary)
{
    components(
        [&](auto c) { too_big[c] = std::min(too_big[c], crop_boundary[c]); });
    return too_big;
}

inline rect_t crop(rect_t too_big, rect_t crop_boundary)
{
    coord_t c1 = crop(too_big.c, crop_boundary);
    coord_t c2 = crop(too_big.r(), crop_boundary);
    return rect_from_2_coord_ts(c1, c2);
}

inline rect_t rect_from_radius(coord_t center, int_t rad)
{
    int_t const d = rad*2 + 1;
    return { center - coord_t{rad,rad}, dimen_t{d,d} };
}

inline coord_t rect_center(rect_t r)
{
    return { (r.c.x + r.ex()) / 2, (r.c.y + r.ey()) / 2 };
}

inline rect_t centered_rect(coord_t center_point, dimen_t dim)
{
    rect_t r;
    components(
        [&](auto c) { r.c[c] = center_point[c] - dim[c] / 2; });
    r.d = dim;
    return r;
}

inline rect_t centered_inside(dimen_t dim, rect_t in)
{
    dim = crop(dim, in.d);
    coord_t center = rect_center(in);
    return { center - to_coord(dim/2), dim };
}

inline rect_t rect_margin(rect_t r, 
                          int_t left, int_t top, int_t right, int_t bottom)
{
    r.c.x += left;
    r.c.y += right;
    r.d.w = std::max(0, r.d.w - left - right);
    r.d.h = std::max(0, r.d.h - top - bottom);
    return r;
}

inline rect_t rect_margin(rect_t r, int_t margin)
{
    return rect_margin(r, margin, margin, margin, margin);
}

inline rect_t rect_margin(rect_t r, int_t x_margin, int_t y_margin)
{
    return rect_margin(r, x_margin, y_margin, x_margin, y_margin);
}

class rect_iterator
: public std::iterator<std::forward_iteratorag, coord_t const>
{
    friend class rect_range;
public:
    rect_iterator() = default;

    coord_t operator*() const { return m_current; }
    coord_t const* operator->() const { return &m_current; }

    rect_iterator& operator++()
    {
        if(++m_current.x == m_rect.ex())
        {
            m_current.x = m_rect.c.x;
            ++m_current.y;
        }
        return *this;
    }

    rect_iterator operator++(int_t)
    {
        rect_iterator ret = *this;
        ++(*this);
        return ret;
    }

    rect_t rect() const { return m_rect; }
private:
    struct begin_tag {};
    struct end_tag {};

    rect_iterator(rect_t r, begin_tag)
    : m_rect(r)
    , m_current(r.c)
    {}

    rect_iterator(rect_t r, end_tag)
    : m_rect(r)
    , m_current{ r.c.x, r.ey() }
    {}

    rect_t m_rect;
    coord_t m_current;
};

inline bool operator==(rect_iterator lhs, rect_iterator rhs)
{
    assert(lhs.rect() == rhs.rect());
    return *lhs == *rhs;
}

inline bool operator!=(rect_iterator lhs, rect_iterator rhs)
{
    assert(lhs.rect() == rhs.rect());
    return !(lhs == rhs);
}


class rect_edge_iterator
: public std::iterator<std::forward_iteratorag, coord_t const>
{
    friend class rect_edge_range;
public:
    rect_edge_iterator() = default;

    coord_t operator*() const { return m_current; }
    coord_t const* operator->() const { return &m_current; }

    rect_edge_iterator& operator++()
    {
        switch(m_mode) {
        case 0:
            if(m_current.x < m_rect.rx())
            {
                ++m_current.x;
                return *this;
            }
            m_mode = 1;
        case 1:
            if(m_current.y < m_rect.ry())
            {
                ++m_current.y;
                return *this;
            }
            m_mode = 2;
        case 2:
            if(m_current.x > m_rect.c.x)
            {
                --m_current.x;
                return *this;
            }
            m_mode = 3;
        case 3:
            if(m_current.y > m_rect.c.y + 1)
            {
                --m_current.y;
                return *this;
            }
            m_mode = 4;
        case 4:
            --m_current.x;
        }
        return *this;
    }

    rect_edge_iterator operator++(int_t)
    {
        rect_edge_iterator ret = *this;
        ++(*this);
        return ret;
    }

    rect_t rect() const { return m_rect; }
private:
    struct begin_tag {};
    struct end_tag {};

    rect_edge_iterator(rect_t r, begin_tag)
    : m_rect(r)
    , m_current(r.c)
    , m_mode(0)
    {}

    rect_edge_iterator(rect_t r, end_tag)
    : m_rect(r)
    , m_current{ r.c.x - 1, r.c.y + 1 }
    , m_mode(4)
    {}

    rect_t m_rect;
    coord_t m_current;
    int_t m_mode;
};

inline bool operator==(rect_edge_iterator lhs,
                       rect_edge_iterator rhs)
{
    assert(lhs.rect() == rhs.rect());
    return *lhs == *rhs;
}

inline bool operator!=(rect_edge_iterator lhs,
                       rect_edge_iterator rhs)
{
    return !(lhs == rhs);
}

template<std::size_t N = 8>
std::array<coord_t, N> dir_range;

template<>
constexpr std::array<coord_t, 8> dir_range<8> =
{{
    {  1,  0 },
    {  1, -1 },
    {  0, -1 },
    { -1, -1 },
    { -1,  0 },
    { -1,  1 },
    {  0,  1 },
    {  1,  1 },
}};

template<>
constexpr std::array<coord_t, 4> dir_range<4> =
{{
    {  1,  0 },
    {  0, -1 },
    { -1,  0 },
    {  0,  1 },
}};

class adjacent_iterator
: public std::iterator<std::forward_iteratorag, coord_t const>
{
friend class adjacent_range;
public:
    adjacent_iterator() = default;

    coord_t operator*() const { return m_current; }
    coord_t const* operator->() const { return &m_current; }

    adjacent_iterator& operator++()
    {
        ++m_current.x;
        if(m_current.x > m_center.x + 1)
        {
            m_current.x = m_center.x - 1;
            ++m_current.y;
        }
        else if(m_current == m_center)
            ++m_current.x;
        return *this;
    }

    adjacent_iterator operator++(int_t)
    {
        adjacent_iterator ret = *this;
        ++(*this);
        return ret;
    }

    coord_t center() const { return m_center; }
private:
    struct begin_tag {};
    struct end_tag {};

    adjacent_iterator(coord_t center, begin_tag)
    : m_current(center + coord_t{ -1, -1 })
    , m_center(center)
    {}

    adjacent_iterator(coord_t center, end_tag)
    : m_current(center + coord_t{ -1, 2 })
    , m_center(center)
    {}

    coord_t m_current;
    coord_t m_center;
};

inline bool operator==(adjacent_iterator lhs, adjacent_iterator rhs)
{
    return *lhs == *rhs;
}

inline bool operator!=(adjacent_iterator lhs, adjacent_iterator rhs)
{
    return *lhs != *rhs;
}

class rect_range
{
public:
    using const_iterator = rect_iterator;

    rect_range() : rect_range(rect_t{}) {}
    rect_range(rect_t r)
    : m_begin(r, rect_iterator::begin_tag())
    , m_end(r, rect_iterator::end_tag())
    {}

    rect_iterator begin() const { return m_begin; }
    rect_iterator end() const { return m_end; }

    rect_iterator cbegin() const { return begin(); }
    rect_iterator cend() const { return end(); }

    rect_t rect() const { return m_begin.rect(); }
private:
    rect_iterator m_begin;
    rect_iterator m_end;
};

inline rect_range dimen_range(dimen_t dim)
{
    return rect_range(to_rect(dim));
}

inline rect_range circular_range(coord_t crd, int_t rad)
{
    return rect_range(rect_from_radius(crd, rad));
}

class rect_edge_range
{
public:
    using const_iterator = rect_edge_iterator;

    rect_edge_range() : rect_edge_range(rect_t{}) {}
    rect_edge_range(rect_t r)
    : m_begin(r, rect_edge_iterator::begin_tag())
    , m_end(r, rect_edge_iterator::end_tag())
    {}

    rect_edge_iterator begin() const { return m_begin; }
    rect_edge_iterator end() const { return m_end; }

    rect_edge_iterator cbegin() const { return begin(); }
    rect_edge_iterator cend() const { return cend(); }

    rect_t rect() const { return m_begin.rect(); }
private:
    rect_edge_iterator m_begin;
    rect_edge_iterator m_end;
};

inline rect_edge_range radius_range(coord_t center, int_t rad)
{
    return rect_edge_range(rect_from_radius(center, rad));
}

class adjacent_range
{
public:
    using const_iterator = adjacent_iterator;

    adjacent_range() = delete;

    adjacent_range(coord_t center)
    : m_begin(center, adjacent_iterator::begin_tag())
    , m_end(center, adjacent_iterator::end_tag())
    {}

    adjacent_iterator begin() const { return m_begin; }
    adjacent_iterator end() const { return m_end; }

    adjacent_iterator cbegin() const { return begin(); }
    adjacent_iterator cend() const { return cend(); }

    coord_t center() const { return m_begin.center(); }

    static constexpr int_t const_size = 8;
    constexpr int_t size() const { return const_size; }
private:
    adjacent_iterator m_begin;
    adjacent_iterator m_end;
};

} // namespace i2d

#endif

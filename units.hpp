#ifndef i2d_UNITS_HPP
#define i2d_UNITS_HPP

// Definitions of simple 2d units without much extra.
// Try to keep functions that work on these units in other files,
// such as in geometry.hpp

// REMEMBER: x=right, y=down.
//    x
//  +---->
//  |
// y|
//  v

#include <cstdint>
#include <type_traits>
#include <utility>

namespace i2d {

using int2d_t = std::int32_t;

// Use this to switch components at compile-time using templates.
template<std::size_t N>
struct component_index : std::integral_constant<std::size_t, N>
{
    static constexpr std::size_t index() { return N; }
};

// Call a function for each component
template<typename Func>
void components(Func func)
{
    func(component_index<0>{});
    func(component_index<1>{});
}

// Map over a unit's components
template<typename T, typename Func>
T mapc(T t, Func func)
{
    T ret;
    components([&](auto c) { ret[c] = func(t[c]); });
    return ret;
}

struct dimen_t
{
    static constexpr std::size_t components = 2;

    int2d_t w;
    int2d_t h;

    int2d_t const& operator[](component_index<0>) const { return w; }
    int2d_t& operator[](component_index<0>) { return w; }
    int2d_t const& operator[](component_index<1>) const { return h; }
    int2d_t& operator[](component_index<1>) { return h; }

    constexpr explicit operator bool() const { return w | h; }
};

[[gnu::always_inline]]
constexpr bool operator==(dimen_t lhs, dimen_t rhs)
{
    return lhs.w == rhs.w && lhs.h == rhs.h;
}

[[gnu::always_inline]]
constexpr bool operator!=(dimen_t lhs, dimen_t rhs)
{
    return !(lhs == rhs);
}

[[gnu::always_inline]]
constexpr bool operator<(dimen_t lhs, dimen_t rhs)
{
    return lhs.w != rhs.w ? lhs.w < rhs.w : lhs.h < rhs.h;
}

constexpr dimen_t operator+(dimen_t lhs, dimen_t rhs)
{
    return { lhs.w + rhs.w, lhs.h + rhs.h };
}

constexpr dimen_t operator-(dimen_t lhs, dimen_t rhs)
{
    return { lhs.w - rhs.w, lhs.h - rhs.h };
}

inline dimen_t& operator+=(dimen_t& lhs, dimen_t rhs)
{
    lhs = lhs + rhs;
    return lhs;
}

inline dimen_t& operator-=(dimen_t& lhs, dimen_t rhs)
{
    lhs = lhs - rhs;
    return lhs;
}

constexpr dimen_t operator+(dimen_t lhs)
{
    return lhs;
}

constexpr dimen_t operator-(dimen_t lhs)
{
    return { -lhs.w, -lhs.h };
}

[[gnu::always_inline]]
constexpr dimen_t operator*(dimen_t lhs, int2d_t scale)
{
    return { lhs.w * scale, lhs.h * scale };
}

[[gnu::always_inline]]
constexpr dimen_t operator/(dimen_t lhs, int2d_t scale)
{
    return { lhs.w / scale, lhs.h / scale };
}

[[gnu::always_inline]]
constexpr dimen_t vec_mul(dimen_t dim, int2d_t v)
{
    return dim * v;
}

[[gnu::always_inline]]
constexpr dimen_t vec_div(dimen_t dim, int2d_t v)
{
    return { dim.w / v, dim.h / v };
}

[[gnu::always_inline]]
constexpr dimen_t dimen_t_add(dimen_t d1, dimen_t d2)
{
    return { d1.w + d2.h, d1.h + d2.h };
}

struct coord_t
{
    static constexpr std::size_t components = 2;

    int2d_t x;
    int2d_t y;

    int2d_t const& operator[](component_index<0>) const { return x; }
    int2d_t& operator[](component_index<0>) { return x; }
    int2d_t const& operator[](component_index<1>) const { return y; }
    int2d_t& operator[](component_index<1>) { return y; }

    constexpr explicit operator bool() const { return x | y; }
};

constexpr bool operator==(coord_t lhs, coord_t rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

constexpr bool operator!=(coord_t lhs, coord_t rhs)
{
    return !(lhs == rhs);
}

constexpr bool operator<(coord_t lhs, coord_t rhs)
{
    return lhs.x != rhs.x ? lhs.x < rhs.x : lhs.y < rhs.y;
}

constexpr coord_t operator+(coord_t lhs, coord_t rhs)
{
    return { lhs.x + rhs.x, lhs.y + rhs.y };
}

constexpr coord_t operator-(coord_t lhs, coord_t rhs)
{
    return { lhs.x - rhs.x, lhs.y - rhs.y };
}

constexpr coord_t& operator+=(coord_t& lhs, coord_t rhs)
{
    lhs = lhs + rhs;
    return lhs;
}

constexpr coord_t& operator-=(coord_t& lhs, coord_t rhs)
{
    lhs = lhs - rhs;
    return lhs;
}

constexpr coord_t operator+(coord_t lhs)
{
    return lhs;
}

constexpr coord_t operator-(coord_t lhs)
{
    return { -lhs.x, -lhs.y };
}

constexpr coord_t vec_mul(coord_t crd, int2d_t v)
{
    return { crd.x * v, crd.y * v };
}

constexpr coord_t vec_div(coord_t crd, int2d_t v)
{
    return { crd.x / v, crd.y / v };
}

struct rect_t
{
    coord_t c;
    dimen_t d;

    constexpr explicit operator bool() const { return (bool)d; }

    // 'end'
    coord_t e() const { return { ex(), ey() }; }
    template<typename C>
    int2d_t e(C c) const { return e()[c]; }
    constexpr int2d_t ex() const { return c.x + d.w; }
    constexpr int2d_t ey() const { return c.y + d.h; }

    // 'rbegin'
    coord_t r() const { return { rx(), ry() }; }
    template<typename C>
    int2d_t r(C c) const { return r()[c]; }
    constexpr int2d_t rx() const { return c.x + d.w - 1; }
    constexpr int2d_t ry() const { return c.y + d.h - 1; }

    coord_t xy() const { return c; }

    coord_t exy() const { return { ex(), c.y }; }
    coord_t xey() const { return { c.x, ey() }; }
    coord_t exey() const { return { ex(), ey() }; }

    coord_t rxy() const { return { rx(), c.y }; }
    coord_t xry() const { return { c.x, ry() }; }
    coord_t rxry() const { return { rx(), ry() }; }

    coord_t nw() const { return c; }
    coord_t ne() const { return rxy(); }
    coord_t sw() const { return xry(); }
    coord_t se() const { return rxry(); }
};

constexpr bool operator==(rect_t lhs, rect_t rhs)
{
    return lhs.c == rhs.c && lhs.d == rhs.d;
}

constexpr bool operator!=(rect_t lhs, rect_t rhs)
{
    return !(lhs == rhs);
}

constexpr bool operator<(rect_t lhs, rect_t rhs)
{
    return lhs.c != rhs.c ? lhs.c < rhs.c : lhs.d < rhs.d;
}

constexpr rect_t operator+(rect_t lhs, coord_t rhs)
{
    lhs.c += rhs;
    return lhs;
}

constexpr rect_t operator+(coord_t lhs, rect_t rhs)
{
    rhs.c += lhs;
    return rhs;
}

constexpr rect_t operator-(rect_t lhs, coord_t rhs)
{
    lhs.c -= rhs;
    return lhs;
}

constexpr rect_t& operator+=(rect_t& lhs, coord_t rhs)
{
    lhs = lhs + rhs;
    return lhs;
}

constexpr rect_t& operator-=(rect_t& lhs, coord_t rhs)
{
    lhs = lhs - rhs;
    return lhs;
}


template<typename X, typename Y>
constexpr coord_t make_coord(X x, Y y)
{
    return coord_t{ static_cast<int2d_t>(x), static_cast<int2d_t>(y) };
}

template<typename W, typename H>
constexpr dimen_t make_dimen(W w, H h)
{
    return dimen_t{ static_cast<int2d_t>(w), static_cast<int2d_t>(h) };
}

template<typename W>
constexpr dimen_t square_dimen(W w) 
{
    return dimen_t{ static_cast<int2d_t>(w), static_cast<int2d_t>(w) };
}

// These can represent 8-way directions
enum dir_t : std::uint8_t
{
    DIR_E  = 0,
    FIRST_DIR = DIR_E,
    DIR_SE = 1,
    DIR_S  = 2,
    DIR_SW = 3,
    DIR_W  = 4,
    DIR_NW = 5,
    DIR_N  = 6,
    DIR_NE = 7,
    NUM_DIRS,
};

constexpr dir_t rot90(dir_t dir, int i = 1) { return dir_t(unsigned(dir + 2 * i) % NUM_DIRS); }
constexpr dir_t rot45(dir_t dir, int i = 1) { return dir_t(unsigned(dir + 1 * i) % NUM_DIRS); }

inline dir_t& operator++(dir_t& dir) { dir = dir_t(dir + 1); return dir; }
inline dir_t operator++(dir_t& dir, int) { dir_t copy = dir; ++dir; return copy; }
inline dir_t& operator--(dir_t& dir) { dir = dir_t(dir - 1); return dir; }
inline dir_t operator--(dir_t& dir, int) { dir_t copy = dir; --dir; return copy; }

constexpr unsigned DIR_FLAG_E  = 1 << DIR_E;
constexpr unsigned DIR_FLAG_NE = 1 << DIR_NE;
constexpr unsigned DIR_FLAG_N  = 1 << DIR_N;
constexpr unsigned DIR_FLAG_NW = 1 << DIR_NW;
constexpr unsigned DIR_FLAG_W  = 1 << DIR_W;
constexpr unsigned DIR_FLAG_SW = 1 << DIR_SW;
constexpr unsigned DIR_FLAG_S  = 1 << DIR_S;
constexpr unsigned DIR_FLAG_SE = 1 << DIR_SE;

} // namespace i2d

#endif

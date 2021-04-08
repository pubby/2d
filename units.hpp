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

#include <type_traits>
#include <utility>

namespace i2d {

using int_t = std::int32_t;

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

    int_t w;
    int_t h;

    int_t const& operator[](component_index<0>) const { return w; }
    int_t& operator[](component_index<0>) { return w; }
    int_t const& operator[](component_index<1>) const { return h; }
    int_t& operator[](component_index<1>) { return h; }
};

constexpr bool operator==(dimen_t lhs, dimen_t rhs)
{
    return lhs.w == rhs.w && lhs.h == rhs.h;
}

constexpr bool operator!=(dimen_t lhs, dimen_t rhs)
{
    return !(lhs == rhs);
}

constexpr bool operator<(dimen_t lhs, dimen_t rhs)
{
    return lhs.w != rhs.w ? lhs.w < rhs.w : lhs.h < rhs.h;
}

constexpr dimen_t operator*(dimen_t lhs, int_t scale)
{
    return { lhs.w * scale, lhs.h * scale };
}

constexpr dimen_t operator/(dimen_t lhs, int_t scale)
{
    return { lhs.w / scale, lhs.h / scale };
}

constexpr dimen_t vec_mul(dimen_t dim, int_t v)
{
    return dim * v;
}

constexpr dimen_t vec_div(dimen_t dim, int_t v)
{
    return { dim.w / v, dim.h / v };
}

constexpr dimen_t dimen_t_add(dimen_t d1, dimen_t d2)
{
    return { d1.w + d2.h, d1.h + d2.h };
}

struct coord_t
{
    static constexpr std::size_t components = 2;

    int_t x;
    int_t y;

    int_t const& operator[](component_index<0>) const { return x; }
    int_t& operator[](component_index<0>) { return x; }
    int_t const& operator[](component_index<1>) const { return y; }
    int_t& operator[](component_index<1>) { return y; }
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

inline coord_t& operator+=(coord_t& lhs, coord_t rhs)
{
    lhs = lhs + rhs;
    return lhs;
}

inline coord_t& operator-=(coord_t& lhs, coord_t rhs)
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

constexpr coord_t vec_mul(coord_t crd, int_t v)
{
    return { crd.x * v, crd.y * v };
}

constexpr coord_t vec_div(coord_t crd, int_t v)
{
    return { crd.x / v, crd.y / v };
}

struct rect_t
{
    coord_t c;
    dimen_t d;

    // 'end'
    coord_t e() const { return { ex(), ey() }; }
    template<typename C>
    int_t e(C c) const { return e()[c]; }
    constexpr int_t ex() const { return c.x + d.w; }
    constexpr int_t ey() const { return c.y + d.h; }

    // 'rbegin'
    coord_t r() const { return { rx(), ry() }; }
    template<typename C>
    int_t r(C c) const { return r()[c]; }
    constexpr int_t rx() const { return c.x + d.w - 1; }
    constexpr int_t ry() const { return c.y + d.h - 1; }

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

template<typename X, typename Y>
coord_t make_coord(X x, Y y)
{
    return coord_t{ static_cast<int_t>(x), static_cast<int_t>(y) };
}

template<typename W, typename H>
dimen_t make_dimen(W w, H h)
{
    return dimen_t{ static_cast<int_t>(w), static_cast<int_t>(h) };
}

} // namespace i2d

#endif

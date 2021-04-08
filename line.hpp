#ifndef INT2D_LINE_HPP
#define INT2D_LINE_HPP

// Generic Bressenham line algorithm code.

#include <cassert>
#include <cstdlib>
#include <functional>
#include <iterator>

#include "geometry.hpp"
#include "units.hpp"

namespace i2d {

// A wrapper around the state of Bressenham's line algorithm.
// NOTE: A direction vector of {0,0} is considered invalid.
struct line_state_t
{
    coord_t position;
    coord_t direction;
    int error;

    // Use these to construct 'line_state_t'.
    // Aggregate initialization isn't recommended because 'error' needs
    // to be set.
    static line_state_t pos_dir(coord_t pos, coord_t dir);
    static line_state_t from_to(coord_t from, coord_t to);

    // The starting 'error' value for a given direction.
    // Starts the line right in the middle.
    static int dir_err(coord_t direction);

    static line_state_t next(line_state_t line);
    static line_state_t next(line_state_t line, int_t n);
    static line_state_t prev(line_state_t line);
    static line_state_t prev(line_state_t line, int_t n);

    static line_state_t hflipped(line_state_t line);
    static line_state_t vflipped(line_state_t line);

    void advance() { *this = next(*this); }
    void advance(int_t n) { *this = next(*this, n); }
    void radvance() { *this = prev(*this); }
    void radvance(int_t n) { *this = prev(*this, n); }
    void hflip() { *this = hflipped(*this); }
    void vflip() { *this = vflipped(*this); }

    explicit operator bool() const { return direction != coord_t{0,0}; }
};

constexpr bool operator==(line_state_t lhs, line_state_t rhs)
{
    return (lhs.position == rhs.position
            && lhs.direction == rhs.direction
            && lhs.error == rhs.error);
}

constexpr bool operator!=(line_state_t lhs, line_state_t rhs)
{
    return !(lhs == rhs);
}

class line_iterator
: public std::iterator<std::random_access_iteratorag, coord_t const>
{
    friend class line_range;
public:
    line_iterator() : m_state{} {}
    explicit line_iterator(line_state_t state) : m_state(state) {}

    coord_t operator*() const { return m_state.position; }
    coord_t const* operator->() const { return &m_state.position; }

    line_iterator& operator+=(int_t n) { m_state.advance(n); return *this; }
    line_iterator& operator++() { m_state.advance(); return *this; }
    line_iterator operator++(int_t)
    {
        line_iterator ret(*this);
        ++(*this);
        return ret;
    }

    line_iterator& operator-=(int_t n) { m_state.radvance(n); return *this; }
    line_iterator& operator--() { m_state.radvance(); return *this; }
    line_iterator operator--(int_t)
    {
        line_iterator ret(*this);
        --(*this);
        return ret;
    }

    line_iterator operator+(int_t rhs) const
    {
        line_iterator lhs = *this;
        lhs += rhs;
        return lhs;
    }

    line_iterator operator-(int_t rhs) const
    {
        line_iterator lhs = *this;
        lhs -= rhs;
        return lhs;
    }

    coord_t operator[](std::size_t i) { return *(*this + i); }

    line_state_t state() const { return m_state; }
private:
    line_state_t m_state;
};

std::size_t operator-(line_iterator lhs, line_iterator rhs);

inline bool operator==(line_iterator lhs, line_iterator rhs)
{
    assert(*lhs != *rhs || lhs.state() == rhs.state());
    return *lhs == *rhs;
}

inline bool operator!=(line_iterator lhs, line_iterator rhs)
{
    return !(lhs == rhs);
}

bool operator<(line_iterator lhs, line_iterator rhs);
bool operator<=(line_iterator lhs, line_iterator rhs);
bool operator>(line_iterator lhs, line_iterator rhs);
bool operator>=(line_iterator lhs, line_iterator rhs);

class line_range
{
public:
    using const_iterator = line_iterator;

    line_range() = default;

    explicit line_range(coord_t crd)
    : m_begin({ crd, { 1, 0 }, 0 })
    , m_end({ crd + coord_t{ 1, 0 }, { 1, 0 }, 0 })
    {}

    line_range(coord_t from, coord_t to)
    : line_range(line_state_t::from_to(from, to), cdistance(from, to) + 1)
    {}

    // NOTE: A direction vector of {0,0} is considered invalid.
    line_range(coord_t pos, coord_t dir, int_t steps)
    : line_range(line_state_t::pos_dir(pos, dir), steps)
    {
        assert(dir != {0,0});
    }

    line_range(line_state_t begin, int_t steps)
    : m_begin(begin)
    , m_end(line_state_t::next(begin, steps))
    {}

    line_iterator begin() const { return m_begin; }
    line_iterator end() const { return m_end; }

    line_iterator cbegin() const { return m_begin; }
    line_iterator cend() const { return m_end; }

    std::size_t size() const { return cend() - cbegin(); }

    coord_t first() const { return *m_begin; }
    coord_t last() const { return *(m_end - 1); }

    void lengthen() { ++m_end; }
    void shorten() { --m_end; }
private:
    line_iterator m_begin;
    line_iterator m_end;
};

namespace impl
{
    constexpr bool is_steep(coord_t dir)
    {
        return sqr(dir.y) > sqr(dir.x);
    }

    inline coord_t coord_abs(coord_t direction)
    {
        return { std::abs(direction.x), std::abs(direction.y) };
    }

    // The version of Bressenham being used requires x and y to swap when
    // the line is steeper than 45 degrees.
    // This function simplifies the swapping.
    template<typename Func>
    auto steep_swap(line_state_t line, Func func)
    {
        if(is_steep(line.direction))
            return func(line, component_index<1>{}, component_index<0>{});
        else
            return func(line, component_index<0>{}, component_index<1>{});
    }

    template<typename T>
    constexpr int_t _signum(T x, std::false_type)
    {
        return T(0) < x;
    }

    template<typename T>
    constexpr int_t _signum(T x, std::true_type)
    {
        return (T(0) < x) - (x < T(0));
    }

    // Returns either -1, 0, or 1, depending on sign of val.
    template<typename T>
    constexpr int_t signum(T x)
    {
        static_assert(std::is_arithmetic<T>::value,
                      "only implemented for arithmetic");
        return _signum(x, std::is_signed<T>());
    }
} // namespace impl


// Calls 'it_func' with each coordinate of the line.
// This may be slightly faster than line_state_t and line_range.
template<typename Func>
void iterate_line(coord_t from, coord_t to, Func it_func)
{
    // Using a slightly different algorithm than line_state_t.
    // This version doesn't need to swap x or y.
    coord_t const direction = to - from;
    coord_t const d = impl::coord_abs(direction);
    coord_t const s = mapc(direction, &impl::signum<int_t>);
    for(int_t err = d.x - d.y; it_func(from), from != to;)
    {
        int_t const err2 = 2 * err;
        if(err2 > -d.y)
        {
            err -= d.y;
            from.x += s.x;
        }
        if(err2 < d.x)
        {
            err += d.x;
            from.y += s.y;
        }
    }
}

line_state_t line_state_t::pos_dir(coord_t pos, coord_t dir)
{
    return { pos, dir, dir_err(dir) };
}

line_state_t line_state_t::from_to(coord_t from, coord_t to)
{
    if(from == to)
        return pos_dir(from, {1,0});
    else
        return pos_dir(from, to - from);
}

int_t line_state_t::dir_err(coord_t direction)
{
    using namespace impl;
    return std::abs(is_steep(direction) ? direction.y : direction.x);
}

line_state_t line_state_t::next(line_state_t line)
{
    using namespace impl;
    assert((line.direction != coord_t{0,0}));
    // This is 1 iteration of Bressenham's line algorithm.
    return impl::steep_swap(line,
        [](line_state_t line, auto cx, auto cy)
        {
            coord_t const d = impl::coord_abs(line.direction);
            line.position[cx] += impl::signum(line.direction[cx]);
            line.error -= d[cy] * 2;
            if(line.error < 0)
            {
                line.position[cy] += impl::signum(line.direction[cy]);
                line.error += d[cx] * 2;
            }
            return line;
        });
}

line_state_t line_state_t::prev(line_state_t line)
{
    assert((line.direction != coord_t{0,0}));
    return impl::steep_swap(line,
        [](line_state_t line, auto cx, auto cy)
        {
            coord_t const d = impl::coord_abs(line.direction);
            line.position[cx] -= impl::signum(line.direction[cx]);
            line.error += d[cy] * 2;
            if(line.error > d[cx] * 2)
            {
                line.position[cy] -= impl::signum(line.direction[cy]);
                line.error -= d[cx] * 2;
            }
            return line;
        });
}

static line_state_t next_impl(line_state_t line, int_t n)
{
    assert((line.direction != coord_t{0,0}));
    return impl::steep_swap(line,
        [n](line_state_t line, auto cx, auto cy)
        {
            coord_t const d2 = vec_mul(impl::coord_abs(line.direction), 2);
            line.position[cx] += n * impl::signum(line.direction[cx]);
            line.error -= d2[cy] * n;
            int_t const y_change = (d2[cx] - line.error - 1) / d2[cx];
            assert(y_change >= 0);
            line.position[cy] += y_change * impl::signum(line.direction[cy]);
            line.error += y_change * d2[cx];
            return line;
        });
}

static line_state_t prev_impl(line_state_t line, int_t n)
{
    assert((line.direction != coord_t{0,0}));
    return impl::steep_swap(line,
        [n](line_state_t line, auto cx, auto cy)
        {
            coord_t const d2 = vec_mul(impl::coord_abs(line.direction), 2);
            line.position[cx] -= n * impl::signum(line.direction[cx]);
            line.error += d2[cy] * n;
            int_t const y_change = (line.error - 1) / d2[cx];
            assert(y_change >= 0);
            line.position[cy] -= y_change * impl::signum(line.direction[cy]);
            line.error -= y_change * d2[cx];
            return line;
        });
}

// This is the same as repeatedly calling next(line) n times,
// except this function has O(1) complexity instead of O(1).
line_state_t line_state_t::next(line_state_t line, int_t n)
{
    if(n < 0)
        return prev_impl(line, -n);
    else
        return next_impl(line, n);
}

line_state_t line_state_t::prev(line_state_t line, int_t n)
{
    if(n < 0)
        return next_impl(line, -n);
    else
        return prev_impl(line, n);
}

line_state_t line_state_t::hflipped(line_state_t line)
{
    // TODO: Should error be updated?
    line.direction.x *= -1;
    return line;
}

line_state_t line_state_t::vflipped(line_state_t line)
{
    // TODO: Should error be changed?
    line.direction.y *= -1;
    return line;
}

std::size_t operator-(line_iterator lhs, line_iterator rhs)
{
    return cdistance(*lhs, *rhs);
}

static int_t iter_cmp(line_iterator lhs, line_iterator rhs)
{
    coord_t p2 = *rhs;
    return impl::steep_swap(lhs.state(),
        [p2](line_state_t l1, auto cx, auto cy)
        {
            return (l1.position[cx] - p2[cx]) * l1.direction[cx];
        });
}

bool operator<(line_iterator lhs, line_iterator rhs)
{
    return iter_cmp(lhs, rhs) < 0;
}

bool operator<=(line_iterator lhs, line_iterator rhs)
{
    return iter_cmp(lhs, rhs) <= 0;
}

bool operator>(line_iterator lhs, line_iterator rhs)
{
    return iter_cmp(lhs, rhs) > 0;
}

bool operator>=(line_iterator lhs, line_iterator rhs)
{
    return iter_cmp(lhs, rhs) >= 0;
}

} // namespace i2d

#endif

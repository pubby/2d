#ifndef INT2D_GRID_HPP
#define INT2D_GRID_HPP

#include <algorithm>
#include <array>
#include <cassert>
#include <functional>
#include <iterator>
#include <utility>
#include <vector>

#include "geometry.hpp"

namespace i2d {

namespace impl {
    template<typename T>
    struct to_void { using type = void; };

    template<typename T>
    using ToVoid = typename to_void<T>::type;
}


constexpr std::size_t grid_index(dimen_t d, coord_t c)
    { return c.y * d.w + c.x; }

constexpr coord_t from_grid_index(dimen_t d, int i)
    { return { i % d.w, i / d.w }; }

template<typename T, typename = void>
struct is_grid : std::false_type {};

template<typename T>
struct is_grid<T, impl::ToVoid<typename T::is_grid> > : std::true_type {};

template<typename T, int2d_t Width, int2d_t Height>
class fixed_grid_t
{
private:
    using array_type = std::array<T, Width*Height>;
public:
    using is_grid = void;
    using value_type = T;
    using iterator = typename array_type::iterator;
    using const_iterator = typename array_type::const_iterator;

    fixed_grid_t() = default;
    fixed_grid_t(T const& value)
    {
        for(T& t : m_arr)
            t = value;
    }

    fixed_grid_t(fixed_grid_t const&) = default;
    fixed_grid_t(fixed_grid_t&&) = default;

    fixed_grid_t& operator=(fixed_grid_t const&) = default;
    fixed_grid_t& operator=(fixed_grid_t&&) = default;

    const_iterator cbegin() const { return m_arr.begin(); }
    const_iterator cend() const { return m_arr.end(); }

    const_iterator begin() const { return cbegin(); }
    const_iterator end() const { return cend(); }

    iterator begin() { return m_arr.begin(); }
    iterator end() { return m_arr.end(); }

    constexpr dimen_t dimen() const { return { Width, Height }; }

    T const& at(coord_t c) const 
    {
        if(!in_bounds(c, dimen()))
            throw std::out_of_range("grid_t::at");
        return m_arr[index(c)];
    }

    T& at(coord_t c)
    {
        if(!in_bounds(c, dimen()))
            throw std::out_of_range("grid_t::at");
        return m_arr[index(c)];
    }

    T const& at(unsigned i) const { return m_arr.at(i); }
    T& at(unsigned i) { return m_arr.at(i); }

    T const& operator[](coord_t c) const { return m_arr[index(c)]; }
    T& operator[](coord_t c) { return m_arr[index(c)]; }

    T const& operator[](unsigned i) const { return m_arr[i]; }
    T& operator[](unsigned i) { return m_arr[i]; }

    T const* data() const { return m_arr.data(); }
    T* data() { return m_arr.data(); }

    std::size_t size() const { return m_arr.size(); }

    void fill(T const& t) { m_arr.fill(t); }

    std::size_t index(coord_t c) const { return grid_index(dimen(), c); }
    coord_t from_index(unsigned i) const { return from_grid_index(dimen(), i); }
private:

    array_type m_arr;
};

// This is like a rect_tangular std::vector.
// It has column-major ordering.
template<typename T, class A = std::allocator<T> >
class grid_t
{
private:
    using vector_type = std::vector<T, A>;
public:
    using is_grid = void;
    using value_type = T;
    using allocator_type = A;

    using iterator = typename vector_type::iterator;
    using const_iterator = typename vector_type::const_iterator;

    grid_t()
    : grid_t(A())
    {}

    explicit grid_t(A const& alloc)
    : grid_t({0,0}, alloc)
    {}

    explicit grid_t(dimen_t dim, A const& alloc = A())
    : m_vec(area(dim), alloc)
    , m_dim(dim)
    {}

    grid_t(dimen_t dim, T const& t, A const& alloc = A())
    : m_vec(area(dim), t, alloc)
    , m_dim(dim)
    {}

    grid_t(grid_t const&) = default;
    grid_t(grid_t&&) = default;

    grid_t& operator=(grid_t const&) = default;
    grid_t& operator=(grid_t&&) = default;

    allocator_type get_allocator() const { return m_vec.get_allocator(); }

    void swap(grid_t& other)
    {
        using std::swap;
        swap(m_vec, other.m_vec);
        swap(m_dim, other.m_dim);
    }

    friend void swap(grid_t& a, grid_t& b) noexcept
    {
        a.swap(b);
    }

    const_iterator cbegin() const { return m_vec.begin(); }
    const_iterator cend() const { return m_vec.end(); }

    const_iterator begin() const { return cbegin(); }
    const_iterator end() const { return cend(); }

    iterator begin() { return m_vec.begin(); }
    iterator end() { return m_vec.end(); }

    dimen_t dimen() const { return m_dim; }

    T const& at(coord_t c) const
    {
        if(!in_bounds(c, dimen()))
            throw std::out_of_range("grid_t::at");
        return m_vec[index(c)];
    }

    T& at(coord_t c)
    {
        if(!in_bounds(c, dimen()))
            throw std::out_of_range("grid_t::at");
        return m_vec[index(c)];
    }

    T const& at(unsigned i) const { return m_vec.at(i); }
    T& at(unsigned i) { return m_vec.at(i); }

    T get(coord_t c, T const& default_) const
    {
        return in_bounds(c, dimen()) ? m_vec[index(c)] : default_;
    }

    T const& operator[](coord_t c) const { return m_vec[index(c)]; }
    T& operator[](coord_t c) { return m_vec[index(c)]; }

    T const& operator[](unsigned i) const { return m_vec[i]; }
    T& operator[](unsigned i) { return m_vec[i]; }

    T const* data() const { return m_vec.data(); }
    T* data() { return m_vec.data(); }

    std::size_t size() const { return m_vec.size(); }

    void resize(dimen_t new_dim)
    {
        grid_t new_grid(new_dim);
        dimen_t const copy_dim = crop(dimen(), new_dim);
        for(auto crd : dimen_range(copy_dim))
            new_grid[crd] = std::move(operator[](crd));
        swap(new_grid);
    }

    void clear()
    {
        m_vec.clear();
        m_dim = {0,0};
    }

    void fill(T const& t)
    {
        m_vec.assign(m_vec.size(), t);
    }

    std::size_t index(coord_t c) const { return c.y * m_dim.w + c.x; }
    coord_t from_index(unsigned i) const { return from_grid_index(dimen(), i); }
private:

    vector_type m_vec;
    dimen_t m_dim;
};

// Blits one grid on top of another using merge_func to combine the values.
// The signature of merge_func should be:
// T(T const& dest_val, T const& src_val)
template<typename Func, typename Grid>
void fblit(Grid& dest,
           coord_t dest_crd,
           Grid const& src,
           rect_t src_rect_t,
           Func merge_func = Func())
{
    static_assert(is_grid<Grid>::value, "must be a Grid");
    ASSERT(in_bounds(src_rect_t, src.to_rect_t()));
    ASSERT(in_bounds(src_rect_t, dest.to_rect_t()));
    for(int2d_t y = 0; y < src_rect_t.d.h; ++y)
    for(int2d_t x = 0; x < src_rect_t.d.w; ++x)
    {
        coord_t const dest_i = { dest_crd.x + x, dest_crd.y + y };
        coord_t const src_i = { src_rect_t.c.x + x, src_rect_t.c.y + y };
        dest[dest_i] = merge_func(static_cast<Grid const&>(dest)[dest_i],
                                  src[src_i]);
    }
}

template<typename Grid>
void blit(Grid& dest,
          coord_t dest_crd,
          Grid const& src,
          rect_t src_rect_t)
{
    static_assert(is_grid<Grid>::value, "must be a Grid");
    using value_type = typename Grid::value_type;
    fblit(
        dest,
        dest_crd,
        src,
        src_rect_t,
        [](value_type const&, value_type const& src_val)
        {
            return src_val;
        });
}

template<typename Func, typename Grid>
void fblit(Grid& dest,
           coord_t dest_crd,
           Grid const& src,
           Func merge_func = Func())
{
    static_assert(is_grid<Grid>::value, "must be a Grid");
    fblit(dest, dest_crd, src, src.to_rect_t(), merge_func);
}

template<typename Grid>
void blit(Grid& dest,
          coord_t dest_crd,
          Grid const& src)
{
    blit(dest, dest_crd, src, src.to_rect_t());
}

namespace impl
{
    inline std::vector<std::string> lines_of(std::string str)
    {
        std::vector<std::string> ret;
        std::string temp;
        for(char ch : str)
        {
            if(ch == '\n')
            {
                ret.push_back(temp);
                temp.clear();
            }
            else
                temp.push_back(ch);
        }
        ret.push_back(temp);
        return ret;
    }
} // namespace impl

inline grid_t<char> string_to_grid(std::string str)
{
    auto lines = impl::lines_of(str);
    dimen_t dim = { 0, static_cast<int2d_t>(lines.size()) };
    for(auto const& line : lines)
        dim.w = std::max<int2d_t>(dim.w, line.size());

    grid_t<char> ret(dim, '\0');

    for(int2d_t y = 0; y < dim.h; ++y)
    for(int2d_t x = 0; x < static_cast<int2d_t>(lines[y].size()); ++x)
        ret[{x,y}] = lines[y][x];

    return ret;
}

} // namespace i2d

#endif

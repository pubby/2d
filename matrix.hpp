#ifndef INT2D_MATRIX_HPP
#define INT2D_MATRIX_HPP

// Some very crude matrix types.
// Functionality is added as-needed; YAGNI!

#include <array>
#include <cassert>

#include "geometry.hpp"
#include "units.hpp"

namespace int2d {

namespace impl
{
    // Simplifies a number of quarter turns into an equivalent direction
    // in the range [0, 4)
    constexpr int _simplify_quarter_turns(int turns) { return turns & 3; }
}

// An integer-based 2d transformation matrix.
// REMEMBER: x=right, y=down. This affects rotations!
template<typename T>
struct imat3
{
    static_assert(std::is_integral<T>::value, "must be integral");

    auto const& operator[](std::size_t i) const { return arr[i]; }
    auto& operator[](std::size_t i) { return arr[i]; }

    std::array<std::array<T, 3>, 3> arr; // [row][col]; [y][x]

    static imat3 id()
    {
        return imat3
        {{{
            {{ 1, 0, 0 }},
            {{ 0, 1, 0 }},
            {{ 0, 0, 1 }},
        }}};
    }

    static imat3 rotate_cw(int n = 1)
    {
        int sin;
        int cos;
        switch(impl::_simplify_quarter_turns(n))
        {
        default:
        case 0: // 0 deg
            sin = 0;
            cos = 1;
            break;
        case 1: // 90 deg
            sin = 1;
            cos = 0;
            break;
        case 2: // 180 deg
            sin = 0;
            cos = -1;
            break;
        case 3: // 270 deg
            sin = -1;
            cos = 0;
            break;
        }

        return imat3
        {{{
            {{  cos, sin, 0 }},
            {{ -sin, cos, 0 }},
            {{    0,   0, 1 }},
        }}};
    }

    static imat3 rotate_ccw(int n = 1)
    {
        return rotate_cw(-n);
    }

    static imat3 translate(coord_t by)
    {
        return imat3
        {{{
            {{    1,    0, 0 }},
            {{    0,    1, 0 }},
            {{ by.x, by.y, 1 }},
        }}};
    }

    static imat3 hmirror()
    {
        return imat3
        {{{
            {{ -1, 0, 0 }},
            {{  0, 1, 0 }},
            {{  0, 0, 1 }},
        }}};
    }

    static imat3 vmirror()
    {
        return imat3
        {{{
            {{ 1,  0, 0 }},
            {{ 0, -1, 0 }},
            {{ 0,  0, 1 }},
        }}};
    }
};

using imat3x3 = imat3<int>;
using lmat3x3 = imat3<long>;
using llmat3x3 = imat3<long long>;

template<typename T>
bool operator==(imat3<T> lhs, imat3<T> rhs)
{
    return lhs.arr == rhs.arr;
}

template<typename T>
bool operator!=(imat3<T> lhs, imat3<T> rhs)
{
    return lhs.arr != rhs.arr;
}

template<typename T>
bool operator<(imat3<T> lhs, imat3<T> rhs)
{
    return lhs.arr < rhs.arr;
}

template<typename T>
imat3<T> operator*(imat3<T> lhs,
                                imat3<T> rhs)
{
    imat3<T> result;
    for(std::size_t i = 0; i != 3; ++i)
    for(std::size_t j = 0; j != 3; ++j)
    {
        result[i][j] = 0;
        for(int k = 0; k != 3; ++k)
            result[i][j] += lhs[i][k] * rhs[k][j];
    }
    return result;
}

template<typename T>
imat3<T>& operator*=(imat3<T>& lhs,
                                  imat3<T> rhs)
{
    lhs = lhs * rhs;
    return lhs;
}

template<typename T>
coord_t transform(imat3<T> mat, coord_t crd)
{
    return
    {
        mat[0][0]*crd.x + mat[1][0]*crd.y + mat[2][0],
        mat[0][1]*crd.x + mat[1][1]*crd.y + mat[2][1],
    };
}

template<typename T>
rect_t transform(imat3<T> mat, rect_t r)
{
    return rect_from_2_coords(transform(mat, r.c), transform(mat,r.r()));
}

template<typename T>
T determinant(imat3<T> m)
{
    return (+ m[0][0]*m[1][1]*m[2][2]
            + m[0][1]*m[1][2]*m[2][0]
            + m[0][2]*m[1][0]*m[2][1]
            - m[0][2]*m[1][1]*m[2][0]
            - m[0][1]*m[1][0]*m[2][2]
            - m[0][0]*m[1][2]*m[2][1]);
}

// Allows transforming a rect_t or coordinate system into a new
// coordinate system with an origin at (0,0).
// REMEMBER: x=right, y=down. This affects rotations!
class view_t
{
public:
    view_t()
    : view_t(coord_t{0,0})
    {}

    explicit view_t(coord_t origin)
    : m_mat(imat3x3::translate(origin))
    , m_inverse(imat3x3::translate(-origin))
    , m_dim{0,0}
    {}

    explicit view_t(rect_t subrect_t)
    : m_mat(imat3x3::translate(subrect_t.c))
    , m_inverse(imat3x3::translate(-subrect_t.c))
    , m_dim(subrect_t.d)
    {}

    view_t(view_t const& cs, coord_t origin)
    : m_mat(imat3x3::translate(origin) * cs.m_mat)
    , m_inverse(cs.m_inverse * imat3x3::translate(-origin))
    , m_dim{0,0}
    {
        assert((cs.dimen() == dimen_t{0,0}));
    }

    view_t(view_t const& cs, rect_t subrect_t)
    : m_mat(imat3x3::translate(subrect_t.c) * cs.m_mat)
    , m_inverse(cs.m_inverse * imat3x3::translate(-subrect_t.c))
    , m_dim(subrect_t.d)
    {
        assert(in_bounds(subrect_t, cs.dimen()));
    }

    // Returns {0,0} for coordinate systems.
    dimen_t dimen() const { return m_dim; }

    void rotate_cw(int_t quarter_turns)
    {
        apply_matrix(imat3x3::rotate_cw(quarter_turns),
                     imat3x3::rotate_cw(-quarter_turns));
    }

    void rotate_ccw(int_t quarter_turns)
    {
        apply_matrix(imat3x3::rotate_ccw(quarter_turns),
                     imat3x3::rotate_ccw(-quarter_turns));
    }

    void hmirror()
    {
        apply_matrix(imat3x3::hmirror(), imat3x3::hmirror());
    }

    void vmirror()
    {
        apply_matrix(imat3x3::vmirror(), imat3x3::vmirror());
    }

    // Converts back to the system passed into the constructor.
    template<typename T>
    auto to_parent(T&& t) const { return transform(m_mat, t); }

    // Inverse of 'to_parent'.
    template<typename T>
    auto from_parent(T&& t) const { return transform(m_inverse, t); }

    imat3x3 matrix() const { return m_mat; }
    imat3x3 inverse_matrix() const { return m_inverse; }
private:
    void apply_matrix(imat3x3 mat, imat3x3 inv)
    {
        if(area(dimen()) != 0)
        {
            rect_t r = transform(mat, rect_t{ {0,0}, m_dim });
            m_mat = mat * imat3x3::translate(-r.c) * m_mat;
            m_inverse *= imat3x3::translate(r.c) * inv;
            m_dim = r.d;
        }
        else
        {
            m_mat = mat * m_mat;
            m_inverse *= inv;
        }
    }

    imat3x3 m_mat;
    imat3x3 m_inverse;
    dimen_t m_dim;
};

} // namespace i2d

#endif

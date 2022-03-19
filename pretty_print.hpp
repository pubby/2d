#ifndef INT2D_PRETTY_PRINT_HPP
#define INT2D_PRETTY_PRINT_HPP

#include <ostream>

#include "line.hpp"
#include "units.hpp"

namespace i2d {

inline std::ostream& operator<<(std::ostream& stream, dimen_t dim)
{
    stream << "dimen_t{ " << dim.w << ", " << dim.h << " }";
    return stream;
}

inline std::ostream& operator<<(std::ostream& stream, coord_t crd)
{
    stream << "coord_t{ " << crd.x << ", " << crd.y << " }";
    return stream;
}

inline std::ostream& operator<<(std::ostream& stream, rect_t r)
{
    stream << "rect_t{ " << r.c << ", " << r.d << " }";
    return stream;
}

inline std::ostream& operator<<(std::ostream& stream, line_state_t st)
{
    stream << "line_state_t{ " << st.pos << ", "
                               << st.dir << ", "
                               << st.error << " }";
    return stream;
}

} // end namespace i2d

#endif

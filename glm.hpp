#ifndef INT2D_GLM_HPP
#define INT2D_GLM_HPP

// Helper functions for interfacing with the GLM library.

#include <glm/glm.hpp>

#include "units.hpp"

namespace i2d {

template<typename T>
T from_vec2(glm::vec2 v)
{
    return T{ v.x, v.y };
}

inline glm::vec2 to_vec2(coord_t c)
{
    return ::glm::vec2(c.x, c.y);
}

inline glm::vec2 to_vec2(dimen_t d)
{
    return ::glm::vec2(d.w, d.h);
}

inline ::glm::vec3 to_vec3(coord_t c, float z = 0.0f)
{
    return ::glm::vec3(c.x, c.y, z);
}

inline ::glm::vec3 to_vec3(dimen_t d, float z = 0.0f)
{
    return ::glm::vec3(d.w, d.h, z);
}

} // namespace i2d

#endif

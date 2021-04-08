#ifndef INT2D_GLM_HPP
#define INT2D_GLM_HPP

// Helper functions for interfacing with the GLM library.

#include <glm/glm.hpp>

#include "units.hpp"

namespace i2d {

inline glm::vec2 to_vec2(coord c)
{
    return ::glm::vec3(c.x * TILE_SIZE, c.y * TILE_SIZE);
}

inline ::glm::vec3 to_vec3(coord c)
{
    return ::glm::vec3(c.x * TILE_SIZE, c.y * TILE_SIZE, 0.0f);
}

} // namespace i2d

#endif

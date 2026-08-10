#include "geometry/position.hpp"

#include "geometry_epsilon.hpp"

bool operator==(Position a, Position b) noexcept {
    return (a - b).lengthSquared() < geometryEpsilon * geometryEpsilon;
}
bool operator!=(Position a, Position b) noexcept { return !(a == b); }
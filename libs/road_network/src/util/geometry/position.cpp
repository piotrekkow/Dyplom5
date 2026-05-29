#include <road_network/util/geometry/position.hpp>

#include "geom_eps.hpp"

bool operator==(Position a, Position b) noexcept {
    return (a - b).lengthSquared() < geomEps * geomEps;
}
bool operator!=(Position a, Position b) noexcept { return !(a == b); }
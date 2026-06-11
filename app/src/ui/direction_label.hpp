#pragma once

#include <QString>
#include <cstdint>
#include <road_network/util/geometry/vector2.hpp>

// 8-way compass quantizer for headings. Domain convention: +y = north, +x =
// east. A heading is quantized to the nearest of the 8 principal directions.
//
// Entries are labelled "approaching from" (the compass word is the side the
// traffic comes from, i.e. opposite the travel heading): e.g. "E ←".
// Exits are labelled "bound" (the compass word is the travel heading): e.g.
// "WB ←". The arrow always points along the travel heading.
namespace direction_label {

enum class Compass8 : std::uint8_t { N, NE, E, SE, S, SW, W, NW };

// Nearest principal direction to a travel heading.
Compass8 quantize(Vector2 heading);

// Two/one-letter compass abbreviation, e.g. "NE", "W".
QString abbrev(Compass8 c);

// Unicode arrow glyph pointing along the given travel heading (nearest of 8).
QString arrow(Vector2 heading);

// "approaching from" label for an entry heading, e.g. "E ←".
QString entryLabel(Vector2 heading);

// "bound" label for an exit heading, e.g. "WB ←".
QString exitLabel(Vector2 heading);

}  // namespace direction_label

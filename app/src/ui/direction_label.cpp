#include "direction_label.hpp"

#include <array>
#include <cmath>

namespace direction_label {
namespace {

// Principal directions ordered by angle index (45° steps starting at East,
// counter-clockwise in domain coords where +y = north).
constexpr std::array<Compass8, 8> kByAngle = {
    Compass8::E,  Compass8::NE, Compass8::N, Compass8::NW,
    Compass8::W,  Compass8::SW, Compass8::S, Compass8::SE};

size_t angleIndex(Vector2 v) {
    // octant index 0..7, 0 == East, increasing counter-clockwise
    double a = std::atan2(static_cast<double>(v.dy),
                          static_cast<double>(v.dx));  // (-pi, pi], +y north
    int idx = static_cast<int>(std::lround(a / (M_PI / 4.0)));
    return static_cast<size_t>(((idx % 8) + 8) % 8);
}

}  // namespace

Compass8 quantize(Vector2 heading) { return kByAngle[angleIndex(heading)]; }

QString abbrev(Compass8 c) {
    switch (c) {
        case Compass8::N:
            return "N";
        case Compass8::NE:
            return "NE";
        case Compass8::E:
            return "E";
        case Compass8::SE:
            return "SE";
        case Compass8::S:
            return "S";
        case Compass8::SW:
            return "SW";
        case Compass8::W:
            return "W";
        case Compass8::NW:
            return "NW";
    }
    return {};
}

QString arrow(Vector2 heading) {
    // glyphs indexed like kByAngle: E, NE, N, NW, W, SW, S, SE
    static const std::array<QString, 8> glyphs = {
        QString::fromUtf8("→"), QString::fromUtf8("↗"),
        QString::fromUtf8("↑"), QString::fromUtf8("↖"),
        QString::fromUtf8("←"), QString::fromUtf8("↙"),
        QString::fromUtf8("↓"), QString::fromUtf8("↘")};
    return glyphs[angleIndex(heading)];
}

QString entryLabel(Vector2 heading) {
    // "approaching from" — opposite the travel heading
    return abbrev(quantize(-heading)) + " " + arrow(heading);
}

QString exitLabel(Vector2 heading) {
    // "bound" — along the travel heading
    return abbrev(quantize(heading)) + "B " + arrow(heading);
}

}  // namespace direction_label

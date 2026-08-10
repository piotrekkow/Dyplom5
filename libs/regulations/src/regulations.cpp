#include "regulations/regulations.hpp"

#include <algorithm>
#include <cmath>
#include <common/overloaded.hpp>

namespace {
constexpr auto idx(StreamKind k) { return static_cast<size_t>(k); }
constexpr auto idx(SignalKind k) { return static_cast<size_t>(k); }
}  // namespace

double Regulations::intergreenFloor(StreamKind evac, StreamKind appr) const {
    for (const auto& f : intergreenParams_.floors) {
        if (f.evac == evac && f.appr == appr) return f.value;
    }
    return intergreenParams_.defaultFloor;
}

double Regulations::approachReactionMargin() const {
    return intergreenParams_.approachReactionMargin;
}

bool Regulations::enforceApproachRelease() const {
    return enforceApproachRelease_;
}

const reg::ResolvedStream Regulations::kinematics(
    StreamKind kind, std::optional<TurnKind> turn) const {
    reg::Stream s = streams_[idx(kind)];
    double evacSpeedCeil =
        std::visit(overloaded{
                       [&](reg::Stream::EvacSpeedCap cap) {
                           return cap.forTurn(turn.value_or(TurnKind::THROUGH));
                       },
                       [&](double cap) { return cap; },
                   },
                   s.evacSpeedCeil);

    return {.yellowTime = s.yellowTime,
            .clearingLength = s.clearingLength,
            .evacSpeedCeil = evacSpeedCeil,
            .approachSpeedFloor = s.approachSpeedFloor};
}

const reg::Aspect& Regulations::aspects(SignalKind kind) const {
    return aspects_[idx(kind)];
}

TurnKind Regulations::classifyTurn(double angle_rad) const {
    // angle ∈ [-throughMax,  throughMax] → THROUGH
    // angle ∈ ( throughMax,  leftMax   ] → LEFT
    // angle ∈ ( leftMax,     +∞        ) → UTURN
    // angle ∈ ( -∞,         -throughMax) → RIGHT
    if (angle_rad < -turnAngles_.throughMax) return TurnKind::RIGHT;
    if (angle_rad <= turnAngles_.throughMax) return TurnKind::THROUGH;
    if (angle_rad <= turnAngles_.leftMax) return TurnKind::LEFT;
    return TurnKind::UTURN;
}

bool Regulations::isParallel(Vector2 vec, Vector2 other) const {
    return std::abs(vec.normalized().dot(other.normalized())) >
           layoutParams_.parallelCosineThreshold;
}

[[nodiscard]] bool Regulations::isOpposing(Vector2 vec, Vector2 other) const {
    double angle = std::abs(vec.angleTo(other));
    return angle >= layoutParams_.opposingThreshold;
}

double Regulations::baseSaturationPerm(TurnKind turn, double Qn) const {
    const auto& s = baseSaturation_.forTurn(turn);
    return s.permBase - s.permQnCoeff * Qn;
}

double Regulations::baseSaturationProt(TurnKind turn) const {
    return baseSaturation_.forTurn(turn).prot;
}

double Regulations::opposingFlowShare(TurnKind turn, size_t laneCount) const {
    if (laneCount == 0) return 0.0;
    const auto& share = baseSaturation_.forTurn(turn).qnShare;
    return share[std::min(laneCount, share.size()) - 1];
}

Regulations Regulations::defaults() {
    using SA = SignalAspect;
    using Cap = reg::Stream::EvacSpeedCap;
    Regulations r;

    // ── Streams (§8.3.4)
    // ────────────────────────────────────────────────────
    auto& S = r.streams_;
    S[idx(StreamKind::VEHICLE)] = {
        .yellowTime = 3.0,
        .clearingLength = 10.0,
        .evacSpeedCeil =
            Cap{.uturn = 5.55, .right = 8.33, .left = 11.11, .through = 14.0},
        .approachSpeedFloor = 16.7 /* no floor */};
    S[idx(StreamKind::CYCLIST)] = {
        .yellowTime = 0.0, .clearingLength = 0.0, .evacSpeedCeil = 4.2};
    S[idx(StreamKind::PEDESTRIAN)] = {
        .yellowTime = 0.0, .clearingLength = 0.0, .evacSpeedCeil = 1.4};
    S[idx(StreamKind::BUS)] = {
        .yellowTime = 3.0,
        .clearingLength = 14.0,
        .evacSpeedCeil =
            Cap{.uturn = 5.55, .right = 8.33, .left = 8.33, .through = 10.0},
        .approachSpeedFloor = 16.7};
    S[idx(StreamKind::TRAM)] = {
        .yellowTime = 3.0,
        .clearingLength = 27.0,  // 2-car (n*13.5)
        .evacSpeedCeil =
            Cap{.uturn = 5.55, .right = 8.33, .left = 8.33, .through = 10.0},
        .approachSpeedFloor = 16.7};

    // ── Aspects (§4.2 / §8.2)
    // ───────────────────────────────────────────────
    auto& A = r.aspects_;
    A[idx(SignalKind::VEHICLE)] = {.active = SA::GREEN,
                                   .inactive = SA::RED,
                                   .toActive = {.aspect = SA::RED_YELLOW,
                                                .duration = 1,
                                                .isIntergreen = true},
                                   .toInactive = {.aspect = SA::YELLOW,
                                                  .duration = 3,
                                                  .isIntergreen = true},
                                   .additionalIntergreenEvac = 0,
                                   .minGreen = {.fixed = 8}};
    A[idx(SignalKind::CONDITIONAL_ARROW)] = {.active = SA::GREEN,
                                             .inactive = SA::DARK,
                                             .toActive = {},
                                             .toInactive = {},
                                             .additionalIntergreenEvac = 2,
                                             .minGreen = {.fixed = 8}};
    A[idx(SignalKind::PEDESTRIAN)] = {
        .active = SA::GREEN,
        .inactive = SA::RED,
        .toActive = {},
        .toInactive = {.aspect = SA::FLASHING_GREEN,
                       .duration = 4,
                       .isIntergreen = false},
        .additionalIntergreenEvac = 0,
        .minGreen = {.fixed = 4}};
    A[idx(SignalKind::CYCLIST)] = {.active = SA::GREEN,
                                   .inactive = SA::RED,
                                   .toActive = {},
                                   .toInactive = {.aspect = SA::FLASHING_GREEN,
                                                  .duration = 4,
                                                  .isIntergreen = false},
                                   .additionalIntergreenEvac = 0,
                                   .minGreen = {.fixed = 4}};
    A[idx(SignalKind::PEDESTRIAN_AND_CYCLIST)] = {
        .active = SA::GREEN,
        .inactive = SA::RED,
        .toActive = {},
        .toInactive = {.aspect = SA::FLASHING_GREEN,
                       .duration = 4,
                       .isIntergreen = false},
        .additionalIntergreenEvac = 0,
        .minGreen = {.fixed = 4}};
    A[idx(SignalKind::BUS)] = {
        .active = SA::VERTICAL_DASH,
        .inactive = SA::HORIZONTAL_DASH,
        .toActive = {},
        .toInactive = {.aspect = SA::HORIZONTAL_DOUBLE_DOT,
                       .duration = 3,
                       .isIntergreen = true},
        .additionalIntergreenEvac = 0,
        .minGreen = {.fixed = 7}};
    A[idx(SignalKind::TRAM)] = {
        .active = SA::VERTICAL_DASH,
        .inactive = SA::HORIZONTAL_DASH,
        .toActive = {},
        .toInactive = {.aspect = SA::HORIZONTAL_DOUBLE_DOT,
                       .duration = 3,
                       .isIntergreen = true},
        .additionalIntergreenEvac = 0,
        .minGreen = {.fixed = 7}};

    r.intergreenParams_ = {.approachReactionMargin = 0.0,
                           .defaultFloor = 0,
                           .floors = {
                               {.evac = StreamKind::VEHICLE,
                                .appr = StreamKind::VEHICLE,
                                .value = 1.0},  // (0)
                               {.evac = StreamKind::VEHICLE,
                                .appr = StreamKind::PEDESTRIAN,
                                .value = 4.0},
                               {.evac = StreamKind::VEHICLE,
                                .appr = StreamKind::CYCLIST,
                                .value = 4.0},
                               {.evac = StreamKind::CYCLIST,
                                .appr = StreamKind::VEHICLE,
                                .value = 1.0},
                               {.evac = StreamKind::PEDESTRIAN,
                                .appr = StreamKind::VEHICLE,
                                .value = 1.0},
                           }};

    r.turnAngles_ = {.throughMax = 0.7854, .leftMax = 2.3562};  // pi/4, 3pi/4
    r.layoutParams_ = {
        .opposingThreshold = 2.3562,
        .parallelCosineThreshold = 0.70711};  // 3pi/4, cos(45deg)
    r.baseSaturation_ = {
        .uturn = {.prot = 1470, .permBase = 1360, .permQnCoeff = 2.00},
        .left = {.prot = 1750, .permBase = 1620, .permQnCoeff = 1.75},
        .through = {.prot = 1900, .permBase = 1900, .qnShare = {1.0, 1.0, 1.0}},
        .right = {.prot = 1600, .permBase = 1300, .qnShare = {1.0, 0.5, 0.0}}};
    return r;
}

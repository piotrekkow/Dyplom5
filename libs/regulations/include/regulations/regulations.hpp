#pragma once

#include <array>
#include <geometry/vector2.hpp>

#include "params/aspect.hpp"
#include "params/base_saturation.hpp"
#include "params/intergreen.hpp"
#include "params/layout.hpp"
#include "params/stream.hpp"
#include "params/turn_angles.hpp"
#include "types/signal_kind.hpp"
#include "types/stream_kind.hpp"

// all values in SI units, angles in radians
class Regulations {
   public:
    static Regulations defaults();

    [[nodiscard]] double intergreenFloor(StreamKind evac,
                                         StreamKind appr) const;
    [[nodiscard]] const reg::ResolvedStream kinematics(
        StreamKind kind, std::optional<TurnKind> turn = std::nullopt) const;
    [[nodiscard]] const reg::Aspect& aspects(SignalKind kind) const;
    [[nodiscard]] TurnKind classifyTurn(double angle_rad) const;
    [[nodiscard]] bool isParallel(Vector2 vec, Vector2 other) const;
    [[nodiscard]] bool isOpposing(Vector2 vec, Vector2 other) const;
    [[nodiscard]] double baseSaturationPerm(TurnKind turn, double Qn) const;
    [[nodiscard]] double baseSaturationProt(TurnKind turn) const;
    [[nodiscard]] double opposingFlowShare(TurnKind turn,
                                           size_t laneCount) const;
    [[nodiscard]] double approachReactionMargin() const;
    [[nodiscard]] bool enforceApproachRelease() const;

   private:
    std::array<reg::Stream, static_cast<size_t>(StreamKind::count)> streams_{};
    std::array<reg::Aspect, static_cast<size_t>(SignalKind::count)> aspects_{};
    reg::TurnAngles turnAngles_{};
    reg::Intergreen intergreenParams_{};
    reg::Layout layoutParams_{};
    reg::BaseSaturation baseSaturation_{};
    bool enforceApproachRelease_{true};
};
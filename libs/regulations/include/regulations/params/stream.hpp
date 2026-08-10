#pragma once

#include <optional>
#include <variant>

#include "regulations/types/turn_kind.hpp"

namespace reg {

struct Stream {
    struct EvacSpeedCap {
        double uturn;
        double right;
        double left;
        double through;

        [[nodiscard]] double forTurn(TurnKind t) const {
            switch (t) {
                case TurnKind::UTURN:
                    return uturn;
                case TurnKind::LEFT:
                    return left;
                default:
                case TurnKind::THROUGH:
                    return through;
                case TurnKind::RIGHT:
                    return right;
            }
        }
    };
    using EvacSpeed = std::variant<double, EvacSpeedCap>;
    double yellowTime{0.0};      // t_ż
    double clearingLength{0.0};  // l_p
    EvacSpeed evacSpeedCeil{0.0};
    std::optional<double> approachSpeedFloor{};
};

struct ResolvedStream {
    double yellowTime;
    double clearingLength;
    double evacSpeedCeil;
    std::optional<double> approachSpeedFloor;
};

}  // namespace reg
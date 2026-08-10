#pragma once

#include <array>

#include "regulations/types/turn_kind.hpp"

namespace reg {

// base saturation flows [E/hz]
struct BaseSaturation {
    struct TurnSat {
        double prot;
        double permBase;
        double permQnCoeff{0.0};
        // share of this turn's flow counted into the opposing flow Qn,
        // indexed by the opposing entry's lane count (1, 2, 3+)
        std::array<double, 3> qnShare{};
    };
    TurnSat uturn{};
    TurnSat left{};
    TurnSat through{};
    TurnSat right{};
    [[nodiscard]] const TurnSat& forTurn(TurnKind t) const {
        switch (t) {
            case TurnKind::UTURN:
                return uturn;
            case TurnKind::LEFT:
                return left;
            case TurnKind::THROUGH:
                return through;
            case TurnKind::RIGHT:
                return right;
        }
        return through;
    }
};

};  // namespace reg
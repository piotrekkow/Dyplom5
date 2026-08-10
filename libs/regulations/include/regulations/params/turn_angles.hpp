#pragma once

namespace reg {

// Turn classification: approach heading→approach point, exit
// point vector→exit heading
//   angle ∈ [-throughMax,  throughMax] → THROUGH
//   angle ∈ ( throughMax,  leftMax   ] → LEFT
//   angle ∈ ( leftMax,     +∞        ) → UTURN
//   angle ∈ ( -∞,         -throughMax) → RIGHT
struct TurnAngles {
    double throughMax{0.7854};  // π/4
    double leftMax{2.3562};     // 3π/4
};

}  // namespace reg
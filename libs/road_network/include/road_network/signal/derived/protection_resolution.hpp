#pragma once

struct ProtectionResolution {
    bool forced;     // ui checkbox grayed-out, checked
    bool requested;  // ui checkbox accepted input
    [[nodiscard]] bool isProtected() const { return forced || requested; }
};
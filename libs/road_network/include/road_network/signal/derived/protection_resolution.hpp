#pragma once

struct ProtectionResolution {
    bool forced;
    bool requested;
    [[nodiscard]] bool isProtected() const { return forced || requested; }
};
#pragma once
#include <optional>

#include "road_network/util/ids.hpp"

class ExitLane {
   public:
    ExitLane(ExitId at) : exit_(at) {}

    [[nodiscard]] ExitId at() const { return exit_; }
    [[nodiscard]] float width() const { return width_; }

    void setWidth(float width) { width_ = width > 0.f ? width : width_; }

   private:
    ExitId exit_;
    float width_ = 3.5f;
    std::optional<float> storageLength_ = std::nullopt;
};

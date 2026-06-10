#pragma once

#include <string>
#include <vector>

#include "road_network/util/ids.hpp"

// A signal group configuration: one set of signal groups at a node. A node
// can have multiple configs; a group belongs to exactly one config.
class SignalConfig {
   public:
    SignalConfig(NodeId at, std::string name)
        : at_(at), name_(std::move(name)) {}

    [[nodiscard]] NodeId at() const { return at_; }
    [[nodiscard]] const std::string& name() const { return name_; }
    void setName(std::string name) { name_ = std::move(name); }

    [[nodiscard]] const std::vector<SignalGroupId>& signalGroups() const {
        return signalGroups_;
    }
    std::vector<SignalGroupId>& signalGroups() { return signalGroups_; }

   private:
    NodeId at_;
    std::string name_;  // empty = anonymous
    std::vector<SignalGroupId> signalGroups_;
};

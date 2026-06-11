#pragma once

#include <string>
#include <vector>

#include "road_network/util/ids.hpp"

// A signalized intersection: a set of nodes controlled together. Owns the
// signal configs spanning those nodes. A node belongs to at most one
// intersection.
class Intersection {
   public:
    explicit Intersection(std::string name) : name_(std::move(name)) {}

    [[nodiscard]] const std::string& name() const { return name_; }
    void setName(std::string name) { name_ = std::move(name); }

    [[nodiscard]] const std::vector<NodeId>& nodes() const { return nodes_; }
    std::vector<NodeId>& nodes() { return nodes_; }

    [[nodiscard]] const std::vector<SignalConfigId>& configs() const {
        return configs_;
    }
    std::vector<SignalConfigId>& configs() { return configs_; }

   private:
    std::string name_;
    std::vector<NodeId> nodes_;
    std::vector<SignalConfigId> configs_;
};

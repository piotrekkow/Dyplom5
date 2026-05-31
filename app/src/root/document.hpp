#pragma once

#include <road_network/demand.hpp>
#include <road_network/graph.hpp>
#include <road_network/signal.hpp>

// Domain root. Responsibility: Owns domain objects.
class Document {
   public:
    const Graph& graph() const { return graph_; }
    const Demand& demand() const { return demand_; }
    const Signal& signal() const { return signal_; }

    Graph& graph() { return graph_; }
    Demand& demand() { return demand_; }
    Signal& signal() { return signal_; }

   private:
    Graph graph_;
    Demand demand_;
    Signal signal_;
};
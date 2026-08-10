#include "delay.hpp"

#include <control/delay.hpp>

#include "types.hpp"

double groupDelay(const GGroup& g, double greenTime, double cycle) {
    if (g.kind == GKind::Crosswalk) return 0.0;

    double D = 0.0;
    for (const auto& gc : g.clusters)
        D += control_theory::weightedDelay(gc.Q, gc.S, greenTime, cycle);
    return D;
}

double totalDelay(const GGreen& alloc, const GConfig& cfg, int T) {
    double D = 0.0;
    auto Td = static_cast<double>(T);
    const auto& groups = cfg.groups;
    for (std::size_t i = 0; i < groups.size(); ++i)
        D += groupDelay(groups[i], static_cast<double>(alloc.tGroup[i]), Td);
    return D;
}

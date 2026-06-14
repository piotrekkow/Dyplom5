#include "signal_debug.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <road_network/signal/data/sequence.hpp>
#include <road_network/signal/data/signal_aspect.hpp>
#include <road_network/signal/data/signal_group.hpp>
#include <road_network/signal/data/timing.hpp>
#include <road_network/util/overloaded.hpp>
#include <string>
#include <vector>
#include <sstream>

namespace {
const char* aspectStr(SignalAspect a) {
    switch (a) {
        case SignalAspect::DARK:
            return "\033[90m▪\033[0m";
        case SignalAspect::RED:
            return "\033[91m■\033[0m";
        case SignalAspect::RED_YELLOW:
            return "\033[38;2;240;160;0m◩\033[0m";
        case SignalAspect::GREEN:
            return "\033[32m□\033[0m";
        case SignalAspect::YELLOW:
            return "\033[93m◪\033[0m";
        case SignalAspect::FLASHING_GREEN:
            return "\033[38;5;82m◫\033[0m";
    }
    return "?";
}

std::string sgLabel(SignalGroupId sgId, const SignalGroup& sg) {
    return std::visit(overloaded{[&](const MovementSignalGroup&) {
                                     return std::to_string(sgId.index()) + "K";
                                 },
                                 [&](const CrosswalkSignalGroup&) {
                                     return std::to_string(sgId.index()) + "P";
                                 },
                                 [&](const auto&) {
                                     return std::to_string(sgId.index()) + "S";
                                 }},
                      sg);
}

std::vector<SignalAspect> buildTimeline(SignalGroupId sgId,
                                        const Timing& timing,
                                        const SignalGroup& sg) {
    const int T = timing.cycleLength();
    const Sequence& seq = timing.sequence(sgId);
    auto inactive =
        std::visit([](const auto& g) { return g.inactiveAspect(); }, sg);

    std::vector<SignalAspect> result(static_cast<size_t>(T), inactive);

    if (std::holds_alternative<AlwaysActive>(seq)) {
        auto active =
            std::visit([](const auto& g) { return g.activeAspect(); }, sg);
        std::ranges::fill(result, active);
        return result;
    }

    for (const Interval& iv : std::get<SignalSequence>(seq).intervals()) {
        std::visit(
            [&](const auto& g) {
                auto appr = g.atomicTransitionAppr();
                for (int i = 0; i < static_cast<int>(appr.size()); ++i)
                    result[static_cast<size_t>(
                        ((iv.intervalStart() + i) % T + T) % T)] =
                        appr[static_cast<size_t>(i)];

                for (int i = 0; i < iv.greenDuration(); ++i)
                    result[static_cast<size_t>(((iv.greenStart() + i) % T + T )% T)] =
                        SignalAspect::GREEN;

                auto evac = g.atomicTransitionEvac();
                for (int i = 0; i < static_cast<int>(evac.size()); ++i)
                    result[static_cast<size_t>(((iv.greenEnd() + i) % T + T) %
                                               T)] =
                        evac[static_cast<size_t>(i)];
            },
            sg);
    }

    return result;
}

void printRuler(int lw, int T) {
    std::string ruler(static_cast<size_t>(T + 7), ' ');
    for (size_t p = 0; p <= static_cast<size_t>(T); p += 5) {
        std::string num = std::to_string(p);

        for (size_t c = 0; c < num.size() && p + 1 + c < ruler.size(); ++c) {
            ruler[p + 1 + c] = num[c];
        }
    }

    std::cout << std::left << std::setw(lw) << " " << ruler << "\n";
}

struct Throughput {
    double Q = 0.0;
    double S = 0.0;
    int t_eg = 0;
    double C = 0.0;
    double x = 0.0;
    double D = 0.0;
};

Throughput computeClusterThroughput(ClusterId cid, bool isProtected,
                                    SignalGroupId sgId, const Timing& timing,
                                    const Graph& graph, const Demand& demand) {
    const int T = timing.cycleLength();
    Throughput tp;

    const auto* cluster = graph.data().clusters().get(cid);
    if (cluster) {
        for (MovementId mid : cluster->movements()) {
            const auto* flow = demand.data().flows().get(mid);
            if (flow) tp.Q += static_cast<double>(flow->eqVehiclesPerHour());
        }
        for (EntryLaneId lid : cluster->lanes()) {
            const auto* sat = demand.derived().laneSaturations().get(lid);
            if (!sat) continue;
            tp.S += isProtected ? static_cast<double>(sat->prot)
                                : static_cast<double>(sat->perm);
        }
    }

    const Sequence& seq = timing.sequence(sgId);
    int t_gg = 0;
    if (std::holds_alternative<AlwaysActive>(seq)) {
        t_gg = T;
    } else {
        for (const Interval& iv : std::get<SignalSequence>(seq).intervals())
            t_gg += iv.greenDuration();
    }
    tp.t_eg = t_gg + 1;

    if (tp.S <= 0.0 || T <= 0) return tp;

    double lambda = static_cast<double>(tp.t_eg) / static_cast<double>(T);
    tp.C = tp.S * lambda;
    tp.x = (tp.C > 0.0) ? tp.Q / tp.C : std::numeric_limits<double>::infinity();

    double denom = 1.0 - std::min(1.0, tp.x) * lambda;
    double d1 = (denom > 1e-9) ? static_cast<double>(T) * (1.0 - lambda) *
                                     (1.0 - lambda) / (2.0 * denom)
                               : std::numeric_limits<double>::infinity();
    double sq =
        (tp.x - 1.0) * (tp.x - 1.0) + (tp.C > 0.0 ? tp.x * tp.x / tp.C : 0.0);
    double d2 = 900.0 * ((tp.x - 1.0) + std::sqrt(std::max(0.0, sq)));
    tp.D = d1 + d2;

    return tp;
}

}  // namespace

namespace {

struct ThroughputRow {
    ClusterId cid;
    std::string clusterLabel;  // "c3 (3, 4)"
    std::string groupLabel;    // "2K [prot]"
    bool firstInGroup = false;
    Throughput tp;
};

std::string formatMovements(const std::vector<MovementId>& mvs) {
    std::ostringstream oss;

    for (size_t i = 0; i < mvs.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << mvs[i].index();
    }

    return oss.str();
}

std::string clusterLabel(ClusterId cid, const std::vector<MovementId>& mvs) {
    return "c" + std::to_string(cid.index()) + " (" + formatMovements(mvs) +
           ")";
}

std::string throughputGroupLabel(SignalGroupId sgId, const SignalGroup& sg,
                                 bool isProtected) {
    return sgLabel(sgId, sg) + (isProtected ? " [prot]" : " [perm]");
}

std::vector<ThroughputRow> buildThroughputRows(const NodeSignal& ns,
                                               const Timing& timing,
                                               const Signal& signal,
                                               const Graph& graph,
                                               const Demand& demand) {
    std::vector<ThroughputRow> rows;

    for (SignalGroupId sgId : ns.signalGroups) {
        if (!timing.hasSequence(sgId)) continue;

        const SignalGroup* sg = signal.data().signalGroups().get(sgId);

        if (!sg) continue;

        const auto* mov = std::get_if<MovementSignalGroup>(sg);
        if (!mov) continue;

        bool firstCluster = true;

        for (ClusterId cid : mov->clusters()) {
            const auto* cluster = graph.data().clusters().get(cid);

            if (!cluster) continue;

            rows.push_back(
                {.cid = cid,
                 .clusterLabel = clusterLabel(cid, cluster->movements()),
                 .groupLabel = firstCluster ? throughputGroupLabel(
                                                  sgId, *sg, mov->isProtected())
                                            : "",
                 .firstInGroup = firstCluster,
                 .tp = computeClusterThroughput(cid, mov->isProtected(), sgId,
                                                timing, graph, demand)});

            firstCluster = false;
        }
    }

    return rows;
}

void printThroughputTable(const std::vector<ThroughputRow>& rows) {
    if (rows.empty()) return;

    constexpr int clusterW = 22;
    constexpr int groupW = 16;

    // everything after the '|'
    constexpr int metricsW = 8      // Q
                             + 8    // S_adj
                             + 6    // teg
                             + 10   // C
                             + 8    // Q/C
                             + 8    // d
                             + 10   // Q*d
                             + 10;  // Q*d/D%

    // left columns + "|" + metrics
    constexpr int tableW = clusterW + groupW + 1 + metricsW;

    auto printTableRule = [&](char c) {
        std::cout << std::string(static_cast<size_t>(tableW), c) << "\n";
    };

    double totalQd = 0.0;

    double maxX = -1.0;
    std::string maxClusterLabel;
    std::string maxGroupLabel;

    for (const auto& row : rows) {
        const double qd = row.tp.Q * row.tp.D;

        if (std::isfinite(qd)) totalQd += qd;

        if (std::isfinite(row.tp.x) && row.tp.x > maxX) {
            maxX = row.tp.x;
            maxClusterLabel = row.clusterLabel;
            maxGroupLabel = row.groupLabel;
        }
    }

    std::cout << std::left << std::setw(clusterW) << "cluster(movements)"

              << std::setw(groupW) << "group"

              << "|"

              << std::right << std::setw(8) << "Q" << std::setw(8) << "S_adj"
              << std::setw(6) << "teg" << std::setw(10) << "C" << std::setw(8)
              << "Q/C" << std::setw(8) << "d" << std::setw(10) << "Q*d"
              << std::setw(10) << "Q*d/D%"
              << "\n";

    printTableRule('=');

    bool first = true;

    for (const auto& row : rows) {
        if (!first && row.firstInGroup) {
            printTableRule('-');
        }

        first = false;

        const double qd = row.tp.Q * row.tp.D;

        const double pct =
            (std::isfinite(qd) && totalQd > 0.0) ? (qd / totalQd * 100.0) : 0.0;

        std::cout << std::left << std::setw(clusterW) << row.clusterLabel

                  << std::setw(groupW) << row.groupLabel

                  << "|"

                  << std::right << std::fixed

                  << std::setprecision(0) << std::setw(8) << row.tp.Q
                  << std::setw(8) << row.tp.S << std::setw(6) << row.tp.t_eg

                  << std::setprecision(1) << std::setw(10) << row.tp.C

                  << std::setprecision(3) << std::setw(8) << row.tp.x

                  << std::setprecision(1) << std::setw(8) << row.tp.D
                  << std::setw(10) << qd << std::setw(10) << pct << "\n";
    }

    printTableRule('-');
    std::cout << "Summary:\n"
              << "    Max Q/C: " << std::fixed << std::setprecision(3) << maxX
              << " @ " << maxClusterLabel << " | " << maxGroupLabel << "\n"
              << "    Total delay: " << std::setprecision(1) << totalQd
              << "[veh*s/h]\n\n";
}

}  // namespace

namespace {

std::string streamLabel(const StreamId& id) {
    return std::visit(
        overloaded{
            [](MovementId mid) { return "m" + std::to_string(mid.index()); },
            [](CrosswalkId cid) { return "c" + std::to_string(cid.index()); }},
        id);
}

std::vector<StreamId> sortedUniqueStreamIds(std::vector<StreamId> streams) {
    std::ranges::sort(streams, [](const StreamId& a, const StreamId& b) {
        if (a.index() != b.index()) return a.index() < b.index();
        return std::visit([](const auto& x) { return x.index(); }, a) <
               std::visit([](const auto& x) { return x.index(); }, b);
    });
    streams.erase(std::ranges::unique(streams).begin(), streams.end());
    return streams;
}

template <typename CellFn>
void printStreamMatrix(const std::string& title,
                       const std::vector<StreamId>& streams, CellFn cellFn) {
    int cw = 3;
    for (const StreamId& s : streams)
        cw = std::max(cw, static_cast<int>(streamLabel(s).size()));

    const std::string pad(static_cast<size_t>(cw), ' ');

    std::cout << title << "\n";

    std::cout << pad;
    for (const StreamId& col : streams)
        std::cout << " " << std::right << std::setw(cw) << streamLabel(col);
    std::cout << "\n";

    for (const StreamId& row : streams) {
        std::cout << std::left << std::setw(cw) << streamLabel(row);
        for (const StreamId& col : streams)
            std::cout << " " << std::right << std::setw(cw) << cellFn(row, col);
        std::cout << "\n";
    }
    std::cout << "\n";
}

}  // namespace

namespace debug_print {
void streamPermitted(NodeId node, const Graph& graph) {
    const auto* spm = graph.derived().streamPermitted().get(node);
    if (!spm) {
        std::cerr << "[graph_debug] no stream permitted map at node "
                  << node.index() << "\n";
        return;
    }

    const auto* nd = graph.data().nodes().get(node);
    if (!nd) return;

    std::vector<StreamId> streams;
    for (EntryId eid : nd->entries()) {
        const auto* entry = graph.data().entries().get(eid);
        if (!entry) continue;
        for (MovementId mid : entry->movements()) streams.emplace_back(mid);
    }
    for (CrosswalkSeriesId csid : nd->crosswalkSeries()) {
        const auto* cs = graph.data().crosswalkSeries().get(csid);
        if (!cs) continue;
        for (CrosswalkId cid : cs->crosswalks()) streams.emplace_back(cid);
    }

    printStreamMatrix(
        "=== StreamPermittedMap  node " + std::to_string(node.index()) + " ===",
        sortedUniqueStreamIds(std::move(streams)),
        [&](const StreamId& row, const StreamId& col) -> std::string {
            if (row == col) return "x";
            auto it = spm->find(row);
            return ((it != spm->end()) && it->second.count(col)) ? "1" : " ";
        });
}

void streamIntergreen(NodeId node, const Graph& graph) {
    const auto* sim = graph.derived().streamIntergreen().get(node);
    if (!sim) {
        std::cerr << "[graph_debug] no stream intergreen map at node "
                  << node.index() << "\n";
        return;
    }

    std::vector<StreamId> streams;
    for (const auto& [key, apprMap] : *sim) {
        streams.push_back(key);
        for (const auto& [apprId, _] : apprMap) streams.push_back(apprId);
    }

    printStreamMatrix(
        "=== StreamIntergreenMap  node " + std::to_string(node.index()) +
            " ===",
        sortedUniqueStreamIds(std::move(streams)),
        [&](const StreamId& row, const StreamId& col) -> std::string {
            if (row == col) return "x";
            auto it = sim->find(row);
            if (it == sim->end()) return " ";
            auto jt = it->second.find(col);
            return (jt != it->second.end()) ? std::to_string(jt->second.t_max)
                                            : " ";
        });
}

void nodeTimings(NodeId node, const Signal& signal, const Graph& graph,
                 const Demand& demand) {
    const auto* ns = signal.data().nodeSignals().get(node);
    if (!ns || ns->timings.empty()) {
        std::cerr << "[signal_debug] no timings at node " << node.index()
                  << "\n";
        return;
    }

    for (TimingId timingId : ns->timings) {
        const Timing* timing = signal.data().timings().get(timingId);
        if (!timing) continue;
        const int T = timing->cycleLength();

        std::cout << "=== Timing " << timingId.index() << "  (node "
                  << node.index() << ", T=" << T << "s) ===\n";

        int lw = 4;
        for (SignalGroupId sgId : ns->signalGroups) {
            if (!timing->hasSequence(sgId)) continue;
            const SignalGroup* sg = signal.data().signalGroups().get(sgId);
            if (sg)
                lw = std::max(lw, static_cast<int>(sgLabel(sgId, *sg).size()));
        }
        const std::string pad(static_cast<size_t>(lw), ' ');

        printRuler(lw, T);

        const auto& sgs = ns->signalGroups;
        for (int gi = 0; gi < static_cast<int>(sgs.size()); ++gi) {
            SignalGroupId sgId = sgs[static_cast<size_t>(gi)];
            if (!timing->hasSequence(sgId)) continue;
            const SignalGroup* sg = signal.data().signalGroups().get(sgId);
            if (!sg) continue;

            auto timeline = buildTimeline(sgId, *timing, *sg);
            std::cout << std::left << std::setw(lw) << sgLabel(sgId, *sg)
                      << " |";
            for (SignalAspect a : timeline) std::cout << aspectStr(a);

            const Sequence& seq = timing->sequence(sgId);
            std::cout << " \033[32m";
            if (std::holds_alternative<AlwaysActive>(seq)) {
                std::cout << "(0-" << (T - 1) << ")";
            } else {
                for (const Interval& iv :
                     std::get<SignalSequence>(seq).intervals()) {
                    if (iv.greenDuration() > 0)
                        std::cout << "(" << iv.greenStart() << "-"
                                  << iv.greenEnd() % T << ") ";
                }
            }
            std::cout << "\033[0m\n";

            bool more = false;
            for (int j = gi + 1; j < static_cast<int>(sgs.size()); ++j)
                if (timing->hasSequence(sgs[static_cast<size_t>(j)])) {
                    more = true;
                    break;
                }
            if (more) {
                std::cout << pad << " |";
                for (int p = 0; p < T; ++p)
                    std::cout << (p % 5 == 4 ? "|" : " ");
                std::cout << "\n";
            }
        }

        printRuler(lw, T);
        std::cout << "\n";

        auto rows = buildThroughputRows(*ns, *timing, signal, graph, demand);
        printThroughputTable(rows);
    }
}
void intergreenMatrix(NodeId node, const Signal& signal) {
    const auto* igm = signal.derived().intergreenMatrix().get(node);
    if (!igm) {
        std::cerr << "[signal_debug] no intergreen matrix at node "
                  << node.index() << "\n";
        return;
    }

    // Collect all unique SignalGroupIds
    std::vector<SignalGroupId> ids;
    for (const auto& [evac, row] : *igm) {
        ids.push_back(evac);
        for (const auto& [appr, _] : row) ids.push_back(appr);
    }
    std::ranges::sort(ids, [](SignalGroupId a, SignalGroupId b) {
        return a.index() < b.index();
    });
    ids.erase(std::ranges::unique(ids).begin(), ids.end());

    auto label = [&](SignalGroupId id) {
        const auto* sg = signal.data().signalGroups().get(id);
        return sgLabel(id, *sg);
    };

    int cw = 3;
    for (SignalGroupId id : ids)

        cw = std::max(cw, static_cast<int>(label(id).size()));

    const std::string pad(static_cast<size_t>(cw), ' ');

    std::cout << "=== IntergreenMatrix  node " << node.index() << " ===\n";

    // Header
    std::cout << pad;
    for (SignalGroupId col : ids)
        std::cout << " " << std::right << std::setw(cw) << label(col);
    std::cout << "\n";

    // Rows
    for (SignalGroupId row : ids) {
        std::cout << std::left << std::setw(cw) << label(row);
        auto it = igm->find(row);
        for (SignalGroupId col : ids) {
            std::string cell;
            if (row == col) {
                cell = "x";
            } else if (it != igm->end()) {
                auto jt = it->second.find(col);
                cell =
                    (jt != it->second.end()) ? std::to_string(jt->second) : " ";
            } else {
                cell = " ";
            }
            std::cout << " " << std::right << std::setw(cw) << cell;
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}
}  // namespace debug_print
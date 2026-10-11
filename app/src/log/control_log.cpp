#include <control/delay.hpp>
#include <control/timing.hpp>
#include <format>
#include <print>
#include <road_network/control.hpp>
#include <road_network/demand.hpp>
#include <road_network/graph.hpp>
#include <road_network/signal.hpp>
#include <string>
#include <variant>
#include <vector>

#include "log.hpp"
#include "util/group_label.hpp"

namespace logs {
namespace {
using labels::groupLabel;

bool isProtected(SignalGroupId id, const Signal& signal) {
    if (const auto* pr = signal.derived().protectionResolutions().get(id))
        return pr->isProtected();
    const auto* g = signal.data().signalGroups().get(id);
    const auto* mg = g ? std::get_if<MovementSignalGroup>(g) : nullptr;
    return mg && mg->params().requestProtected;
}

control::StageSequence<SignalGroupId> stageSequence(const Timing& timing,
                                                    const ControlData& data) {
    control::StageSequence<SignalGroupId> seq;
    for (const StageStep& step : timing.params().steps) {
        const auto* s = data.stages().get(step.stage());
        const auto* t = data.stageTransitions().get(step.transition());
        if (!s || !t) continue;
        seq.push_back({.stage = s->params().core,
                       .transition = t->params().core,
                       .stageDuration = step.stageDuration()});
    }
    return seq;
}

int greenTime(const std::vector<control::AspectInterval>& seq) {
    int g = 0;
    for (const auto& iv : seq)
        if (!iv.isIntergreen) g += iv.duration;
    return g;
}

template <typename Id>
std::string indexList(const std::vector<Id>& ids) {
    std::string s;
    for (const Id& id : ids)
        s += std::format("{}{}", s.empty() ? "" : ", ", id.index());
    return s;
}

constexpr std::size_t kLeftCols = 2;

struct Table {
    std::vector<std::string> header;
    std::vector<std::vector<std::string>> rows;
    std::vector<bool> sepAfter;
};

void printTable(const Table& t) {
    std::vector<std::size_t> w(t.header.size());
    for (std::size_t c = 0; c < w.size(); ++c) w[c] = t.header[c].size();
    for (const auto& row : t.rows)
        for (std::size_t c = 0; c < row.size() && c < w.size(); ++c)
            w[c] = std::max(w[c], row[c].size());

    auto formatRow = [&](const std::vector<std::string>& cells) {
        std::string line;
        for (std::size_t c = 0; c < w.size(); ++c) {
            const std::string& cell =
                c < cells.size() ? cells[c] : std::string{};
            if (c == kLeftCols) line += "|";
            if (c > 0) line += "   ";
            line += c < kLeftCols ? std::format("{:<{}}", cell, w[c])
                                  : std::format("{:>{}}", cell, w[c]);
        }
        return line;
    };

    const std::string header = formatRow(t.header);
    std::println("{}", header);
    std::println("{}", std::string(header.size(), '='));
    for (std::size_t i = 0; i < t.rows.size(); ++i) {
        std::println("{}", formatRow(t.rows[i]));
        if (i < t.sepAfter.size() && t.sepAfter[i])
            std::println("{}", std::string(header.size(), '-'));
    }
}

const char* aspectSymbol(SignalAspect aspect) {
    switch (aspect) {
        case SignalAspect::DARK:
            return "\033[90m▪\033[0m";
        case SignalAspect::RED:
            return "\033[91m■\033[0m";
        case SignalAspect::RED_YELLOW:
            return "\033[38;2;240;160;0m◩\033[0m";
        case SignalAspect::GREEN:
            return "\033[32m□\033[0m";
        case SignalAspect::YELLOW:
            return "\033[38;2;255;255;0m◪\033[0m";
        case SignalAspect::FLASHING_GREEN:
            return "\033[38;5;82m◫\033[0m";
        default:
            return "?";
    }
}

// "    5   10   15 ..."
std::string numberRuler(int T) {
    std::string s(static_cast<std::size_t>(T), ' ');
    for (int t = 5; t <= T; t += 5) {
        const std::string num = std::to_string(t);
        for (std::size_t j = 0; j < num.size(); ++j)
            s[static_cast<std::size_t>(t) - num.size() + j] = num[j];
    }
    return s;
}

// "    |    |    | ..."
std::string tickRuler(int T) {
    std::string s(static_cast<std::size_t>(T), ' ');
    for (int t = 5; t <= T; t += 5) s[static_cast<std::size_t>(t) - 1] = '|';
    return s;
}
}  // namespace

void throughput(TimingId timing, const Control& control, const Signal& signal,
                const Graph& graph, const Demand& demand) {
    const Timing* t = control.data().timings().get(timing);
    if (!t) {
        std::println("[log] no timing {}", timing.index());
        return;
    }
    const auto plan =
        control::buildTimingPlan(stageSequence(*t, control.data()));
    const int T = plan.cycleLength();
    if (T <= 0) {
        std::println("[log] timing {} ({}) has no steps", timing.index(),
                     t->params().name);
        return;
    }
    const auto* nodeGroups = signal.data().nodeGroups().get(t->at());
    if (!nodeGroups) return;

    auto green = [&](SignalGroupId gid) {
        const auto it = plan.sequences.find(gid);
        return it == plan.sequences.end() ? 0 : greenTime(it->second);
    };

    Table cs{
        .header = {"cs (c)", "group", "len[m]", "min[s]", "g[s]", "g/T[-]"},
        .rows = {},
        .sepAfter = {}};
    for (SignalGroupId gid : *nodeGroups) {
        const auto* g = signal.data().signalGroups().get(gid);
        const auto* cg = g ? std::get_if<CrosswalkSignalGroup>(g) : nullptr;
        if (!cg) continue;

        std::string streams =
            std::format("cs? ({})", indexList(cg->crosswalks()));
        double len = 0.0;
        if (!cg->crosswalks().empty()) {
            if (const auto* cw =
                    graph.data().crosswalks().get(cg->crosswalks().front())) {
                streams = std::format("cs{} ({})", cw->parent().index(),
                                      indexList(cg->crosswalks()));
                if (const auto* series =
                        graph.data().crosswalkSeries().get(cw->parent()))
                    len = series->params().criticalGreenPath.length();
            }
        }
        cs.rows.push_back({
            streams,
            groupLabel(gid, signal.data()),
            std::format("{:.1f}", len),
            std::format("{}", cg->params().intervalMinFixed),
            std::format("{}", green(gid)),
            std::format("{:.2f}", static_cast<double>(green(gid)) / T),
        });
        cs.sepAfter.push_back(true);
    }
    if (!cs.rows.empty()) {
        printTable(cs);
        std::println("");
    }

    struct ClusterRow {
        std::string label;
        double Q = 0.0;
        double S = 0.0;
        control_theory::StreamDelay sd;
    };
    struct GroupRows {
        std::string label;
        int green = 0;
        std::vector<ClusterRow> clusters;
    };

    std::vector<GroupRows> groups;
    double totalD = 0.0;
    double maxX = 0.0;
    std::string maxXLabel;

    for (SignalGroupId gid : *nodeGroups) {
        const auto* g = signal.data().signalGroups().get(gid);
        const auto* mg = g ? std::get_if<MovementSignalGroup>(g) : nullptr;
        if (!mg) continue;

        const bool prot = isProtected(gid, signal);
        GroupRows gr{
            .label = std::format("{} [{}]", groupLabel(gid, signal.data()),
                                 prot ? "prot" : "perm"),
            .green = green(gid),
            .clusters = {}};

        for (ClusterId cid : mg->clusters()) {
            const auto* cluster = graph.data().clusters().get(cid);
            if (!cluster) continue;

            ClusterRow row{.label =
                               std::format("cl{} ({})", cid.index(),
                                           indexList(cluster->movements()))};
            for (MovementId mid : cluster->movements())
                if (const auto* Q = demand.data().flows().get(mid))
                    row.Q += Q->eqVehiclesPerHour();

            Saturation sat{.prot = 0.0, .perm = 0.0};
            if (const auto* laneSat =
                    demand.derived().laneSaturations().get(cluster->at())) {
                const auto& lanes = cluster->lanes();
                for (int li = lanes.firstIdx; li <= lanes.lastIdx(); ++li)
                    if (li >= 0 &&
                        static_cast<std::size_t>(li) < laneSat->size())
                        sat += (*laneSat)[static_cast<std::size_t>(li)];
            }
            row.S = prot ? sat.prot : sat.perm;

            row.sd = control_theory::streamDelay(row.Q, row.S, gr.green, T);
            totalD += row.Q * row.sd.delay;
            if (row.sd.saturation > maxX) {
                maxX = row.sd.saturation;
                maxXLabel = gr.label;
            }
            gr.clusters.push_back(std::move(row));
        }
        groups.push_back(std::move(gr));
    }

    Table cl{
        .header = {"cl (mov)", "group", "Q[veh]", "S[E/hz]", "ge[s]", "λ[-]",
                   "C[E/h]", "Q/C[-]", "d[s/veh]", "Q*d[s]", "Q*d/D[%]"},
        .rows = {},
        .sepAfter = {}};
    for (const auto& gr : groups) {
        for (std::size_t i = 0; i < gr.clusters.size(); ++i) {
            const auto& row = gr.clusters[i];
            const bool first = i == 0;
            const double Qd = row.Q * row.sd.delay;
            cl.rows.push_back({
                row.label,
                first ? gr.label : "",
                std::format("{:.0f}", row.Q),
                std::format("{:.0f}", row.S),
                first ? std::format("{}", gr.green) : "",
                first ? std::format("{:.2f}", row.sd.greenRatio) : "",
                std::format("{:.1f}", row.sd.capacity),
                std::format("{:.3f}", row.sd.saturation),
                std::format("{:.1f}", row.sd.delay),
                std::format("{:.1f}", Qd),
                std::format("{:.1f}", totalD > 0.0 ? Qd / totalD * 100.0 : 0.0),
            });
            cl.sepAfter.push_back(i + 1 == gr.clusters.size());
        }
    }
    printTable(cl);

    std::println("");
    std::println("Summary:");
    std::println("    Max Q/C: {:.3f} @ {}", maxX, maxXLabel);
    std::println("    Total delay: {:.1f}[s/h]", totalD);
}

void timingDiagram(TimingId timing, const Control& control,
                   const Signal& signal) {
    const Timing* t = control.data().timings().get(timing);
    if (!t) {
        std::println("[log] no timing {}", timing.index());
        return;
    }
    const auto plan =
        control::buildTimingPlan(stageSequence(*t, control.data()));
    const int cyc = plan.cycleLength();
    if (cyc <= 0) {
        std::println("[log] timing {} ({}) has no steps", timing.index(),
                     t->params().name);
        return;
    }
    const auto* nodeGroups = signal.data().nodeGroups().get(t->at());
    if (!nodeGroups) return;

    std::size_t labelW = 0;
    std::vector<std::string> labels;
    labels.reserve(nodeGroups->size());
    for (SignalGroupId gid : *nodeGroups) {
        labels.push_back(groupLabel(gid, signal.data()));
        labelW = std::max(labelW, labels.back().size());
    }
    labelW += 2;
    const std::string pad(labelW, ' ');

    std::println("{}{}", pad, numberRuler(cyc));
    for (std::size_t i = 0; i < nodeGroups->size(); ++i) {
        const auto it = plan.sequences.find((*nodeGroups)[i]);
        if (it == plan.sequences.end()) continue;

        std::string row;
        for (const auto& iv : it->second)
            for (int k = 0; k < iv.duration; ++k)
                row += aspectSymbol(iv.aspect);
        std::string windows;
        for (const auto& w : control::activeWindows(it->second))
            windows += std::format("  ({}-{})", w.onset, w.end);

        std::println("{:<{}}{}{}", labels[i], labelW, row, windows);
        if (i + 1 < nodeGroups->size())
            std::println("{}{}", pad, tickRuler(cyc));
    }
    std::println("{}{}", pad, numberRuler(cyc));
    std::println();
}
}  // namespace logs

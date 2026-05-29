#pragma once

#include <unordered_map>

#include "road_network/util/ids.hpp"

struct StreamIntergreen {
    float s_evac;
    float s_appr;
    float v_evac;
    float v_appr;
    float l_evac;
    float t_y;
    float t_min;
    float t_evac;
    float t_appr;
    float t_raw;
};

struct StreamIntergreenEntry {
    int t_max;
    StreamIntergreen diag;
};

using StreamIntergreenMap =
    std::unordered_map<StreamId,
                       std::unordered_map<StreamId, StreamIntergreenEntry>>;

using StreamIntergreenOverrideMap =
    std::unordered_map<StreamId, std::unordered_map<StreamId, int>>;

// #include <QDebug>
// #include <QString>
// #include <variant>
// inline void debugPrint(const StreamIntergreenMap& map) {
//     qDebug().noquote() << "StreamIntergreen Map:";
//     qDebug().noquote() << QString::asprintf(
//         "%8s %8s %8s %8s %8s %8s %8s %8s %8s %8s %8s %8s", "evac_id",
//         "appr_id", "s_evac", "l_evac", "v_evac", "t_evac", "s_appr",
//         "v_appr", "t_appr", "t_min", "t_raw", "t_max");

//     for (const auto& [evacId, apprMap] : map) {
//         uint32_t evacIdx =
//             std::visit([](auto id) { return id.index(); }, evacId);
//         char evacType = std::holds_alternative<MovementId>(evacId) ? 'K' :
//         'P';

//         for (const auto& [apprId, ig] : apprMap) {
//             uint32_t apprIdx =
//                 std::visit([](auto id) { return id.index(); }, apprId);
//             char apprType =
//                 std::holds_alternative<MovementId>(apprId) ? 'K' : 'P';

//             qDebug().noquote() << QString::asprintf(
//                 "%6u %1c %6u %1c %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f "
//                 "%8.2f %8.2f %8.2f %8.2f",
//                 evacIdx, evacType, apprIdx, apprType, ig.s_evac, ig.l_evac,
//                 ig.v_evac, ig.t_evac, ig.s_appr, ig.v_appr, ig.t_appr,
//                 ig.t_min, ig.t_raw, ig.t_max);
//         }
//     }
// }
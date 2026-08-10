#include "application.hpp"

#include <QDebug>
#include <memory>

#include "log/log.hpp"
#include "road_network/graph/data/connector/movement_path_spec.hpp"

namespace {
void seedDemoContent(Project& project) {
    project.run([&](Transaction& tx, Editor& e) {
        auto& g = e.graph();
        auto& d = e.demand();

        const auto& data = project.document().graph().data();
        auto movementsOf = [&](const std::vector<ClusterId>& ids) {
            std::vector<MovementId> out;
            for (ClusterId c : ids)
                for (MovementId m : data.clusters().get(c)->movements())
                    out.push_back(m);
            return out;
        };

        const NodeId n1 = g.createNode(tx, {.x = 10.0, .y = 5.0});

        // N
        const EntryId e1n =
            g.createEntry(tx, n1, {.x = -24.7, .y = 6.2}, {.dx = 1, .dy = 0});
        g.setEntryLanes(tx, e1n, {{.width = 3.0}, {}, {}, {.width = 3.0}});
        const ExitId x1n =
            g.createExit(tx, n1, {.x = -24.7, .y = 8.7}, {.dx = -1, .dy = 0});
        g.setExitLanes(tx, x1n, {3.0, 3.2, 3.5});

        // S
        const EntryId e1s =
            g.createEntry(tx, n1, {.x = 46.7, .y = 3.2}, {.dx = -1, .dy = 0});
        g.setEntryLanes(
            tx, e1s,
            {{.width = 3.0}, {.width = 3.2}, {.width = 3.4}, {.width = 3.0}});
        const ExitId x1s =
            g.createExit(tx, n1, {.x = 46.7, .y = 0.8}, {.dx = 1, .dy = 0});
        g.setExitLanes(tx, x1s, {3.5, 3.5, 3.2});

        // E
        const EntryId e1e =
            g.createEntry(tx, n1, {.x = 5, .y = 37.2}, {.dx = .08, .dy = -.7});
        g.setEntryLanes(tx, e1e,
                        {{.width = 3.3}, {.width = 3.3}, {.width = 3.3}});
        const ExitId x1e =
            g.createExit(tx, n1, {.x = 9.7, .y = 37.5}, {.dx = -.08, .dy = .7});
        g.setExitLanes(tx, x1e, {3.5, 3.5, 3.2});

        // W
        const EntryId e1w =
            g.createEntry(tx, n1, {.x = 14, .y = -25}, {.dx = -.1, .dy = .7});
        g.setEntryLanes(tx, e1w,
                        {{.width = 3.2}, {.width = 3.2}, {.width = 3.2}});
        const ExitId x1w =
            g.createExit(tx, n1, {.x = 10, .y = -25}, {.dx = .12, .dy = -.7});
        g.setExitLanes(tx, x1w, {3.0, 3.0});

        // N
        auto l1n = g.setEntryLayout(tx, e1n,
                                    {{.exit = x1e, .laneCount = 1},
                                     {.exit = x1s, .laneCount = 2},
                                     {.exit = x1w, .laneCount = 1}},
                                    {false, false});
        if (!l1n) qDebug() << "m1n err" << static_cast<int>(l1n.error());
        auto m1n = movementsOf(l1n.value());
        g.setMovementPathSpec(
            tx, m1n[0], {.dE = 10, .dX = 6, .params = QuadBezierParams{}});
        g.setMovementPathSpec(tx, m1n[1],
                              {.dE = 16, .dX = 16, .params = LineParams{}});
        g.setMovementPathSpec(
            tx, m1n[2], {.dE = 15, .dX = 10, .params = QuadBezierParams{}});

        // S
        auto l1s = g.setEntryLayout(tx, e1s,
                                    {{.exit = x1s, .laneCount = 1},
                                     {.exit = x1w, .laneCount = 1},
                                     {.exit = x1n, .laneCount = 2},
                                     {.exit = x1e, .laneCount = 1}},
                                    {true, false, false});
        if (!l1s) qDebug() << "m1s err" << static_cast<int>(l1s.error());
        auto m1s = movementsOf(l1s.value());
        g.setMovementPathSpec(
            tx, m1s[0],
            {.dE = 12,
             .dX = 11,
             .params = CubicBezierParams{.cE = 14, .cX = 12}});
        g.setMovementPathSpec(
            tx, m1s[1], {.dE = 15, .dX = 10, .params = QuadBezierParams{}});
        g.setMovementPathSpec(tx, m1s[2],
                              {.dE = 16, .dX = 16, .params = LineParams{}});
        g.setMovementPathSpec(
            tx, m1s[3], {.dE = 15, .dX = 10, .params = QuadBezierParams{}});

        // E
        auto l1e = g.setEntryLayout(tx, e1e,
                                    {{.exit = x1s, .laneCount = 1},
                                     {.exit = x1w, .laneCount = 2},
                                     {.exit = x1n, .laneCount = 1}},
                                    {false, true});
        if (!l1e) qDebug() << "m1e err" << static_cast<int>(l1e.error());
        auto m1e = movementsOf(l1e.value());
        g.setMovementPathSpec(
            tx, m1e[0], {.dE = 12, .dX = 9, .params = QuadBezierParams{}});
        g.setMovementPathSpec(tx, m1e[2],
                              {.dE = 2, .dX = 7, .params = QuadBezierParams{}});

        // W
        auto l1w = g.setEntryLayout(tx, e1w,
                                    {{.exit = x1n, .laneCount = 1},
                                     {.exit = x1e, .laneCount = 2},
                                     {.exit = x1s, .laneCount = 1}},
                                    {false, true});
        if (!l1w) qDebug() << "m1w err" << static_cast<int>(l1w.error());
        auto m1w = movementsOf(l1w.value());
        g.setMovementPathSpec(
            tx, m1w[0], {.dE = 14, .dX = 11, .params = QuadBezierParams{}});
        g.setMovementPathSpec(tx, m1w[1],
                              {.dE = 0, .dX = 0, .params = LineParams{}});
        g.setMovementPathSpec(
            tx, m1w[2], {.dE = 2, .dX = 11, .params = QuadBezierParams{}});

        auto setEdge = [&](CrosswalkId id,
                           CrosswalkGeometryInput::Segment edge) {
            auto input = data.crosswalks().get(id)->params().input;
            input.edge = edge;
            g.setCrosswalkGeometry(tx, id, input);
        };
        auto setSplit = [&](CrosswalkId id,
                            CrosswalkGeometryInput::Segment split) {
            auto input = data.crosswalks().get(id)->params().input;
            input.split = split;
            g.setCrosswalkGeometry(tx, id, input);
        };

        // CW N
        auto csn = g.createCrosswalkSeries(tx, n1);
        g.setCSeriesType(tx, csn, CrosswalkType::PEDESTRIAN_AND_CYCLIST);
        auto c1n = g.createCrosswalk(
            tx, csn,
            {.p1 = {.x = -21.8, .y = 8.6}, .p2 = {.x = -22.1, .y = 18}});
        setEdge(c1n, {.width = 7.5, .x1 = 0, .x2 = 0});
        setSplit(c1n, {.width = 4.0, .x1 = 0, .x2 = 0});

        auto c2n = g.createCrosswalk(
            tx, csn,
            {.p1 = {.x = -21.8, .y = -6.6}, .p2 = {.x = -21.8, .y = 6.2}});
        setEdge(c2n, {.width = 7.5, .x1 = 0, .x2 = 0});
        setSplit(c2n, {.width = 4.0, .x1 = 0, .x2 = 0});

        Polyline csnPath;
        csnPath.setStart({.x = -20, .y = -6.6});
        csnPath.addLine({.x = -20, .y = 18});
        g.setCSeriesCriticalPath(tx, csn, csnPath);

        // CW S
        auto css = g.createCrosswalkSeries(tx, n1);
        g.setCSeriesType(tx, css, CrosswalkType::PEDESTRIAN_AND_CYCLIST);
        auto c1s = g.createCrosswalk(
            tx, css,
            {.p1 = {.x = 43.4, .y = 15.6}, .p2 = {.x = 43.4, .y = 3.1}});
        setEdge(c1s, {.width = 7.5, .x1 = 0, .x2 = 0});
        setSplit(c1s, {.width = 4.0, .x1 = 0, .x2 = 0});

        auto c2s = g.createCrosswalk(
            tx, css,
            {.p1 = {.x = 43.4, .y = 0.6}, .p2 = {.x = 43.4, .y = -9.2}});
        setEdge(c2s, {.width = 7.5, .x1 = 0, .x2 = 0});
        setSplit(c2s, {.width = 4.0, .x1 = 0, .x2 = 0});

        Polyline cssPath;
        cssPath.setStart({.x = 42, .y = -9.2});
        cssPath.addLine({.x = 42, .y = 15.6});
        g.setCSeriesCriticalPath(tx, css, cssPath);

        // CW E
        auto cse = g.createCrosswalkSeries(tx, n1);
        g.setCSeriesType(tx, cse, CrosswalkType::PEDESTRIAN_AND_CYCLIST);
        auto c1e = g.createCrosswalk(
            tx, cse,
            {.p1 = {.x = 10, .y = 33.3}, .p2 = {.x = 20.4, .y = 33.45}});
        setEdge(c1e, {.width = 7.5, .x1 = 0, .x2 = 1});
        setSplit(c1e, {.width = 4.0, .x1 = 0, .x2 = 0});

        auto c2e = g.createCrosswalk(
            tx, cse,
            {.p1 = {.x = -4.9, .y = 33.4}, .p2 = {.x = 5.45, .y = 33.4}});
        setEdge(c2e, {.width = 7.5, .x1 = 0.5, .x2 = 1});
        setSplit(c2e, {.width = 4.0, .x1 = 0, .x2 = 0});

        Polyline csePath;
        csePath.setStart({.x = 20.7, .y = 30.9});
        csePath.addLine({.x = -4.4, .y = 30.8});
        g.setCSeriesCriticalPath(tx, cse, csePath);

        // CW W
        auto csw = g.createCrosswalkSeries(tx, n1);
        g.setCSeriesType(tx, csw, CrosswalkType::PEDESTRIAN_AND_CYCLIST);
        auto c1w = g.createCrosswalk(
            tx, csw,
            {.p1 = {.x = 23.3, .y = -21.4}, .p2 = {.x = 13.5, .y = -22.0}});
        setEdge(c1w, {.width = 7.5, .x1 = 2, .x2 = 0});
        setSplit(c1w, {.width = 4.0, .x1 = 0, .x2 = 0});

        auto c2w = g.createCrosswalk(
            tx, csw,
            {.p1 = {.x = 9.8, .y = -22.3}, .p2 = {.x = 3.6, .y = -22.7}});
        setEdge(c2w, {.width = 7.5, .x1 = -0.5, .x2 = 0.5});
        setSplit(c2w, {.width = 4.0, .x1 = 0, .x2 = 0});

        Polyline cswPath;
        cswPath.setStart({.x = 23.1, .y = -19.6});
        cswPath.addLine({.x = 3.3, .y = -20.7});
        g.setCSeriesCriticalPath(tx, csw, cswPath);

        g.setIntergreenOverride(tx, n1, m1n[0], m1s[0], 7, 7);
        g.setIntergreenOverride(tx, n1, m1e[0], m1w[0], 7, 7);

        // demand
        d.setFlow(tx, m1n[0], n1, Flow(275));
        d.setFlow(tx, m1n[1], n1, Flow(1150));
        d.setFlow(tx, m1n[2], n1, Flow(100));

        d.setFlow(tx, m1s[0], n1, Flow(40));
        d.setFlow(tx, m1s[1], n1, Flow(115));
        d.setFlow(tx, m1s[2], n1, Flow(680));
        d.setFlow(tx, m1s[3], n1, Flow(195));

        d.setFlow(tx, m1e[0], n1, Flow(185));
        d.setFlow(tx, m1e[1], n1, Flow(460));
        d.setFlow(tx, m1e[2], n1, Flow(200));

        d.setFlow(tx, m1w[0], n1, Flow(185));
        d.setFlow(tx, m1w[1], n1, Flow(700));
        d.setFlow(tx, m1w[2], n1, Flow(170));
    });
}
}  // namespace

Application::Application() {
    project_ = std::make_unique<Project>(Document(regulations_));
    window_ = std::make_unique<MainWindow>(*project_);

    seedDemoContent(*project_);
    std::vector<TimingId> optimized;
    project_->run([&](Transaction& tx, Editor& e) {
        for (const auto& n : project_->document().graph().data().nodes()) {
            logs::streamIntergreen(n.id, project_->document().graph());
            logs::streamPermitted(n.id, project_->document().graph());
            if (auto t = e.runOptimizer(tx, n.id, 120)) optimized.push_back(*t);
            logs::intergreenMatrix(n.id, project_->document().signal());
        }
    });

    const auto& doc = project_->document();
    for (TimingId t : optimized) {
        logs::throughput(t, doc.control(), doc.signal(), doc.graph(),
                         doc.demand());
        logs::timingDiagram(t, doc.control(), doc.signal());
    }
}

Application::~Application() = default;

void Application::show() { window_->show(); }

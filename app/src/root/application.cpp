#include "application.hpp"

#include <QCoreApplication>
#include <QDebug>

#include "road_network/graph/data/movement.hpp"
#include "road_network/util/geometry/polyline.hpp"
#include "signal_debug.hpp"
#include "ui/network/network_scene.hpp"
#include "use_cases/demand_use_cases.hpp"
#include "use_cases/graph_use_cases.hpp"
#include "use_cases/signal_use_cases.hpp"

Application::Application() {
    document_ = std::make_unique<Document>();
    editorCtx_ = std::make_unique<EditorContext>(*document_, networkBus_);
    window_ = std::make_unique<MainWindow>();
    populator_ = std::make_unique<NetworkScenePopulator>(
        *window_->scene(), networkBus_, document_->graph().data(),
        document_->graph().derived(), document_->signal().data(),
        CoordTransform{});
}
Application::~Application() {
    populator_.reset();
    window_.reset();
    editorCtx_.reset();
    document_.reset();
}

void Application::show() {
    window_->show();
    window_->scene()->setBackdrop(
        QCoreApplication::applicationDirPath() +
            "/../../app/assets/Lazurowa-Czluchowska.jpg",
        {-100, -70}, 0.041667);
    auto& g = editorCtx_->graph();
    auto& d = editorCtx_->demand();
    auto& s = editorCtx_->signal();

    auto n1 = g.createNode({.x = 10.0f, .y = 5.0f});
    // N
    auto e1n = g.createEntry(n1, {.x = -24.7f, .y = 6.2f}, {.dx = 1, .dy = 0});
    g.setEntryLanes(e1n, {{.width = 3.f}, {}, {}, {.width = 3.f}});
    auto x1n = g.createExit(n1, {.x = -24.7f, .y = 8.7f}, {.dx = -1, .dy = 0});
    g.setExitLanes(x1n, {{3.f}, {3.2f}, {}});

    // S
    auto e1s = g.createEntry(n1, {.x = 46.7f, .y = 3.2f}, {.dx = -1, .dy = 0});
    g.setEntryLanes(e1s, {{3.f}, {3.2f}, {3.4f}, {3.f}});
    auto x1s = g.createExit(n1, {.x = 46.7f, .y = 0.8f}, {.dx = 1, .dy = 0});
    g.setExitLanes(x1s, {{}, {}, {3.2f}});

    // E
    auto e1e =
        g.createEntry(n1, {.x = 5, .y = 37.2f}, {.dx = .08f, .dy = -.7f});
    g.setEntryLanes(e1e, {{3.3f}, {3.3f}, {3.3f}});
    auto x1e =
        g.createExit(n1, {.x = 9.7f, .y = 37.5f}, {.dx = -.08f, .dy = .7f});
    g.setExitLanes(x1e, {{}, {}, {3.2f}});

    // W
    auto e1w = g.createEntry(n1, {.x = 14, .y = -25}, {.dx = -.1f, .dy = .7f});
    g.setEntryLanes(e1w, {{3.2f}, {3.2f}, {3.2f}});
    auto x1w = g.createExit(n1, {.x = 10, .y = -25}, {.dx = .12f, .dy = -.7f});
    g.setExitLanes(x1w, {{3}, {3}});

    // N
    auto m1n = g.setEntryLayout(e1n,
                                {{.exit = x1e, .laneCount = 1},
                                 {.exit = x1s, .laneCount = 2},
                                 {.exit = x1w, .laneCount = 1}},
                                {false, false});
    if (!m1n) qDebug() << "m1n err" << static_cast<int>(m1n.error());
    g.setMovementPathSpec(m1n.value()[0], {10, 6, QuadBezierParams{}});
    g.setMovementPathSpec(m1n.value()[1], {16, 16, LineParams{}});
    g.setMovementPathSpec(m1n.value()[2], {15, 10, QuadBezierParams{}});

    // S
    auto m1s = g.setEntryLayout(e1s,
                                {{.exit = x1s, .laneCount = 1},
                                 {.exit = x1w, .laneCount = 1},
                                 {.exit = x1n, .laneCount = 2},
                                 {.exit = x1e, .laneCount = 1}},
                                {true, false, false});
    if (!m1s) qDebug() << "m1s err" << static_cast<int>(m1s.error());
    g.setMovementPathSpec(m1s.value()[0], {12, 11, CubicBezierParams{14, 12}});
    g.setMovementPathSpec(m1s.value()[1], {15, 10, QuadBezierParams{}});
    g.setMovementPathSpec(m1s.value()[2], {16, 16, LineParams{}});
    g.setMovementPathSpec(m1s.value()[3], {15, 10, QuadBezierParams{}});

    // E
    auto m1e = g.setEntryLayout(e1e,
                                {{.exit = x1s, .laneCount = 1},
                                 {.exit = x1w, .laneCount = 2},
                                 {.exit = x1n, .laneCount = 1}},
                                {false, true});
    if (!m1e) qDebug() << "m1e err" << static_cast<int>(m1e.error());
    g.setMovementPathSpec(m1e.value()[0], {12, 9, QuadBezierParams{}});
    g.setMovementPathSpec(m1e.value()[2], {2, 7, QuadBezierParams{}});

    // W
    auto m1w = g.setEntryLayout(e1w,
                                {{.exit = x1n, .laneCount = 1},
                                 {.exit = x1e, .laneCount = 2},
                                 {.exit = x1s, .laneCount = 1}},
                                {false, true});
    if (!m1w) qDebug() << "m1w err" << static_cast<int>(m1w.error());
    g.setMovementPathSpec(m1w.value()[0], {14, 11, QuadBezierParams{}});
    g.setMovementPathSpec(m1w.value()[2], {2, 11, QuadBezierParams{}});

    // CW N
    auto csn = g.createCrosswalkSeries(n1);
    auto c1n = g.createCrosswalk(
        csn, {.p1 = {.x = -21.8f, .y = 8.6f}, .p2 = {.x = -22.1f, .y = 18}});
    g.setCrosswalkType(c1n, CrosswalkType::PEDESTRIAN_AND_CYCLIST);
    g.setCrosswalkEdge(c1n, {.width = 7.5f, .x1 = 0, .x2 = 0});

    auto c2n = g.createCrosswalk(
        csn, {.p1 = {.x = -21.8f, .y = -6.6f}, .p2 = {.x = -21.8f, .y = 6.2f}});
    g.setCrosswalkType(c2n, CrosswalkType::PEDESTRIAN_AND_CYCLIST);
    g.setCrosswalkEdge(c2n, {.width = 7.5f, .x1 = 0, .x2 = 0});

    Polyline csnPath;
    csnPath.setStart({.x = -20, .y = -6.6f});
    csnPath.addLine({.x = -20, .y = 18});
    g.setCrosswalkSeriesPedPath(csn, csnPath);

    // CW S
    auto css = g.createCrosswalkSeries(n1);
    auto c1s = g.createCrosswalk(
        css, {.p1 = {.x = 43.4f, .y = 15.6f}, .p2 = {.x = 43.4f, .y = 3.1f}});
    g.setCrosswalkType(c1s, CrosswalkType::PEDESTRIAN_AND_CYCLIST);
    g.setCrosswalkEdge(c1s, {.width = 7.5f, .x1 = 0, .x2 = 0});

    auto c2s = g.createCrosswalk(
        css, {.p1 = {.x = 43.4f, .y = 0.6f}, .p2 = {.x = 43.4f, .y = -9.2f}});
    g.setCrosswalkType(c2s, CrosswalkType::PEDESTRIAN_AND_CYCLIST);
    g.setCrosswalkEdge(c2s, {.width = 7.5f, .x1 = 0, .x2 = 0});

    Polyline cssPath;
    cssPath.setStart({.x = 42, .y = -9.2f});
    cssPath.addLine({.x = 42, .y = 15.6f});
    qDebug() << "css len =" << cssPath.length();
    g.setCrosswalkSeriesPedPath(css, cssPath);

    // CW E
    auto cse = g.createCrosswalkSeries(n1);
    auto c1e = g.createCrosswalk(
        cse, {.p1 = {.x = 10, .y = 33.3f}, .p2 = {.x = 20.4f, .y = 33.45f}});
    g.setCrosswalkType(c1e, CrosswalkType::PEDESTRIAN_AND_CYCLIST);
    g.setCrosswalkEdge(c1e, {.width = 7.5f, .x1 = 0, .x2 = 1});

    auto c2e = g.createCrosswalk(
        cse, {.p1 = {.x = -4.9f, .y = 33.4f}, .p2 = {.x = 5.45f, .y = 33.4f}});
    g.setCrosswalkType(c2e, CrosswalkType::PEDESTRIAN_AND_CYCLIST);
    g.setCrosswalkEdge(c2e, {.width = 7.5f, .x1 = 0.5f, .x2 = 1});

    Polyline csePath;
    csePath.setStart({.x = 20.7f, .y = 30.9f});
    csePath.addLine({.x = -4.4f, .y = 30.8f});
    qDebug() << "cse len =" << csePath.length();
    g.setCrosswalkSeriesPedPath(cse, csePath);

    // CW W
    auto csw = g.createCrosswalkSeries(n1);
    auto c1w = g.createCrosswalk(csw, {.p1 = {.x = 23.3f, .y = -21.4f},
                                       .p2 = {.x = 13.5f, .y = -22.0f}});
    g.setCrosswalkType(c1w, CrosswalkType::PEDESTRIAN_AND_CYCLIST);
    g.setCrosswalkEdge(c1w, {.width = 7.5f, .x1 = 2, .x2 = 0});

    auto c2w = g.createCrosswalk(
        csw, {.p1 = {.x = 9.8f, .y = -22.3f}, .p2 = {.x = 3.6f, .y = -22.7f}});
    g.setCrosswalkType(c2w, CrosswalkType::PEDESTRIAN_AND_CYCLIST);
    g.setCrosswalkEdge(c2w, {.width = 7.5f, .x1 = -0.5f, .x2 = 0.5f});

    Polyline cswPath;
    cswPath.setStart({.x = 23.1f, .y = -19.6f});
    cswPath.addLine({.x = 3.3f, .y = -20.7f});
    g.setCrosswalkSeriesPedPath(csw, cswPath);

    g.setStreamIntergreenOverride(n1, m1n.value()[0], m1s.value()[0], 7, 7);
    g.setStreamIntergreenOverride(n1, m1e.value()[0], m1w.value()[0], 7, 7);
    // g.setStreamPermittedOverride(n1, m1n.value()[0], m1s.value()[0], false);

    // demand
    d.setFlow(m1n.value()[0], Flow(255));  // 255
    d.setFlow(m1n.value()[1], Flow(1150));
    d.setFlow(m1n.value()[2], Flow(100));

    d.setFlow(m1s.value()[0], Flow(60));   // 60
    d.setFlow(m1s.value()[1], Flow(100));  // 100
    d.setFlow(m1s.value()[2], Flow(1000));
    d.setFlow(m1s.value()[3], Flow(219));

    d.setFlow(m1e.value()[0], Flow(1));  // 280
    d.setSaturationOverride(m1e.value()[0], {.prot = std::nullopt, .perm = 0});
    d.setFlow(m1e.value()[1], Flow(800));
    d.setFlow(m1e.value()[2], Flow(255));

    d.setFlow(m1w.value()[0], Flow(1));  // 145
    d.setSaturationOverride(m1w.value()[0], {.prot = std::nullopt, .perm = 0});
    d.setFlow(m1w.value()[1], Flow(500));
    d.setFlow(m1w.value()[2], Flow(155));

    debug_print::streamPermitted(n1, document_->graph());
    debug_print::streamIntergreen(n1, document_->graph());

    auto result = s.runOptimizer(n1, 120);

    debug_print::intergreenMatrix(n1, document_->signal());
    debug_print::nodeTimings(n1, document_->signal(), document_->graph(),
                             document_->demand());
}

#pragma once

#include <QObject>
#include <road_network/util/ids.hpp>

#include "bus/subscription_bag.hpp"

struct UiCore;
class SceneItemRegistry;

// Bus -> scene: creates, updates and removes items as domain events arrive,
// and pushes geometry into them
class SceneSync : public QObject {
    Q_OBJECT
   public:
    SceneSync(UiCore& core, SceneItemRegistry& items,
              QObject* parent = nullptr);

   private:
    UiCore& core_;
    SceneItemRegistry& items_;
    SubscriptionBag subs_;

    // project load
    void populate();

    void syncNode(NodeId id);
    void syncEntry(EntryId id);
    void syncExit(ExitId id);
    void syncMovement(MovementId id);
    void syncCrosswalkSeries(CrosswalkSeriesId id);
    void syncCrosswalk(CrosswalkId id);

    // everything owned by the node: nodeUpdated (post-solve)
    void syncNodeDeep(NodeId id);
};

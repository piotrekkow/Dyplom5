#pragma once

#include <road_network/demand/data/flow.hpp>
#include <road_network/demand/data/saturation_override.hpp>
#include <road_network/util/ids.hpp>

class DemandEditor;
class CommandStack;

class DemandUseCases {
   public:
    DemandUseCases(DemandEditor& editor, CommandStack& cmdStack);

    void setFlow(MovementId at, Flow value);
    void eraseFlow(MovementId at);
    void setSaturationOverride(MovementId at, SaturationOverride override);
    void eraseSaturationOverride(MovementId at);

   private:
    DemandEditor& editor_;
    CommandStack& cmdStack_;
};

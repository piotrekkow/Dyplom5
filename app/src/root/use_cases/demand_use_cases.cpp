#include "demand_use_cases.hpp"

#include "root/command/transaction.hpp"
#include "root/command_stack.hpp"
#include "root/editor/demand_editor.hpp"

DemandUseCases::DemandUseCases(DemandEditor& editor, CommandStack& cmdStack)
    : editor_(editor), cmdStack_(cmdStack) {}

void DemandUseCases::setFlow(MovementId at, Flow value) {
    Transaction tx;
    editor_.setFlowTx(tx, at, value);
    cmdStack_.push(tx.commit());
}

void DemandUseCases::eraseFlow(MovementId at) {
    Transaction tx;
    editor_.eraseFlowTx(tx, at);
    cmdStack_.push(tx.commit());
}

void DemandUseCases::setSaturationOverride(MovementId at,
                                           SaturationOverride override) {
    Transaction tx;
    editor_.setSaturationOverrideTx(tx, at, override);
    cmdStack_.push(tx.commit());
}

void DemandUseCases::eraseSaturationOverride(MovementId at) {
    Transaction tx;
    editor_.eraseSaturationOverrideTx(tx, at);
    cmdStack_.push(tx.commit());
}

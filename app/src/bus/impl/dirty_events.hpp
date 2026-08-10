#pragma once

#include <road_network/util/ids.hpp>

class Transaction;

template <typename ChildId, typename ParentId = void>
struct CreatedTx {
    ChildId id;
    ParentId at;
    Transaction& tx;
};

template <typename ChildId, typename ParentId = void>
struct UpdatedTx {
    ChildId id;
    ParentId at;
    Transaction& tx;
};

template <typename ChildId, typename ParentId = void>
struct RemovedTx {
    ChildId id;
    ParentId at;
    Transaction& tx;
};

template <typename ChildId>
struct CreatedTx<ChildId, void> {
    ChildId id;
    Transaction& tx;
};

template <typename ChildId>
struct UpdatedTx<ChildId, void> {
    ChildId id;
    Transaction& tx;
};

template <typename ChildId>
struct RemovedTx<ChildId, void> {
    ChildId id;
    Transaction& tx;
};

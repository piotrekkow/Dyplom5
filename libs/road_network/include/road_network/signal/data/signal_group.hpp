#pragma once

#include <array>
#include <variant>
#include <vector>

#include "road_network/util/ids.hpp"
#include "signal_aspect.hpp"

class CrosswalkSignalGroup {
   public:
    CrosswalkSignalGroup(NodeId at, std::vector<CrosswalkId> crosswalks,
                         int intervalMin)
        : at_(at),
          crosswalks_(std::move(crosswalks)),
          intervalMin_(intervalMin) {}

    [[nodiscard]] NodeId at() const { return at_; }
    [[nodiscard]] const std::vector<CrosswalkId>& crosswalks() const {
        return crosswalks_;
    }
    [[nodiscard]] std::vector<StreamId> streams() const {
        return {crosswalks_.begin(), crosswalks_.end()};
    }
    [[nodiscard]] int intervalMin() const { return intervalMin_; }
    void setIntervalMin(int min) { intervalMin_ = min; }

    static constexpr SignalAspect activeAspect() { return SignalAspect::GREEN; }
    static constexpr SignalAspect inactiveAspect() { return SignalAspect::RED; }
    static constexpr std::array<SignalAspect, 0> atomicTransitionAppr() {
        return {};
    }
    static constexpr std::array<SignalAspect, 4> atomicTransitionEvac() {
        return {SignalAspect::FLASHING_GREEN, SignalAspect::FLASHING_GREEN,
                SignalAspect::FLASHING_GREEN, SignalAspect::FLASHING_GREEN};
    }

   private:
    NodeId at_;
    std::vector<CrosswalkId> crosswalks_;
    int intervalMin_;
};

class MovementSignalGroup {
   public:
    MovementSignalGroup(NodeId at, std::vector<ClusterId> clusters,
                        std::vector<MovementId> movements, bool isProtected,
                        int intervalMin)
        : at_(at),
          clusters_(std::move(clusters)),
          movements_(std::move(movements)),
          isProtected_(isProtected),
          intervalMin_(intervalMin) {}

    [[nodiscard]] NodeId at() const { return at_; }
    [[nodiscard]] const std::vector<ClusterId>& clusters() const {
        return clusters_;
    }
    [[nodiscard]] std::vector<StreamId> streams() const {
        return {movements_.begin(), movements_.end()};
    }
    [[nodiscard]] int intervalMin() const { return intervalMin_; }
    [[nodiscard]] bool isProtected() const { return isProtected_; }
    void setIntervalMin(int min) { intervalMin_ = min; }
    void setProtected(bool p) { isProtected_ = p; }

    static constexpr SignalAspect activeAspect() { return SignalAspect::GREEN; }
    static constexpr SignalAspect inactiveAspect() { return SignalAspect::RED; }
    static constexpr std::array<SignalAspect, 1> atomicTransitionAppr() {
        return {SignalAspect::RED_YELLOW};
    }
    static constexpr std::array<SignalAspect, 3> atomicTransitionEvac() {
        return {SignalAspect::YELLOW, SignalAspect::YELLOW,
                SignalAspect::YELLOW};
    }

   private:
    NodeId at_;
    std::vector<ClusterId> clusters_;
    std::vector<MovementId> movements_;
    bool isProtected_;
    int intervalMin_;
};

class ConditionalTurnArrowSignalGroup {
   public:
    ConditionalTurnArrowSignalGroup(NodeId at, MovementId movement,
                                    int intervalMin)
        : at_(at), movement_(movement), intervalMin_(intervalMin) {}

    [[nodiscard]] NodeId at() const { return at_; }
    [[nodiscard]] MovementId movement() const { return movement_; }
    [[nodiscard]] std::vector<StreamId> streams() const {
        return {static_cast<StreamId>(movement_)};
    }
    [[nodiscard]] int intervalMin() const { return intervalMin_; }
    void setIntervalMin(int min) { intervalMin_ = min; }

    static constexpr SignalAspect activeAspect() { return SignalAspect::GREEN; }
    static constexpr SignalAspect inactiveAspect() {
        return SignalAspect::DARK;
    }
    static constexpr std::array<SignalAspect, 0> atomicTransitionAppr() {
        return {};
    }
    static constexpr std::array<SignalAspect, 0> atomicTransitionEvac() {
        return {};
    }

   private:
    NodeId at_;
    MovementId movement_;
    int intervalMin_;
};

using SignalGroup = std::variant<CrosswalkSignalGroup, MovementSignalGroup,
                                 ConditionalTurnArrowSignalGroup>;
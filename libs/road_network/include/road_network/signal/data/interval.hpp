#pragma once

class Interval {
   public:
    Interval(int onset, int duration, int apprTransitionDuration,
             int evacTransitionDuration)
        : onset_(onset),
          duration_(duration),
          apprTransitionDuration_(apprTransitionDuration),
          evacTransitionDuration_(evacTransitionDuration) {}

    [[nodiscard]] int intervalStart() const {
        return onset_ - apprTransitionDuration_;
    }
    [[nodiscard]] int intervalEnd() const {
        return onset_ + duration_ + evacTransitionDuration_;
    }
    [[nodiscard]] int greenStart() const { return onset_; }
    [[nodiscard]] int greenEnd() const { return onset_ + duration_; }
    [[nodiscard]] int greenDuration() const { return duration_; }
    [[nodiscard]] int totalDuration() const {
        return apprTransitionDuration_ + duration_ + evacTransitionDuration_;
    }

   private:
    int onset_;
    int duration_;
    int apprTransitionDuration_;
    int evacTransitionDuration_;
};
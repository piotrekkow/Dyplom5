#pragma once

class Flow {
   public:
    Flow(double vehiclesPerHour, double uc = 0.f) {
        setVehiclesPerHour(vehiclesPerHour);
        setUc(uc);
    }

    [[nodiscard]] double vehiclesPerHour() const { return vehiclesPerHour_; }
    [[nodiscard]] double eqVehiclesPerHour() const {
        return vehiclesPerHour_ * (1 + uc_);
    }
    [[nodiscard]] double uc() const { return uc_; }

    void setVehiclesPerHour(double value) { vehiclesPerHour_ = value; }
    void setUc(double value) { uc_ = value; }

   private:
    double vehiclesPerHour_ = 0.f;
    double uc_ = 0.f;
};
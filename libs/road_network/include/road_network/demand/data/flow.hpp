#pragma once

class Flow {
   public:
    Flow(float vehiclesPerHour, float uc = 0.f) {
        setVehiclesPerHour(vehiclesPerHour);
        setUc(uc);
    }

    [[nodiscard]] float vehiclesPerHour() const { return vehiclesPerHour_; }
    [[nodiscard]] float eqVehiclesPerHour() const {
        return vehiclesPerHour_ * (1 + uc_);
    }
    [[nodiscard]] float uc() const { return uc_; }

    void setVehiclesPerHour(float value);
    void setUc(float value);

   private:
    float vehiclesPerHour_ = 0.f;
    float uc_ = 0.f;
};
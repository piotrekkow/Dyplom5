#pragma once

struct IntergreenDiagnostic {
    struct Base {
        double v_evac, v_appr, l_evac, t_y, t_min, t_apprAdd;
    };
    Base base;
    double s_evac{}, s_appr{};
    [[nodiscard]] double t_evac() const {
        return (s_evac + base.l_evac) / base.v_evac;
    }
    [[nodiscard]] double t_appr() const {
        return (s_appr / base.v_appr) + base.t_apprAdd;
    }
    [[nodiscard]] double t_raw() const { return base.t_y + t_evac() - t_appr(); }
};
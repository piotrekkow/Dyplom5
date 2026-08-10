#pragma once

struct Saturation {
    double prot;
    double perm;

    Saturation operator+(Saturation o) const {
        return {.prot = prot + o.prot, .perm = perm + o.perm};
    }
    Saturation operator-(Saturation o) const {
        return {.prot = prot - o.prot, .perm = perm - o.perm};
    }
    Saturation operator*(Saturation o) const {
        return {.prot = prot * o.prot, .perm = perm * o.perm};
    }
    Saturation operator/(Saturation o) const {
        return {.prot = prot / o.prot, .perm = perm / o.perm};
    }
    Saturation operator*(double f) const {
        return {.prot = prot * f, .perm = perm * f};
    }
    Saturation operator/(double f) const {
        return {.prot = prot / f, .perm = perm / f};
    }
    Saturation& operator+=(Saturation o) {
        prot += o.prot;
        perm += o.perm;
        return *this;
    }

    friend Saturation operator*(double f, Saturation s) { return s * f; }
    friend Saturation operator/(double f, Saturation s) {
        return {.prot = f / s.prot, .perm = f / s.perm};
    }
};
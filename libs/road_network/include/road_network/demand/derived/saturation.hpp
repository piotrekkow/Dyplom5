#pragma once

struct Saturation {
    float prot;
    float perm;

    Saturation operator+(Saturation o) const {
        return {prot + o.prot, perm + o.perm};
    }
    Saturation operator-(Saturation o) const {
        return {prot - o.prot, perm - o.perm};
    }
    Saturation operator*(Saturation o) const {
        return {prot * o.prot, perm * o.perm};
    }
    Saturation operator/(Saturation o) const {
        return {prot / o.prot, perm / o.perm};
    }
    Saturation operator*(float f) const { return {prot * f, perm * f}; }
    Saturation operator/(float f) const { return {prot / f, perm / f}; }
    Saturation& operator+=(Saturation o) {
        prot += o.prot;
        perm += o.perm;
        return *this;
    }

    friend Saturation operator*(float f, Saturation s) { return s * f; }
    friend Saturation operator/(float f, Saturation s) {
        return {f / s.prot, f / s.perm};
    }
};
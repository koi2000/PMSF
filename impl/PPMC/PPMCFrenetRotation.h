// SPDX-License-Identifier: Apache-2.0
//
// PPMC-local, double-precision Frenet rotation (bijection) for geometry
// residuals.  The upstream PPMC port in deps/MCGAL (MeshUtils.h) keeps the
// reference algorithm but degrades the intermediate arithmetic to float
// (Vector3 accumulation, float matProd/compRotationMat), which makes the
// round-trip lossy for larger residuals (e.g. curvature-predicted values):
// observed +-1..2 component errors on ~1/3000 residuals.  The reference
// implementation explicitly documents "Double precision needed for the frenet
// rotation computations" (CGAL::MatrixC33<MyKernelDouble>).
//
// This header is a line-for-line double-precision port used only by
// impl/PPMC (encoder + decoder) so that other algorithms (PCLTTM) keep their
// frozen bytes.  Both sides use it, so the transform is symmetric by
// construction; the frame itself still comes from pmsf::safeHalfedgeFrame.

#ifndef PMSF_PPMC_FRENET_ROTATION_DOUBLE_H
#define PMSF_PPMC_FRENET_ROTATION_DOUBLE_H

#include "Point.h"
#include "Halfedge.h"
#include "Facet.h"
#include "Mesh.h"
#include "predict/FrenetResidualCodec.h"

#include <cmath>
#include <array>
#include <vector>

namespace ppmc {

struct Mat3d {
    double c[3][3];

    Mat3d() {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) c[i][j] = 0.0;
    }
    Mat3d(double a, double b, double cc, double d, double e, double f, double g, double h, double i) {
        c[0][0] = a; c[0][1] = b; c[0][2] = cc;
        c[1][0] = d; c[1][1] = e; c[1][2] = f;
        c[2][0] = g; c[2][1] = h; c[2][2] = i;
    }
};

inline Mat3d matProd3(const Mat3d& m, const Mat3d& n) {
    Mat3d o;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) s += m.c[i][k] * n.c[k][j];
            o.c[i][j] = s;
        }
    }
    return o;
}

inline void matVec3(const Mat3d& m, const double u[3], double out[3]) {
    for (int i = 0; i < 3; ++i)
        out[i] = m.c[i][0] * u[0] + m.c[i][1] * u[1] + m.c[i][2] * u[2];
}

inline int signeD(double x) { return (x < 0.0) ? -1 : 1; }

// Same decision tree as reference frenetRotation.cpp::compRotationMat, but all
// intermediate quantities are double.
inline std::array<Mat3d, 15> compRotationMatD(const double T1[3], const double T2[3],
                                               const double normal[3]) {
    Mat3d rotMat(T1[0], T2[0], normal[0],
                 T1[1], T2[1], normal[1],
                 T1[2], T2[2], normal[2]);
    Mat3d M = rotMat;

    const Mat3d D1(1, 0, 0, 0, 1, 0, 0, 0, 1);
    const Mat3d D2(0, 1, 0, 1, 0, 0, 0, 0, 1);
    const Mat3d D3(-1, 0, 0, 0, -1, 0, 0, 0, 1);
    const Mat3d D4(1, 0, 0, 0, 0, 1, 0, 1, 0);
    const Mat3d D5(1, 0, 0, 0, -1, 0, 0, 0, -1);

    std::array<Mat3d, 15> S;  // perf: stack-allocated (no per-vertex heap vector)

    // Smallest rotation angle first.
    if (std::fabs(M.c[0][2]) > std::fabs(M.c[1][2]))
        S[0] = D2;
    else
        S[0] = D1;
    M = matProd3(S[0], M);

    if (M.c[1][2] < 0.0)
        S[1] = D3;
    else
        S[1] = D1;
    M = matProd3(S[1], M);

    // First rotation angle phi.
    double len = M.c[0][2] * M.c[0][2] + M.c[1][2] * M.c[1][2];
    double phi = (len == 0.0) ? 0.0 : signeD(-M.c[0][2]) * std::acos(M.c[1][2] / std::sqrt(len));

    S[2] = Mat3d(1, -std::tan(phi / 2.0), 0, 0, 1, 0, 0, 0, 1);
    S[3] = Mat3d(1, 0, 0, std::sin(phi), 1, 0, 0, 0, 1);
    S[4] = S[2];

    Mat3d R1inv(std::cos(phi), std::sin(phi), 0,
                -std::sin(phi), std::cos(phi), 0,
                0, 0, 1);
    M = matProd3(R1inv, M);

    if (std::fabs(M.c[1][2]) > std::fabs(M.c[2][2]))
        S[5] = D4;
    else
        S[5] = D1;
    M = matProd3(S[5], M);

    if (M.c[2][2] < 0.0)
        S[6] = D5;
    else
        S[6] = D1;
    M = matProd3(S[6], M);

    // Second rotation angle psi.
    len = M.c[1][2] * M.c[1][2] + M.c[2][2] * M.c[2][2];
    double psi = (len == 0.0) ? 0.0 : signeD(-M.c[1][2]) * std::acos(M.c[2][2] / std::sqrt(len));

    S[7] = Mat3d(1, 0, 0, 0, 1, -std::tan(psi / 2.0), 0, 0, 1);
    S[8] = Mat3d(1, 0, 0, 0, 1, 0, 0, std::sin(psi), 1);
    S[9] = S[7];

    Mat3d R2inv(1, 0, 0,
                0, std::cos(psi), std::sin(psi),
                0, -std::sin(psi), std::cos(psi));
    M = matProd3(R2inv, M);

    if (std::fabs(M.c[0][1]) > std::fabs(M.c[1][1]))
        S[10] = D2;
    else
        S[10] = D1;
    M = matProd3(S[10], M);

    if (M.c[1][1] < 0.0)
        S[11] = D3;
    else
        S[11] = D1;
    M = matProd3(S[11], M);

    // Last rotation angle theta.
    len = M.c[0][1] * M.c[0][1] + M.c[1][1] * M.c[1][1];
    double theta = (len == 0.0) ? 0.0 : signeD(-M.c[0][1]) * std::acos(M.c[1][1] / std::sqrt(len));

    S[12] = Mat3d(1, -std::tan(theta / 2.0), 0, 0, 1, 0, 0, 0, 1);
    S[13] = Mat3d(1, 0, 0, std::sin(theta), 1, 0, 0, 0, 1);
    S[14] = S[12];

    return S;
}

inline MCGAL::Vector3i frenetRotationD(MCGAL::Vector3i dist, const double T1[3],
                                      const double T2[3], const double normal[3]) {
    const std::array<Mat3d, 15> S = compRotationMatD(T1, T2, normal);

    double u[3] = {static_cast<double>(dist[0]), static_cast<double>(dist[1]),
                   static_cast<double>(dist[2])};
    for (int i = 0; i < 15; ++i) {
        Mat3d mi;
        if (i == 0 || i == 1 || i == 5 || i == 6 || i == 10 || i == 11) {
            mi = S[i];
        } else {
            mi = Mat3d(S[i].c[0][0], -S[i].c[0][1], -S[i].c[0][2],
                       -S[i].c[1][0], S[i].c[1][1], -S[i].c[1][2],
                       -S[i].c[2][0], -S[i].c[2][1], S[i].c[2][2]);
        }
        double nu[3];
        matVec3(mi, u, nu);
        u[0] = std::round(nu[0]);
        u[1] = std::round(nu[1]);
        u[2] = std::round(nu[2]);
    }
    return MCGAL::Vector3i(static_cast<int>(u[0]), static_cast<int>(u[1]),
                           static_cast<int>(u[2]));
}

inline MCGAL::Vector3i invFrenetRotationD(MCGAL::Vector3i frenet, const double T1[3],
                                        const double T2[3], const double normal[3]) {
    const std::array<Mat3d, 15> S = compRotationMatD(T1, T2, normal);

    double u[3] = {static_cast<double>(frenet[0]), static_cast<double>(frenet[1]),
                   static_cast<double>(frenet[2])};
    for (int i = 14; i > -1; --i) {
        double nu[3];
        matVec3(S[i], u, nu);
        u[0] = std::round(nu[0]);
        u[1] = std::round(nu[1]);
        u[2] = std::round(nu[2]);
    }
    return MCGAL::Vector3i(static_cast<int>(u[0]), static_cast<int>(u[1]),
                           static_cast<int>(u[2]));
}

// Deterministic gate-anchored Frenet frame.
//
// `pmsf::safeHalfedgeFrame` takes the face normal from
// MCGAL::computeNormalFromHeh(), whose accumulation starts at the face's
// proxyHalfedge.  Encoder and decoder may assign different proxy halfedges to
// the same reconstructed face, and the float accumulation order then differs
// by ~1e-6 — enough to flip a rotation rounding for a handful of residuals and
// break decoder symmetry.  This version computes the normal with a fixed
// Newell traversal anchored at the gate halfedge (identical input on both
// sides => bit-identical frame), in double precision.
// Deterministic quantized-coordinate frame: positions come from the integer
// quantization lattice (mesh->floatPosToInt) instead of raw float vertex
// coordinates.  Encoder and decoder hold dequantized floats that can differ in
// the last mantissa bits, which perturbs the Newell normal by ~1e-6 and flips a
// rotation rounding for a handful of residuals (D28).  Integer lattice input is
// bit-identical on both sides.
inline pmsf::FrenetFrame ppmcHalfedgeFrameQuantized(MCGAL::Halfedge* h, MCGAL::Mesh* mesh) {
    pmsf::FrenetFrame frame;
    if (mesh == nullptr) return pmsf::FrenetFrame{};
    auto qpos = [&](MCGAL::Vertex* v) -> std::array<double, 3> {
        if (!v) return {0.0, 0.0, 0.0};
        const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return {static_cast<double>(q[0]), static_cast<double>(q[1]), static_cast<double>(q[2])};
    };
    double nx = 0.0, ny = 0.0, nz = 0.0;
    if (h && h->face()) {
        MCGAL::Halfedge* e = h;
        int guard = 0;
        do {
            MCGAL::Vertex* vi = e->vertex();
            MCGAL::Vertex* vj = e->end_vertex();
            if (vi && vj) {
                const auto pi = qpos(vi);
                const auto pj = qpos(vj);
                const double xi = pi[0], yi = pi[1], zi = pi[2];
                const double xj = pj[0], yj = pj[1], zj = pj[2];
                nx += (yi - yj) * (zi + zj);
                ny += (zi - zj) * (xi + xj);
                nz += (xi - xj) * (yi + yj);
            }
            e = e->next();
        } while (e && e != h && ++guard < 4096);
    }
    double nl = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (!(nl > 0.0) || !std::isfinite(nl)) { nx = 0.0; ny = 0.0; nz = 1.0; nl = 1.0; }
    frame.normal = MCGAL::Vector3(static_cast<float>(nx / nl), static_cast<float>(ny / nl),
                                  static_cast<float>(nz / nl));
    std::array<double, 3> v1 = {0.0, 0.0, 0.0};
    std::array<double, 3> v2 = {1.0, 0.0, 0.0};
    if (h && h->vertex()) v1 = qpos(h->vertex());
    if (h && h->opposite() && h->opposite()->vertex()) v2 = qpos(h->opposite()->vertex());
    else if (h && h->end_vertex()) v2 = qpos(h->end_vertex());
    const double gx = v1[0] - v2[0], gy = v1[1] - v2[1], gz = v1[2] - v2[2];
    const double ex = static_cast<double>(frame.normal[0]);
    const double ey = static_cast<double>(frame.normal[1]);
    const double ez = static_cast<double>(frame.normal[2]);
    const double dot = gx * ex + gy * ey + gz * ez;
    double t1x = gx - dot * ex, t1y = gy - dot * ey, t1z = gz - dot * ez;
    const double t1l = std::sqrt(t1x * t1x + t1y * t1y + t1z * t1z);
    if (t1l > 0.0 && std::isfinite(t1l)) { t1x /= t1l; t1y /= t1l; t1z /= t1l; }
    else { t1x = 1.0; t1y = 0.0; t1z = 0.0; }
    frame.t1 = MCGAL::Vector3(static_cast<float>(t1x), static_cast<float>(t1y), static_cast<float>(t1z));
    frame.t2 = MCGAL::Vector3(static_cast<float>(ey * t1z - ez * t1y),
                              static_cast<float>(ez * t1x - ex * t1z),
                              static_cast<float>(ex * t1y - ey * t1x));
    return frame;
}



}  // namespace ppmc

#endif  // PMSF_PPMC_FRENET_ROTATION_DOUBLE_H
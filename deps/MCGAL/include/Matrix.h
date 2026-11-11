/*
 * @Author: koi
 * @Date: 2025-09-12 17:26:50
 * @Description:
 */
#ifndef MATRIX_H
#define MATRIX_H

namespace MCGAL {
#include <string.h>
#include "Point.h"

class Matrix {
  public:
    Matrix() {
        memset(c, 0, sizeof(c));
    }

    Matrix(double r0x, double r0y, double r0z, double r1x, double r1y, double r1z, double r2x, double r2y, double r2z) {
        c[0][0] = r0x, c[0][1] = r0y;
        c[0][2] = r0z;
        c[1][0] = r1x, c[1][1] = r1y;
        c[1][2] = r1z;
        c[2][0] = r2x, c[2][1] = r2y;
        c[2][2] = r2z;
    }

    Matrix& operator=(const Matrix& m) {
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 3; ++j)
                c[i][j] = m.c[i][j];
        return *this;
    }

    Vector3 operator*(Vector3 v) {
        return Vector3(c[0][0] * v[0] + c[0][1] * v[1] + c[0][2] * v[2], c[1][0] * v[0] + c[1][1] * v[1] + c[1][2] * v[2],
                     c[2][0] * v[0] + c[2][1] * v[1] + c[2][2] * v[2]);
    }

    Vector3 r0() const {
        return Vector3(c[0][0], c[0][1], c[0][2]);
    }
    Vector3 r1() const {
        return Vector3(c[1][0], c[1][1], c[1][2]);
    }
    Vector3 r2() const {
        return Vector3(c[2][0], c[2][1], c[2][2]);
    }

  private:
    double c[3][3];
};

}  // namespace MCGAL

#endif
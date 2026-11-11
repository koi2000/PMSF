#ifndef POINT_H
#define POINT_H
#include <assert.h>
#include <math.h>
#include <stdexcept>
namespace MCGAL {
class Vector3;

class Point {
  public:
    Point() {
        v[0] = 0.0;
        v[1] = 0.0;
        v[2] = 0.0;
    }

    Point(float x, float y, float z) {
        v[0] = x;
        v[1] = y;
        v[2] = z;
    }

    Point(float x, float y, float z, int id) {
        v[0] = x;
        v[1] = y;
        v[2] = z;
    }

    Point(Point* pt) {
        assert(pt);
        for (int i = 0; i < 3; i++) {
            v[i] = pt->v[i];
        }
    };

    float x() const {
        return v[0];
    }

    float y() const {
        return v[1];
    }

    float z() const {
        return v[2];
    }

    // 两个点减法返回一个向量
    Point operator-(const Point& p) const {
        return Point(v[0] - p.x(), v[1] - p.y(), v[2] - p.z());
    }

    Point operator/(const double div) const {
        return Point(v[0] / div, v[1] / div, v[2] / div);
    }

    float& operator[](int index) {
        if (index >= 0 && index < 3) {
            return v[index];
        } else {
            throw std::out_of_range("Index out of range");
        }
    }

    bool operator==(const Point& p) {
        return v[0] == p.x() && v[1] == p.y() && v[2] == p.z();
        // return v[0] == p.x() && v[1] == p.y() && v[2] == p.z();
    }

    bool operator<(const Point& p) const {
        if (v[0] != p.v[0])
            return v[0] < p.v[0];
        if (v[1] != p.v[1])
            return v[1] < p.v[1];
        return v[2] < p.v[2];
    }

    bool operator>(const Point& p) const {
        if (v[0] != p.v[0])
            return v[0] > p.v[0];
        if (v[1] != p.v[1])
            return v[1] > p.v[1];
        return v[2] > p.v[2];
    }

    Point operator-(const Point& p) {
        return Point(v[0] - p.x(), v[1] - p.y(), v[2] - p.z());
    }

    Point operator+(const Point& p) {
        return Point(v[0] + p.x(), v[1] + p.y(), v[2] + p.z());
    }

    float length() {
        return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    // 向量叉积
    Point cross(const Point& v) const {
        return Point(y() * v.z() - z() * v.y(), z() * v.x() - x() * v.z(), x() * v.y() - y() * v.x());
    }

    // 向量点积
    double dot(const Point& v) const {
        return x() * v.x() + y() * v.y() + z() * v.z();
    }

    void normalize() {
        float len = length();
        if (len == 0) {
            throw std::runtime_error("Cannot normalize a zero vector");
        }
        v[0] /= len;
        v[1] /= len;
        v[2] /= len;
    }

  public:
    float v[3];
};

class PointInt {
  public:
    PointInt() {
        v[0] = 0.0;
        v[1] = 0.0;
        v[2] = 0.0;
    }

    PointInt(int x, int y, int z) {
        v[0] = x;
        v[1] = y;
        v[2] = z;
    }

    PointInt(PointInt* pt) {
        assert(pt);
        for (int i = 0; i < 3; i++) {
            v[i] = pt->v[i];
        }
    };

    int x() const {
        return v[0];
    }

    int y() const {
        return v[1];
    }

    int z() const {
        return v[2];
    }

    // 两个点减法返回一个向量
    PointInt operator-(const PointInt& p) const {
        return PointInt(v[0] - p.x(), v[1] - p.y(), v[2] - p.z());
    }

    PointInt operator/(const int div) const {
        return PointInt(v[0] / 2, v[1] / 2, v[2] / 2);
    }

    int& operator[](int index) {
        if (index >= 0 && index < 3) {
            return v[index];
        } else {
            throw std::out_of_range("Index out of range");
        }
    }

    const int& operator[](int index) const {
        if (index >= 0 && index < 3) {
            return v[index];
        } else {
            throw std::out_of_range("Index out of range");
        }
    }

    bool operator==(const PointInt& p) {
        return v[0] == p.x() && v[1] == p.y() && v[2] == p.z();
        // return v[0] == p.x() && v[1] == p.y() && v[2] == p.z();
    }

    bool operator<(const PointInt& p) const {
        if (v[0] != p.v[0])
            return v[0] < p.v[0];
        if (v[1] != p.v[1])
            return v[1] < p.v[1];
        return v[2] < p.v[2];
    }

    bool operator>(const PointInt& p) const {
        if (v[0] != p.v[0])
            return v[0] > p.v[0];
        if (v[1] != p.v[1])
            return v[1] > p.v[1];
        return v[2] > p.v[2];
    }

    PointInt operator-(const PointInt& p) {
        return PointInt(v[0] - p.x(), v[1] - p.y(), v[2] - p.z());
    }

    PointInt operator+(const PointInt& p) {
        return PointInt(v[0] + p.x(), v[1] + p.y(), v[2] + p.z());
    }

    float length() {
        return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    // 向量叉积
    PointInt cross(const PointInt& v) const {
        return PointInt(y() * v.z() - z() * v.y(), z() * v.x() - x() * v.z(), x() * v.y() - y() * v.x());
    }

    // 向量点积
    double dot(const PointInt& v) const {
        return x() * v.x() + y() * v.y() + z() * v.z();
    }

    void normalize() {
        float len = length();
        if (len == 0) {
            throw std::runtime_error("Cannot normalize a zero vector");
        }
        v[0] /= len;
        v[1] /= len;
        v[2] /= len;
    }

  public:
    int v[3];
};

// 定义向量类
class Vector3 : public Point {
  public:
    Vector3() {
        v[0] = 0;
        v[1] = 0;
        v[2] = 0;
    }

    Vector3(double x, double y, double z) {
        v[0] = x;
        v[1] = y;
        v[2] = z;
    }

    Vector3(Point p) {
        v[0] = p.x();
        v[1] = p.y();
        v[2] = p.z();
    }

    // 向量减法
    Vector3 operator-(const Point& p) const {
        return Vector3(v[0] - p.x(), v[1] - p.y(), v[2] - p.z());
    }

    Vector3 operator+(const Point& p) const {
        return Vector3(v[0] + p.x(), v[1] + p.y(), v[2] + p.z());
    }

    // 向量叉积
    Vector3 cross(const Vector3& v) const {
        return Vector3(y() * v.z() - z() * v.y(), z() * v.x() - x() * v.z(), x() * v.y() - y() * v.x());
    }

    // 向量点积
    double dot(const Vector3& v) const {
        return x() * v.x() + y() * v.y() + z() * v.z();
    }

    Vector3 operator%(const Vector3& v) const {
        return Vector3(y() * v.z() - z() * v.y(), z() * v.x() - x() * v.z(), x() * v.y() - y() * v.x());
    }

    double operator|(const Vector3& v) const {
        return x() * v.x() + y() * v.y() + z() * v.z();
    }

    Vector3 operator*(double scalar) {
        return Vector3(v[0] * scalar, v[1] * scalar, v[2] * scalar);
    }

    Vector3 normalize() {
        float len = length();
        // if (len == 0) {
        //     throw std::runtime_error("Cannot normalize a zero vector");
        // }
        return Vector3(v[0] / len, v[1] / len, v[2] / len);
    }

    float& operator[](int index) {
        if (index >= 0 && index < 3) {
            return v[index];
        } else {
            throw std::out_of_range("Index out of range");
        }
    }
};

class Vector3i : public PointInt {
  public:
    Vector3i(int x, int y, int z) {
        v[0] = x;
        v[1] = y;
        v[2] = z;
    }

    Vector3i(PointInt p) {
        v[0] = p.x();
        v[1] = p.y();
        v[2] = p.z();
    }

    // 向量减法
    Vector3i operator-(const PointInt& p) const {
        return Vector3i(v[0] - p.x(), v[1] - p.y(), v[2] - p.z());
    }

    Vector3i operator+(const PointInt& p) const {
        return Vector3i(v[0] + p.x(), v[1] + p.y(), v[2] + p.z());
    }

    // 向量叉积
    Vector3i cross(const Vector3i& v) const {
        return Vector3i(y() * v.z() - z() * v.y(), z() * v.x() - x() * v.z(), x() * v.y() - y() * v.x());
    }

    // 向量点积
    double dot(const Vector3i& v) const {
        return x() * v.x() + y() * v.y() + z() * v.z();
    }

    int& operator[](int index) {
        if (index >= 0 && index < 3) {
            return v[index];
        } else {
            throw std::out_of_range("Index out of range");
        }
    }
};

}  // namespace MCGAL
#endif
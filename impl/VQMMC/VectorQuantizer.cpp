#include "VectorQuantizer.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <vector>

namespace {

Vec3d zeroPoint() {
    return Vec3d(0.0, 0.0, 0.0);
}

Vec3d addPoint(const Vec3d& a, const Vec3d& b) {
    return Vec3d(a.x() + b.x(), a.y() + b.y(), a.z() + b.z());
}

Vec3d subPoint(const Vec3d& a, const Vec3d& b) {
    return Vec3d(a.x() - b.x(), a.y() - b.y(), a.z() - b.z());
}

Vec3d scalePoint(const Vec3d& p, double s) {
    return Vec3d(p.x() * s, p.y() * s, p.z() * s);
}

Vec3d centroidOfSamples(const std::vector<Vec3d>& data,
                        const std::vector<int>& samples) {
    if (samples.empty()) return zeroPoint();
    double sx = 0.0, sy = 0.0, sz = 0.0;
    for (int idx : samples) {
        sx += data[(size_t)idx].x();
        sy += data[(size_t)idx].y();
        sz += data[(size_t)idx].z();
    }
    const double inv = 1.0 / (double)samples.size();
    return Vec3d(sx * inv, sy * inv, sz * inv);
}

Vec3d centroidOfRange(const std::vector<Vec3d>& data, int begin, int end) {
    if (begin >= end) return zeroPoint();
    double sx = 0.0, sy = 0.0, sz = 0.0;
    for (int i = begin; i < end; ++i) {
        sx += data[(size_t)i].x();
        sy += data[(size_t)i].y();
        sz += data[(size_t)i].z();
    }
    const double inv = 1.0 / (double)(end - begin);
    return Vec3d(sx * inv, sy * inv, sz * inv);
}

double squaredDistance(const Vec3d& a, const Vec3d& b) {
    const double dx = (double)a.x() - (double)b.x();
    const double dy = (double)a.y() - (double)b.y();
    const double dz = (double)a.z() - (double)b.z();
    return dx * dx + dy * dy + dz * dz;
}

double dotPoint(const Vec3d& a, const Vec3d& b) {
    return (double)a.x() * (double)b.x() +
           (double)a.y() * (double)b.y() +
           (double)a.z() * (double)b.z();
}

Vec3d normalizeOrFallback(const Vec3d& v) {
    const double n = std::sqrt(dotPoint(v, v));
    if (n <= 1e-20) return Vec3d(1.0, 0.0, 0.0);
    return scalePoint(v, 1.0 / n);
}

int previousPowerOfTwo(int n) {
    if (n <= 1) return 1;
    int p = 1;
    while (p <= n / 2) p <<= 1;
    return p;
}

void sortByProjection(const std::vector<Vec3d>& data,
                      std::vector<int>& samples,
                      const Vec3d& centroid,
                      const Vec3d& axis) {
    std::stable_sort(samples.begin(), samples.end(), [&](int a, int b) {
        const double pa = dotPoint(subPoint(data[(size_t)a], centroid), axis);
        const double pb = dotPoint(subPoint(data[(size_t)b], centroid), axis);
        if (pa != pb) return pa < pb;
        return a < b;
    });
}

}  // namespace

VectorQuantizer::VectorQuantizer(int codebookSize, int dimension) {
    codebook_.codebookSize = std::max(1, codebookSize);
    codebook_.dimension = dimension;
    codebook_.codewords.resize((size_t)codebook_.codebookSize);
}

VectorQuantizer::~VectorQuantizer() {}

void VectorQuantizer::trainCodebook(const std::vector<Vec3d>& trainingData,
                                    int maxIterations,
                                    double convergenceThreshold) {
    if (trainingData.empty()) {
        std::cerr << "Error: Empty VQMMC training data" << std::endl;
        codebook_.codewords.clear();
        codebook_.progressiveTree.clear();
        codebook_.codebookSize = 0;
        return;
    }

    const int cappedSize =
        std::min(codebook_.codebookSize, (int)trainingData.size());
    const int targetSize = previousPowerOfTwo(cappedSize);

    initializeWithBinaryLBG(trainingData, targetSize);
    codebook_.codebookSize = (int)codebook_.codewords.size();

    double prevDistortion = computeDistortion(trainingData);
    const int iterations = std::max(1, maxIterations);
    for (int iter = 0; iter < iterations; ++iter) {
        const bool changed = lloydIteration(trainingData);
        const double distortion = computeDistortion(trainingData);
        const double improvement =
            prevDistortion > 0.0 ? (prevDistortion - distortion) / prevDistortion : 0.0;
        prevDistortion = distortion;
        if (!changed || (iter > 8 && improvement >= 0.0 &&
                         improvement < convergenceThreshold)) {
            break;
        }
    }

    buildProgressiveTree();
}

void VectorQuantizer::initializeWithBinaryLBG(const std::vector<Vec3d>& data,
                                              int targetSize) {
    codebook_.codewords.clear();
    codebook_.progressiveTree.clear();

    std::vector<int> allSamples(data.size());
    std::iota(allSamples.begin(), allSamples.end(), 0);
    buildInitialTree(data, allSamples, std::max(1, targetSize), 0);
    codebook_.codebookSize = (int)codebook_.codewords.size();
}

int VectorQuantizer::buildInitialTree(const std::vector<Vec3d>& data,
                                      const std::vector<int>& samples,
                                      int leafBudget,
                                      int depth) {
    const int nodeId = (int)codebook_.progressiveTree.size();
    codebook_.progressiveTree.emplace_back();
    PTTreeNode& node = codebook_.progressiveTree.back();
    node.nodeId = nodeId;
    node.depth = depth;
    node.codeword = centroidOfSamples(data, samples);

    if (leafBudget <= 1 || samples.size() <= 1) {
        node.leftChild = -1;
        node.rightChild = -1;
        codebook_.codewords.push_back(node.codeword);
        return nodeId;
    }

    std::vector<int> left;
    std::vector<int> right;
    splitSamples(data, samples, left, right);
    if (left.empty() || right.empty()) {
        node.leftChild = -1;
        node.rightChild = -1;
        codebook_.codewords.push_back(node.codeword);
        return nodeId;
    }

    int leftBudget = std::max(1, (int)std::llround(
        (double)leafBudget * (double)left.size() / (double)samples.size()));
    leftBudget = std::min(leftBudget, (int)left.size());
    int rightBudget = leafBudget - leftBudget;
    if (rightBudget < 1) {
        rightBudget = 1;
        leftBudget = leafBudget - rightBudget;
    }
    if (rightBudget > (int)right.size()) {
        const int overflow = rightBudget - (int)right.size();
        rightBudget -= overflow;
        leftBudget += overflow;
    }
    leftBudget = std::max(1, std::min(leftBudget, (int)left.size()));
    rightBudget = std::max(1, std::min(rightBudget, (int)right.size()));

    const int leftChild = buildInitialTree(data, left, leftBudget, depth + 1);
    const int rightChild = buildInitialTree(data, right, rightBudget, depth + 1);
    codebook_.progressiveTree[(size_t)nodeId].leftChild = leftChild;
    codebook_.progressiveTree[(size_t)nodeId].rightChild = rightChild;
    return nodeId;
}

void VectorQuantizer::splitSamples(const std::vector<Vec3d>& data,
                                   const std::vector<int>& samples,
                                   std::vector<int>& left,
                                   std::vector<int>& right) const {
    left.clear();
    right.clear();
    if (samples.size() <= 1) {
        left = samples;
        return;
    }

    const Vec3d centroid = centroidOfSamples(data, samples);
    double cov[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    for (int idx : samples) {
        const Vec3d d = subPoint(data[(size_t)idx], centroid);
        const double v[3] = { d.x(), d.y(), d.z() };
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) cov[r][c] += v[r] * v[c];
        }
    }

    Vec3d axis(1.0, 0.0, 0.0);
    for (int iter = 0; iter < 24; ++iter) {
        const double a[3] = { axis.x(), axis.y(), axis.z() };
        Vec3d next(
            cov[0][0] * a[0] + cov[0][1] * a[1] + cov[0][2] * a[2],
            cov[1][0] * a[0] + cov[1][1] * a[1] + cov[1][2] * a[2],
            cov[2][0] * a[0] + cov[2][1] * a[1] + cov[2][2] * a[2]);
        axis = normalizeOrFallback(next);
    }

    std::vector<int> ordered = samples;
    sortByProjection(data, ordered, centroid, axis);
    const size_t mid = ordered.size() / 2;
    left.assign(ordered.begin(), ordered.begin() + (long)mid);
    right.assign(ordered.begin() + (long)mid, ordered.end());
    if (left.empty() || right.empty()) return;

    Vec3d c0 = centroidOfSamples(data, left);
    Vec3d c1 = centroidOfSamples(data, right);
    for (int iter = 0; iter < 16; ++iter) {
        std::vector<int> nextLeft;
        std::vector<int> nextRight;
        nextLeft.reserve(samples.size());
        nextRight.reserve(samples.size());

        for (int idx : samples) {
            const double d0 = squaredDistance(data[(size_t)idx], c0);
            const double d1 = squaredDistance(data[(size_t)idx], c1);
            if (d0 <= d1) nextLeft.push_back(idx);
            else nextRight.push_back(idx);
        }

        if (nextLeft.empty() || nextRight.empty()) {
            left.assign(ordered.begin(), ordered.begin() + (long)mid);
            right.assign(ordered.begin() + (long)mid, ordered.end());
            return;
        }

        const bool stable = nextLeft == left && nextRight == right;
        left.swap(nextLeft);
        right.swap(nextRight);
        c0 = centroidOfSamples(data, left);
        c1 = centroidOfSamples(data, right);
        if (stable) break;
    }

    const double p0 = dotPoint(subPoint(c0, centroid), axis);
    const double p1 = dotPoint(subPoint(c1, centroid), axis);
    if (p0 > p1) std::swap(left, right);
}

bool VectorQuantizer::lloydIteration(const std::vector<Vec3d>& data) {
    const int K = (int)codebook_.codewords.size();
    if (K <= 0) return false;

    std::vector<Vec3d> sums((size_t)K, zeroPoint());
    std::vector<int> counts((size_t)K, 0);
    std::vector<int> assignment(data.size(), 0);

    for (size_t i = 0; i < data.size(); ++i) {
        const int nearest = findNearestCodeword(data[i]);
        assignment[i] = nearest;
        sums[(size_t)nearest] = addPoint(sums[(size_t)nearest], data[i]);
        counts[(size_t)nearest]++;
    }

    std::vector<double> sampleError(data.size(), 0.0);
    for (size_t i = 0; i < data.size(); ++i) {
        sampleError[i] = computeDistance(data[i],
                                         codebook_.codewords[(size_t)assignment[i]]);
    }

    bool changed = false;
    for (int k = 0; k < K; ++k) {
        Vec3d next = codebook_.codewords[(size_t)k];
        if (counts[(size_t)k] > 0) {
            next = scalePoint(sums[(size_t)k], 1.0 / (double)counts[(size_t)k]);
        } else {
            const auto farthest = std::max_element(sampleError.begin(), sampleError.end());
            const size_t idx = (size_t)std::distance(sampleError.begin(), farthest);
            next = data[idx];
            *farthest = -1.0;
        }
        if (computeDistance(next, codebook_.codewords[(size_t)k]) > 1e-24) {
            changed = true;
        }
        codebook_.codewords[(size_t)k] = next;
    }
    return changed;
}

double VectorQuantizer::computeDistortion(const std::vector<Vec3d>& data) const {
    if (data.empty() || codebook_.codewords.empty()) return 0.0;

    double totalDistortion = 0.0;
    for (const auto& vec : data) {
        int nearestIdx = findNearestCodeword(vec);
        totalDistortion += computeDistance(vec, codebook_.codewords[(size_t)nearestIdx]);
    }
    return totalDistortion / (double)data.size();
}

void VectorQuantizer::buildProgressiveTree() {
    codebook_.progressiveTree.clear();
    if (codebook_.codewords.empty()) return;
    buildTreeRange(0, (int)codebook_.codewords.size(), 0);
}

void VectorQuantizer::buildTreeRange(int begin, int end, int depth) {
    const int nodeId = (int)codebook_.progressiveTree.size();
    codebook_.progressiveTree.emplace_back();
    PTTreeNode& node = codebook_.progressiveTree.back();
    node.nodeId = nodeId;
    node.depth = depth;
    node.codeword = centroidOfRange(codebook_.codewords, begin, end);

    if (end - begin <= 1) {
        node.leftChild = -1;
        node.rightChild = -1;
        if (begin < end) node.codeword = codebook_.codewords[(size_t)begin];
        return;
    }

    const int mid = begin + (end - begin) / 2;
    const int leftChild = (int)codebook_.progressiveTree.size();
    buildTreeRange(begin, mid, depth + 1);
    const int rightChild = (int)codebook_.progressiveTree.size();
    buildTreeRange(mid, end, depth + 1);
    codebook_.progressiveTree[(size_t)nodeId].leftChild = leftChild;
    codebook_.progressiveTree[(size_t)nodeId].rightChild = rightChild;
}

double VectorQuantizer::computeDistance(const Vec3d& v1, const Vec3d& v2) const {
    return squaredDistance(v1, v2);
}

int VectorQuantizer::findNearestCodeword(const Vec3d& vector) const {
    int nearestIdx = 0;
    double minDist = std::numeric_limits<double>::max();

    for (int i = 0; i < (int)codebook_.codewords.size(); ++i) {
        double dist = computeDistance(vector, codebook_.codewords[(size_t)i]);
        if (dist < minDist) {
            minDist = dist;
            nearestIdx = i;
        }
    }
    return nearestIdx;
}

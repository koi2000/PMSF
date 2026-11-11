/*
 * @Author: koi
 * @Date: 2025-09-14 23:06:44
 * @Description:
 */
#ifndef SEGMENTATION_OPTION_H
#define SEGMENTATION_OPTION_H

#include "Halfedge.h"
#include <string>
#include <vector>

/**
  启用seg后，seg的配置项也有很多
  1. 是否指定数量，指定数量就pre seg，不指定就由系统内定
  2. 指定数量后，是否指定种子点，指定种子点就按照种子点进行seg，不指定就由系统内定
  3. 是否压缩边界，不压缩边界的话
  4.
 */
// 选择分区方案
enum class SegmentCompressionScheme {
    PredefinedSegments = 0,
    LowestLODAsSegments = 1
};

class SegmentationOption {
  public:
    SegmentationOption() : scheme_(SegmentCompressionScheme::PredefinedSegments), numOfSegments_(0), compressBoundary_(false) {}

    void setScheme(SegmentCompressionScheme s) {
        scheme_ = s;
    }
    SegmentCompressionScheme getScheme() const {
        return scheme_;
    }

    void setNumSegments(int n) {
        numOfSegments_ = n;
    }
    int getNumSegments() const {
        return numOfSegments_;
    }

    void setSeeds(const std::vector<MCGAL::Halfedge*>& list) {
        seeds_ = list;
    }
    const std::vector<MCGAL::Halfedge*>& getSeeds() const {
        return seeds_;
    }

    void setCompressBoundary(bool enable) {
        compressBoundary_ = enable;
    }
    bool isCompressBoundaryEnabled() const {
        return compressBoundary_;
    }

    class Builder {
      public:
        Builder() : scheme_(SegmentCompressionScheme::PredefinedSegments), numOfSegments_(0), compressBoundary_(false) {}

        Builder& setSegmentationType(SegmentCompressionScheme scheme) {
            scheme_ = scheme;
            return *this;
        }

        Builder& setNumSegments(int n) {
            numOfSegments_ = n;
            return *this;
        }

        Builder& setSeeds(const std::vector<MCGAL::Halfedge*>& seeds) {
            seeds_ = seeds;
            return *this;
        }

        Builder& setCompressBoundary(bool enable) {
            compressBoundary_ = enable;
            return *this;
        }

        SegmentationOption build() {
            SegmentationOption option;
            option.scheme_ = scheme_;
            option.numOfSegments_ = numOfSegments_;
            option.seeds_ = seeds_;
            option.compressBoundary_ = compressBoundary_;
            return option;
        }

      private:
        SegmentCompressionScheme scheme_;
        int numOfSegments_;
        std::vector<MCGAL::Halfedge*> seeds_;
        bool compressBoundary_;
    };

  private:
    SegmentCompressionScheme scheme_;
    int numOfSegments_;
    std::vector<MCGAL::Halfedge*> seeds_;
    bool compressBoundary_;
};

#endif
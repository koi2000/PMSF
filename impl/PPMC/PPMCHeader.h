/*
 * @Author: koi
 * @Date: 2025-09-01 10:04:28
 * @Description:
 */
#ifndef PPMC_HEADER_H
#define PPMC_HEADER_H

#include "BufferUtils.h"
#include "header/PMSFHeader.h"

class PPMCHeader : public PMSFHeader {
  public:
    bool b_useAdaptiveQuantization = false;
    bool b_useLiftingScheme = false;
    bool b_useCurvaturePrediction = false;
    bool b_useConnectivityPredictionFaces = false;
    bool b_useConnectivityPredictionEdges = false;
    bool b_useTriangleMeshConnectivityPredictionFaces = false;

  public:
    PPMCHeader() = default;

    // Builder模式`

    // getter
    int getRound() const {
        return round;
    }
    int getGroupNumber() const {
        return groupNumber;
    }
    bool isEnableCompress() const {
        return enableCompress;
    }
    bool isEnableQuantization() const {
        return enableQuantization;
    }
    bool isEnableSegmentation() const {
        return enableSegmentation;
    }
    bool isEnableAdjustment() const {
        return enableAdjustment;
    }

    // setter
    void setRound(int r) {
        round = r;
    }
    void setGroupNumber(int g) {
        groupNumber = g;
    }
    void setEnableCompress(bool v) {
        enableCompress = v;
    }
    void setEnableQuantization(bool v) {
        enableQuantization = v;
    }
    void setEnableSegmentation(bool v) {
        enableSegmentation = v;
    }
    void setEnableAdjustment(bool v) {
        enableAdjustment = v;
    }

    // 完整的序列化方法
    virtual void serialize(char* buffer, size_t& offset) const {
        writeInt(buffer, offset, round);
        unsigned bitOffset = 0;
        writeBool(buffer, bitOffset, offset, enableCompress);
        writeBool(buffer, bitOffset, offset, enableQuantization);
        writeBool(buffer, bitOffset, offset, enableSegmentation);
        writeBool(buffer, bitOffset, offset, enableAdjustment);
        offset += 1;
        bitOffset = 0;
        if (enableSegmentation) {
            writeInt(buffer, offset, groupNumber);
        }
    }

    // 反序列化方法（可选）
    virtual void deserialize(char* buffer, size_t& offset) {
        round = readInt(buffer, offset);
        unsigned bitOffset = 0;
        enableCompress = readBool(buffer, bitOffset, offset);
        enableQuantization = readBool(buffer, bitOffset, offset);
        enableSegmentation = readBool(buffer, bitOffset, offset);
        enableAdjustment = readBool(buffer, bitOffset, offset);
        bitOffset = 0;
        offset += 1;
        if (enableSegmentation) {
            groupNumber = readInt(buffer, offset);
        }
    }
};

#endif
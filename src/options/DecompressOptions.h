/*
 * @Author: koi
 * @Date: 2025-08-27 21:18:59
 * @Description: 存储用于解压缩所使用的各种 option
 */

#include "decompress/DeserializeOperator.h"
#include <array>
#include <functional>
#include <memory>
#pragma once

#include <string>

class DecompressOptions {
  public:
    DecompressOptions() = default;
    ~DecompressOptions() = default;

    class Builder {
      public:
        Builder& setDeserializeOperator(const std::shared_ptr<DeserializeOperator>& op) {
            deserializeOperator_ = op;
            return *this;
        }
        Builder& setPath(const std::string& p) {
            path_ = p;
            return *this;
        }
        Builder& setOutputPath(const std::string& out) {
            outputPath_ = out;
            return *this;
        }
        /// @brief 设置目标轮次：解压到第 N 轮（0 = 仅基网，<0 = 解压到文件中存储的全部轮次）
        Builder& setTargetRound(int r) {
            targetRound_ = r;
            return *this;
        }
        /// @brief 设置目标分区：仅解压指定分区；-1 表示解压所有分区
        Builder& setTargetGroup(int g) {
            targetGroup_ = g;
            return *this;
        }
        /// @brief 算法自定义网格池容量（origV, origF -> {V, H, F}）。
        /// 为空时走框架默认（PMSF_POOL_* env / 估计值）。
        Builder& setPoolSizer(std::function<std::array<int, 3>(int, int)> f) {
            poolSizer_ = std::move(f);
            return *this;
        }
        DecompressOptions build() {
            DecompressOptions options;
            options.deserializeOperator_ = deserializeOperator_;
            options.path = path_;
            options.outputPath = outputPath_;
            options.targetRound = targetRound_;
            options.targetGroup = targetGroup_;
            options.poolSizer = poolSizer_;
            return options;
        }

      private:
        std::shared_ptr<DeserializeOperator> deserializeOperator_;
        std::string path_;
        std::string outputPath_;
        int targetRound_ = -1;
        int targetGroup_ = -1;
        std::function<std::array<int, 3>(int, int)> poolSizer_;
    };

    // Getter
    std::shared_ptr<DeserializeOperator> getDeserializeOperator() const {
        return deserializeOperator_;
    }
    std::string getPath() const {
        return path;
    }
    std::string getOutputPath() const {
        return outputPath;
    }
    int getTargetRound() const {
        return targetRound;
    }
    int getTargetGroup() const {
        return targetGroup;
    }
    const std::function<std::array<int, 3>(int, int)>& getPoolSizer() const {
        return poolSizer;
    }
    void setPoolSizer(std::function<std::array<int, 3>(int, int)> f) {
        poolSizer = std::move(f);
    }

  private:
    std::shared_ptr<DeserializeOperator> deserializeOperator_;
    std::string path;
    std::string outputPath;
    int targetRound = -1;  // <0 表示解压全部轮次
    int targetGroup = -1;  // <0 表示解压全部分区
    std::function<std::array<int, 3>(int, int)> poolSizer;
};

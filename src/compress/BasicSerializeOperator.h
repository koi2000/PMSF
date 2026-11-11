/*
 * @Author: koi
 * @Date: 2025-09-01 11:13:52
 * @Description:
 */
/*
 * @Author: koi
 * @Date: 2025-08-29 17:49:34
 * @Description:
 */
#ifndef BASIC_SERIALIZEOPERATOR_H
#define BASIC_SERIALIZEOPERATOR_H

#include "SerializeOperator.h"
#include <string>

/**
 * @brief 序列化操作符
 * @details 这里的序列化类型有很多区别
 * 首先header就有区别，然后就是序列化每一轮的symbol
 */
class BasicSerializeOperator : public SerializeOperator {
  public:
    BasicSerializeOperator(std::shared_ptr<SymbolCollectOperator> collector,
                           bool enablePrediction = false,
                           bool enableQuantization = false,
                           bool enableCompress = false)
        : SerializeOperator(collector, enablePrediction, enableQuantization, enableCompress) {}


    void init();

    /// @brief header 是唯一权威：它说走 range-coder 残差路径，序列化器就走。
    ///
    /// 不能只看构造时的 options.prediction —— 算法可以在 beforeCompressLoop 里
    /// 按自己的语义改写 header（例如把这一位关掉、改用连通性预测位）。解码端
    /// 读的是 header，序列化器读的是 options，两边一旦分叉，产物就不可解。
    void onHeaderFinalized(const PMSFHeader& header) override {
        enablePrediction_ = header.isEnablePrediction();
    }

    void serializeInt(int val);

    void serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh);

    void serializeBaseMeshWithSeed(std::shared_ptr<MCGAL::Mesh> mesh, std::vector<MCGAL::Halfedge*> seed);

    void serialize(std::string path);
};
#endif
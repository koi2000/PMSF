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
#ifndef PPMC_SERIALIZEOPERATOR_H
#define PPMC_SERIALIZEOPERATOR_H

#include "compress/SerializeOperator.h"
#include <string>

/**
 * @brief 序列化操作符
 * @details 这里的序列化类型有很多区别
 * 首先header就有区别，然后就是序列化每一轮的symbol
 */
class PPMCSerializeOperator : public SerializeOperator {
  public:
  PPMCSerializeOperator(std::shared_ptr<SymbolCollectOperator> collector,
                           bool enablePrediction = false,
                           bool enableQuantization = false,
                           bool enableCompress = false)
        : SerializeOperator(collector, enablePrediction, enableQuantization, enableCompress) {}


    void init();

    void serializeInt(int val);

    void serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh);

    void serializeBaseMeshWithSeed(std::shared_ptr<MCGAL::Mesh> mesh, std::vector<MCGAL::Halfedge*> seed);

    void serialize(std::string path);

    int i_curQuantizationId = 0;
};
#endif
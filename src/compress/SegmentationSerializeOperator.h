/*
 * @Author: koi
 * @Date: 2025-09-18 16:35:01
 * @Description:
 */
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
#ifndef SEGMENTATION_SERIALIZEOPERATOR_H
#define SEGMENTATION_SERIALIZEOPERATOR_H

#include "SerializeOperator.h"
#include <string>

/**
 * @brief 序列化操作符
 * @details 这里的序列化类型有很多区别
 * 首先header就有区别，然后就是序列化每一轮的symbol
 */
class SegmentationSerializeOperator : public SerializeOperator {
  public:
    SegmentationSerializeOperator(std::shared_ptr<SymbolCollectOperator> collector,
                                  bool enablePrediction = false,
                                  bool enableQuantization = false,
                                  bool enableCompress = false)
        : SerializeOperator(collector, enablePrediction, enableQuantization, enableCompress) {}

    void init();

    void serializeInt(int val);

    void serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh);

    void calculateGroupSize(std::shared_ptr<MCGAL::Mesh> mesh,
                            std::vector<MCGAL::Vertex*>& vs,
                            MCGAL::Halfedge* seed,
                            std::set<MCGAL::Vertex*> vset,
                            int& idx);

    void serializeBaseMeshWithSeed(std::shared_ptr<MCGAL::Mesh> mesh, std::vector<MCGAL::Halfedge*> seed);

    void serialize(std::string path);
};
#endif
/*
 * @Author: pomar-refactor
 * @Description: Pomar 存储模块重构版 —— 符号读取器实现
 */

#include "POMAR/decompress/PomarColumnarSymbolReadOperator.h"

#include <cassert>

void PomarColumnarSymbolReadOperator::resizeClusters(unsigned nbClusters) {
    nbSplitSymbols_.assign(nbClusters, {});
    nbSkipsHehSymbols_.assign(nbClusters, {});
    vertexSplitSymbols_.assign(nbClusters, {});
    geometrySymbols_.assign(nbClusters, {});
    geometryNormalSymbols_.assign(nbClusters, {});
}

unsigned PomarColumnarSymbolReadOperator::decodeNbBits(rangecoder* rc, unsigned i_nbBits) {
    unsigned tmp = decode_culshift(rc, i_nbBits);
    decode_update(rc, 1, tmp, (freq)1 << i_nbBits);
    return tmp;
}

void PomarColumnarSymbolReadOperator::decodeAndInitModels(unsigned i_clusterId) {
    unsigned i_nbBits = i_clusterId > 0 ? i_nbBitsNbSplits_ : 3;
    i_maxNbSplits_ = decodeNbBits(&rangeCoder_, i_nbBits);
    initqsmodel(&nbSplitSymbolsModel_, i_maxNbSplits_ + 1, 10, 1 << 9, NULL, 0);

    i_maxNbSkipsHehSymbol_ = 0;
    i_maxVertexSplits_ = 0;
    i_minResidual_ = 0;
    i_maxResidual_ = 0;
    i_minResidualNormal_ = 0;
    i_maxResidualNormal_ = 0;

    if (i_maxNbSplits_ > 0) {
        i_nbBits = i_clusterId > 0 ? i_nbBitsNbSkips_ : 4;
        i_maxNbSkipsHehSymbol_ = decodeNbBits(&rangeCoder_, i_nbBits);
        initqsmodel(&nbSkipsHehSymbolsModel_, i_maxNbSkipsHehSymbol_ + 1, 10, 1 << 9, NULL, 0);

        i_nbBits = i_clusterId > 0 ? i_nbBitsVertexSplits_ : 5;
        i_maxVertexSplits_ = decodeNbBits(&rangeCoder_, i_nbBits);
        initqsmodel(&vertexSplitSymbolsModel_, i_maxVertexSplits_ + 1, 10, 1 << 9, NULL, 0);

        if (i_clusterId > 0) {
            i_minResidual_ = decodeNbBits(&rangeCoder_, i_nbBitsMinResidual_) + i_minMinResidual_;
            unsigned residualRange = decodeNbBits(&rangeCoder_, i_nbBitsResidualRange_);
            i_maxResidual_ = i_minResidual_ + residualRange - 1;
        } else {
            uint16_t i16_min = decode_short(&rangeCoder_);
            i_minResidual_ = *(int16_t*)&i16_min;
            unsigned residualRange = decode_short(&rangeCoder_);
            i_maxResidual_ = i_minResidual_ + residualRange - 1;
        }
        initqsmodel(&geometrySymbolsModel_, (i_maxResidual_ - i_minResidual_ + 1), 18, 1 << 17, NULL, 0);

        if (i_clusterId > 0) {
            i_minResidualNormal_ = decodeNbBits(&rangeCoder_, i_nbBitsMinResidualNormal_) + i_minMinResidualNormal_;
            unsigned residualNormalRange = decodeNbBits(&rangeCoder_, i_nbBitsResidualNormalRange_);
            i_maxResidualNormal_ = i_minResidualNormal_ + residualNormalRange - 1;
        } else {
            uint16_t i16_min = decode_short(&rangeCoder_);
            i_minResidualNormal_ = *(int16_t*)&i16_min;
            unsigned residualNormalRange = decode_short(&rangeCoder_);
            i_maxResidualNormal_ = i_minResidualNormal_ + residualNormalRange - 1;
        }
        initqsmodel(&geometryNormalSymbolsModel_, (i_maxResidualNormal_ - i_minResidualNormal_ + 1), 18, 1 << 17, NULL, 0);
    }
}

void PomarColumnarSymbolReadOperator::deleteModels() {
    deleteqsmodel(&nbSplitSymbolsModel_);
    if (i_maxNbSplits_ > 0) {
        deleteqsmodel(&nbSkipsHehSymbolsModel_);
        deleteqsmodel(&vertexSplitSymbolsModel_);
        deleteqsmodel(&geometrySymbolsModel_);
        deleteqsmodel(&geometryNormalSymbolsModel_);
    }
}

void PomarColumnarSymbolReadOperator::decodeCluster(unsigned i_clusterId, unsigned i_maxLOD, unsigned i_nbBaseVertices) {
    start_decoding(&rangeCoder_);
    decodeAndInitModels(i_clusterId);

    unsigned i_nbVertices = i_clusterId == 0 ? i_nbBaseVertices : 1;
    for (unsigned i_LOD = 0; i_LOD < i_maxLOD; ++i_LOD) {
        unsigned i_nbAddedVertices = 0;

        for (unsigned i_vertex = 0; i_vertex < i_nbVertices; ++i_vertex) {
            int syfreq, ltfreq;
            ltfreq = decode_culshift(&rangeCoder_, 10);
            unsigned i_nbSplits = qsgetsym(&nbSplitSymbolsModel_, ltfreq);
            qsgetfreq(&nbSplitSymbolsModel_, i_nbSplits, &syfreq, &ltfreq);
            decode_update(&rangeCoder_, syfreq, ltfreq, 1 << 10);
            qsupdate(&nbSplitSymbolsModel_, i_nbSplits);
            nbSplitSymbols_[i_clusterId].push_back(i_nbSplits);

            for (unsigned i_split = 0; i_split < i_nbSplits; ++i_split) {
                ltfreq = decode_culshift(&rangeCoder_, 10);
                unsigned i_nbSkipsHehSymbol = qsgetsym(&nbSkipsHehSymbolsModel_, ltfreq);
                qsgetfreq(&nbSkipsHehSymbolsModel_, i_nbSkipsHehSymbol, &syfreq, &ltfreq);
                decode_update(&rangeCoder_, syfreq, ltfreq, 1 << 10);
                qsupdate(&nbSkipsHehSymbolsModel_, i_nbSkipsHehSymbol);
                nbSkipsHehSymbols_[i_clusterId].push_back(i_nbSkipsHehSymbol);

                ltfreq = decode_culshift(&rangeCoder_, 10);
                unsigned i_vertexSplitSymbol = qsgetsym(&vertexSplitSymbolsModel_, ltfreq);
                qsgetfreq(&vertexSplitSymbolsModel_, i_vertexSplitSymbol, &syfreq, &ltfreq);
                decode_update(&rangeCoder_, syfreq, ltfreq, 1 << 10);
                qsupdate(&vertexSplitSymbolsModel_, i_vertexSplitSymbol);
                vertexSplitSymbols_[i_clusterId].push_back(i_vertexSplitSymbol);

                ltfreq = decode_culshift(&rangeCoder_, 10);
                i_vertexSplitSymbol = qsgetsym(&vertexSplitSymbolsModel_, ltfreq);
                qsgetfreq(&vertexSplitSymbolsModel_, i_vertexSplitSymbol, &syfreq, &ltfreq);
                decode_update(&rangeCoder_, syfreq, ltfreq, 1 << 10);
                qsupdate(&vertexSplitSymbolsModel_, i_vertexSplitSymbol);
                vertexSplitSymbols_[i_clusterId].push_back(i_vertexSplitSymbol);

                ltfreq = decode_culshift(&rangeCoder_, 18);
                unsigned i_geomTangentialSymbol = qsgetsym(&geometrySymbolsModel_, ltfreq);
                qsgetfreq(&geometrySymbolsModel_, i_geomTangentialSymbol, &syfreq, &ltfreq);
                decode_update(&rangeCoder_, syfreq, ltfreq, 1 << 18);
                qsupdate(&geometrySymbolsModel_, i_geomTangentialSymbol);
                geometrySymbols_[i_clusterId].push_back(i_geomTangentialSymbol + i_minResidual_);

                ltfreq = decode_culshift(&rangeCoder_, 18);
                i_geomTangentialSymbol = qsgetsym(&geometrySymbolsModel_, ltfreq);
                qsgetfreq(&geometrySymbolsModel_, i_geomTangentialSymbol, &syfreq, &ltfreq);
                decode_update(&rangeCoder_, syfreq, ltfreq, 1 << 18);
                qsupdate(&geometrySymbolsModel_, i_geomTangentialSymbol);
                geometrySymbols_[i_clusterId].push_back(i_geomTangentialSymbol + i_minResidual_);

                ltfreq = decode_culshift(&rangeCoder_, 18);
                unsigned i_geomNormalSymbol = qsgetsym(&geometryNormalSymbolsModel_, ltfreq);
                qsgetfreq(&geometryNormalSymbolsModel_, i_geomNormalSymbol, &syfreq, &ltfreq);
                decode_update(&rangeCoder_, syfreq, ltfreq, 1 << 18);
                qsupdate(&geometryNormalSymbolsModel_, i_geomNormalSymbol);
                geometryNormalSymbols_[i_clusterId].push_back(i_geomNormalSymbol + i_minResidualNormal_);
            }
            i_nbAddedVertices += i_nbSplits;
        }
        i_nbVertices += i_nbAddedVertices;
    }
    done_decoding(&rangeCoder_);
    deleteModels();
}

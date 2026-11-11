#ifndef VECTOR_QUANTIZER_H
#define VECTOR_QUANTIZER_H

#include "Types.h"

/**
 * VQMMC full-rate vector quantizer.
 *
 * The paper uses a trained full-rate codebook, then puts a progressive
 * transmission tree over that codebook.  This implementation keeps the same
 * shape in the current framework: deterministic binary LBG splitting produces
 * the leaf codewords, Lloyd refinement improves the full-rate codebook, and
 * buildProgressiveTree() stores the corresponding root-to-leaf hierarchy.
 */
class VectorQuantizer {
  public:
    VectorQuantizer(int codebookSize = 256, int dimension = 3);
    ~VectorQuantizer();

    void trainCodebook(const std::vector<Vec3d>& trainingData,
                       int maxIterations = 1000,
                       double convergenceThreshold = 1e-6);
    void buildProgressiveTree();
    const VQCodebook& getCodebook() const { return codebook_; }

  private:
    VQCodebook codebook_;

    double computeDistortion(const std::vector<Vec3d>& data) const;
    bool lloydIteration(const std::vector<Vec3d>& data);
    void initializeWithBinaryLBG(const std::vector<Vec3d>& data, int targetSize);
    int buildInitialTree(const std::vector<Vec3d>& data,
                         const std::vector<int>& samples,
                         int leafBudget,
                         int depth);
    void buildTreeRange(int begin, int end, int depth);
    void splitSamples(const std::vector<Vec3d>& data,
                      const std::vector<int>& samples,
                      std::vector<int>& left,
                      std::vector<int>& right) const;

    double computeDistance(const Vec3d& v1, const Vec3d& v2) const;
    int findNearestCodeword(const Vec3d& vector) const;
};

#endif  // VECTOR_QUANTIZER_H

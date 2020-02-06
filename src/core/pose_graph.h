#pragma once

#include "core/common_include.h"
#include "core/g2o_types.h"

namespace core
{

class PoseGraph
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;

    g2o::OptimizationAlgorithmLevenberg* solver_;
    g2o::SparseOptimizer optimizer_;

    std::vector<EdgePose*> edges_;

public:

    PoseGraph();

    ~PoseGraph();

    void Clear();

    void AddEdge(unsigned long from, unsigned long to, const SE3& measurement, const Mat66& information);

    void SetInitial(unsigned long id, const SE3& initial_estimate);

    std::vector<SE3> GetPoses() const;

    unsigned long Size() const;

    void Save(const std::string& filename) const;

    void Load(const std::string& filename);

    bool Optimize(int num_iters = 10);
};

}



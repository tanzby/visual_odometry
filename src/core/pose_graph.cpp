#include "pose_graph.h"

namespace core
{
    PoseGraph::PoseGraph()
    {
        solver_ = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

        optimizer_.setAlgorithm(solver_);
        optimizer_.setVerbose(true);
    }

    PoseGraph::~PoseGraph() {}

    void PoseGraph::Clear()
    {
        optimizer_.clear();
        edges_.clear();
    }

    void PoseGraph::AddEdge(unsigned long from, unsigned long to, const SE3& measurement, const Mat66& information)
    {
        ///! add Vertex first in g2o
        auto vertices_map = optimizer_.vertices();
        if (!vertices_map.count(from))
        {
            LOG(WARNING) << "no found Vertex[id="<< from <<"]" << "in graph";
            return;
        }
        if (!vertices_map.count(to))
        {
            LOG(WARNING) << "no found Vertex[id="<< to <<"]" << "in graph";
            return;
        }

        EdgePose* e = new EdgePose();

        e->setId(edges_.size());
        e->setVertex(0, optimizer_.vertices()[from]);
        e->setVertex(1, optimizer_.vertices()[to]);

        e->setMeasurement(measurement);
        e->setInformation(information);

        optimizer_.addEdge(e);
        edges_.emplace_back(e);
    }

    void PoseGraph::SetInitial(unsigned long id, const SE3& initial_estimate)
    {
        auto vertices_map = optimizer_.vertices();

        if (vertices_map.count(id))
        {
            auto v = static_cast<VertexPose*>(vertices_map[id]);
            v->setEstimate(initial_estimate);
        }
        else
        {
            VertexPose *v = new core::VertexPose();

            v->setId(id);
            v->setEstimate(initial_estimate);
            optimizer_.addVertex(v);

            if (id == 0)
            {
                v->setFixed(true);
            }
        }

    }
    std::vector<SE3> PoseGraph::GetPoses() const
    {
        std::vector<SE3> poses(optimizer_.vertices().size());

        for (const auto& pair : optimizer_.vertices())
        {
            if (pair.first>= poses.size()) continue;

            poses[pair.first] = static_cast<VertexPose*>(pair.second)->estimate();
        }

        return poses;
    }

    unsigned long PoseGraph::Size() const
    {
        return optimizer_.vertices().size();
    }

    void PoseGraph::Save(const std::string& filename) const
    {
        LOG(INFO) << "save PoseGraph to '" << filename <<"'";
        try
        {
            std::ofstream f_out(filename);
            if (!f_out)
            {
                LOG(INFO) << "cannot open '" << filename <<"' to save PoseGraph";
            }
            for (const auto& item: optimizer_.vertices())
            {
                f_out << "VERTEX_SE3:QUAT ";
                static_cast<VertexPose*>(item.second)->write(f_out);
            }
            for (const auto& item: optimizer_.edges())
            {
                f_out << "EDGE_SE3:QUAT ";
                static_cast<EdgePose*>(item)->write(f_out);
            }
            f_out.close();
        }
        catch (std::exception& e)
        {
            LOG(ERROR) << "error occurred when save PoseGraph to '" << filename <<"': "<< e.what();
        }
    }

    void PoseGraph::Load(const std::string& filename)
    {
        LOG(INFO) << "load PoseGraph from '" << filename <<"'";
        if(!optimizer_.load(filename.c_str()))
        {
            LOG(ERROR) << "cannot load PoseGraph from '" << filename <<"'";
        }
    }

    bool PoseGraph::Optimize(int num_iters)
    {
        optimizer_.initializeOptimization();
        optimizer_.optimize(num_iters);
        return true;
    }
}
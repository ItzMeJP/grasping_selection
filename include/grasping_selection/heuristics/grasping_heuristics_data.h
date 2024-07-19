//
// Created by joaopedro on 1/2/23.
//

#ifndef GRASPING_SELECTION_GRASPING_HEURISTICS_DATA_H
#define GRASPING_SELECTION_GRASPING_HEURISTICS_DATA_H

#include <string>
#include <memory>



namespace grasping_selection {
    class GraspingHeuristicsArgumentBase {
    public:
        typedef std::shared_ptr<GraspingHeuristicsArgumentBase> Ptr;

        GraspingHeuristicsArgumentBase() {};
        ~GraspingHeuristicsArgumentBase() {};

        double weight;

    };

    class EuclideanDistanceArgument : public GraspingHeuristicsArgumentBase {
    public:
        double distance_threshold;
    };

    class DepthDistanceArgument : public GraspingHeuristicsArgumentBase {
    public:
        double distance_threshold;
    };

    class JointGraspingArgument : public GraspingHeuristicsArgumentBase {
    public:
        std::string chain_start, chain_end, urdf_param;
        double timeout;
        int insist;
    };
    /// ########################################################################

    class GraspingHeuristicsDataBase {
    public:

        GraspingHeuristicsDataBase() {};
        ~GraspingHeuristicsDataBase() {};

    };

    class CandidateCollisionData : public GraspingHeuristicsDataBase {
    public:

    };

    class GeneralCollisionData : public GraspingHeuristicsDataBase {
    public:

    };

    class JointData : public GraspingHeuristicsDataBase {
    public:

    };

    class WorkspaceData : public GraspingHeuristicsDataBase {
    public:

    };
}

#endif //GRASPING_SELECTION_GRASPING_HEURISTICS_DATA_H

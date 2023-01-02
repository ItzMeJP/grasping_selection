//
// Created by joaopedro on 1/2/23.
//

#ifndef GRASPING_SELECTION_GRASPING_HEURISTICS_DATA_H
#define GRASPING_SELECTION_GRASPING_HEURISTICS_DATA_H

#include <string>
#include <memory>



namespace grasping_selection {
    class GraspingHeuristicsParameterBase {
    public:
        GraspingHeuristicsParameterBase() {};

        ~GraspingHeuristicsParameterBase() {};
        double weight;
        typedef std::shared_ptr<GraspingHeuristicsParameterBase> Ptr;
    };




    class EuclideanDistanceParameter : public GraspingHeuristicsParameterBase {
        double distance_threshold;
    };

    class DepthDistanceParameter : public GraspingHeuristicsParameterBase {
        double distance_threshold;
    };

    class JointGraspingParameter : public GraspingHeuristicsParameterBase {
        std::string chain_start, chain_end, urdf_param;
        double timeout;
        int insist;
    };
}

#endif //GRASPING_SELECTION_GRASPING_HEURISTICS_DATA_H

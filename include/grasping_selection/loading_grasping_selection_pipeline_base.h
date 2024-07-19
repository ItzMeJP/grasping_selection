//
// Created by joaopedro on 1/6/23.
//

#ifndef GRASPING_SELECTION_LOADING_GRASPING_SELECTION_PIPELINE_H
#define GRASPING_SELECTION_LOADING_GRASPING_SELECTION_PIPELINE_H

#include <memory>

namespace grasping_selection {
    class LoadingGraspingSelectionPipelineBase {
    public:

        LoadingGraspingSelectionPipelineBase(){};
        ~LoadingGraspingSelectionPipelineBase(){};

        typedef std::shared_ptr<LoadingGraspingSelectionPipelineBase> Ptr;

        virtual bool loadPipeline() = 0;

    };
}

#endif //GRASPING_SELECTION_LOADING_GRASPING_SELECTION_PIPELINE_H

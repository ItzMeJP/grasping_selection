//
// Created by joaopedro on 1/5/23.
//

#ifndef GRASPING_SELECTION_LOADING_GRASPING_DATASET_FROM_JSON_H
#define GRASPING_SELECTION_LOADING_GRASPING_DATASET_FROM_JSON_H

#include "loading_grasping_selection_pipeline_base.h"
#include "iostream"

namespace grasping_selection {
    class LoadingGraspingSelectionPipelineFromJSON : public LoadingGraspingSelectionPipelineBase {
    public:
        LoadingGraspingSelectionPipelineFromJSON();
        ~LoadingGraspingSelectionPipelineFromJSON();

        bool loadPipeline();

    };
}

#endif //GRASPING_SELECTION_LOADING_GRASPING_DATASET_FROM_JSON_H

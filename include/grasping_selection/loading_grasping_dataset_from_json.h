//
// Created by joaopedro on 1/5/23.
//

#ifndef GRASPING_SELECTION_LOADING_GRASPING_DATASET_FROM_JSON_H
#define GRASPING_SELECTION_LOADING_GRASPING_DATASET_FROM_JSON_H

#include "loading_grasping_dataset_base.h"
#include "iostream"

namespace grasping_selection {
    class LoadingGraspingDatasetFromJSON : public LoadingGraspingDatasetBase {
    public:
        LoadingGraspingDatasetFromJSON();
        ~LoadingGraspingDatasetFromJSON();

        bool loadObjectCandidatesDataset();
        bool loadObjectCenterOfGravity();
        bool loadObjectCenterOfBoundingBox();

    };
}

#endif //GRASPING_SELECTION_LOADING_GRASPING_DATASET_FROM_JSON_H

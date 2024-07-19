//
// Created by joaopedro on 1/5/23.
//

#ifndef GRASPING_SELECTION_LOADING_GRASPING_DATASET_BASE_H
#define GRASPING_SELECTION_LOADING_GRASPING_DATASET_BASE_H

#include <memory>

namespace grasping_selection {
    class LoadingGraspingDatasetBase {
    public:

        LoadingGraspingDatasetBase(){};
        ~LoadingGraspingDatasetBase(){};

        typedef std::shared_ptr<LoadingGraspingDatasetBase> Ptr;

        virtual bool loadObjectCandidatesDataset() = 0;
        virtual bool loadObjectCenterOfGravity() = 0;
        virtual bool loadObjectCenterOfBoundingBox() = 0;

    };
}

#endif //GRASPING_SELECTION_LOADING_GRASPING_DATASET_BASE_H

//
// Created by joaopedro on 1/5/23.
//

#include "loading_grasping_dataset_from_json.h"

namespace grasping_selection {

    LoadingGraspingDatasetFromJSON::LoadingGraspingDatasetFromJSON () {   };
    LoadingGraspingDatasetFromJSON::~LoadingGraspingDatasetFromJSON () {   };

    bool LoadingGraspingDatasetFromJSON::loadObjectCandidatesDataset()  {

        std::cout << "Testando o JSON para dataset" << std::endl;
        return true;
    }

    bool LoadingGraspingDatasetFromJSON::loadObjectCenterOfBoundingBox() {

        std::cout << "Testando o JSON para COBB dataset " << std::endl;
        return true;
    }

    bool LoadingGraspingDatasetFromJSON::loadObjectCenterOfGravity() {

        std::cout << "Testando o JSON para COBB dataset " << std::endl;
        return true;
    }

}
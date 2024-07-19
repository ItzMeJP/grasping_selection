//
// Created by joaopedro on 1/6/23.
//

#ifndef GRASPING_SELECTION_CORE_SETUP_FROM_JSON_H
#define GRASPING_SELECTION_CORE_SETUP_FROM_JSON_H
#include "core_setup_base.h"
#include "iostream"

namespace grasping_selection {
    class CoreSetupFromJSON : public CoreSetupBase {
    public:
        CoreSetupFromJSON();
        ~CoreSetupFromJSON();

    private:
        bool loadCoreConfiguration();

    };
}
#endif //GRASPING_SELECTION_CORE_SETUP_FROM_JSON_H

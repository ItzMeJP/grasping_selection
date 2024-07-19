//
// Created by joaopedro on 1/6/23.
//

#ifndef GRASPING_SELECTION_CORE_SETUP_BASE_H
#define GRASPING_SELECTION_CORE_SETUP_BASE_H

#include <memory>
#include <string>
#include <sstream>

namespace grasping_selection {
    class CoreSetupBase {
    public:

        struct Data { //TODO: remove this hard code. Set the JSON import
            std::string
                    log_folder_path = "/home/joaopedro/",
                    grasp_candidates_namespace = "candidate",
                    cog_namespace = "center_of_gravity",
                    cog_namespace_tf_header_frame_id_override = "gripper",
                    cobb_namespace_ = "center_of_bounding_box",
                    cobb_namespace_tf_header_frame_id_override = "gripper",
                    approach_origin_namespace = "approach",
                    robot_base_frame = "base_link",
                    reference_frame = "gripper",
                    selected_candidate_tf_frame = "grasp";
        };

        CoreSetupBase(){};
        ~CoreSetupBase(){};

        typedef std::shared_ptr<CoreSetupBase> Ptr;

        bool setupCoreConfiguration();
        bool checkConfig();
        std::string getErrorMsg();

        Data getData();

    protected:

        Data data_;
        std::stringstream error_msg_ss_;

    private:
        virtual bool loadCoreConfiguration() = 0;


    };
}
#endif //GRASPING_SELECTION_CORE_SETUP_BASE_H

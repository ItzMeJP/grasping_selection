/**\file core.h
 * * \brief File with grasping_selection core class declaration
 *
 * @version 28_12_2022
 * @author João Pedro Carvalho de Souza
 */

#ifndef GRASPING_SELECTION_CORE
#define GRASPING_SELECTION_CORE

#pragma once

#include <string>
#include <iostream>
#include <ctime> // to get date info
#include <string.h>
#include <sstream> //to manip strings
#include <fstream> // to generate files
#include <sys/stat.h> // to check the path existence

#include "grasping_heuristics_data.h"

#define MSG_PREFIX "<GraspingSelection::core> "

#ifndef NDEBUG
#define DEBUG_GRASPING_SELECTION_CORE_MSG(str) do { std::cout << "\033[;33m" << MSG_PREFIX << str << "\033[0m"<< std::endl; } while( false )
#else
#define DEBUG_GRASPING_SELECTION_CORE_MSG(str) do { } while ( false )
#endif

#define ERROR_GRASPING_SELECTION_CORE_MSG(str) do { std::cout << "\033[;31m" << MSG_PREFIX << str << "\033[0m"<< std::endl; } while( false )


namespace grasping_selection {

    struct RequestInput {

        std::string detected_object_name,
                    detected_object_tf_name;
                int operation_mode;

    };

    struct Configuration{
        std::string
                log_folder_path = "",
                grasp_candidates_namespace = "candidate",
                cog_namespace_ = "center_of_gravity",
                cog_namespace_tf_header_frame_id_override_ = "gripper",
                cobb_namespace_ = "center_of_bounding_box",
                cobb_namespace_tf_header_frame_id_override_ = "gripper",
                approach_origin_namespace_ = "approach",
                robot_base_frame_ = "base_link",
                reference_frame_ = "gripper",
                selected_candidate_tf_frame = "grasp";
    };

    class GraspingSelection{
    public:
        GraspingSelection ();
        ~GraspingSelection (void);

        enum OPERATION_MODE {
            DIRECT,
            PRE_LOAD,
            STANDALONE_RUN
        };

        enum CORE_FEEDBACK_CODE {
            LOG_PATH_ERROR = 101,
            LOG_FOLDER_CREATION_ERROR,
            SETUP_CONFIGURATION_ERROR,
            REQUEST_INPUT_ERROR,
            PRELOAD_STANDALONE_INCONSISTENCY
        };

        bool setupConfiguration(Configuration _config);
        bool requestSelection(RequestInput _in);

        void buildTheLogFile(); //TODO:  put this method as private

        int getErrorCode();

    protected:
        std::string log_folder_name_,
                    current_log_folder_path_,
                    detected_object_name_loaded_,
                    detected_object_tf_name_loaded_, //name of the tf, that can be different from the name of the object
                    candidate_chosen_;

        int request_counter_ = 0,
            error_code_;
        Configuration configuration_;
        bool checkConfig(Configuration _in);
        bool checkInputRequest(RequestInput _in);
        bool setupLog(std::string _path_log_folder);
        void updateOutputMsg(std::string _msg, bool _error_msg);

        GraspEstimationBase::ArrPtr                    estimationPipelineArrPtr_ = std::make_shared<GraspEstimationBase::Arr>();

        bool
                preload_ok_             = false,
                occur_joint_filter_     = false,
                occur_collision_filter_ = false,
                occur_ws_filter_        = false;

    private:

        bool checkIfPathExist(std::string _path);
        bool createLogFolder(std::string _path);
        void generateDatetimeFolderName(std::string &_folder_name);

        std::ofstream outputLogFile_;
        std::stringstream contentToServerLogFile_;

    };

} //end namespace

#endif // GRASPING_SELECTION_CORE
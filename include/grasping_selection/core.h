/**\file core.h
 * * \brief File with grasping_selection core class declaration
 *
 * @version 28_12_2022
 * @author João Pedro Carvalho de Souza
 */

#ifndef GRASPING_SELECTION_CORE
#define GRASPING_SELECTION_CORE

#pragma once


#include <ctime> // to get date info
#include <sstream> //to manip strings
#include <fstream> // to generate files
#include <sys/stat.h> // to check the path existence

#include "core_data.h" //also includes the std_vector_complement and the transform_manipulation libs
#include "grasping_heuristics_data.h"
#include "grasping_heuristics_base.h"
#include "loading_grasping_dataset_base.h"
#include "loading_grasping_dataset_from_json.h"

#define MSG_GRASPING_SELECTION_PREFIX "<GraspingSelection::core> "

#ifndef NDEBUG
#define DEBUG_GRASPING_SELECTION_CORE_MSG(str) do { std::cout << "\033[;33m" << MSG_GRASPING_SELECTION_PREFIX << str << "\033[0m"<< std::endl; } while( false )
#else
#define DEBUG_GRASPING_SELECTION_CORE_MSG(str) do { } while ( false )
#endif

#define ERROR_GRASPING_SELECTION_CORE_MSG(str) do { std::cout << "\033[;31m" << MSG_GRASPING_SELECTION_PREFIX << str << "\033[0m"<< std::endl; } while( false )


namespace grasping_selection {

    class GraspingSelection{
    public:
        GraspingSelection ();
        ~GraspingSelection (void);

        bool setupConfiguration(LoadingGraspingDatasetBase::Ptr _readDataFunc,Configuration _config);
        bool requestSelection(RequestInput _in);

        void buildTheLogFile(); //TODO:  put this method as private

        int getErrorCode();

        bool testing();

    protected:
        std::string log_folder_name_,
                    current_log_folder_path_,
                    detected_object_name_loaded_,
                    detected_object_tf_name_loaded_, //name of the tf, that can be different from the name of the object
                    candidate_chosen_;

        int request_counter_ = 0,
            error_code_;
        Configuration configuration_;
        LoadingGraspingDatasetBase::Ptr readDataFunc_;
        bool checkConfig(Configuration _in);
        bool checkInputRequest(RequestInput _in);
        bool setupLog(std::string _path_log_folder);
        void updateOutputMsg(std::string _msg, bool _error_msg);

        GraspingHeuristicsBase::ArrPtr estimationPipelineArrPtr_ = std::make_shared<GraspingHeuristicsBase::Arr>();
        CandidateArr grasp_candidates_arr_;
        std::vector<double> pipeline_scores_arr_;
        std::vector<bool> pipeline_extrapolation_arr_;
        std::vector<std::pair<double, int>> pipeline_scores_pair_arr_; //used for sorts scores_arr
        std::vector<std::string> candidates_tf_names_arr_;
        MarkerArr collisions_marker_arr_;

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
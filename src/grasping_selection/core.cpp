/**\file core.h
 * * \brief File with grasping_selection core class definition
 *
 * @version 28_12_2022
 * @author João Pedro Carvalho de Souza
 */

#include "core.h"

namespace grasping_selection {

    GraspingSelection::GraspingSelection () {    }
    GraspingSelection::~GraspingSelection () {    }

    bool GraspingSelection::setupConfiguration(Configuration _config){

        configuration_ = _config;

        if(!setupLog(configuration_.log_folder_path) ||
           !checkConfig(configuration_)){
            return false;
        }

        return true;

    }

    bool GraspingSelection::setupLog(std::string _path_log_folder){

        if(!checkIfPathExist(_path_log_folder)){
            error_code_ = LOG_PATH_ERROR;
            return false;
        }

        if(!createLogFolder(_path_log_folder)){
            error_code_ = LOG_FOLDER_CREATION_ERROR;
            return false;
        }

        outputLogFile_.open(current_log_folder_path_ + "/core.log", std::ios::out | std::ios::trunc );

        return true;

    };

    bool GraspingSelection::requestSelection(RequestInput _in) {

        if(!checkInputRequest(_in)) {
            return false;
        }

        request_counter_ ++;

        updateOutputMsg("####################################################################################"  ,0);
        updateOutputMsg("                             Request #" + std::to_string(request_counter_)         ,0);
        updateOutputMsg("####################################################################################"  ,0);
        updateOutputMsg("Requested object name : " + _in.detected_object_name, 0);
        updateOutputMsg("Requested object tf name : " + _in.detected_object_tf_name, 0);
        updateOutputMsg("Operation Mode : " + std::to_string(_in.operation_mode), 0);

        if (_in.operation_mode != OPERATION_MODE::DIRECT) {
            if (_in.operation_mode == OPERATION_MODE::PRE_LOAD) { //if load operation mode

                estimationPipelineArrPtr_->clear();
                grasp_candidates_arr_.candidates.clear();
                pipeline_extrapolation_arr_.clear();
                candidates_tf_names_arr_.clear();
                pipeline_scores_arr_.clear();
                pipeline_scores_pair_arr_.clear();
                collisions_marker_arr_.markers.clear();

                occur_collision_filter_ = false;
                occur_joint_filter_     = false;
                occur_ws_filter_        = false;

                candidate_chosen_ = "NOT_CONVERGED";

                detected_object_name_loaded_    = _in.detected_object_name;
                detected_object_tf_name_loaded_ = _in.detected_object_tf_name;
            } else {

                if ((detected_object_name_loaded_.compare(_in.detected_object_name) != 0) ||
                    (detected_object_tf_name_loaded_.compare(_in.detected_object_tf_name) != 0)) {

                    updateOutputMsg("Detected object and/or TF name(s) are different from the ones defined by load option.",1);
                    error_code_ = PRELOAD_STANDALONE_INCONSISTENCY;
                    return false;

                }

            }
        } else {
            estimationPipelineArrPtr_->clear();
            grasp_candidates_arr_.candidates.clear();
            pipeline_extrapolation_arr_.clear();
            candidates_tf_names_arr_.clear();
            pipeline_scores_arr_.clear();
            pipeline_scores_pair_arr_.clear();
            collisions_marker_arr_.markers.clear();

            occur_collision_filter_ = false;
            occur_joint_filter_     = false;
            occur_ws_filter_        = false;

            candidate_chosen_ = "NOT_CONVERGED";
        }

        GraspingHeuristicsParameterBase::Ptr config;
        config.reset(new EuclideanDistanceParameter());


        const clock_t begin_time = clock();

        //bool success = executeProcess(_in.operation_mode);

        float elapsed_time = float (clock () - begin_time) / CLOCKS_PER_SEC;

        if (_in.operation_mode == OPERATION_MODE::DIRECT || _in.operation_mode == OPERATION_MODE::STANDALONE_RUN)
            updateOutputMsg("Decision Elapsed Time :" + std::to_string(elapsed_time),0 );
        else
            updateOutputMsg("Loading Elapsed Time :" + std::to_string(elapsed_time),0 );
/*
        LogData l;
        l.decision_time    = elapsed_time;
        l.detected_object  = detected_object_name_;
        l.candidate_chosen = candidate_chosen_;
        log_data_arr_.push_back(l);
*/

        return success;

    }

    bool GraspingSelection::checkConfig(Configuration _in){

        DEBUG_GRASPING_SELECTION_CORE_MSG("Checking the configuration parameters.");
        contentToServerLogFile_ <<  "Checking the configuration parameters.\n";

        bool success = true;

        if(_in.approach_origin_namespace_.empty())
        {
            updateOutputMsg("Approach Origin Namespace parameter is Empty.",1);
            success = false;
        }
        if(_in.cobb_namespace_.empty())
        {
            updateOutputMsg("COBB Namespace parameter is Empty.",1);
            success = false;
        }
        if(_in.cobb_namespace_tf_header_frame_id_override_.empty())
        {
            updateOutputMsg("COBB Namespace Override parameter is Empty.",1);
            success = false;
        }
        if(_in.cog_namespace_.empty())
        {
            updateOutputMsg("COG Namespace parameter is Empty.",1);
            success = false;
        }
        if(_in.cog_namespace_tf_header_frame_id_override_.empty())
        {
            updateOutputMsg("COG Namespace Override parameter is Empty.",1);
            success = false;
        }
        if(_in.grasp_candidates_namespace.empty())
        {
            updateOutputMsg("Grasp Candidate Namespace parameter is Empty.",1);
            success = false;
        }
        if(_in.reference_frame_.empty())
        {
            updateOutputMsg("Reference frame parameter is Empty.",1);
            success = false;
        }
        if(_in.robot_base_frame_.empty())
        {
            updateOutputMsg("Robot Base Frame parameter is Empty.",1);
            success = false;
        }
        if(_in.selected_candidate_tf_frame.empty())
        {
            updateOutputMsg("Selected Candidate TF frame parameter is Empty.",1);
            success = false;
        }

        success?0:error_code_ =SETUP_CONFIGURATION_ERROR;

        return success;
    }

    bool GraspingSelection::checkInputRequest(RequestInput _in){

        bool success = true;

        updateOutputMsg("Checking the input parameters.",0);

        if(!(_in.operation_mode == OPERATION_MODE::DIRECT
             || _in.operation_mode == OPERATION_MODE::PRE_LOAD
             || _in.operation_mode == OPERATION_MODE::STANDALONE_RUN  ))
        {
            updateOutputMsg("Invalid operation mode.",1);
            success = false;
        }
        if(_in.detected_object_name.empty())
        {
            updateOutputMsg("Object detected name is Empty.",1);
            success = false;
        }
        if(_in.detected_object_tf_name.empty())
        {
            updateOutputMsg("Object detected tf name is Empty.",1);
            success = false;
        }

        success?0:error_code_ = REQUEST_INPUT_ERROR;

        return success;
    }

    bool GraspingSelection::checkIfPathExist(std::string _path){

        DEBUG_GRASPING_SELECTION_CORE_MSG("Checking if the LOGs path exists.");

        struct stat sb;
        const char* str = _path.c_str();

        if (stat(str, &sb) == 0){
            return true;
        }
        else{
            return false;
        }

    }

    void GraspingSelection::updateOutputMsg(std::string _msg, bool _error_msg){

        if(!_error_msg){
            contentToServerLogFile_ << _msg+"\n";
            DEBUG_GRASPING_SELECTION_CORE_MSG(_msg);}
        else{
            contentToServerLogFile_ << "<ERROR> " + _msg+"\n";
            ERROR_GRASPING_SELECTION_CORE_MSG(_msg);
        }
    }

    void GraspingSelection::buildTheLogFile() {
        outputLogFile_ << contentToServerLogFile_.str() ;
        outputLogFile_.close();
    }

    bool GraspingSelection::createLogFolder(std::string _path){

        DEBUG_GRASPING_SELECTION_CORE_MSG("Creating the LOGs folder.");

        generateDatetimeFolderName(log_folder_name_);
        log_folder_name_ = "grasping_selection_" + log_folder_name_;
        current_log_folder_path_ = configuration_.log_folder_path+"/"+log_folder_name_;

        if (mkdir(current_log_folder_path_.c_str(), 0777) == -1) {
            ERROR_GRASPING_SELECTION_CORE_MSG ("Error:  " << strerror(errno));
            return false;
        }
        else {
            DEBUG_GRASPING_SELECTION_CORE_MSG ("Logs directory created.");
            return true;
        }
    }

    int GraspingSelection::getErrorCode(){
        return error_code_;
    }

    void GraspingSelection::generateDatetimeFolderName(std::string &_folder_name){

        std::time_t t = std::time(nullptr);
        std::tm* now = std::localtime(&t);

        std::stringstream name;
        int aux1 = now->tm_mon + 1,
                aux2 = now->tm_year + 1900;

        name << now->tm_mday << aux1 << aux2 << "_" << now->tm_hour << now->tm_min << now->tm_sec;
        _folder_name = name.str();

//        std::cout << "Current Date: " << now->tm_mday << '/' << (now->tm_mon + 1) << '/'<< (now->tm_year + 1900) << std::endl;
//        std::cout << "Current Hour: " << now->tm_hour << ':' << now->tm_min << ':'<< now->tm_sec << std::endl;

    }

} //end namespace
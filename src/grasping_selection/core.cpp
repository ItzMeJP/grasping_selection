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

    bool GraspingSelection::setupConfiguration(CoreSetupBase::Ptr _readCoreSetupFunc, LoadingGraspingDatasetBase::Ptr _readDatasetFunc ){

        readDatasetFunc_ = _readDatasetFunc;
        readCoreSetupFunc_ = _readCoreSetupFunc;

        if(!readCoreSetupFunc_->setupCoreConfiguration()){
            ERROR_GRASPING_SELECTION_CORE_MSG(readCoreSetupFunc_->getErrorMsg());
            error_code_ = CORE_FEEDBACK_CODE::SETUP_CONFIGURATION_ERROR;
            return false;
        }

        if(!setupLog(readCoreSetupFunc_->getData().log_folder_path)){
            ERROR_GRASPING_SELECTION_CORE_MSG("LOG_PATH_ERROR. CODE: " << this->getErrorCode());
            return false;
        }

        DEBUG_GRASPING_SELECTION_CORE_MSG("Grasping selection server setup finished. Running...");

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

        request_counter_ ++;

        updateOutputMsg("####################################################################################"  ,0);
        updateOutputMsg("                             Request #" + std::to_string(request_counter_)         ,0);
        updateOutputMsg("####################################################################################"  ,0);

        if(!checkInputRequest(_in)) {
            return false;
        }

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


        const clock_t begin_time = clock();

        bool success = executeProcess(_in.operation_mode);

        float elapsed_time = float (clock () - begin_time) / CLOCKS_PER_SEC;

        if (_in.operation_mode == OPERATION_MODE::DIRECT || _in.operation_mode == OPERATION_MODE::STANDALONE_RUN)
            updateOutputMsg("Decision Elapsed Time [s]:" + std::to_string(elapsed_time),0 );
        else
            updateOutputMsg("Loading Elapsed Time [s]:" + std::to_string(elapsed_time),0 );
/*
        LogData l;
        l.decision_time    = elapsed_time;
        l.detected_object  = detected_object_name_;
        l.candidate_chosen = candidate_chosen_;
        log_data_arr_.push_back(l);
*/

        //return success;
        return true;

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

        DEBUG_GRASPING_SELECTION_CORE_MSG("Checking if the LOGs path exists. Current value: \"" << _path << "\"");

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
        current_log_folder_path_ = readCoreSetupFunc_->getData().log_folder_path+"/"+log_folder_name_;

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

    bool GraspingSelection::executeProcess (int _operation_mode) {

        if (_operation_mode == OPERATION_MODE::DIRECT){ //Load and Run the Pipeline
            return (executeDirectProcess());
        }
        else if (_operation_mode == OPERATION_MODE::PRE_LOAD) { // Just Load
            return (executePreLoad());
        }
        else if (_operation_mode == OPERATION_MODE::STANDALONE_RUN && preload_ok_) { // Just Run the Pipeline
            preload_ok_ = false;
            return (executeStandloneProcess());
        }
        else if (!preload_ok_) {
            ROS_ERROR_STREAM("No loaded data found");
            return false;
        }
        return true;

    }

    /// <summary>
    /// Execute the load and run mode together
    /// </summary>
    /// <returns>true if succeeded, otherwise false </returns>
    bool GraspingSelection::executeDirectProcess () {

        //start the procedure after goal acquisition.     It loads parameter server and run the procedure START
        if (
            //    !getNumberOfCandidates() ||
               !readDatasetFunc_->loadObjectCandidatesDataset()
            || !readDatasetFunc_->loadObjectCenterOfGravity()
            //|| !pubCogTF()
            || !readDatasetFunc_->loadObjectCenterOfBoundingBox()
            //|| !pubCobbTF()
            //|| !pubCandidates()
            || !loadEstimationPipeline()
            || !buildTFsArr()
            || !runMethods()
            || !sortScores()
            || !getResult()
            || !pubChoosenCandidate()
                ){
            if (debug_tools_) {
                runDebugTools(); // only pub debug tools that are not related to a candidate
            }

            return false;
        }

        return true;
    }

    /// <summary>
    /// Load the object pipeline from parameter server. Supported methods's tag names (X is an alpha_numeric that define the order):
    /// X_euclidean: euclidean distance evaluator
    /// X_depth: depth distance evaluator
    /// X_roll: roll distance evaluator
    /// X_pitch: pitch distance evaluator
    /// X_joints: joint filter
    /// X_workspace: workspace filter
    /// X_collision: collision filter
    /// </summary>
    /// <returns> false if the pipeline is empty (no identified tag), otherwise true </returns>
    bool GraspingSelection::loadEstimationPipeline () {

        //just to know the order of the pipeline metrics
        XmlRpc::XmlRpcValue xml_param;
        std::string         configuration_namespace =
                private_node_handle_->getNamespace() + "/" + detected_object_name_ + "/pipeline";
        ROS_DEBUG_STREAM("Config_ns to load pipelines: " << configuration_namespace);
        if (private_node_handle_->getParam(configuration_namespace, xml_param) &&
            xml_param.getType() == XmlRpc::XmlRpcValue::TypeStruct) {

            //ROS_DEBUG_STREAM("xml_param: " << xml_param);

            GraspEstimationBase::Ptr           method;
            //int i = 1;
            for (XmlRpc::XmlRpcValue::iterator it = xml_param.begin(); it != xml_param.end(); ++it) {
                std::string method_name = it->first;

                if (method_name.find("euclidean") != std::string::npos) {
                    method.reset(new EuclideanGraspEstimation());
                } else if (method_name.find("depth") != std::string::npos) {
                    method.reset(new DepthGraspEstimation());
                } else if (method_name.find("roll") != std::string::npos) {
                    method.reset(new RollGraspEstimation());
                } else if (method_name.find("pitch") != std::string::npos) {
                    method.reset(new PitchGraspEstimation());
                } else if (method_name.find("yaw") != std::string::npos) {
                    method.reset(new YawGraspEstimation());
                } else if (method_name.find("joints") != std::string::npos) {
                    method.reset(new JointsGraspEstimation());
                } else if (method_name.find("workspace") != std::string::npos) {
                    method.reset(new ConeWSGraspEstimation());
                } else if (method_name.find("collision") != std::string::npos) {
                    method.reset(new CollisionGraspEstimation());
                } else if (method_name.find("cog_distance") != std::string::npos) {
                    method.reset(new COGDistanceGraspEstimation());
                } else if (method_name.find("cobb_distance") != std::string::npos) {
                    method.reset(new COBBDistanceGraspEstimation());
                }

                //TODO: insert new methods ...

                if (method) {
                    ROS_INFO_STREAM("Loading parameters for : " << configuration_namespace + "/" + method_name);
                    method->setTfBuffer(tf_buffer_);
                    method->setupMethodConfigurationFromParameterServer(node_handle_, private_node_handle_,
                                                                        configuration_namespace + "/" + method_name +
                                                                        "/");
                    estimationPipelineArrPtr_->push_back(method);
                }
            }

        }
        if (estimationPipelineArrPtr_->empty()) {
            ROS_ERROR_STREAM("The grasp estimation pipeline is empty");
            return false;
        }

        return true;
    }











    bool GraspingSelection::testing() {


        LoadingGraspingDatasetBase::Ptr graspingDatasetLoader;
        graspingDatasetLoader.reset(new LoadingGraspingDatasetFromJSON());

        CoreSetupBase::Ptr coreSetupLoader;
        coreSetupLoader.reset(new CoreSetupFromJSON());


        if(!this->setupConfiguration(coreSetupLoader,graspingDatasetLoader) ) {
            return false;
        }


        RequestInput in;

        in.operation_mode = OPERATION_MODE::DIRECT;
        in.detected_object_name = "object";
        in.detected_object_tf_name = "object";

        if( this->requestSelection(in) )
            std::cout <<  "Success" << std::endl;

        this->buildTheLogFile();


        return 0;
    }

} //end namespace
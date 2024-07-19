//
// Created by joaopedro on 1/6/23.
//
#include "core_setup_base.h"

namespace grasping_selection{

    bool CoreSetupBase::checkConfig(){

        bool success = true;
        error_msg_ss_.clear();

        if(data_.approach_origin_namespace.empty())
        {
            error_msg_ss_ <<"Approach Origin Namespace parameter is Empty.\n";
            success = false;
        }
        if(data_.cobb_namespace_.empty())
        {
            error_msg_ss_ <<"COBB Namespace parameter is Empty.\n";
            success = false;
        }
        if(data_.cobb_namespace_tf_header_frame_id_override.empty())
        {
            error_msg_ss_ <<"COBB Namespace Override parameter is Empty.""\n";
            success = false;
        }
        if(data_.cog_namespace.empty())
        {
            error_msg_ss_ <<"COG Namespace parameter is Empty.""\n";
            success = false;
        }
        if(data_.cog_namespace_tf_header_frame_id_override.empty())
        {
            error_msg_ss_ <<"COG Namespace Override parameter is Empty.""\n";
            success = false;
        }
        if(data_.grasp_candidates_namespace.empty())
        {
            error_msg_ss_ <<"Grasp Candidate Namespace parameter is Empty.""\n";
            success = false;
        }
        if(data_.reference_frame.empty())
        {
            error_msg_ss_ <<"Reference frame parameter is Empty.""\n";
            success = false;
        }
        if(data_.robot_base_frame.empty())
        {
            error_msg_ss_ <<"Robot Base Frame parameter is Empty.""\n";
            success = false;
        }
        if(data_.selected_candidate_tf_frame.empty())
        {
            error_msg_ss_ <<"Selected Candidate TF frame parameter is Empty.""\n";
            success = false;
        }

        if(success){
            error_msg_ss_ << "None error.\n" ;
        }

        return success;
    }

    bool CoreSetupBase::setupCoreConfiguration(){

        if(!this->loadCoreConfiguration() || ! this->checkConfig()){
            return false;
        }

        return true;
    }

    std::string CoreSetupBase::getErrorMsg() {
        return error_msg_ss_.str();
    }

    CoreSetupBase::Data CoreSetupBase::getData() {
        return data_;
    }




}
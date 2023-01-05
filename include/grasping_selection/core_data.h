//
// Created by joaopedro on 1/5/23.
//

#ifndef GRASPING_SELECTION_GRASPING_SELECTION_DATA_H
#define GRASPING_SELECTION_GRASPING_SELECTION_DATA_H

#include <string>
#include <iostream>

#include "transform_manipulation/pose.h"
#include "std_vector_complement/std_vector_operations.h"

namespace grasping_selection {

    struct RequestInput {

        std::string detected_object_name,
                detected_object_tf_name;
        int operation_mode;

    };

    struct Configuration {
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

    enum MARKER_TYPE{
        ARROW = 0u,
        CUBE = 1u,
        SPHERE = 2u,
        CYLINDER = 3u,
        LINE_STRIP = 4u,
        LINE_LIST = 5u,
        CUBE_LIST = 6u,
        SPHERE_LIST = 7u,
        POINTS = 8u,
        TEXT_VIEW_FACING = 9u,
        MESH_RESOURCE = 10u,
        TRIANGLE_LIST = 11u,
    };

    struct GripperData{
        double type;
        std::vector<double> parameters;
    };

    struct Candidate{
        GripperData gripper_data;
        Pose pose,
             approach;
    };

    struct CandidateArr{
        std::string object_name;
        std::vector<Candidate> candidates;
    };

    struct Marker{
        Pose pose;
        struct {double x,y,z;} scale;
        struct {double r,g,b,a;} color;
        int index, type;

    };

    struct MarkerArr{
        std::vector<Marker> markers;
    };
}

#endif //GRASPING_SELECTION_GRASPING_SELECTION_DATA_H

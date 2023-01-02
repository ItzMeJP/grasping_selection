/**\file grasp_estimation_skill_server.cpp
 * \brief File with grasping_heuristics_base class definition
 *
 * @version 1.0
 * @author João Pedro Carvalho de Souza
 */

#ifndef GRASPING_SELECTION_GRASPING_HEURISTICS_BASE_H
#define GRASPING_SELECTION_GRASPING_HEURISTICS_BASE_H

#include <memory> // to use shared_ptr
#include <std_vector_complement/std_vector_operations.h>

#include "grasping_heuristics_data.h"

namespace grasping_selection {
    class GraspingHeuristicsBase {
    public:
        GraspingHeuristicsBase();
        ~GraspingHeuristicsBase();

        typedef std::shared_ptr<GraspingHeuristicsBase> Ptr;
        typedef std::vector<GraspingHeuristicsBase::Ptr> Arr;
        typedef std::shared_ptr<GraspingHeuristicsBase::Arr> ArrPtr;


        enum GRIPPER_ID {        // Type of gripper used
            ROBOTIQ_2F_85,       // two adaptive fingers gripper from Robotiq with opening of 85mm
            ROBOTIQ_2F_140,      // two adaptive fingers gripper from Robotiq with opening of 140mm
            ROBOTIQ_3F,           // three adaptive fingers gripper from Robotiq
            SUCTION,
            FESTO_2F_HGPC_16_A_30,   //Pneumatic parallel 2F gripper from FESTO with fingers with opening of 30mm
            SCHMALZ_FOAM_SUCTION_CUP_FMSW_N10_76x22, // Small rectangular foam suction cup from Schmalz
        };

        enum SYNTHESIS_METHOD {
            MANUAL,
            SANN_MULTIFINGERED,
            SANN_SUCTION,
            MIMIC_GRASPING,
        };

        enum ESTIMATION_METHOD{
            EUCLIDEAN_DISTANCE,
            DEPTH_DISTANCE,
            ROLL_DISTANCE,
            PITCH_DISTANCE,
            YAW_DISTANCE,
            JOINTS_FILTER,
            WORKSPACE_FILTER,
            COLLISION_FILTER,
            COG_DISTANCE,
            COBB_DISTANCE
        };

        struct candidateCollisionData{
/*
            int _size;
            std::vector<grasp_estimation_skill_msgs::GeometricShape> _shapes_arr;
            std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> _cloud_arr;
*/
        };

        struct generalCollisionData{
/*
            int _size;
            std::vector<grasp_estimation_skill_msgs::GeometricShape> calib_shapes;
            pcl::PointCloud<pcl::PointXYZRGBNormal> scene_cloud;
*/
        };

        struct jointData{
  //          std::vector<Eigen::VectorXd> kdl_solutions_arr_;
        };

        struct wsData{
//            pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
        };

 /*       struct tfsArr{
            std::vector<geometry_msgs::TransformStamped> child_from_reference_frame_arr, // candidates from custom frame, typically gripper tcp frame. It is needed to be updated every time a method is called to run.
            child_from_robot_frame_arr,     // candidates from robot base frame. It is needed to be updated every time a method is called to run.
            child_from_cog_frame_arr,       // candidates from cog frame. It is needed to be updated only once since its object property.
            child_from_cobb_frame_arr;      // candidates from cobb frame. It is needed to be updated only once since its object property.
        } tfs_arr_;
*/
        virtual void setupMethodConfiguration(GraspingHeuristicsParameterBase _g) = 0;
        //void setupBaseConfigurationFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle, std::string _configuration_namespace);
        //void setTfBuffer(std::shared_ptr<tf2_ros::Buffer>& _tf_buffer) { tf_buffer_ = _tf_buffer; }

        bool start (std::string _robot_base_frame, std::string _tf_base_reference_name_,
                    std::vector<std::string> _tf_candidates_name_arr_, tfsArr _tfs_arr, grasp_estimation_skill_msgs::GraspCandidateArr &candidates_arr,
                    std::vector<bool>  _previously_extrapolation_arr); //TODO: change all these vector to only candidates_arr

        // Arrays
        std::vector<double> getScoreArr();
        std::vector <double> getWeightedScoreArr();
        std::vector<bool> getExtrapolationArr();

        virtual int getMethodName() = 0;

        //Specific public method and parameters
        ///Collision
        // bool getDataFromCollisionFilter(std::vector<grasp_estimation_skill_msgs::GeometricShape> &_shapes_arr,
        //                                 int &_size,
        //                                 std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> &_cloud_arr);

        virtual bool getData(grasp_estimation_skill::GraspEstimationBase::candidateCollisionData &_d) = 0;
        virtual bool getData(grasp_estimation_skill::GraspEstimationBase::generalCollisionData &_d) = 0;
        virtual bool getData(grasp_estimation_skill::GraspEstimationBase::jointData &_d) = 0;
        virtual bool getData(grasp_estimation_skill::GraspEstimationBase::wsData &_d) = 0;

    protected:
        bool getTFsFromReference ();
        bool getTFsFromRobotBase ();
        virtual bool run() = 0;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        grasp_estimation_skill_msgs::GraspCandidateArr candidates_arr_;


        std::string tf_base_reference_name_, robot_base_frame_;

        // Arrays
        std::vector<std::string> tf_candidates_name_arr_;


        std::vector<double> scores_arr_, weighted_scores_arr_;
        std::vector<bool>   extrapolation_arr_;
        std::vector<bool>   previously_extrapolation_arr_;

        double weight_;

        //Specific private method and parameters
        /// Collision
        //std::vector<grasp_estimation_skill_msgs::GeometricShape> collision_shapes_each_candidate_arr_;
        //std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> collision_clouds_each_candidate_arr_;
        //int number_of_collision_shapes_per_candidate_;

    };
}

#endif //GRASPING_SELECTION_GRASPING_HEURISTICS_BRASE_H

#pragma once
#include "CGAL_includes.h"
#include <Eigen/Dense>
#include <vector>

class UAV

{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        UAV() {
            pose <<     1.f, 0.f, 0.f, 0.f,
                        0.f, 1.f, 0.f, 12.f,
                        0.f, 0.f, 1.f, 0.f,
                        0.f, 0.f, 0.f, 1.f;
            //x kenet -9
            // view vector for the camera  - first column of the rotation matrix
           // up vector for the camera    - second column of the rotation matix
           // right vector for the camera - third column of the rotation matrix
           // The (X, Y, Z) position of the camera w.r.t origin - 4th column
            //convert rotation to quaternion  and assign
            Eigen::Matrix3f rot = pose.block(0, 0, 3, 3);
            q = Eigen::Quaternionf(rot);
        }
        
    Eigen::Matrix4f get_pose() { return pose; }
    void set_pose(Eigen::Matrix4f& pose) { this->pose = pose; }
    float get_hfov() { return hfov_; }
    float get_vfov() { return vfov_; }
    float get_np_dist() { return np_dist_; }
    float get_fp_dist() { return fp_dist_; }
    Eigen::Quaternionf get_orientiation() { return q; }
    void set_orientation(Eigen::Quaternionf& rot) { q = rot; }

private:
    /** \brief The camera pose */
    Eigen::Matrix4f pose;
    /** \brief Horizontal field of view */
    float hfov_;
    /** \brief Vertical field of view */
    float vfov_;
    /** \brief Near plane distance */
    float np_dist_;
    /** \brief Far plane distance */
    float fp_dist_;

    //orientation
    Eigen::Quaternionf q;


};


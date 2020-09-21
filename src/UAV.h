#pragma once
#include "CGAL_includes.h"
#include <Eigen/Dense>
#include <vector>

class UAV

{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    Eigen::Matrix4f get_pos() { return position; }
    float get_hfov() { return hfov_; }
    float get_vfov() { return vfov_; }
    float get_np_dist() { return np_dist_; }
    float get_fp_dist() { return fp_dist_; }
    Eigen::Quaternionf get_orientiation() { return q; }

private:
    /** \brief The camera pose */
    Eigen::Matrix4f position;
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


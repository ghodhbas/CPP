#pragma once
#include "CGAL_includes.h"
#include <Eigen/Dense>
#include <vector>

class UAV

{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
	void get_pos();

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


};


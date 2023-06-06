#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>

#include <glm/glm.hpp>
#include <glm/gtx/matrix_decompose.hpp>

using namespace adsk::core;
using namespace adsk::fusion;

struct Pose {
	glm::dvec3 translation;
	glm::dquat orientation;

	glm::dvec3 euler_angles();
};

glm::dmat4 vec_to_glm_mat4(std::vector<double>& vec);

Pose get_pose_from_fusion_mat4(Ptr<Matrix3D> mat);

glm::dvec3 get_vec3_from_fusion_point(Ptr<Point3D> point);

glm::dvec3 get_vec3_from_native_vec3(Ptr<Vector3D> vec);

glm::dquat get_quat_from_vec(glm::dvec3 vec);
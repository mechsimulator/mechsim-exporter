#include "Math.h"

glm::dmat4 vec_to_glm_mat4(std::vector<double>& v) {
	if (v.size() != 16) return glm::dmat4();
	return glm::dmat4(
		v[0], v[1], v[2], v[3],
		v[4], v[5], v[6], v[7],
		v[8], v[9], v[10], v[11],
		v[12], v[13], v[14], v[15]
	);
}

Pose get_pose_from_fusion_mat4(Ptr<Matrix3D> mat) {
	auto vec = mat->asArray();
	glm::dmat4 transformMat = vec_to_glm_mat4(vec);
	glm::dvec3 scale;
	glm::dquat orientation;
	glm::dvec3 translation;
	glm::dvec3 skew;
	glm::dvec4 perspective;
	glm::decompose(transformMat, scale, orientation, translation, skew, perspective);

	auto nativeTranslation = mat->translation()->asArray();
	translation.x = nativeTranslation[0];
	translation.y = nativeTranslation[1];
	translation.z = nativeTranslation[2];

	return Pose{ translation, orientation };
}

glm::dvec3 Pose::euler_angles() {
	return glm::eulerAngles(orientation);
}

glm::dvec3 get_vec3_from_fusion_point(Ptr<Point3D> point) {
	return glm::dvec3(point->x(), point->y(), point->z());
}

glm::dvec3 get_vec3_from_native_vec3(Ptr<Vector3D> vec) {
	auto arr = vec->asArray();
	return glm::dvec3(arr[0], arr[1], arr[2]);
}

glm::dquat get_quat_from_vec(glm::dvec3 vec) {
	return glm::dquat{ vec.x, vec.y, vec.z, 0 };
}
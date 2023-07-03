#include "Parser.h"
#include "Common.h"

#include <algorithm>

template<typename J>
glm::dvec3 getOriginFromJoint(Ptr<J> joint, Ptr<JointGeometry> geometryOrOrigin) {

	Ptr<JointGeometry> geometry;
	if (!strcmp(geometryOrOrigin->objectType(), "adsk::fusion::JointGeometry")) {
		geometry = geometryOrOrigin->cast<JointGeometry>();
		auto ent = geometry->entityOne();
		if (!strcmp(ent->objectType(), "adsk::fusion::BRepEdge")) {
			auto entEdge = ent->cast<BRepEdge>();
			if (!entEdge->assemblyContext()) {
				auto newEnt = entEdge->createForAssemblyContext(joint->occurrenceOne());
				auto min = newEnt->boundingBox()->minPoint();
				auto max = newEnt->boundingBox()->maxPoint();
				return glm::dvec3((max->x() + min->x()) / 2.0, (max->y() + min->y()) / 2.0, (max->z() + min->z()) / 2.0);
			} else {
				return glm::dvec3();
			}
		}
		
		if (!strcmp(ent->objectType(), "adsk::fusion::BRepFace")) {
			auto entFace = ent->cast<BRepFace>();
			if (!entFace->assemblyContext()) {
				auto newEnt = entFace->createForAssemblyContext(joint->occurrenceOne());
				auto centroid = newEnt->centroid();
				return glm::dvec3(centroid->x(), centroid->y(), centroid->z());
			} else {
				return glm::dvec3();
			}
		} else {
			return glm::dvec3();
		}
	} else { // JointOrigin
		auto origin = geometryOrOrigin->cast<JointOrigin>()->geometry()->origin();
		return glm::dvec3(origin->x(), origin->y(), origin->z());
	}
}

/// Joint or AsBuiltJoint
template<typename T>
void Parser::parseJoint(Ptr<T> joint, Ptr<JointGeometry> geometry) {
	glm::dvec3 translation = getOriginFromJoint(joint, geometry);

	app->log(ms_format(translation.x << ", " << translation.y << ", " << translation.z));
	auto vec = get_vec3_from_native_vec3(geometry->primaryAxisVector());
	jointMap.emplace(joint->entityToken(), std::make_pair(JointEntry{ static_cast<JointType>(joint->jointMotion()->jointType()), Pose{ translation, get_quat_from_vec(vec) } }, jointMap.size() + 1));
}

void Parser::parse() {
	if (
		root->allJoints().size() == 0 &&
		root->allAsBuiltJoints().size() == 0
	) parse_warn("No joints found.");

	for (auto& joint : root->allJoints())
		parseJoint(joint, joint->geometryOrOriginOne()->cast<JointGeometry>());

	for (auto &asBuiltJoint : root->allAsBuiltJoints()) 
		parseJoint(asBuiltJoint, asBuiltJoint->geometry());

	if (root->allRigidGroups().size() == 0) parse_warn("No rigid groups found.");
	
	for (auto rigidGroup : root->allRigidGroups()) {
		rigidGroupMap.emplace(rigidGroup->entityToken(), rigidGroupMap.size() + 1);
	}

	for (auto part : root->allOccurrences()) {
		PartEntry partEntry;

		partEntry.name = part->fullPathName();
		partEntry.pose = get_pose_from_fusion_mat4(part->transform2());
		
		for (auto joint : part->joints()) {
			partEntry.joints.push_back(jointMap[joint->entityToken()].second);
		}

		for (auto asBuiltJoint : part->asBuiltJoints()) {
			partEntry.joints.push_back(jointMap[asBuiltJoint->entityToken()].second);
		}

		for (auto rigidGroup : part->rigidGroups()) {
			partEntry.rigidGroups.push_back(rigidGroupMap[rigidGroup->entityToken()]);
		}

		for (auto body : part->bRepBodies()) {
			auto meshCalculator = body->meshManager()->createMeshCalculator();
			meshCalculator->setQuality(TriangleMeshQualityOptions::VeryHighQualityTriangleMesh);
			auto mesh = meshCalculator->calculate();
			partEntry.bodies.push_back(BodyEntry{
				mesh->triangleCount(),
				mesh->nodeCoordinatesAsFloat(),
				mesh->nodeIndices(),
				mesh->normalVectorsAsFloat(),
				mesh->textureCoordinatesAsFloat()
			});
		}

		partsEntries.push_back(partEntry);
	}
}

void Parser::serialize(std::ofstream& file) {
	file << MRR_IDENTIFIER;

	// Joint palette
	size_t jointListSize = jointMap.size();
	file.write(reinterpret_cast<const char*>(&jointListSize), sizeof(jointListSize));
	
	for (auto jointMapEntry : jointMap) {
		JointEntry entry = jointMapEntry.second.first;
		file.write(reinterpret_cast<const char*>(&entry), sizeof(entry));
	}

	size_t partsListSize = partsEntries.size();
	file.write(reinterpret_cast<const char*>(&partsListSize), sizeof(partsListSize));
	
	for (auto& partEntry : partsEntries) {
		size_t nameSize = partEntry.name.size();
		file.write(reinterpret_cast<const char*>(&nameSize), sizeof(nameSize));
		
		file << partEntry.name;
		file.write(reinterpret_cast<const char*>(&partEntry.pose), sizeof(partEntry.pose));
		
		size_t jointRefListSize = partEntry.joints.size();
		file.write(reinterpret_cast<const char*>(&jointRefListSize), sizeof(jointRefListSize));
		file.write(reinterpret_cast<const char*>(partEntry.joints.data()), partEntry.joints.size() * sizeof(uint32_t));

		size_t rigidGroupRefListSize = partEntry.rigidGroups.size();
		file.write(reinterpret_cast<const char*>(&rigidGroupRefListSize), sizeof(rigidGroupRefListSize));
		file.write(reinterpret_cast<const char*>(partEntry.rigidGroups.data()), partEntry.rigidGroups.size() * sizeof(uint32_t));

		size_t bodiesListSize = partEntry.bodies.size();
		file.write(reinterpret_cast<const char*>(&bodiesListSize), sizeof(bodiesListSize));
		for (auto& body : partEntry.bodies) {
			file.write(reinterpret_cast<const char*>(&body.triangleCount), sizeof(body.triangleCount));
			size_t vertsSize = body.verts.size();
			file.write(reinterpret_cast<const char*>(&vertsSize), sizeof(vertsSize));
			file.write(reinterpret_cast<const char*>(body.verts.data()), body.verts.size() * sizeof(float));

			size_t indicesSize = body.indices.size();
			file.write(reinterpret_cast<const char*>(&indicesSize), sizeof(indicesSize));
			file.write(reinterpret_cast<const char*>(body.indices.data()), body.indices.size() * sizeof(int));

			size_t normalsSize = body.normals.size();
			file.write(reinterpret_cast<const char*>(&normalsSize), sizeof(normalsSize));
			file.write(reinterpret_cast<const char*>(body.normals.data()), body.normals.size() * sizeof(float));

			size_t uvSize = body.uv.size();
			file.write(reinterpret_cast<const char*>(&uvSize), sizeof(uvSize));
			file.write(reinterpret_cast<const char*>(body.uv.data()), body.uv.size() * sizeof(float));
		}
		
	}

	file.close();
}

const char* Parser::joint_type_to_str(JointTypes type) {
	static const char* jointTypes[] = {
		"RIGID",
		"REVOLUTE",
		"SLIDER",
		"CYLINDRICAL",
		"PINSLOT",
		"PLANAR",
		"BALL",
	};

	return jointTypes[type];
}

void Parser::parse_error(const char* msg) {
	error("Parse error", msg);
}

void Parser::parse_warn(const char* msg) {
	warn("Parse warning", msg);
}
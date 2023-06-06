#include "Parser.h"
#include "Common.h"

#include <algorithm>

/// Joint or AsBuiltJoint
template<typename JointType>
void Parser::parseJoint(Ptr<JointType> joint, Ptr<JointGeometry> geometry) {
	if (!strcmp(joint->objectType(), "adsk::fusion::JointGeometry")) {
		parse_error("Error parsing joint: joint objectType != adsk::fusion::JointGeometry");
	}

	glm::dvec3 translation;
	geometry->origin()->getData(translation.x, translation.y, translation.z);
	auto vec = get_vec3_from_native_vec3(geometry->primaryAxisVector());
	jointMap.emplace(joint->entityToken(), std::make_pair(JointEntry{ joint->jointMotion()->jointType(), Pose{ translation, get_quat_from_vec(vec) } }, jointMap.size() + 1));
}

void Parser::parse() {
	
	if (
		root->allJoints().size() == 0 &&
		root->allAsBuiltJoints().size() == 0
	) parse_warn("No joints found.");

	for (auto joint : root->allJoints())
		parseJoint(joint, joint->geometryOrOriginTwo()->cast<JointGeometry>());

	for (auto asBuiltJoint : root->allAsBuiltJoints()) 
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
			auto mesh = body->meshManager()->createMeshCalculator()->calculate();
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
		file << partEntry.name << '\0';
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
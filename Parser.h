#pragma once

#include <vector>
#include <fstream>
#include <map>

#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>

#include "Math.h"

#define MRR_IDENTIFIER "MRR (MechSim Robot Representation)"

using namespace adsk::core;
using namespace adsk::fusion;

struct JointEntry {
	JointTypes type;
	Pose pose;
};

struct BodyEntry {
	int32_t triangleCount;
	std::vector<float> verts;
	std::vector<int> indices;
	std::vector<float> normals;
	std::vector<float> uv;
};

struct PartEntry {
	std::string name;
	Pose pose;
	std::vector<uint32_t> joints;
	std::vector<uint32_t> rigidGroups;
	std::vector<BodyEntry> bodies;
};

class Parser {
public:
	Parser(Ptr<Component> root) : root(root) {}

	void parse();

	void serialize(std::ofstream &file);

private:
	template<typename JointType>
	void parseJoint(Ptr<JointType> joint, Ptr<JointGeometry> geometry);

	static const char* Parser::joint_type_to_str(JointTypes type);
	static void Parser::parse_error(const char* msg);
	static void Parser::parse_warn(const char* msg);
private:
	Ptr<Component> root;
	std::map<std::string, std::pair<JointEntry, uint32_t>> jointMap;
	std::map<std::string, uint32_t> rigidGroupMap;
	std::vector<PartEntry> partsEntries;
};
# MechSim Exporter Docs

A Fusion 360 add-in to export assemblies to MRR (MechSim Robot Representation) format.


## Format

### Overview

|Type  |Name  |Description |
|--|--|--|
|String  |MRR_IDENTIFIER  |Magic string used to identify the MRR format.  |
|size_t |jointListSize |Size of the joint list. |
|JointEntry[] |jointList |List of joints parts reference to. |
|size_t |partsListSize |Size of the parts list. |
|PartEntry[] |partsList |List of all parts (components) in assembly. |

### JointEntry

|Type  |Name  |Description |
|--|--|--|
|JointType |type |The joint type that describes the relative motion between two parts. (e.g. rigid, revolute, slider) |
|Pose |pose |Position and orientation of joint. |

### PartEntry

|Type  |Name  |Description |
|--|--|--|
|size_t |nameSize |Size of name string. |
|String |name |Unique name for the part. |
|Pose |pose |Position and orientation of part. |
|size_t |jointReferenceListSize |Size of the joint reference list. |
|uint32_t[] |jointReferenceList |List of ids that correspond to the joints the part is linked to. |
|size_t |rigidGroupReferenceListSize |Size of the rigid group reference list. |
|uint32_t |rigidGroupReferenceList |List of ids that correspond to the rigid groups the part is a member of. |
|size_t |bodyListSize |Size of body list. |
|BodyEntry[] |bodyList |List of all bodies in part. |

### BodyEntry

|Type  |Name  |Description |
|--|--|--|
|int32_t |triangleCount |Number of all triangles in body mesh. |
|float[] |verticies |List of verticies describing the mesh of the body. |
|int[] |indices |List of indices for verticies. |
|float[] |normals |List of normals. |
|float[] |uvs |List of UV coordinates for texture mapping. |

### Pose
|Type  |Name  |Description |
|--|--|--|
|dvec3 |translation |Translation vector of the pose. |
|dquat |orientation |Orientation represented as a quaternion. |
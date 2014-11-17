import os
import BVHToolkit
from cgkit.bvh import Node
from BVHToolkit import Animation, Pose, Bone

def listAll():
    #"""Load BVH and get node positions."""
    bvh_serial_path = os.path.join(os.path.dirname(__file__), "Example1.bvh")
    bvh_serial = {"root": None, "frames": None}
    reader = BVHToolkit.BVHAnimationReader(bvh_serial_path)
    reader.read()
    bvh_serial["root"] = reader.root
    bvh_serial["frames"] = reader.frames
    #bone = BVHToolkit.Bone(bvh_serial["root"])
    #for i in range(len(bone.node_list)):
        #print bone.node_list[i].name
        #print bone.get_param_offset(i)

    anim = BVHToolkit.Animation(BVHToolkit.Bone(bvh_serial["root"]))
    for i in range(len(reader.animation.frames)):
        anim.add_frame(bvh_serial["frames"][i])

    for j in range(len(anim.frames)):
        print "pose " + str(j) + ":"
        pose = anim.get_pose(j)
        for i in range(len(pose.positions)):
            print str(pose.bone.node_list[i].name) + ": " + str(pose.get_position(i))

def getPosesX(node):
    result={}
    #"""Load BVH and get node positions."""
    bvh_serial_path = os.path.join(os.path.dirname(__file__), "Example1.bvh")
    bvh_serial = {"root": None, "frames": None}
    reader = BVHToolkit.BVHAnimationReader(bvh_serial_path)
    reader.read()
    bvh_serial["root"] = reader.root
    bvh_serial["frames"] = reader.frames
    #bone = BVHToolkit.Bone(bvh_serial["root"])
    #for i in range(len(bone.node_list)):
        #print bone.node_list[i].name
        #print bone.get_param_offset(i)

    anim = BVHToolkit.Animation(BVHToolkit.Bone(bvh_serial["root"]))
    for i in range(len(reader.animation.frames)):
        anim.add_frame(bvh_serial["frames"][i])

    for j in range(len(anim.frames)):
        #print "pose " + str(j) + ":"
        pose = anim.get_pose(j)
        #print str(pose.bone.node_list[node].name) + ": " + str(pose.get_position(node)[0])
        result[j]=pose.get_position(node)[0]
    return result

def getPosesY(node):
    result={}
    #"""Load BVH and get node positions."""
    bvh_serial_path = os.path.join(os.path.dirname(__file__), "Example1.bvh")
    bvh_serial = {"root": None, "frames": None}
    reader = BVHToolkit.BVHAnimationReader(bvh_serial_path)
    reader.read()
    bvh_serial["root"] = reader.root
    bvh_serial["frames"] = reader.frames
    #bone = BVHToolkit.Bone(bvh_serial["root"])
    #for i in range(len(bone.node_list)):
        #print bone.node_list[i].name
        #print bone.get_param_offset(i)

    anim = BVHToolkit.Animation(BVHToolkit.Bone(bvh_serial["root"]))
    for i in range(len(reader.animation.frames)):
        anim.add_frame(bvh_serial["frames"][i])

    for j in range(len(anim.frames)):
        #print "pose " + str(j) + ":"
        pose = anim.get_pose(j)
        #print str(pose.bone.node_list[node].name) + ": " + str(pose.get_position(node)[0])
        result[j]=pose.get_position(node)[1]
    return result

def getPosesZ(node):
    result={}
    #"""Load BVH and get node positions."""
    bvh_serial_path = os.path.join(os.path.dirname(__file__), "Example1.bvh")
    bvh_serial = {"root": None, "frames": None}
    reader = BVHToolkit.BVHAnimationReader(bvh_serial_path)
    reader.read()
    bvh_serial["root"] = reader.root
    bvh_serial["frames"] = reader.frames
    #bone = BVHToolkit.Bone(bvh_serial["root"])
    #for i in range(len(bone.node_list)):
        #print bone.node_list[i].name
        #print bone.get_param_offset(i)

    anim = BVHToolkit.Animation(BVHToolkit.Bone(bvh_serial["root"]))
    for i in range(len(reader.animation.frames)):
        anim.add_frame(bvh_serial["frames"][i])

    for j in range(len(anim.frames)):
        #print "pose " + str(j) + ":"
        pose = anim.get_pose(j)
        #print str(pose.bone.node_list[node].name) + ": " + str(pose.get_position(node)[0])
        result[j]=pose.get_position(node)[2]
    return result

u = getPosesX(0)
v = getPosesY(0)
z = getPosesZ(0)
print u
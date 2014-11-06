import os
import BVHToolkit
from cgkit.bvh import Node
from BVHToolkit import Animation, Pose, Bone

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

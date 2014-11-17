from BVHToolkit import Animation, Pose, Bone
from cgkit.bvh import Node
import os

#"""Load BVH and get node positions."""
bvh_serial_path = os.path.join(os.path.dirname(__file__), "Example1.bvh")
animation = Animation.from_bvh(bvh_serial_path)
pose = animation.get_pose(0)    # get pose at third frame
print str(pose.bone.node_list[0].name) + ": " + str(pose.get_position(0)) # get position of the root node
print str(pose.bone.node_list[1].name) + ": " + str(pose.get_position(1)) # get position of the 1th node


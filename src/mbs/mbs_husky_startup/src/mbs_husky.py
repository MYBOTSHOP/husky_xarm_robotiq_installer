import rospy, roslaunch, os, rospkg


husky_base = os.path.join(rospkg.RosPack().get_path("mbs_husky_startup"), 'launch', 'husky_base.launch')


rospy.init_node('mbs_husky_system_launch', anonymous=False)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

husky_base = ['mbs_husky_startup', 'husky_base.launch']


roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(husky_base)

launch_files = [(roslaunch_file, [])]

parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

parent.start()
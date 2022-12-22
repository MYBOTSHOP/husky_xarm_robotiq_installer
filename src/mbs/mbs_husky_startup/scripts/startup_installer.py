#!/usr/bin/env python3

import os
import robot_upstart
import rospkg

job_name = "mbs_husky"

os.system("sudo service {} stop".format(job_name))

uninstall_job = robot_upstart.Job(name=job_name, rosdistro=os.environ['ROS_DISTRO'])
uninstall_job.uninstall()

main_job = robot_upstart.Job(name=job_name, 
                             user='root',
                            #  interface="enp2s0",
                             master_uri=os.environ['ROS_MASTER_URI'],
                             rosdistro=os.environ['ROS_DISTRO'],
                             workspace_setup=os.path.join(rospkg.RosPack().get_path('mbs_husky_startup'), 'config', 'setup.bash'))
main_job.add(package="mbs_husky_startup", filename="launch/mbs_husky.launch")
main_job.install()

os.system("sudo systemctl daemon-reload && sudo systemctl start {}".format(job_name))

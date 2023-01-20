#sdkeli_klm_udp
####################
#### HOW TO RUN ####
# 1: build:
	catkin_make
# 2: install:
	catkin_make install
# 3.1: run with node:
	roslaunch sdkeli_klm_udp sdkeli_klm_udp.launch
# 3.2: run with nodelet:
	roslaunch sdkeli_klm_udp sdkeli_klm_udp_nodelet.launch
#       NOTE:keli_laser_manager nodelet will be launch by default, you can specify 
#	     the nodelet manager what you want by setting arguments "bringup_self_manager" and
#	     "manager_name" in the "sdkeli_klm_tcp_nodelet.launch"

####################
#### HOW TO VIEW FROM RVIZ ###
# 1: Run the package as introduced .
# 2: Start another terminal.
# 3: run rviz:
	rosrun rviz rviz
# 4: In the popup window, click [Add] on the left-bottum
# 5: In the popup window, select  [By topic] tab.
# 6: Select [LaserScan], then [OK]
# 7: Back to the main window, in the left operation panel:
     [Displays]->[Global Options]->[Fixed Frame]->Change the value of this item to [laser]
# 8: Then, you should be able to see the result in display window.

#################### 
#### HOW TO DISABLE DEBUG MODE ####
# 1: In cfg/SDKeliKlm.cfg
# 2: Set the default value of debug_mode to False

# HOW TO DISABLE CHECK FRAME MODE
# 1: In launch/sdkeli_klm_udp.launch or launch/sdkeli_klm_udp_nodelet.launch
# 2: Set the checkframe = false in xml 

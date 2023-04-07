from std_msgs.msg import String, Empty, Bool

from bebop_msgs.msg import Ardrone3PilotingStateAlertStateChanged
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged

from geometry_msgs.msg import Twist, Point, Pose
import roslib
import os
import rospy
import smach
import smach_ros
from std_msgs.msg import String, Bool, UInt8, Float32

from cv_bridge import CvBridge, CvBridgeError

pub_state = rospy.Publisher("/state_machine/state",
                            String, queue_size=1)

import rospkg
import json
import ros_numpy
import numpy as np
rospack = rospkg.RosPack()
positions_path = str(rospack.get_path('drone_control')+'/config/recorded_routines.json')

with open(positions_path, 'r') as json_data_file:
    moving_routines = json.load(json_data_file)
    

class takeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'error'])    
        self.takeoff_topic = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
        self.camera_angle_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)
        self.running_aligned_pub = rospy.Publisher("/control/align_reference/set_running_state", Bool, queue_size=1)
        self.angle_msg = Twist()
        self.angle_msg.angular.y = 3
        self.z = 0
    def altitude_callback(self, data):
        self.z = data.altitude
        print(data.altitude)

    def execute(self, userdata):
        altitude_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",Ardrone3PilotingStateAltitudeChanged, self.altitude_callback)
        rospy.loginfo('Executing state takeoff')
        for i in range(10):
            self.camera_angle_pub.publish(self.angle_msg)
            self.running_aligned_pub.publish(False)
            rospy.sleep(0.05)
        rospy.loginfo('Camera Alined')

        for i in range(10):
            self.takeoff_topic.publish(Empty())
            rospy.sleep(0.1)
        
        t = rospy.Time.now()
        while self.z < 0.8 and not rospy.is_shutdown() and (rospy.Time.now()-t).to_sec()<5.0:
            rospy.sleep(0.1)
        # if (rospy.Time.now()-t).to_sec()<5.0:
        #     return 'error'
        
        # rospy.sleep(1)
        
        altitude_sub.unregister()
        return 'done'


class land_now(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

        self.land_topic = rospy.Publisher("/bebop/land", Empty,queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('land')

        for i in range(10):
            self.land_topic.publish(Empty())
            rospy.sleep(0.1)
        rospy.sleep(1)
        # self.condition = ""
        return 'done'


# ======================== Flag ================================
class align_flag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['flag_aligned','reference_lost', 'too_many_attempts'])
        self.camera_topic = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)

        self.running_feature_pub= rospy.Publisher("cv_detection/feature_detector/set_running_state", Bool, queue_size=1)
        self.running_rect_pub= rospy.Publisher("/cv_detection/rectangle_detector/set_running_state", Bool,queue_size=1)
        self.ref_image_pub= rospy.Publisher("cv_detection/feature_detector/set_ref_image", String,queue_size=1)

        self.camera_angle_pub= rospy.Publisher("/control/align_reference/set_camera_angle", Float32,queue_size=1)
        self.speed_pub= rospy.Publisher("control/align_reference/set_speed", Float32,queue_size=1)
        self.precision_pub= rospy.Publisher("/control/align_reference/set_precision", Point,queue_size=1)
        self.pid_config_pub= rospy.Publisher("/control/align_reference/set_pid_config", String,queue_size=1)
        self.goal_point_pub = rospy.Publisher("/control/align_reference/set_goal_point", Point, queue_size=1)
        self.running_aligned_pub = rospy.Publisher("/control/align_reference/set_running_state", Bool, queue_size=1)
        
    def execute(self, userdata):
        # rospy.sleep(3)
        camera_init_angle = Twist()
        camera_init_angle.angular.y = 3 # looking down
        for i in range(1): # to make sure that the camera is on the right position
            self.camera_topic.publish(camera_init_angle)
            rospy.sleep(0.1)

        # self.color_pub.publish("blue")
        self.ref_image_pub.publish("flag2.png")

        p = Point()
        p.x, p.y, p.z = [30,30,20]
        self.precision_pub.publish(p)
        self.camera_angle_pub.publish(0)
        self.speed_pub.publish(0.5)
        self.pid_config_pub.publish("flag") #changes the pid values
        # self.running_feature_pub.publish(True)
        self.running_rect_pub.publish(True)

        p = Point()
        p.x = 856/2
        p.y = 480/2
        p.z = 160
        self.goal_point_pub.publish(p)
        self.running_aligned_pub.publish(True)
        print("ok")

        rospy.wait_for_message("/control/align_reference/aligned", Bool)
        rospy.loginfo("ALIGNED")
        # rospy.sleep(1)
        # self.running_feature_pub.publish(False)
        self.running_rect_pub.publish(False)
        self.running_aligned_pub.publish(False)
        return 'flag_aligned'

class face_flag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','error'])
        self.camera_topic = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)

        self.running_feature_pub= rospy.Publisher("cv_detector/feature_detector/set_running_state", Bool, queue_size=1)
        self.ref_image_pub= rospy.Publisher("cv_detector/feature_detector/set_ref_image", String,queue_size=1)

        self.pose_pub = rospy.Publisher("/control/position", Pose, queue_size=1)
        self.running_control_pub = rospy.Publisher("/control/set_running_state", Bool, queue_size=1)
        
    def execute(self, userdata):
        # rospy.sleep(3)
        camera_init_angle = Twist()
        camera_init_angle.angular.y = 3 # looking down
        for i in range(1): # to make sure that the camera is on the right position
            self.camera_topic.publish(camera_init_angle)
            rospy.sleep(0.1)

        # self.color_pub.publish("blue")
        self.ref_image_pub.publish("flag.png")
        self.running_feature_pub.publish(True)

        # p = Point()
        # p.x, p.y, p.z = [30,30,20]
        # self.precision_pub.publish(p)

        #check location of the flag
        
        routine_name = 'face_all_flags'

        # if routine_name in moving_routines.keys():
        #     new_pose = ros_numpy.msgify(Pose,np.array(moving_routines[routine_name][0]))
        #     print(new_pose)
        #     self.pose_pub.publish(new_pose)
        #     self.running_control_pub.publish(True)
        #     rospy.wait_for_message("/control/aligned", Bool)
        # else:
        #     print("no routine named: "+routine_name)
        #     print(moving_routines.keys())
        #     return 'error'

        p = rospy.wait_for_message("/cv_detection/feature_detector/features_center", Point)
        if p.x < 250:
            routine_name = 'flag1'
        elif p.x < 600:
            routine_name = 'flag2'
        else:
            routine_name = 'flag3'
        print(routine_name)

        # if routine_name in moving_routines.keys():
        #     new_pose = ros_numpy.msgify(Pose,np.array(moving_routines[routine_name][0]))
        #     print(new_pose)
        #     self.pose_pub.publish(new_pose)
        #     self.running_control_pub.publish(True)
        #     rospy.wait_for_message("/control/aligned", Bool)
        # else:
        #     print("no routine named: "+routine_name)
        #     print(moving_routines.keys())
        #     return 'error'

        print("ok")


        rospy.wait_for_message("/control/align_reference/aligned", Bool)
        rospy.loginfo("ALIGNED")

        self.running_feature_pub.publish(False)
        self.running_control_pub.publish(False)
        return 'done'


class change_view_flag (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):

        return 'done'

class capture_flag (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','error'])

        self.running_rect_pub= rospy.Publisher("/cv_detection/rectangle_detector/set_running_state", Bool,queue_size=1)
        self.save_detection_pub = rospy.Publisher("cv_detection/rectangle_detector/save_detection", String, queue_size=1)
        self.save_image_raw_pub = rospy.Publisher("cv_detection/rectangle_detector/save_image_raw", String, queue_size=1)

    def execute(self, userdata):
        self.running_rect_pub.publish(True)

        count_erros = 0
        detection_saved = False
        while not detection_saved:
            self.save_image_raw_pub.publish('flag_raw.png')

            self.save_detection_pub.publish('flag.png')
            try:
                detection_saved = rospy.wait_for_message('cv_detection/rectangle_detector/detection_saved', Bool, 0.5)
            except:
                count_erros += 1
                print(count_erros)
                if count_erros >10:
                    return 'error'
        self.running_rect_pub.publish(False)
        return 'done'





class routine_interface(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['execute','land','custom', 'takeoff'], output_keys=['routine_name'])

        self.routine_name_sub = rospy.Subscriber("/traj_name", String, self.traj_name_callback, queue_size=1)
        self.execute_sub = rospy.Subscriber("/execute", Empty, self.execute_callback, queue_size=1)
        self.land_sub = rospy.Subscriber("/land", Empty, self.land_callback, queue_size=1)
        self.takeoff_sub = rospy.Subscriber("/takeoff", Empty, self.takeoff_callback, queue_size=1)
        self.custom_sub = rospy.Subscriber("/custom_action", String, self.custom_callback, queue_size=1)


        self.running_control_pub = rospy.Publisher("/control/set_running_state", Bool, queue_size=1)


        self.action = None

        self.routine_name = None

        # self.pose_pub = rospy.Publisher("/control/position", Pose, queue_size=1)
        # self.running_control_pub = rospy.Publisher("/control/set_running_state", Bool, queue_size=1)
        # self.running_control_pub = rospy.Publisher("/control/aligned", Bool, queue_size=1)
        # self.routine_name = routine_name

    def traj_name_callback(self, msg):
        self.routine_name = msg.data

    def land_callback(self,msg):
        self.action = "land"
    def takeoff_callback(self,msg):
        self.action = "takeoff"
    def execute_callback(self,msg):
        self.action = "execute"
    def custom_callback(self,msg):
        self.action = msg.data


    def execute(self, userdata):
        self.action = None
        self.running_control_pub.publish(True)

        while not rospy.is_shutdown() and self.action is None:
            pass
    
        userdata.routine_name = self.routine_name
        self.running_control_pub.publish(False)

        return self.action
        

class follow_routine (smach.State):
    def __init__(self, routine_name=None, interface=True):
        smach.State.__init__(self, outcomes=['done','error'],input_keys=['routine_name'])


        self.pose_pub = rospy.Publisher("/control/position", Pose, queue_size=1)
        self.running_control_pub = rospy.Publisher("/control/set_running_state", Bool, queue_size=1)
        # self.running_control_pub = rospy.Publisher("/control/aligned", Bool, queue_size=1)
        self.routine_name = routine_name

    def execute(self, userdata):

        self.routine_name = userdata.routine_name
        # rospy.sleep(2)
        self.running_control_pub.publish(True)
        # self.pose_pub.publish()

        if self.routine_name in moving_routines:
            for position in moving_routines[self.routine_name]:
                print("------------- pose ----------")
                new_pose = ros_numpy.msgify(Pose,np.array(position))
                print(new_pose)
                self.pose_pub.publish(new_pose)
                rospy.wait_for_message("/control/aligned", Bool)
                rospy.sleep(0.1)

        else:
            print("no routine named: "+self.routine_name)
            print(moving_routines)
            return 'error'

        self.running_control_pub.publish(False)
        return 'done'






class align_window(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.camera_topic = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)   
        # self.running_rect_pub= rospy.Publisher("/cv_detection/rectangle_detector/set_running_state", Bool,queue_size=1)
        self.running_color_pub= rospy.Publisher("cv_detection/color_range/set_running_state", Bool, queue_size=1)
        self.color_pub= rospy.Publisher("cv_detection/color_range/set_color", String,queue_size=1)

        self.camera_angle_pub= rospy.Publisher("/control/align_reference/set_camera_angle", Float32,queue_size=1)
        self.speed_pub= rospy.Publisher("control/align_reference/set_speed", Float32,queue_size=1)
        self.precision_pub= rospy.Publisher("/control/align_reference/set_precision", Point,queue_size=1)
        self.pid_config_pub= rospy.Publisher("/control/align_reference/set_pid_config", String,queue_size=1)
        self.goal_point_pub = rospy.Publisher("/control/align_reference/set_goal_point", Point, queue_size=1)
        self.running_aligned_pub = rospy.Publisher("/control/align_reference/set_running_state", Bool, queue_size=1)
    #     self.aligned_sub = rospy.Subscriber("/control/align_reference/aligned", Bool, self.aligned_callback, queue_size=1)

    # def aligned_callback()
    def execute(self, userdata):
        self.color_pub.publish("blue")
        print("ok")
        self.running_color_pub.publish(True)
        camera_init_angle = Twist()
        camera_init_angle.angular.y = 3 # looking down
        for i in range(1): # to make sure that the camera is on the right position
            self.camera_topic.publish(camera_init_angle)
            rospy.sleep(0.1)
        self.pid_config_pub.publish("default")
        self.camera_angle_pub.publish(0)

        self.color_pub.publish("blue")
        self.running_aligned_pub.publish(True)
        p = Point()
        p.x = 856/2
        p.y = 480/2
        p.z = 300
        self.goal_point_pub.publish(p)
        print("ok")


        rospy.wait_for_message("/control/align_reference/aligned", Bool)
        # rospy.sleep(1)
        # print(done)
        self.running_color_pub.publish(False)
        self.running_aligned_pub.publish(False)
        

        return 'done'

class pass_through_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

        self.pose_pub = rospy.Publisher("/control/position", Pose, queue_size=1)
        self.relative_pose_pub = rospy.Publisher("/control/position_relative", Pose, queue_size=1)
        self.running_control_pub = rospy.Publisher("/control/set_running_state", Bool, queue_size=1)
        # self.running_control_pub = rospy.Publisher("/control/aligned", Bool, queue_size=1)


    def execute(self, userdata):
        # rospy.sleep(2)
        current_pose = rospy.wait_for_message("control/current_position", Pose)
        print(current_pose)
        self.pose_pub.publish(current_pose) #sticks to current position
        self.running_control_pub.publish(True)
        rospy.sleep(0.1)
        p = Pose()
        p.position.z = 0.10
        p.orientation.w = 1
        self.relative_pose_pub.publish(p)
        rospy.sleep(0.3)
        rospy.wait_for_message("/control/aligned", Bool)
        p = Pose()
        p.position.x = 3
        p.orientation.w = 1
        self.relative_pose_pub.publish(p)
        rospy.sleep(0.1)
        rospy.wait_for_message("/control/aligned", Bool)
        rospy.sleep(2)
        self.running_control_pub.publish(False)
        return 'done'

# ================================= QR Code ===================================
class align_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['shelf_aligned','reference_lost', 'too_many_attempts'])

    def execute(self, userdata):

        rospy.sleep(1)
        return 'done'

class change_view_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):

        

        return 'done'

class face_box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.camera_topic = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)

        # self.running_rect_pub= rospy.Publisher("/cv_detection/rectangle_detector/set_running_state", Bool,queue_size=1)
        
        self.running_color_pub= rospy.Publisher("cv_detection/color_range/set_running_state", Bool, queue_size=1)
        self.color_pub= rospy.Publisher("cv_detection/color_range/set_color", String,queue_size=1)
        self.camera_angle_pub= rospy.Publisher("/control/align_reference/set_camera_angle", Float32,queue_size=1)
        self.speed_pub= rospy.Publisher("control/align_reference/set_speed", Float32,queue_size=1)

        self.precision_pub= rospy.Publisher("/control/align_reference/set_precision", Point,queue_size=1)
        self.pid_config_pub= rospy.Publisher("/control/align_reference/set_pid_config", String,queue_size=1)
        self.goal_point_pub = rospy.Publisher("/control/align_reference/set_goal_point", Point, queue_size=1)
        self.running_aligned_pub = rospy.Publisher("/control/align_reference/set_running_state", Bool, queue_size=1)
        
    def execute(self, userdata):

        camera_init_angle = Twist()
        camera_init_angle.angular.y = -100 # looking down
        for i in range(1): # to make sure that the camera is on the right position
            self.camera_topic.publish(camera_init_angle)
            rospy.sleep(0.1)

        p = Point()
        p.x, p.y, p.z = [20,20,7]
        self.precision_pub.publish(p)
        self.color_pub.publish("blue")
        self.camera_angle_pub.publish(1.57)
        self.speed_pub.publish(0.5)
        self.pid_config_pub.publish("box") #changes the pid values
        self.running_color_pub.publish(True)
        self.running_aligned_pub.publish(True)
        p = Point()
        p.x = 856/2
        p.y = 480/2-50
        p.z = 90
        self.goal_point_pub.publish(p)
        print("ok")

        rospy.wait_for_message("/control/align_reference/aligned", Bool)
        rospy.loginfo("ALIGNED")
        # rospy.sleep(1)
        # print(done)
        self.running_color_pub.publish(False)
        self.running_aligned_pub.publish(False)
        return 'done'

class pickup_box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.pickup_box_pub = rospy.Publisher("/control/pickup_box", Empty, queue_size=1)


    def execute(self, userdata):

        self.pickup_box_pub.publish(Empty())
        rospy.sleep(0.25)
        return 'done'

# ====================== QR code =================
class inventory(smach.State):
    def __init__(self,shelf = 1):
        smach.State.__init__(self, outcomes=['done','error'])

        self.pose_pub = rospy.Publisher("/control/position", Pose, queue_size=1)
        self.running_control_pub = rospy.Publisher("/control/set_running_state", Bool, queue_size=1)
        self.running_inventory_pub= rospy.Publisher("cv_detection/inventory/set_runnig_state", Bool, queue_size=1)

        
        self.read_tag_pub= rospy.Publisher("cv_detection/inventory/read_tag", Empty, queue_size=1)
        self.stop_reading_qr_pub= rospy.Publisher("cv_detection/inventory/stop_reading_qr", Empty, queue_size=1)
        self.running_color_pub= rospy.Publisher("cv_detection/color_range/set_running_state", Bool, queue_size=1)
        self.color_pub= rospy.Publisher("cv_detection/color_range/set_color", String,queue_size=1)

        self.color_ref_point_sub  = rospy.Subscriber("/control/align_reference/ref_point", Point, self.color_rect_callback)
        self.current_pose_sub = rospy.Subscriber("/odom_slam_sf/current_pose", Pose, self.pose_callback)

        self.shelf = shelf

        self.current_pose  = None

    def pose_callback(self,data):#pose
        self.current_pose = data

    def color_rect_callback(self,data):# Point
        global frame_coord
        # print("sdfhnsolg")
        print(data.z)
        if data.z > 300:
            print("detected")
            frame_pose = self.current_pose

    def exit(self):
        self.stop_reading_qr_pub.publish(Empty())            
        self.running_inventory_pub.publish(False)            
        self.running_control_pub.publish(False)
        self.running_color_pub.publish(False)

        # self.color_ref_point_sub.unregister()
        # self.current_pose_sub.unregister()

    def execute(self, userdata):

        

        self.running_inventory_pub.publish(True)
        self.running_control_pub.publish(True)


        self.running_color_pub.publish(True)
        self.color_pub.publish("yellow")

        routine_name = 'qr'+str(self.shelf)
        # routine_name = 'test'

        if routine_name in moving_routines:
            for i, position in enumerate(moving_routines[routine_name]):
                print("------------- pose ----------")

                self.stop_reading_qr_pub.publish(Empty())                        

                new_pose = ros_numpy.msgify(Pose,np.array(position))
                print(new_pose)
                self.pose_pub.publish(new_pose)
                rospy.wait_for_message("/control/aligned", Bool)
                self.read_tag_pub.publish(Empty())
                rospy.sleep(0.5)
                new_pose.position.z += 1.5
                new_pose = ros_numpy.msgify(Pose,np.array(position))
                print(new_pose)
                self.pose_pub.publish(new_pose)
                rospy.wait_for_message("/control/aligned", Bool)
                self.read_tag_pub.publish(Empty())
                rospy.sleep(0.5)
                new_pose.position.z += 1.3
                new_pose = ros_numpy.msgify(Pose,np.array(position))
                print(new_pose)
                self.pose_pub.publish(new_pose)
                rospy.wait_for_message("/control/aligned", Bool)

            self.exit()

            
            return 'done'
        else:
            print("no routine named: "+routine_name)
            print(moving_routines)
            
        self.exit()

        return 'error'


class change_view_qr_code(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):

        rospy.sleep(1)
        return 'done'




class drop_box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.vel_pub = rospy.Publisher("/control/velocity", Point, queue_size=10)

    def control(self,vec,t):
        p = Point()
        p.x, p.y, p.z = vec
        for i in range(int(t*50)):
            self.vel_pub.publish(p)
            rospy.sleep(0.02)

    def execute(self, userdata):
        rospy.loginfo('drop_box')
        self.control([0,0,-1],0.5)
        self.control([-0.5,0,1],0.8)
        self.control([-0.5,0,0],1.5)
        print("ok")
        self.control([0,0,0],0.6)
        return 'done'

class face_recharge_station(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.camera_topic = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)

        # self.running_rect_pub= rospy.Publisher("/cv_detection/rectangle_detector/set_running_state", Bool,queue_size=1)
        self.running_pub= rospy.Publisher("cv_detection/feature_detector/set_running_state", Bool, queue_size=1)
        self.ref_image_pub= rospy.Publisher("cv_detection/feature_detector/set_ref_image", String,queue_size=1)

        self.camera_angle_pub= rospy.Publisher("control/feature_detector/set_camera_angle", Float32,queue_size=1)
        self.precision_pub= rospy.Publisher("/control/align_reference/set_precision", Point,queue_size=1)
        self.speed_pub= rospy.Publisher("control/align_reference/set_speed", Float32,queue_size=1)
        self.pid_config_pub= rospy.Publisher("/control/align_reference/set_pid_config", String,queue_size=1)
        self.goal_point_pub = rospy.Publisher("/control/align_reference/set_goal_point", Point, queue_size=1)
        self.running_aligned_pub = rospy.Publisher("/control/align_reference/set_running_state", Bool, queue_size=1)
        
    def execute(self, userdata):
        # rospy.sleep(3)
        camera_init_angle = Twist()
        camera_init_angle.angular.y = -100 # looking down
        for i in range(1): # to make sure that the camera is on the right position
            self.camera_topic.publish(camera_init_angle)
            rospy.sleep(0.3)

        p = Point()
        p.x, p.y, p.z = [30,30,10]
        self.pid_config_pub.publish("box")
        self.precision_pub.publish(p)
        self.ref_image_pub.publish("landingpad.png")
        self.camera_angle_pub.publish(1.57)
        self.speed_pub.publish(0.5)
        self.running_pub.publish(True)
        self.running_aligned_pub.publish(True)
        p = Point()
        p.x = 856/2
        p.y = 480/2-50
        p.z = 230
        self.goal_point_pub.publish(p)
        print("ok")

        rospy.wait_for_message("/control/align_reference/aligned", Bool)
        rospy.loginfo("ALIGNED")
        # rospy.sleep(1)
        # print(done)
        self.running_pub.publish(False)
        self.running_aligned_pub.publish(False)
        return 'done'


# define state find_window

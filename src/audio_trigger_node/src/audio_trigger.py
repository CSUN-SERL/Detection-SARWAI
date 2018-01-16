import rospy
import yaml
import sys

from nav_msgs.msg import Odometry
from multiprocessing import Process
from audio_trigger_node.msg import Trigger


global trigger_list
trigger_list = {}

def main():

    class Struct:
        def __init__(self, **entries):
            self.__dict__.update(entries)
    # define the class for our location data
    class trigger_info(yaml.YAMLObject):
        yaml_tag = u'!Location'
        def __init__(self,x,y,width,height,audio_file):
            self.x = x
            self.y = y
            self.w = width
            self.h = height
            self.audio_path = audio_file

    # load our yaml file
    with open('../location_test.yaml', 'r') as file_stream:
        location_info_yaml = yaml.safe_load(file_stream)

    # populate location dictionary like so
    # location name : Location_data object

    for key,value in location_info_yaml["locations"].iteritems():
        print key
        trigger = trigger_info(**location_info_yaml["locations"][key])
        trigger_list[key] = trigger

    #subscribe to pose_mock
    #publish pose_mock
    p1 = Process(target = pose_mock)
    p1.start()
    p2 = Process(target = listen_for_pose)
    p2.start()


def trigger_callback(data):

    pub_trigger = rospy.Publisher('/audio_trigger',Trigger,queue_size )

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    for key,trigger in trigger_list.iteritems():
        if x >= trigger.x and y >= trigger.y and x <= trigger.x+trigger.w and y <= trigger.y+trigger.h:
            print "husky in box ", key
            msg = Trigger()
            msg.trigger_location = key
            msg.audio_file = trigger.audio_path

    #distance formula
    #for future implementation of volume as a function
    #of distace from center of trigger point
    msg.distance_from_center = pow(pow(x - (trigger.x+.5*(trigger.w)),2) + pow(y - (trigger.y+.5*(trigger.h)),2),.5)
    print msg.distance_from_center




#publishing node for testing
def pose_mock():
    pub_test = rospy.Publisher('/pose_mock',Odometry,queue_size = 1000)
    rospy.init_node('pose_mock')
    r = rospy.Rate(10)

    msg = Odometry()
    msg.pose.pose.position.x = 12.3
    msg.pose.pose.position.y = 23.2

    print "publisher start"
    while not rospy.is_shutdown():
        pub_test.publish(msg)
        r.sleep()


def listen_for_pose():
    rospy.init_node('listen_for_pose',anonymous = True)
    rospy.Subscriber('/pose_mock',Odometry,trigger_callback)

    rospy.spin()


if __name__ == "__main__":
    main()

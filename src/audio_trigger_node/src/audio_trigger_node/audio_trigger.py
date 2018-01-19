import rospy
import yaml
import sys

from nav_msgs.msg import Odometry
from multiprocessing import Process
from audio_trigger_node.msg import Trigger
from detection_msgs.msg import AudioDetection
import shout

# Topic used to send logging message
pubtopic = '/sarwai_detection/detection_audio'
pub = rospy.Publisher(pubtopic, AudioDetection, queue_size = 1000)

# found locations
activatedLocations = {}

#Set up icecast shout service
s = shout.Shout()
s.host = 'localhost'
s.port = 8050
s.password = 'SERLstream'
s.mount = '/BuddhaStream'
s.format = 'mp3'
s.protocol = 'http'
# s.audio_info = {
#   shout.SHOUT_AI_BITRATE : ,
#   shout.SHOUT_AI_SAMPLERATE : ,
#   shout.SHOUT_AI_CHANNELS : ,
#   shout.SHOUT_AI_QUALITY : 
# }

# s.open()

# Define function to loop background file to icecast
def bgLoop():
    global bg_playing
    bg_playing = True

    bgfilename = '/audio/audio_loop.mp3'
    bg = open(bgfilename)

    while True:
        if bg_playing:
            abuf = f.read(4096)
            if len(abuf) == 0:
                bg.close()
                bg = open(bgfilename)
                continue
            s.send(abuf)
            s.sync()



def main():
    rospy.init_node('listen_for_pose',anonymous = True)

    global trigger_list
    trigger_list = {}

    global query_list
    query_list = {}

    # define the class for our trigger location data
    class trigger_info(yaml.YAMLObject):
        yaml_tag = u'!Location'
        def __init__(self,x,y,width,height,query):
            self.x = x
            self.y = y
            self.w = width
            self.h = height
            self.q = query

    # load our yaml file

    with open('location_test.yaml', 'r') as file_stream:
        location_info_yaml = yaml.safe_load(file_stream)
    
    with open('premade_query.yaml', 'r') as file_stream:
        query_info_yaml = yaml.safe_load(file_stream)

    # populate location dictionary like so
    # location name : Location_data object

    for key,value in location_info_yaml["locations"].iteritems():
        print key
        trigger = trigger_info(**location_info_yaml["locations"][key])
        trigger_list[key] = trigger

    for key,value in query_info_yaml['queries'].iteritems():
        #query_list[key] = QueryData(**query_info_yaml['queries'][key])
        query_list[key] = value

    # Begin background audio stream
    # pbg = Process(target = bgLoop)
    # pbg.start()


    #IGNORING POSE FAKER FOR NOW
    #subscribe to pose_mock
    #publish pose_mock
#    p11 = Process(target = pose_mock(1))
#    p12 = Process(target = pose_mock(2))
#    p11.start()
#    p12.start()

    # Begin monitoring robot location
    p21 = Process(target = listen_for_pose, args = ('/robot1/odometry/filtered',0))
    p22 = Process(target = listen_for_pose, args = ('/robot2/odometry/filtered',1))
    p21.start()
    p22.start()



def trigger_callback(data):

    pub_trigger = rospy.Publisher('/audio_trigger',Trigger,queue_size=100 )

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
    pub_trigger.publish(msg)


    #stream audio to icecast to icecast




#publishing node for testing
def pose_mock(robot_num):

    pub_test = rospy.Publisher('/pose_mock_'+str(robot_num),Odometry,queue_size = 1000)
    rospy.init_node('pose_mock')
    r = rospy.Rate(10)

    msg = Odometry()
    msg.pose.pose.position.x = 10+robot_num
    msg.pose.pose.position.y = 20+robot_num

    print "publisher start"
    while not rospy.is_shutdown():
        pub_test.publish(msg)
        r.sleep()

def listen_for_pose(topic, robotId):
    while True:
        # get latest odometry message from specified topic
        msg = rospy.wait_for_message(topic, Odometry)

        #get x and y from message
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        currentzone = ''
        #check if odom x and y are in a trigger zone
        for trigger,tval in trigger_list.iteritems():
            if ( tval['x'] < pos[0] and
                 tval['y'] < pos[1] and
                 tval['x'] + tval['width'] > pos[0] and
                 tval['y'] + tval['height'] > pos[1]):
                currentzone = trigger
                break
        #if not, continue
        else:
            continue
        #check if location has been activated before
        if currentzone in activatedLocations:
            #if so, continue
            continue

        #mark location as activated
        activatedLocations[currentzone] = True

        #get query associated with location
        querynum = trigger_list[currentzone][query]
        query = querylist['query' + str(querynum)]

        #Cast audio file to icecast
        # TODO

        #publish query to topic for logging in audio logger
        audiomsg = AudioDetection()
        audiomsg.robotId = robotId
        audiomsg.confidence = query['confidence']
        audiomsg.filename = query['file_name']
        audiomsg.robotX = pose[0]
        audiomsg.robotY = pose[1]

        pub.publish(audiomsg)



if __name__ == "__main__":
    main()

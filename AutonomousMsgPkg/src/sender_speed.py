#! /usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32
import time

rospy.init_node('Sender',anonymous=False)
pub=rospy.Publisher('long_string',String,queue_size=1)
rate=rospy.Rate(1)

pub_size=1000000 #1Mbyte

data_cnt=0
start_time=time.time()

#while(1):
while rospy.is_shutdown()==False:
    pub_str="#" * pub_size + "/" + str(time.time())
    pub.publish(pub_str)
    data_cnt+=1
    duration=time.time()
    if duration >= 1*60:
        sending_speed=(pub_size/(10**6))*data_cnt/duration
        print("\n\n----------------[Sending REPORT]----------------")
        print("Total Sending Size: {0} Mb\nDuration : {1} sec\nData Cnt : {2}"\
            .format(pub_size*data_cnt/(10**6), round(duration,5), data_cnt))
        print("Sending Speed : %f Mbps" % round(sending_speed, 5))
        break
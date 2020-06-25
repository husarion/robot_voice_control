from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion, PoseStamped

def planar_to_posestamped(x=0., y=0., yaw=0., frame=""):
    posestamped = PoseStamped()
    posestamped.pose.position.x = x
    posestamped.pose.position.y = y          
    posestamped.header.frame_id = frame
    posestamped.pose.orientation = yaw_to_quat(yaw)

    return posestamped

def yaw_to_quat(yaw):
    q = quaternion_from_euler(0, 0, yaw)
    return Quaternion(q[0], q[1], q[2], q[3])  


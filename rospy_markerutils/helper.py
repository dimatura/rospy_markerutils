import rospy
from visualization_msgs.msg import Marker

from . import make_markers

class MarkerHelper(object):

    def __init__(self, topic, frame_id='/world'):
        self.pub = rospy.Publisher(topic, Marker)
        self.frame_id = frame_id


    def publish(self, marker):
        self.pub.publish(marker.to_msg(frame_id=self.frame_id))


    def wireframe_box(*args, **kwargs):
        marker = make_markers.WireframeBoxMarker(*args, **kwargs)
        self.publish(marker)

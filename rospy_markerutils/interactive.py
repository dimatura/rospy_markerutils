"""
Some code from wu_ros_tools/easy_markers.
"""

from collections import OrderedDict
import rospy

from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *

from . import make_markers


TYPEDATA = {
    'rotate_x': [1,1,0,0, InteractiveMarkerControl.ROTATE_AXIS],
    'move_x'  : [1,1,0,0, InteractiveMarkerControl.MOVE_AXIS],
    'rotate_z': [1,0,1,0, InteractiveMarkerControl.ROTATE_AXIS],
    'move_z'  : [1,0,1,0, InteractiveMarkerControl.MOVE_AXIS],
    'rotate_y': [1,0,0,1, InteractiveMarkerControl.ROTATE_AXIS],
    'move_y'  : [1,0,0,1, InteractiveMarkerControl.MOVE_AXIS]
}


class InteractiveMarkerManager(object):
    def __init__(self, name='interactive_markers', frame_id='/world'):
        self.server = InteractiveMarkerServer(name)
        self.frame_id = frame_id
        self.marker_ctr = 0
        self.markers = OrderedDict()

    def add_marker(self,
                   init_position,
                   callback,
                   controls = [],
                   scale=1.,
                   marker=None,
                   name=None,
                   fixed_orientation=False):
        if name is None:
            name = 'control%d'%(self.marker_ctr)
            self.marker_ctr += 1

        if marker is None:
            marker = make_markers.make_marker()
            marker.header.frame_id = self.frame_id
        marker.header.frame_id = self.frame_id

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.pose.position = init_position
        int_marker.scale = scale

        int_marker.name = name

        vm_control = InteractiveMarkerControl()
        vm_control.always_visible = True
        vm_control.markers.append(marker)
        int_marker.controls.append(vm_control)
        #int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

        for control_name in controls:
            data = TYPEDATA[control_name]
            control = InteractiveMarkerControl()
            control.orientation.w = data[0]
            control.orientation.x = data[1]
            control.orientation.y = data[2]
            control.orientation.z = data[3]
            control.name = control_name
            control.interaction_mode = data[4]
            if fixed_orientation:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

        self.server.insert(int_marker, callback)
        self.markers[name] = int_marker
        #self.server.applyChanges()

    def apply_changes(self):
        self.server.applyChanges()


def make_6dof_marker(init_position, frame_id='/world', fixed_orientation=True, viz_marker=None):
    """ make marker. remember to set header!
    visible_marker is a marker that has the same pose and is useful
    for visualization.
    """

    # some reasonable defaults
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.pose.position = init_position
    int_marker.scale = 1.

    int_marker.name = 'simple_6dof'
    int_marker.description = 'simple_6dof'

    # add the visible_marker
    if viz_marker is not None:
        vm_control = InteractiveMarkerControl()
        vm_control.always_visible = True
        vm_control.markers.append(viz_marker)
        int_marker.controls.append(vm_control)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

    if fixed_orientation:
        int_marker.name += "_fixed"
        int_marker.description += "_fixed"
    int_marker.name += "_MOVE_ROTATE_3D"

    # add visible 6dof thing
    def make_rotate_axis_fixed(axis_name, orientation):
        controlR = InteractiveMarkerControl()
        controlR.orientation.w = orientation[0]
        controlR.orientation.x = orientation[1]
        controlR.orientation.y = orientation[2]
        controlR.orientation.z = orientation[3]
        controlR.name = "rotate_"+axis_name
        controlR.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed_orientation:
            controlR.orientation_mode = InteractiveMarkerControl.FIXED
        controlM = InteractiveMarkerControl()
        controlM.orientation.w = orientation[0]
        controlM.orientation.x = orientation[1]
        controlM.orientation.y = orientation[2]
        controlM.orientation.z = orientation[3]
        controlM.name = "move_"+axis_name
        controlM.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        controlM.orientation_mode = InteractiveMarkerControl.FIXED
        return (controlR, controlM)

    cR, cM = make_rotate_axis_fixed('x', (1, 1, 0, 0))
    int_marker.controls.append(cR)
    int_marker.controls.append(cM)

    # yeah z not y?
    # also why is orientation not normalized
    cR, cM = make_rotate_axis_fixed('z', (1, 0, 1, 0))
    int_marker.controls.append(cR)
    int_marker.controls.append(cM)

    cR, cM = make_rotate_axis_fixed('y', (1, 0, 0, 1))
    int_marker.controls.append(cR)
    int_marker.controls.append(cM)
    return int_marker

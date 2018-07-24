#
# @author  Daniel Maturana
# @year    2015
#
# @attention Copyright (c) 2015
# @attention Carnegie Mellon University
# @attention All rights reserved.
#
# @=


import numpy as np

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import rospy_geomutils


class MarkerWrapper(object):
    """ Base object to generate marker messages.
    Use .to_msg() to get the marker message.
    """

    def __init__(self):
        self.marker = Marker()
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD

        self.quat = rospy_geomutils.Quaternion.identity()
        self.tvec = np.array([0., 0., 0.])
        self.pose = rospy_geomutils.RigidTransform(self.quat, self.tvec)
        self.marker.pose = self.pose.to_pose_msg()

        self.marker.scale.x = 1.
        self.marker.scale.y = 1.
        self.marker.scale.z = 1.

        self.marker.ns = 'marker'
        self.marker.id = 1
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

    def set_color_int(self, rgb):
        rgb = np.asarray(rgb)
        if len(rgb) != 3:
            raise ValueError('RGB should have three elements')
        if rgb.dtype.kind not in 'ui':
            raise ValueError('RGB should be integral')
        self.marker.color.r = rgb[0]/255.
        self.marker.color.g = rgb[1]/255.
        self.marker.color.b = rgb[2]/255.

    def set_color_float(self, rgb):
        rgb = np.asarray(rgb)
        if len(rgb) != 3:
            raise ValueError('RGB should have three elements')
        if rgb.dtype.kind != 'f':
            raise ValueError('RGB should be floating point')
        self.marker.color.r = rgb[0]
        self.marker.color.g = rgb[1]
        self.marker.color.b = rgb[2]

    def set_color_hex(self, hexstr):
        hexstr = hexstr.replace('#', '')
        hex_to_float = lambda x: float(int(x, base=16))/255.
        self.marker.color.r = hex_to_float(hexstr[0:2])
        self.marker.color.g = hex_to_float(hexstr[2:4])
        self.marker.color.b = hex_to_float(hexstr[4:6])

    def set_alpha_float(self, alpha):
        self.marker.color.a = alpha

    def set_frame_id(self, frame_id):
        self.marker.header.frame_id = frame_id

    def set_scale(self, scale):
        try:
            self.marker.scale.x = scale[0]
            self.marker.scale.y = scale[1]
            self.marker.scale.z = scale[2]
        except TypeError:
            self.marker.scale.x = scale
            self.marker.scale.y = scale
            self.marker.scale.z = scale

    def set_id(self, id):
        self.marker.id = id

    def set_ns(self, ns):
        self.marker.ns = ns

    def set_quat(self, quat):
        self.quat = quat
        self.pose = rospy_geomutils.RigidTransform(self.quat, self.tvec)
        self.marker.pose = self.pose.to_pose_msg()

    def set_translation(self, tvec):
        self.tvec = np.asarray(tvec)
        self.pose = rospy_geomutils.RigidTransform(self.quat, self.tvec)
        self.marker.pose = self.pose.to_pose_msg()

    def to_msg(self, stamp=None, frame_id=None):
        if stamp is not None:
            self.marker.header.stamp = stamp
        if frame_id is not None:
            self.marker.header.frame_id = frame_id
        return self.marker


class BoxMarker(MarkerWrapper):
    def __init__(self, box_dim):
        super(BoxMarker, self).__init__()
        self.marker.scale.x = box_dim[0]
        self.marker.scale.y = box_dim[1]
        self.marker.scale.z = box_dim[2]


class RectangleMarker(MarkerWrapper):
    """ internally, a triangle list."""

    def __init__(self, dim):
        super(RectangleMarker, self).__init__()

        self.marker.type = Marker.TRIANGLE_LIST

        half_len = dim[0]*.5
        half_breadth = dim[1]*.5

        corners = []
        corners.append([ half_len,  half_breadth, 0.])
        corners.append([ half_len, -half_breadth, 0.])
        corners.append([-half_len, -half_breadth, 0.])
        corners.append([-half_len,  half_breadth, 0.])

        points = [Point(x=c[0], y=c[1], z=c[2]) for c in corners]
        self.marker.points.append(points[0])
        self.marker.points.append(points[1])
        self.marker.points.append(points[2])
        self.marker.points.append(points[0])
        self.marker.points.append(points[2])
        self.marker.points.append(points[3])
        self.set_rgb_int([0x52, 0x19, 0])


class WireframeBoxMarker(MarkerWrapper):
    def __init__(self, box_dim, scale=0.25):
        """ TODO use dim initialization?
        """
        super(WireframeBoxMarker, self).__init__()
        self.marker.type = Marker.LINE_LIST
        self.set_scale(scale)
        self.set_color_float((0.5, 1.0, 0.2))
        box_dim = np.asarray(box_dim)
        min_pt, max_pt = -box_dim*.5, box_dim*.5
        self.marker.points = _bounding_box_to_lines(min_pt, max_pt)

    @staticmethod
    def from_min_max_pt(min_pt, max_pt):
        min_pt, max_pt = np.asarray(min_pt), np.asarray(max_pt)
        dim = max_pt-min_pt
        center = (max_pt+min_pt)*.5
        marker = WireframeBoxMarker(dim)
        marker.set_translation(center)
        return marker


class ArrowMarker(MarkerWrapper):
    def __init__(self, start_pt, end_pt):
        super(ArrowMarker, self).__init__()
        self.marker.type = Marker.ARROW
        self.start_pt = start_pt
        self.end_pt = end_pt
        self.set_dimensions(0.05, 0.10, 0.10)
        self.marker.points = []
        self.marker.points.append(Point(x=start_pt[0], y=start_pt[1], z=start_pt[2]))
        self.marker.points.append(Point(x=end_pt[0], y=end_pt[1], z=end_pt[2]))

    def set_dimensions(self, shaft_diam, head_diam, head_len):
        self.marker.scale.x = shaft_diam
        self.marker.scale.y = head_diam
        self.marker.scale.z = head_len

    @staticmethod
    def from_base_and_direction(base, direction):
        start_pt = np.asarray(base)
        end_pt = base+np.asarray(direction)
        return ArrowMarker(start_pt, end_pt)


class PathMarker(MarkerWrapper):
    def __init__(self, points):
        super(PathMarker, self).__init__()
        self.marker.type = Marker.LINE_LIST
        self.marker.points = []
        for ix in xrange(len(points)-1):
            self.marker.points.append(Point(x=points[ix][0], y=points[ix][1], z=points[ix][2]))
            self.marker.points.append(Point(x=points[ix+1][0], y=points[ix+1][1], z=points[ix+1][2]))
        self.marker.scale.x = 0.2


class AxesMarker(MarkerWrapper):
    def __init__(self, T, axis_size=1.0):
        """ T is 4x4 or 3x4 matrix representing a pose A in
        some external frame frame_id """
        super(AxesMarker, self).__init__()
        self.axis_size = axis_size
        self.T = T

        red = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        green = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        blue = ColorRGBA(0.0, 0.0, 1.0, 1.0)
        self.marker.type = Marker.LINE_LIST

        origin = T[:3,3]
        porigin = Point(*origin)
        xaxis = Point(*(origin + T[:3, 0] * axis_size))
        yaxis = Point(*(origin + T[:3, 1] * axis_size))
        zaxis = Point(*(origin + T[:3, 2] * axis_size))
        self.marker.points.append(porigin)
        self.marker.points.append(xaxis)
        self.marker.colors.append(red)
        self.marker.colors.append(red)

        self.marker.points.append(porigin)
        self.marker.points.append(yaxis)
        self.marker.colors.append(green)
        self.marker.colors.append(green)

        self.marker.points.append(porigin)
        self.marker.points.append(zaxis)
        self.marker.colors.append(blue)
        self.marker.colors.append(blue)


class TextMarker(MarkerWrapper):
    def __init__(self, text, scale):
        super(TextMarker, self).__init__()
        self.marker.type = Marker.TEXT_VIEW_FACING
        self.marker.text = text
        self.marker.scale.z = scale  # height of an uppercase A
        # TODO jsk overlay text


def _bounding_box_to_lines(min_pt, max_pt):
    """
    given a bounding box represented as two corners,
    calculate the lines representing the edges of this box.
    each line is represented as a pair of points in bb.
    """
    minmax = np.zeros((3, 2))
    minmax[:, 0] = min_pt
    minmax[:, 1] = max_pt
    seq = [
      0,0,0,
      0,1,0,
      0,1,0,
      1,1,0,
      1,1,0,
      1,0,0,
      1,0,0,
      1,0,1,
      1,0,1,
      1,1,1,
      1,1,1,
      0,1,1,
      0,1,1,
      0,0,1,
      0,0,1,
      0,0,0,
      0,1,0,
      0,1,1,
      1,1,1,
      1,1,0,
      0,0,1,
      1,0,1,
      1,0,0,
      0,0,0
    ]

    bb = []
    for row in xrange(24):
        i = seq[3*row + 0]
        j = seq[3*row + 1]
        k = seq[3*row + 2]

        pm = Point()
        pm.x = minmax[0, i]
        pm.y = minmax[1, j]
        pm.z = minmax[2, k]
        bb.append(pm)
    return bb


def make_marker(stamp=0, frame_id=''):
    """ just a reasonable default marker
    """
    return MarkerWrapper().to_msg(stamp, frame_id)


def make_box_marker(box_center, box_dim):
    """ a reasonable default box marker.
    """
    marker = BoxMarker(box_dim)
    marker.set_translation(box_center)
    marker.set_rgb_float([1., 0., 0.])
    marker.set_alpha(0.28)
    return marker.to_msg()

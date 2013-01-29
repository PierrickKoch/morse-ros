#!/usr/bin/env python
"""
usage:
rosrun octomap_morse map.py cmd:=/robot/motion map:=/explore/map
"""

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('nav_msgs')
roslib.load_manifest('geometry_msgs')
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
import wx
import sys
import threading
import array

def known_perimeter(data, width, height):
    """ get the perimeter of the known area from the occupancy grid

    :param data: The map data, in row-major order, starting with (0,0).
                 Occupancy probabilities are in the range [0,100].
                 Unknown is -1.
    :param width: grid width
    :param height: grid height
    :returns: list of point [(x1,y1), (x2,y2), ...]
    """
    p = []
    def is_unknown_with_known_neighbour(d,w,x,y):
        pos = y*w + x
        pos_above = pos - w # (y-1)*w + x
        pos_below = pos + w # (y+1)*w + x
        return d[pos] == -1 and ( d[pos-1] != -1 or d[pos+1] != -1 or 
          d[pos_above-1] != -1 or d[pos_above] != -1 or d[pos_above+1] != -1 or 
          d[pos_below-1] != -1 or d[pos_below] != -1 or d[pos_below+1] != -1)
    # ignore the main border (1 px)
    for x in xrange(1, width-1):
        for y in xrange(1, height-1):
            if is_unknown_with_known_neighbour(data, width, x, y):
                p.append((x, y))
    return p

def dump(data, width, height):
    buf = ""
    for y in range(height):
        for x in range(width):
            value = data[y*width+x]
            if   value == -1: buf += '?'
            elif value >=  0: buf += '%i'%value if value < 9 else 'P'
            elif value == -2: buf += '*'
        buf += '\n'
    return buf

class TwistPublisher(threading.Thread):
    """ ROS Twist (v,w) Publisher
    http://www.ros.org/doc/api/geometry_msgs/html/msg/Twist.html
    """
    def __init__(self):
        threading.Thread.__init__(self)
        self._cmd = Twist()
        self._cmd_linear_released = False
        self._cmd_angular_released = False
        self.cont = True
    def run(self):
        def slowing(vel, coef=0.8, limit=0.01):
            return 0.0 if abs(vel) < limit else vel * coef
        publisher = rospy.Publisher('/cmd', Twist)
        while not rospy.is_shutdown() and self.cont:
            publisher.publish(self._cmd)
            if self._cmd_linear_released:
                self._cmd.linear.x = slowing(self._cmd.linear.x)
            if self._cmd_angular_released:
                self._cmd.angular.z = slowing(self._cmd.angular.z)
            rospy.sleep(.1)

    def cmd_forward(self, released=False):
        self._cmd_linear_released = released
        self._cmd.linear.x = 1 # forward
    def cmd_backward(self, released=False):
        self._cmd_linear_released = released
        self._cmd.linear.x = -1 # backward
    def cmd_left(self, released=False):
        self._cmd_angular_released = released
        self._cmd.angular.z = 1 # turn left
    def cmd_right(self, released=False):
        self._cmd_angular_released = released
        self._cmd.angular.z = -1 # turn right
    def cmd_stop(self, unused=False):
        self._cmd.linear.x = 0
        self._cmd.angular.z = 0

class ImageViewPanel(wx.Panel):
    """ class ImageViewPanel creates a panel with an image on it, inherits wx.Panel 
    http://ros.org/doc/api/nav_msgs/html/msg/OccupancyGrid.html
    """
    display = None
    def update(self, occupancy_grid):
        w = occupancy_grid.info.width
        h = occupancy_grid.info.height
        d = occupancy_grid.data
        if not self.display:
            if w > h:
                self._width = 512
                self._height = 512 * h / w
            else:
                self._height = 512
                self._width = 512 * w / h
            self.display = wx.StaticBitmap(self)
            frame = self.GetParent()
            frame.SetSize((self._width, self._height))
        # from list of point (representation from 2D space) to Bitmap RGB
        bmp = wx.BitmapFromBuffer(w, h, self.dataToRGB(d))
        # scale the image to max 512
        img = bmp.ConvertToImage()
        # frontier
        for (x,y) in known_perimeter(d,w,h):
            img.SetRGB(x, y, 255, 0, 0) 
        img.Rescale(self._width, self._height)
        # display the image in our Panel
        self.display.SetBitmap(wx.BitmapFromImage(img))
    def dataToRGB(self, data):
        ar = array.array('b')
        for pix in data:
            ar.extend([0, 0, pix])
        return ar

class KeyEventFrame(wx.Frame):
    def __init__(self, parent = None, id = -1, title = __file__):
        wx.Frame.__init__(self, parent, id, title)
        self.mapKeyFn = {}

        self.panel = ImageViewPanel(self)
        self.panel.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)
        self.panel.Bind(wx.EVT_KEY_UP, self.OnKeyUp)
        self.panel.SetFocus()
        self.publisher = TwistPublisher()
        self.DoBindKeys()

        self.Centre()
        self.Show()
        rospy.init_node('wxKeyTwist')
        rospy.Subscriber('/map', OccupancyGrid, self.HandleImage)
        self.publisher.start()

    def BindKey(self, key, function):
        self.mapKeyFn[key] = function

    def OnKeyDown(self, event):
        keycode = event.GetKeyCode()
        if keycode in self.mapKeyFn:
            self.mapKeyFn[keycode]()
        event.Skip()

    def OnKeyUp(self, event):
        keycode = event.GetKeyCode()
        if keycode in self.mapKeyFn:
            self.mapKeyFn[keycode](True)
        event.Skip()

    def HandleImage(self, image):
        # make sure we update in the UI thread
        if self.IsShown():
            wx.CallAfter(self.panel.update, image)
        # http://wiki.wxpython.org/LongRunningTasks

    def DoBindKeys(self):
        """ Bind keys to Twist command
        http://wxpython.org/docs/api/wx.KeyEvent-class.html
        """
        self.BindKey(ord('Z'), self.publisher.cmd_forward)
        self.BindKey(ord('S'), self.publisher.cmd_backward)
        self.BindKey(ord('Q'), self.publisher.cmd_left)
        self.BindKey(ord('D'), self.publisher.cmd_right)
        self.BindKey(wx.WXK_UP,    self.publisher.cmd_forward)
        self.BindKey(wx.WXK_DOWN,  self.publisher.cmd_backward)
        self.BindKey(wx.WXK_LEFT,  self.publisher.cmd_left)
        self.BindKey(wx.WXK_RIGHT, self.publisher.cmd_right)
        self.BindKey(wx.WXK_SPACE, self.publisher.cmd_stop)
        self.BindKey(wx.WXK_ESCAPE,self.quit)
    def quit(self, isKeyUp=False):
        self.publisher.cont = False
        self.Close()

def main(argv):
    app = wx.App()
    KeyEventFrame()
    print(__doc__)
    app.MainLoop()
    rospy.signal_shutdown("MainLoop")
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))

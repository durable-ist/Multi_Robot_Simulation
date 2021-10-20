
import sys
import rospy
import actionlib
import numpy as np
from geodesy import utm
import tf2_ros
import tf2_geometry_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class gps_converter:

  def __init__(self):

    rospy.init_node('gps_converter_pub', anonymous=True)
    self.mb_client = actionlib.SimpleActionClient('/jackal0/move_base',MoveBaseAction)
    self.mb_client.wait_for_server()
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    wp_file = sys.argv[1]
    self.execute(wp_file)

  def execute(self, wp_file):
    with open(wp_file) as f:
      lista = f.read().splitlines()
    key = raw_input("Press Enter to begin...")
    idx = 0
    lat, lon = lista[idx].split()
    lat = float(lat)
    lon = float(lon)
    while(key != "Q" and key != "q" and not rospy.is_shutdown()):
      print "----------------------------------------------------"
      print "waypoin no: ", idx
      if self.waypoint_pub(lat, lon):
        if (len(lista) == idx + 1):
          print "all waypoints sent"
          exit()
        idx = idx + 1
        lat, lon = lista[idx].split()
        lat = float(lat)
        lon = float(lon)
      #key = raw_input("Press Enter to continue...")
    exit()

  def waypoint_pub(self, lat, lon):
    utm_conversion = utm.fromLatLong(lat,lon)
    print(utm_conversion)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "utm"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = utm_conversion.easting
    goal.target_pose.pose.position.y = utm_conversion.northing
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.w = 1.0

    try:
      transform = self.tf_buffer.lookup_transform("map",
                                       goal.target_pose.header.frame_id, #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(2.0)) #wait for 1 second

      goal.target_pose = tf2_geometry_msgs.do_transform_pose(goal.target_pose, transform)
      goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = goal.target_pose.pose.position.y, -goal.target_pose.pose.position.x
      goal.target_pose.header.stamp = rospy.Time.now()
      print "goal transformed\n", goal
      self.mb_client.send_goal(goal)
      wait = self.mb_client.wait_for_result()
      return wait
      #return True
    except:
      print "Couldnt find transform from utm to map"
      return False
    #exit()

def main(args):
  ic = gps_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
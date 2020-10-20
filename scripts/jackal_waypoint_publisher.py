#!/usr/bin/env python

import sys
import rospy
import tf
from threading import Thread
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
ROBOT_NAME = "jackal"
FRAME_ID = "world"

class Waypoint_Publish:

  def __init__(self, txt_file):
    rospy.init_node('jackal_waypoint_publisher', anonymous=True)
    self.publishers = []
    self.results = []
    self.result_dict = dict()
#iterate waypoint list to create a matrix for each jackal
    temp = []
    self.waypoints = []
    firstline = True
    with open(txt_file) as f:
      for line in f:
        if firstline:
          firstline = False
          continue
        if (ROBOT_NAME in line) and (temp != []):
          self.waypoints.append(temp)
          temp = []
        temp.append(list(line.split(" ")))
      self.waypoints.append(temp)
#create publishers for topics
    for i in range(len(self.waypoints)):
      self.result_dict[self.waypoints[i][0][0].rstrip("\n")] = False

      topic_name = "/" + self.waypoints[i][0][0].rstrip("\n") + '/move_base_simple/goal'
      self.publishers.append(rospy.Publisher(topic_name, PoseStamped, queue_size=10, latch=True))
      result_name = "/" + self.waypoints[i][0][0].rstrip("\n") + '/move_base/result'
      self.results.append(rospy.Subscriber(result_name, MoveBaseActionResult, self.result_handler))

  def movebase_goal(self, x, y, z, roll, pitch, yaw, frame):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = frame
    return pose


  def handle_waypoints(self, waypoints, publisher):
    for line in waypoints:
      if ROBOT_NAME in line[0]:
        name = line[0].rstrip("\n")
        continue
      else:
        pose = self.movebase_goal(float(line[0]), float(line[1]), float(line[2]), float(line[3]), float(line[4]), float(line[5]), FRAME_ID)
        print "publishing goal to " + name, pose
        publisher.publish(pose)
        while (self.result_dict[name] == False):
          pass
        self.result_dict[name] = False
        rospy.sleep(float(line[6]))
    
  def result_handler(self, data):
    for item in self.result_dict:
      if item in data.status.goal_id.id:
        self.result_dict[item] = True
        print str(item) + " result handle detected"

  def execute(self):
    threads = []
    for i, publisher in enumerate(self.publishers):
      threads.append(Thread(target=self.handle_waypoints, args=[self.waypoints[i], publisher]))
    for x in threads:
      x.start()
    for x in threads:
      x.join()
    rospy.loginfo("waypoint publishing finished successfully")
    rospy.signal_shutdown("waypoint publishing finished")

def main(args):
  node = Waypoint_Publish(args[1])
  try:
    while not rospy.is_shutdown():
      node.execute()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
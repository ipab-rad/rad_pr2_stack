#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Author: GreatAlexander
# @Date:   2016-07-06
# @Last Modified by:   GreatAlexander

import rospy
from std_srvs.srv import *
from std_msgs.msg import *
from pr2_natural_language.srv import *
from parse_recipes import *
from parse_recipes import initializeGraph, single_query_to_speech

class ParserWrapper:
  def __init__(self):
    self.graph = initializeGraph()
    self.pr2_speak = rospy.ServiceProxy('/pr2_natural_language/say', Query)
    self.look_at = rospy.Publisher('/pr2_natural_language/target_object', String,
     queue_size=10)

  def pr2_say(self, sentence):
    rospy.wait_for_service('/pr2_natural_language/say')
    try:
      self.pr2_speak(sentence)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def handle_query(self, req):
    print req.query
    response = single_query_to_speech(req.query, self.graph)
    print response[0] # Print speech response
    print response[1] # Print set of action, target and tool

    self.look_at.publish(response[1][0]) # Look at conversation item
    self.pr2_say(response[0]) # Respond to requested query
    return True

if __name__ == "__main__":
  rospy.init_node('parser_wrapper')
  pw = ParserWrapper()
  s = rospy.Service('/pr2_natural_language/query', Query, pw.handle_query)
  print "Ready to take query"
  rospy.spin()

/******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the
 * names of its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
/**
 *  @file    roomba_walker.hpp
 *  @author  rohithjayarajan
 *  @date 11/18/2018
 *  @version 1.0
 *
 *  @brief definition file for RoombaWalker class
 *
 *  @section DESCRIPTION
 *
 *  file for roomba_walker which contains the definition of RoombaWalker
 * class
 *
 */

// inlcude roomba_walker header file
#include "roomba_walker.hpp"

RoombaWalker::RoombaWalker() {
  // set frequency of publishing to 15Hz
  frequency_ = 15;
  // set velocity for linear motion
  linearVel_ = 0.5;
  // set velocity for angular motion
  angularVel_ = 0.75;
  // set minimum distance at which collision is a threat
  minDist_ = 0.8;
  // set collision threat flag to false initially
  isCollision_ = false;
  // initialize linear and angular motion values of the robot
  twistMsg_.linear.x = 0.0;
  twistMsg_.linear.y = 0.0;
  twistMsg_.linear.z = 0.0;
  twistMsg_.angular.x = 0.0;
  twistMsg_.angular.y = 0.0;
  twistMsg_.angular.z = 0.0;
  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  vel_ = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",
                                            1000);
  /**
   * The publish() function is how you send messages. The parameter
   * is the message object. The type of this object must agree with the type
   * given as a template parameter to the advertise<>() call, as was done
   * in the constructor above.
   */
  vel_.publish(twistMsg_);
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the
   * Subscriber object go out of scope, this callback will automatically be
   * unsubscribed from this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to
   * throw away the oldest ones.
   */
  laserScan_ = nh.subscribe("/scan", 1000, &RoombaWalker::laserCallback, this);
}

RoombaWalker::~RoombaWalker() {
  // stop the linear and angular motion of the robot
  twistMsg_.linear.x = 0.0;
  twistMsg_.linear.y = 0.0;
  twistMsg_.linear.z = 0.0;
  twistMsg_.angular.x = 0.0;
  twistMsg_.angular.y = 0.0;
  twistMsg_.angular.z = 0.0;
  /**
   * The publish() function is how you send messages. The parameter
   * is the message object. The type of this object must agree with the type
   * given as a template parameter to the advertise<>() call, as was done
   * in the constructor above.
   */
  vel_.publish(twistMsg_);
}

void RoombaWalker::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  // temporary variable to help find distance to collision
  double tempDist_ = 10000;
  // loop through laserscan reading to find distance to collisoin and if there
  // is a threat of collision
  for (int &n : msg->ranges) {
    if (n < minDist_) {
      isCollision_ = true;
    }
    if (n < tempDist_) {
      tempDist_ = n;
    }
  }
  collisionDist_ = tempDist_;
}

void RoombaWalker::startExploration() {
  // specifying rate at which to loop
  ros::Rate loop_rate(frequency_);
  while (ros::ok) {
    // if a collision threat is detected, stop linear motion of robot and rotate
    // it around z-axis by a value
    // else, supply only linear motion to the robot
    if (isCollision_) {
      twistMsg_.linear.x = 0.0;
      twistMsg_.angular.z = angularVel_;
    } else {
      twistMsg_.linear.x = linearVel_;
      twistMsg_.angular.z = 0.0;
    }
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    vel_.publish(twistMsg_);
    // call the callbacks
    ros::spinOnce();
    // sleep for remaining time to hit 10Hz publish rate
    loop_rate.sleep();
  }
}

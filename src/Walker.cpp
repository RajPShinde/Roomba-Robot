/***************************************************************************
 MIT License
 Copyright © 2019 Raj Shinde
 
 Permission is hereby granted, free of charge, to any person
 obtaining a copy of this software and associated documentation
 Files (the “Software”), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge,
 publish, distribute, sublicense, and/or sell copies of the Software,
 and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:
 The above copyright notice and this permission notice shall be included 
 in all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS
 OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 OTHER DEALINGS IN THE SOFTWARE.
 ***************************************************************************/

/**
 *  @copyright MIT License, © 2019 Raj Shinde
 *  @file    Walker.cpp
 *  @author  Raj Shinde
 *  @date    11/17/2019
 *  @version 1.0
 *  @brief   walker algorithm
 *  @section DESCRIPTION
 *  Uses laser data to walk and avoid obstacles
 */

#include <iostream>
#include <Walker.hpp>


Walker::Walker() {
  // Initialize variables in constructors
  flag = false;
  velocity = 0.5;
  heading = 5;
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
}

Walker::~Walker() {
// Destroy Objects
vel.publish(msg);
}

/**
 * @brief function to detect obstacle
 * @param sensor_msgs::LaserScan::ConstPtr& msg sensor data
 * @return None
 */
void Walker::senseObstacle(const sensor_msgs::LaserScan::ConstPtr& msg) {
flag = false;
for (auto m : msg->ranges) {
  if (m < 0.6) {
    flag = true;
    ROS_INFO_STREAM("Obstacle detected");
    return;
    }
  }
}

/**
 * @brief function to navigate robot
 * @param None
 * @return None
 */
void Walker::navigate() {
  // Advertise the velocity data
  auto vel = n.advertise<geometry_msgs::Twist>
         ("/cmd_vel_mux/input/navi", 1000);

  // Subscribe to laserScan
  auto s = n.subscribe <sensor_msgs::LaserScan>
        ("/scan", 300, &Walker::senseObstacle, this);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    // Turn if obstacle is detected
    if (flag == true) {
      msg.linear.x = 0.0;
      msg.angular.z = heading;
      // Display LOG Message
      ROS_INFO("Avoiding Collision");
    } else {
      msg.angular.z = 0.0;
      msg.linear.x = velocity;
    }

    // Publish velocity
    vel.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}


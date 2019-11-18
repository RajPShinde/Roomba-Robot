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
 *  @file    Walker.hpp
 *  @author  Raj Shinde
 *  @date    11/17/2019
 *  @version 1.0
 *  @brief   Contains Walker class
 *  @section DESCRIPTION
 *  Walker class with all the methods
 */

#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

 /**
 * @brief Class Walker for robot
 */
class Walker {
 public:
  bool flag;
 /**
  * @brief Default Constructor
  */
  Walker();

 /**
  * @brief Default Destructor
  */
  ~Walker();

 /**
  * @brief function to detect obstacle
  * @param sensor_msgs::LaserScan::ConstPtr& msg sensor data
  * @return None
  */
  void senseObstacle(const sensor_msgs::LaserScan::ConstPtr& msg);

 /**
  * @brief function to navigate robot
  * @param None
  * @return None
  */
  void navigate();
 private:
  // Creating objects and Node Handler
  ros::NodeHandle n;
  geometry_msgs::Twist msg;
  ros::Publisher vel;
  ros::Subscriber s;

  // Initializing Private Variables
  float velocity, heading;
};

#endif  // INCLUDE_WALKER_HPP_

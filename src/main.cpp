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
 *  @file    main.cpp
 *  @author  Raj Shinde
 *  @date    11/17/2019
 *  @version 1.0
 *  @brief   main file to call walker
 *  @section DESCRIPTION
 *  File to call walker that holdes the node
 */

#include <ros/ros.h>
#include <Walker.hpp>

/**
 * @brief Main function for calling
 * @param argc no of argumnets 
 * @param argv char pointer consisting arguments 
 * @return 0
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "walker");
  // Created an object of walker class
  Walker x;
  // call to navigate method
  x.navigate();
  return 0;
}
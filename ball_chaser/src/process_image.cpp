#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the
// specified direction
void drive_robot(float lin_x, float ang_z) {
  // TODO: Request a service and pass the velocities to it to drive the robot
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;
  if (!client.call(srv)) {
    ROS_ERROR("Failed to call service DriveToTarget");
  }
}

// Calculate linear X and angular Z velocities that result in bringing the ball
// to the center of camera image
std::pair<float, float> decide_drive_command(uint32_t img_width,
                                             uint32_t cols) {
  // Use a constant linear X
  constexpr float lin_x = 0.5f;
  // The maximum turn angular Z in radians
  constexpr float abs_max_ang_z = M_PI_2;
  // How far the pixel is off from the center of the image in percentage. +ve%
  // means on the right, -ve% means to the left, 0 means exactly in the center
  // The simple control loop tries to keep the ball in the center of the image
  float error = (static_cast<float>(img_width / 2) - static_cast<float>(cols)) /
                static_cast<float>(img_width / 2);
  // Round error % to closest .10s to reduce rotation command noise
  error = (error / 10.0) * 10.0;
  float ang_z = error * abs_max_ang_z;
  ROS_INFO_STREAM("error:" << error << " ang_z:" << ang_z);
  return {lin_x, ang_z};
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img) {
  int white_pixel = 255;

  // TODO: Loop through each pixel in the image and check if there's a bright
  // white one Then, identify if this pixel falls in the left, mid, or right
  // side of the image Depending on the white ball position, call the drive_bot
  // function and pass velocities to it Request a stop when there's no white
  // ball seen by the camera

  const uint32_t pixel_comps = img.step / img.width;
  for (uint32_t rows = 0; rows < img.height; rows++) {
    for (uint32_t cols = 0; cols < img.width; cols++) {
      // Assuming an 8-bit n-componenet image format, this is the byte index of
      // the first component of the pixel (the R in an RGB image format)
      uint32_t pixel_idx = (img.step * rows) + (cols * pixel_comps);
      bool is_white = true;
      // A pixel is white if all components are white
      for (uint32_t i = 0; i < pixel_comps; i++) {
        if (img.data[pixel_idx + i] != 255) {
          is_white = false;
          break;
        }
      }

      // Found a white pixel, compute and execute drive command to go towards
      // the ball
      if (is_white) {
        auto cmd = decide_drive_command(img.width, cols);
        drive_robot(cmd.first, cmd.second);
        return;
      }
    }
  }

  // Found no white pixels, stop
  drive_robot(0.0, 0.0);
}

int main(int argc, char **argv) {
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from command_robot
  client =
      n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the
  // process_image_callback function
  ros::Subscriber sub1 =
      n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  ROS_INFO("Proces Image Node is up and running");
  // Handle ROS communication events
  ros::spin();

  return 0;
}
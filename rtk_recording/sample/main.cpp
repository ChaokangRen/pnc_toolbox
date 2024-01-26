#include <ros/ros.h>
#include <sglog/sglog.h>

#include <cmath>
#include <ctime>
#include <fstream>
#include <string>
#include <vector>

#include "rtk_recording.h"
#include "sg_sensor_msgs/Inspva.h"
#include "sg_vehicle_msgs/Speed.h"

using namespace std;
using namespace jarvis::rtk_recording;

RtkRecording::RtkData current_rtk_point;
RtkRecording::RtkData previous_rtk_point;
RtkRecording::RtkData current_output_point;
RtkRecording::RtkData previous_output_point;
RtkRecording::RtkData init_point;

bool is_update = false;
bool is_init = false;

double velocity;
double length = 0.05;

vector<vector<double> > output_data;
vector<double> output_point;
static RtkRecording rtk_recording;
vector<vector<double> > inter;

void SpeedCallback(const sg_vehicle_msgs::SpeedConstPtr &vehicle_speed_ptr) {
  velocity = vehicle_speed_ptr->mps;
}

void InspvaCallback(const sg_sensor_msgs::Inspva::ConstPtr &Inspva) {
  double lon = Inspva->longitude * M_PI / 180;
  double lat = Inspva->latitude * M_PI / 180;
  double alt = Inspva->altitude * M_PI / 180;
  double echo_yaw = Inspva->yaw * M_PI / 180 + M_PI / 2;
  //   if (echo_yaw > -M_PI / 2 * 3 && echo_yaw < -M_PI) {
  //     echo_yaw += M_PI * 2;
  //   }
  double echo_pitch = Inspva->pitch * M_PI / 180;
  double echo_roll = Inspva->roll * M_PI / 180;
  double echo_x;
  double echo_y;
  double echo_z;

  rtk_recording.Wgs84ToLocalCoord(lon, lat, alt, &echo_x, &echo_y, &echo_z);
  is_update = true;
  init_point = {echo_x, echo_y, echo_z, echo_yaw, echo_pitch, echo_roll};
  current_rtk_point = init_point;

  if (is_init == false) {
    output_point.emplace_back(echo_x);
    output_point.emplace_back(echo_y);
    // output_point.emplace_back(echo_z);
    output_point.emplace_back(echo_yaw);
    // output_point.emplace_back(echo_pitch);
    // output_point.emplace_back(echo_roll);
    output_point.emplace_back(velocity);
    output_data.emplace_back(output_point);

    output_point.clear();

    previous_rtk_point = init_point;
    previous_output_point = init_point;
    current_output_point = init_point;

    is_init = true;
  }
  if (is_update == true) {
    inter = rtk_recording.Interpolation(previous_rtk_point, current_rtk_point,
                                        length);

    previous_rtk_point = current_rtk_point;

    if (!(inter.empty() || inter[0].empty() || isnan(inter[0][0]))) {
      for (int i = 0; i < inter.size(); i++) {
        for (int j = 0; j < 3; j++) {
          output_point.emplace_back(inter[i][j]);
        }
        output_point.emplace_back(velocity);
        SG_INFO("x = %lf,y = %lf,yaw = %lf,vel = %lf", output_point[0],
                output_point[1], output_point[2], output_point[3]);
        output_data.emplace_back(output_point);
        output_point.clear();
      }
    }
    is_update = false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rtk_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");
  ros::Subscriber inspva_sub =
      nh.subscribe("/sensor/ins/fusion", 10, InspvaCallback);
  ros::Subscriber speed_sub =
      nh.subscribe("/vehicle/chassis/vehicle_speed", 10, SpeedCallback);
  rtk_recording.Init();

  while (ros::ok()) {
    ros::spinOnce();
  }

  time_t now = time(0);
  tm *ltm = localtime(&now);
  string str_year = to_string(ltm->tm_year + 1900);
  string str_mon = to_string(ltm->tm_mon + 1);
  string str_mday = to_string(ltm->tm_mday);
  string str_hour = to_string(ltm->tm_hour);
  string str_min = to_string(ltm->tm_min);
  string filename = "/mnt/Downloads/rtk_trajectory_" + str_year + "_" +
                    str_mon + "_" + str_mday + "_" + str_hour + "_" + str_min +
                    ".json";

  ofstream out_txt_file(filename);
  out_txt_file << "{" << endl << "\"reference_line\":" << endl << "[" << endl;

  if (!output_data.empty()) {
    for (int i = 0; i < output_data.size() - 1; ++i) {
      out_txt_file << "[" << output_data[i][0];
      for (int j = 1; j < 4; ++j) {
        out_txt_file << "," << output_data[i][j];
      }
      out_txt_file << "]," << endl;
    }

    out_txt_file << "[" << output_data[output_data.size() - 1][0] << ","
                 << output_data[output_data.size() - 1][1] << ","
                 << output_data[output_data.size() - 1][2] << ","
                 << output_data[output_data.size() - 1][3] << "]" << endl;
    out_txt_file << "]" << endl << "}" << endl;
  }
  out_txt_file.close();
  return 0;
}

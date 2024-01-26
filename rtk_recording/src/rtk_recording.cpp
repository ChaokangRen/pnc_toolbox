#include "rtk_recording.h"

#include "math.h"
#include "sglog/sglog.h"

const double semimajor_len = 6378137;
const double semiminor_len = 6356752.31414;
const double earth_radius = 6378137;
const double oblateness = 1 / 298.257222101;
const double first_eccentricity = 0.0818191910428;

const double orgin_longitude = 2.074710701656759;
const double orgin_latitude = 0.5586257075569977;
const double orgin_altitude = 0.22758597622763593;

const double orgin_x = -2614020.578497937;
const double orgin_y = 4740731.728376352;
const double orgin_z = 3361079.9529776173;
double output_length = 0.05;

using namespace jarvis::rtk_recording;

void RtkRecording::Init() { SG_INFO("rtk recording start"); }

void RtkRecording::Wgs84ToLocalCoord(const double longitude,
                                     const double latitude,
                                     const double altitude, double *local_x,
                                     double *local_y, double *local_z) {
  double earth_radius_p =
      earth_radius * (1 + oblateness * sin(latitude) * sin(latitude));
  double spre_coord_x =
      (earth_radius_p + altitude) * cos(latitude) * cos(longitude);
  double spre_coord_y =
      (earth_radius_p + altitude) * cos(latitude) * sin(longitude);
  double spre_coord_z =
      ((1 - first_eccentricity * first_eccentricity) * earth_radius_p +
       altitude) *
      sin(latitude);

  *local_x = -sin(orgin_longitude) * (spre_coord_x - orgin_x) +
             cos(orgin_longitude) * (spre_coord_y - orgin_y);
  *local_y =
      -sin(orgin_latitude) * cos(orgin_longitude) * (spre_coord_x - orgin_x) -
      sin(orgin_latitude) * sin(orgin_longitude) * (spre_coord_y - orgin_y) +
      cos(orgin_latitude) * (spre_coord_z - orgin_z);
  *local_z =
      cos(orgin_latitude) * cos(orgin_longitude) * (spre_coord_x - orgin_x) -
      cos(orgin_latitude) * sin(orgin_longitude) * (spre_coord_y - orgin_y) +
      sin(orgin_latitude) * (spre_coord_z - orgin_z);
}

vector<vector<double> > RtkRecording::Interpolation(
    RtkRecording::RtkData previous_point, RtkRecording::RtkData current_point,
    double length) {
  vector<double> output_data;
  vector<vector<double> > output;

  double point_distance = sqrt(pow((current_point.x - previous_point.x), 2) +
                               pow((current_point.y - previous_point.y), 2));

  double output_x;
  double output_y;
  double output_yaw;

  if (point_distance >= rest_length_) {
    output_x =
        (rest_length_ / point_distance) * (current_point.x - previous_point.x) +
        previous_point.x;
    output_y =
        (rest_length_ / point_distance) * (current_point.y - previous_point.y) +
        previous_point.y;
    output_yaw = (rest_length_ / point_distance) *
                     (current_point.yaw - previous_point.yaw) +
                 previous_point.yaw;

    output_data.emplace_back(output_x);
    output_data.emplace_back(output_y);
    output_data.emplace_back(output_yaw);
    output.emplace_back(output_data);
    output_data.clear();

    double rest_distance = point_distance - rest_length_;
    if (rest_distance >= length) {
      int cnt = std::floor(rest_distance / length);
      for (int i = 0; i < cnt; i++) {
        output_x =
            (length / rest_distance) * (current_point.x - output_x) + output_x;
        output_y =
            (length / rest_distance) * (current_point.y - output_y) + output_y;
        output_yaw =
            (length / rest_distance) * (current_point.yaw - output_yaw) +
            output_yaw;
        output_data.emplace_back(output_x);
        output_data.emplace_back(output_y);
        output_data.emplace_back(output_yaw);
        output.emplace_back(output_data);
        output_data.clear();
      }
      rest_length_ = length - (rest_distance - cnt * length);
    } else {
      rest_length_ = length - rest_distance;
    }
  } else {
    rest_length_ = rest_length_ - point_distance;
  }
  return output;
}
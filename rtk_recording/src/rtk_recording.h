#pragma once
#include <vector>

using namespace std;
namespace jarvis {
namespace rtk_recording {
class RtkRecording {
 public:
  RtkRecording() = default;

  void Wgs84ToLocalCoord(const double longitude, const double latitude,
                         const double altitude, double *local_x,
                         double *local_y, double *local_z);
  void Init();

  struct RtkData {
    double x;
    double y;
    double z;
    double yaw;
    double pitch;
    double roll;
  };

  vector<vector<double>> Interpolation(RtkRecording::RtkData previous_point,
                                       RtkRecording::RtkData current_point,
                                       double length);

 private:
  double rest_length_ = 0.0;
};
}  // namespace rtk_recording
}  // namespace jarvis

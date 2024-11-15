#ifndef GPS_UTIL_H_
#define GPS_UTIL_H_

#include <math.h>

#include <cmath>
#include <string>
#include <tuple>

#include "eigen3/Eigen/Dense"
#include "shared/math/math_util.h"
#include "shared/util/helpers.h"

using Eigen::Rotation2Dd;
using Eigen::Vector2d;
using math_util::DegToRad;
using std::string;

namespace gps_util {

struct GPSPoint {
  // Convenience data structure for GPS points.
  GPSPoint() : time(0), lat(0), lon(0) {}
  GPSPoint(double time, double lat, double lon)
      : time(time), lat(lat), lon(lon) {}
  GPSPoint(double time, double lat, double lon, double heading)
      : time(time), lat(lat), lon(lon), heading(heading) {}
  GPSPoint(double lat, double lon) : time(0), lat(lat), lon(lon) {}

  bool operator==(const GPSPoint& other) const {
    return lat == other.lat && lon == other.lon;
  }

  bool operator!=(const GPSPoint& other) const { return !(*this == other); }

  double time;
  double lat;
  double lon;
  double heading;  // True North
};

struct GPSTranslator {
  string GetMapFileFromName(const string& maps_dir, const string& map) {
    return maps_dir + "/" + map + "/" + map + ".gpsmap.txt";
  }

  bool Load(const string& maps_dir, const string& map) {
    const string file = GetMapFileFromName(maps_dir, map);
    ScopedFile fid(file, "r", true);
    if (fid() == nullptr) return false;
    if (fscanf(fid(), "%lf, %lf, %lf", &gps_origin_latitude,
               &gps_origin_longitude, &map_orientation) != 3) {
      return false;
    }
    printf("Map origin: %12.8lf, %12.8lf\n", gps_origin_latitude,
           gps_origin_longitude);
    return true;
  }

  void SetOrigin(double latitude, double longitude, double orientation) {
    gps_origin_latitude = latitude;
    gps_origin_longitude = longitude;
    map_orientation = orientation;
  }

  Vector2d GPSToMetric(const double latitude, const double longitude) {
    const double theta = DegToRad(latitude);
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double r =
        sqrt(Sq(wgs_84_a * wgs_84_b) / (Sq(c * wgs_84_b) + Sq(s * wgs_84_a)));
    const double dlat = DegToRad(latitude - gps_origin_latitude);
    const double dlong = DegToRad(longitude - gps_origin_longitude);
    const double r1 = r * c;
    const double x = r1 * dlong;
    const double y = r * dlat;
    return Rotation2Dd(map_orientation) * Vector2d(x, y);
  }

  void MetricToGPS(const Vector2d& loc, double* longitude, double* latitude) {
    const double theta = DegToRad(gps_origin_latitude);
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double r =
        sqrt(Sq(wgs_84_a * wgs_84_b) / (Sq(c * wgs_84_b) + Sq(s * wgs_84_a)));
    const double r1 = r * c;
    const double dlat = loc.y() / r;
    const double dlong = loc.x() / r1;
    *longitude = gps_origin_longitude + dlong;
    *latitude = gps_origin_latitude + dlat;
  }

  double gps_origin_longitude;
  double gps_origin_latitude;
  double map_orientation;

  // Earth geoid parameters from WGS 84 system
  // https://en.wikipedia.org/wiki/World_Geodetic_System#A_new_World_Geodetic_System:_WGS_84
  // a = Semimajor (Equatorial) axis
  static constexpr double wgs_84_a = 6378137.0;
  // b = Semiminor (Polar) axis
  static constexpr double wgs_84_b = 6356752.314245;
};

inline std::tuple<double, double, int> gpsToUTM(double lat, double lon) {
  // Constants for UTM conversion
  constexpr double a = 6378137.0;          // WGS-84 major axis
  constexpr double f = 1 / 298.257223563;  // WGS-84 flattening
  constexpr double k0 = 0.9996;            // UTM scale factor
  constexpr double e = sqrt(f * (2 - f));  // Eccentricity
  constexpr double e2 = e * e;

  // UTM Zone
  int zone = static_cast<int>((lon + 180) / 6) + 1;

  // Convert latitude and longitude to radians
  double lat_rad = lat * M_PI / 180.0;
  double lon_rad = lon * M_PI / 180.0;

  // Central meridian of the UTM zone
  double lon_origin = (zone - 1) * 6 - 180 + 3;
  double lon_origin_rad = lon_origin * M_PI / 180.0;

  // Calculations for UTM coordinates
  double N = a / sqrt(1 - e2 * sin(lat_rad) * sin(lat_rad));
  double T = tan(lat_rad) * tan(lat_rad);
  double C = e2 / (1 - e2) * cos(lat_rad) * cos(lat_rad);
  double A = cos(lat_rad) * (lon_rad - lon_origin_rad);

  double M =
      a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * lat_rad -
           (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) *
               sin(2 * lat_rad) +
           (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * sin(4 * lat_rad) -
           (35 * e2 * e2 * e2 / 3072) * sin(6 * lat_rad));

  double easting =
      k0 * N *
          (A + (1 - T + C) * A * A * A / 6 +
           (5 - 18 * T + T * T + 72 * C - 58 * e2) * A * A * A * A * A / 120) +
      500000.0;

  double northing =
      k0 *
      (M + N * tan(lat_rad) *
               (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
                (61 - 58 * T + T * T + 600 * C - 330 * e2) * A * A * A * A * A *
                    A / 720));

  // Correct for southern hemisphere
  if (lat < 0) {
    northing += 10000000.0;
  }

  return std::make_tuple(easting, northing, zone);
}

inline double gpsDistance(double lat0, double lon0, double lat1, double lon1) {
  // Convert latitude and longitude to radians
  double lat0_rad = lat0 * M_PI / 180.0;
  double lon0_rad = lon0 * M_PI / 180.0;
  double lat1_rad = lat1 * M_PI / 180.0;
  double lon1_rad = lon1 * M_PI / 180.0;

  // Haversine formula for distance between two GPS coordinates
  double dlat = lat1_rad - lat0_rad;
  double dlon = lon1_rad - lon0_rad;
  double a = pow(sin(dlat / 2), 2) +
             cos(lat0_rad) * cos(lat1_rad) * pow(sin(dlon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return 6371000 * c;  // Radius of Earth in meters
}

inline std::tuple<double, double> gpsToGlobalCoord(double lat0, double lon0,
                                                   double lat1, double lon1) {
  // Convert latitude and longitude to utm coordinates
  const auto& [e0, n0, zone0] = gpsToUTM(lat0, lon0);
  const auto& [e1, n1, zone1] = gpsToUTM(lat1, lon1);
  return {e1 - e0, n1 - n0};
}

inline Eigen::Affine2f gpsToLocal(double current_lat, double current_lon,
                                  double current_heading, double goal_lat,
                                  double goal_lon, double goal_heading) {
  // Step 1: Convert current and goal GPS coordinates to UTM
  const auto& [curr_easting, curr_northing, curr_zone] =
      gpsToUTM(current_lat, current_lon);
  const auto& [goal_easting, goal_northing, goal_zone] =
      gpsToUTM(goal_lat, goal_lon);
  // Ensure that both coordinates are in the same UTM zone
  if (goal_zone != curr_zone) {
    throw std::runtime_error("GPS points are in different UTM zones.");
  }

  // Step 2: Calculate the translation vector from the current position to the
  // goal in UTM
  double dx = goal_easting - curr_easting;
  double dy = goal_northing - curr_northing;

  // Step 3: Convert the current heading to radians and adjust for x-axis
  // reference Since 0 degrees points along the y-axis and rotates
  // counter-clockwise, convert to radians with 0 degrees aligned along the
  // x-axis
  double current_heading_rad = (90.0 - current_heading) * M_PI / 180.0;

  // Step 4: Rotate the translation vector to the robot's local frame
  double local_x =
      dx * cos(-current_heading_rad) - dy * sin(-current_heading_rad);
  double local_y =
      dx * sin(-current_heading_rad) + dy * cos(-current_heading_rad);

  // Step 5: Convert the goal heading to the local frame
  double goal_heading_rad = (90.0 - goal_heading) * M_PI / 180.0;
  double local_heading = goal_heading_rad - current_heading_rad;

  // Normalize local heading to the range [-pi, pi]
  while (local_heading > M_PI) local_heading -= 2 * M_PI;
  while (local_heading < -M_PI) local_heading += 2 * M_PI;

  // Step 6: Create the affine transformation for the local pose of the goal
  Eigen::Affine2f transform = Eigen::Translation2f(local_x, local_y) *
                              Eigen::Rotation2Df(local_heading);

  return transform;
}

}  // namespace gps_util

#endif
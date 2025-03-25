#ifndef GPS_UTIL_H_
#define GPS_UTIL_H_

#include <math.h>

#include <cmath>
#include <string>
#include <tuple>

#include "eigen3/Eigen/Dense"
#include "shared/math/math_util.h"
#include "shared/util/helpers.h"

using Eigen::Affine2d;
using Eigen::Rotation2Dd;
using Eigen::Vector2d;
using math_util::AngleMod;
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
  double heading;  // True North (degrees)
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

inline double gpsDistance(const GPSPoint& p0, const GPSPoint& p1) {
  const auto& [lat0, lon0] = std::make_tuple(p0.lat, p0.lon);
  const auto& [lat1, lon1] = std::make_tuple(p1.lat, p1.lon);
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

inline Eigen::Vector2d gpsToGlobalCoord(const GPSPoint& p0,
                                        const GPSPoint& p1) {
  const auto& [lat0, lon0] = std::make_tuple(p0.lat, p0.lon);
  const auto& [lat1, lon1] = std::make_tuple(p1.lat, p1.lon);

  // Convert latitude and longitude to utm coordinates
  const auto& [e0, n0, zone0] = gpsToUTM(lat0, lon0);
  const auto& [e1, n1, zone1] = gpsToUTM(lat1, lon1);

  if (zone0 != zone1) {
    throw std::runtime_error("GPS points are in different UTM zones.");
  }

  return {e1 - e0, n1 - n0};  // x y
}

inline double gpsToGlobalHeading(const GPSPoint& p0) {
  double heading =
      M_PI / 2.0 - DegToRad(p0.heading);  // convert to radians expected REP 105
  return AngleMod(heading);
}

struct GPSTranslator {
  // Convenience wrapper for GPS to map coordinate transformations
  GPSTranslator()
      : is_gps_origin_set(false),
        is_map_origin_set(false),
        use_map_as_origin(false) {}

  bool Initialized() const { return is_gps_origin_set && is_map_origin_set; }

  void SetReferenceFrame(const string& frame) {
    if (frame == "map") {
      use_map_as_origin = true;
    } else if (frame == "utm") {
      use_map_as_origin = false;
    } else {
      throw std::runtime_error("Invalid reference frame.");
    }
  }

  void SetGPSOrigin(const GPSPoint& p) {
    gps_origin = p;
    is_gps_origin_set = true;
    T_initial_gps = Eigen::Affine2d::Identity();
  }

  void SetMapOrigin(const Affine2d& p) {
    T_initial_map = p;
    is_map_origin_set = true;
  }

  Eigen::Vector2d GpsToGlobalCoord(const GPSPoint& p1) {
    if (!is_gps_origin_set || !is_map_origin_set) {
      throw std::runtime_error("GPS or map origin not set.");
    }

    Eigen::Vector2d gps_loc = gpsToGlobalCoord(gps_origin, p1);
    Eigen::Affine2d T_base_gps = Eigen::Rotation2Dd(gpsToGlobalHeading(p1)) *
                                 Eigen::Translation2d(gps_loc);
    if (use_map_as_origin) {
      Eigen::Affine2d T_gps_map = T_initial_gps * T_initial_map.inverse();
      return (T_gps_map * T_base_gps).translation();
    }

    return gps_loc;
  }

  double GpsToGlobalHeading(const GPSPoint& p0) {
    if (!is_gps_origin_set || !is_map_origin_set) {
      throw std::runtime_error("GPS or map origin not set.");
    }
    double heading_global = gpsToGlobalHeading(p0);
    if (use_map_as_origin) {
      Eigen::Affine2d T_gps_map = T_initial_gps * T_initial_map.inverse();
      Eigen::Rotation2Dd R_base_gps = Eigen::Rotation2Dd(heading_global);
      Eigen::Rotation2Dd R_gps_map(T_gps_map.linear());
      return (R_gps_map * R_base_gps).angle();
    }
    return heading_global;
  }

  bool is_gps_origin_set;
  bool is_map_origin_set;
  bool use_map_as_origin;
  GPSPoint gps_origin;
  Affine2d T_initial_gps;
  Affine2d T_initial_map;
};

}  // namespace gps_util

#endif
#pragma once

#include "../../core/types.h"
#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/motion.h"

static constexpr float FEEDRATE_XY_MM_S = 150;
static constexpr float FEEDRATE_Z_MM_S = 25;
static constexpr float FEEDRATE_EXTRUDE_MM_S = 70;

void go_to_xy(const xy_pos_t position) {
  do_blocking_move_to_xy(position, FEEDRATE_XY_MM_S);
}

void go_to_xy_z(const xy_pos_t position, const float z) {
  do_blocking_move_to_xy_z(position, z, FEEDRATE_XY_MM_S);
}

void go_to_z(const float z) {
  do_blocking_move_to_z(z, FEEDRATE_Z_MM_S);
}

class Dabber {
public:
  static constexpr float PROBE_TO_NOZZLE_Z_OFFSET = -1.5;

private:
  const xy_pos_t _upper_left_corner;
  const float _surface_height;
  const std::vector<std::pair<xy_pos_t, float>> _probed_surface_heights;
  float _min_surface_height;
  float _max_surface_height;

  static constexpr xy_pos_t PROBE_TO_NOZZLE_XY_OFFSET = {-25.25, 0};
  //static constexpr float DABBING_ELEVATION_ABOVE_SURFACE_HEIGHT = 0.45;
  static constexpr float DABBING_ELEVATION_ABOVE_SURFACE_HEIGHT = 0.35;
  static constexpr float APPROACH_FROM_Z_OFFSET = 1;
  static constexpr float CRUISING_ALTITUDE_SURFACE_HEIGHT_OFFSET = 4;
  static constexpr float STAINING_EXTRUSION_MULTIPLIER = 9.0;
  static constexpr float RETRACTION_MM = 0.0/STAINING_EXTRUSION_MULTIPLIER;

  void go_to_cruising_altitude(const xy_pos_t position) const {
    go_to_z_from_surface(position, CRUISING_ALTITUDE_SURFACE_HEIGHT_OFFSET);
  }
  
  void extrude_stain(const float extrusion_mm) const {
    unscaled_e_move(STAINING_EXTRUSION_MULTIPLIER * extrusion_mm, FEEDRATE_EXTRUDE_MM_S);
  }

  void unretract_and_extrude_stain(const float extrusion_mm) const {
    extrude_stain(RETRACTION_MM + extrusion_mm);
  }

  void retract() const {
    extrude_stain(-1 * RETRACTION_MM);
  }
  
  void unretract() const {
    extrude_stain(RETRACTION_MM);
  }
  
  bool vector_is_zero(const xy_pos_t v) const {
    return fabs(v.x) < 0.01 && fabs(v.y) < 0.01;
  }
  
  float vector_magnitude(const xy_pos_t v) const {
    return sqrt(v.x * v.x + v.y * v.y);
  }
  
  xy_pos_t calculate_xy(const xy_pos_t position) const {
    return _upper_left_corner + PROBE_TO_NOZZLE_XY_OFFSET + position;
  }
  
  float interpolate_surface_height(const xy_pos_t position) const {
    float sum_of_inverse_squared_distances = 0;
    float total_surface_height = 0;
    for (unsigned int i = 0; i < _probed_surface_heights.size(); ++i) {
      float diff_x = _probed_surface_heights[i].first.x - position.x;
      float diff_y = _probed_surface_heights[i].first.y - position.y;
      float squared_distance = diff_x * diff_x + diff_y * diff_y;
      // If we're super close to a point, just use its surface level.
      if (squared_distance < 4) {
        return _probed_surface_heights[i].second;
      }
      float inverse = 1/squared_distance;
      sum_of_inverse_squared_distances += inverse;
      total_surface_height += _probed_surface_heights[i].second * inverse;
    }
    float interpolated_surface_height = total_surface_height / sum_of_inverse_squared_distances;
    if (interpolated_surface_height > _max_surface_height + .01 || interpolated_surface_height < _min_surface_height - .01) {
      SERIAL_ECHOLNPGM("Calculated in invalid interpolated surface height of ", interpolated_surface_height, " when max is ", _max_surface_height, " and min is ", _min_surface_height);
      gcode.dwell(500000000);
    }
    return interpolated_surface_height;
  }
  
  float calculate_z(const xy_pos_t position, const float z) const {
    return interpolate_surface_height(position) + PROBE_TO_NOZZLE_Z_OFFSET + z;
  }
  
  void go_to_xy_from_upper_left_corner(const xy_pos_t position) const {
    go_to_xy(calculate_xy(position));
  }

  void go_to_xy_z_from_upper_left_corner(const xy_pos_t position, const float z) const {
    go_to_xy_z(calculate_xy(position), calculate_z(position, z));
  }

  void go_to_z_from_surface(const xy_pos_t position, const float z) const {
    const float surface_z = calculate_z(position, z);
    go_to_z(surface_z);
  }

public:
  Dabber(const xy_pos_t upper_left_corner, 
         const float surface_height,
         const std::vector<std::pair<xy_pos_t, float>> probed_surface_heights) :
    _upper_left_corner(upper_left_corner), 
    _surface_height(surface_height), 
    _probed_surface_heights(probed_surface_heights) {
    _min_surface_height = 1000;
    _max_surface_height = 0;
    for (unsigned int i = 0; i < _probed_surface_heights.size(); ++i) {
      _min_surface_height = std::min(_min_surface_height, _probed_surface_heights[i].second);
      _max_surface_height = std::max(_max_surface_height, _probed_surface_heights[i].second);
    }
  }

  void dab(const xy_pos_t position, const float extrusion_mm, const float offset_from_surface_height, const xy_pos_t post_dab_scoot, const xy_pos_t approach_from) const {
    // Make sure we're at cruising altitude
    go_to_cruising_altitude(position);
    // Even if approach_from is the zero vector, this works
    go_to_xy_from_upper_left_corner(position + approach_from);
    // Extrude the stain for this dab
    unretract_and_extrude_stain(extrusion_mm);
    // Lower and do the actual dab
    const float dabbing_height = DABBING_ELEVATION_ABOVE_SURFACE_HEIGHT + offset_from_surface_height;
    if (!vector_is_zero(approach_from)) {
      go_to_z_from_surface(position, dabbing_height + APPROACH_FROM_Z_OFFSET + vector_magnitude(approach_from));
      go_to_xy_z_from_upper_left_corner(position, dabbing_height + APPROACH_FROM_Z_OFFSET);
    }
    go_to_z_from_surface(position, dabbing_height);
    //gcode.dwell(5000000);
    if (!vector_is_zero(post_dab_scoot)) {
      go_to_xy_from_upper_left_corner(position + post_dab_scoot);
      gcode.dwell(500);
      go_to_xy_from_upper_left_corner(position);
    }
    go_to_cruising_altitude(position);
    retract();
  }
};

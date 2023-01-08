#pragma once

#include "../../core/types.h"
#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/motion.h"

static constexpr float FEEDRATE_XY_MM_S = 80;
static constexpr float FEEDRATE_Z_MM_S = 12;
static constexpr float FEEDRATE_EXTRUDE_MM_S = 20;

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
private:
  const xy_pos_t _upper_left_corner;
  const float _surface_height;

  static constexpr xy_pos_t PROBE_TO_NOZZLE_XY_OFFSET = {-30.25, 0};
  static constexpr float PROBE_TO_NOZZLE_Z_OFFSET = -4.1;
  static constexpr float DABBING_ELEVATION_ABOVE_SURFACE_HEIGHT = .2;
  static constexpr float APPROACH_FROM_Z_OFFSET = 1;
  static constexpr float CRUISING_ALTITUDE_SURFACE_HEIGHT_OFFSET = 2;
  static constexpr float RETRACTION_MM = 0.4;
  static constexpr float STAINING_EXTRUSION_MULTIPLIER = 9.0;

  void go_to_cruising_altitude() const {
    go_to_z_from_surface(CRUISING_ALTITUDE_SURFACE_HEIGHT_OFFSET);
  }
  
  void extrude_stain(const float extrusion_mm) const {
    unscaled_e_move(STAINING_EXTRUSION_MULTIPLIER * extrusion_mm, FEEDRATE_EXTRUDE_MM_S);
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
  
  float calculate_z(const float z) const {
    return _surface_height + PROBE_TO_NOZZLE_Z_OFFSET + z;
  }
  
  void go_to_xy_from_upper_left_corner(const xy_pos_t position) const {
    go_to_xy(calculate_xy(position));
  }

  void go_to_xy_z_from_upper_left_corner(const xy_pos_t position, const float z) const {
    go_to_xy_z(calculate_xy(position), calculate_z(z));
  }

  void go_to_z_from_surface(const float z) const {
    const float surface_z = calculate_z(z);
    go_to_z(surface_z);
  }

public:
  Dabber(const xy_pos_t upper_left_corner, const float surface_height) : 
    _upper_left_corner(upper_left_corner), 
    _surface_height(surface_height) {
  }

  void dab(const xy_pos_t position, const float extrusion_mm, const float offset_from_surface_height, const xy_pos_t post_dab_scoot, const xy_pos_t approach_from) const {
    // Make sure we're at cruising altitude
    go_to_cruising_altitude();
    // Even if approach_from is the zero vector, this works
    go_to_xy_from_upper_left_corner(position + approach_from);
    unretract();
    // Extrude the stain for this dab
    extrude_stain(extrusion_mm);
    // Lower and do the actual dab
    const float dabbing_height = DABBING_ELEVATION_ABOVE_SURFACE_HEIGHT + offset_from_surface_height;
    if (!vector_is_zero(approach_from)) {
      go_to_z_from_surface(dabbing_height + APPROACH_FROM_Z_OFFSET + vector_magnitude(approach_from));
      go_to_xy_z_from_upper_left_corner(position, dabbing_height + APPROACH_FROM_Z_OFFSET);
    }
    go_to_z_from_surface(dabbing_height);
    //gcode.dwell(5000000);
    if (!vector_is_zero(post_dab_scoot)) {
      go_to_xy_from_upper_left_corner(position + post_dab_scoot);
      gcode.dwell(500);
      go_to_xy_from_upper_left_corner(position);
    }
    go_to_cruising_altitude();
    retract();
  }
};

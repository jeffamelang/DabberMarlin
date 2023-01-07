#pragma once

#include "../../core/types.h"
#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/motion.h"

static constexpr float FEEDRATE_XY_MM_S = 30;
static constexpr float FEEDRATE_Z_MM_S = 6;
static constexpr float FEEDRATE_EXTRUDE_MM_S = 6;

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

  static constexpr float DABBING_ELEVATION_ABOVE_SURFACE_HEIGHT = .4;
  static constexpr float APPROACH_FROM_Z_OFFSET = 1;
  static constexpr float CRUISING_ALTITUDE_SURFACE_HEIGHT_OFFSET = 2;
  static constexpr float RETRACTION_MM = 0.4;

  void go_to_cruising_altitude() const {
    go_to_z(_surface_height + CRUISING_ALTITUDE_SURFACE_HEIGHT_OFFSET);
  }
  
  void extrude_stain(const float extrusion_mm) const {
    unscaled_e_move(extrusion_mm, FEEDRATE_EXTRUDE_MM_S);
  }
  
  bool vector_is_zero(const xy_pos_t v) const {
    return fabs(v.x) < 0.01 && fabs(v.y) < 0.01;
  }
  
  float vector_magnitude(const xy_pos_t v) const {
    return sqrt(v.x * v.x + v.y * v.y);
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
    go_to_xy(position + approach_from);
    // Extrude the stain for this dab
    extrude_stain(extrusion_mm);
    // Lower and do the actual dab
    const float dabbing_height = _surface_height + DABBING_ELEVATION_ABOVE_SURFACE_HEIGHT + offset_from_surface_height;
    if (!vector_is_zero(approach_from)) {
      go_to_z(dabbing_height + APPROACH_FROM_Z_OFFSET + vector_magnitude(approach_from));
      go_to_xy_z(position, dabbing_height + APPROACH_FROM_Z_OFFSET);
    }
    go_to_z(dabbing_height);
    if (!vector_is_zero(post_dab_scoot)) {
      go_to_xy(position + post_dab_scoot);
      gcode.dwell(500);
      go_to_xy(position);
    }
    go_to_cruising_altitude();
  }

  void retract() {
    extrude_stain(-1 * RETRACTION_MM);
  }
  
  void unretract() {
    extrude_stain(RETRACTION_MM);
  }
};

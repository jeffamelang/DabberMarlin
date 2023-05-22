#pragma once

#include "../../core/types.h"
#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/motion.h"
#include <map>
#include <vector>

static constexpr float FEEDRATE_XY_MM_S = 150;
static constexpr float FEEDRATE_Z_MM_S = 35;
static constexpr float FEEDRATE_EXTRUDE_MM_S = 70;

void validate_xy(const xy_pos_t position) {
  if (position.x < 0 || position.x > X_BED_SIZE || position.y < 0 || position.y > Y_BED_SIZE) {
    SERIAL_ECHOLNPGM("Cannot move to position of (", position.x, ", ", position.y, "), it's not a valid position.");
    gcode.dwell(500000000);
  }
}

void go_to_xy(const xy_pos_t position) {
  validate_xy(position);
  do_blocking_move_to_xy(position, FEEDRATE_XY_MM_S);
}

void go_to_xy_z(const xy_pos_t position, const float z) {
  validate_xy(position);
  do_blocking_move_to_xy_z(position, z, FEEDRATE_XY_MM_S);
}

void go_to_z(const float z) {
  do_blocking_move_to_z(z, FEEDRATE_Z_MM_S);
}

void display_message(const char* message) {
  LCD_MESSAGE_F(message);
}

void display_message_and_wait_for_button_push(const char* message) {
  display_message(message);
  while (!ui.button_pressed()) {
    safe_delay(50);
  }
}

class Dabber {
public:
  static constexpr float PROBE_TO_NOZZLE_Z_OFFSET = -1.5;

private:
  const xy_pos_t _upper_left_corner;
  const float _surface_height;
  const std::map<double, std::vector<std::pair<xy_pos_t, float>>> _all_elevations_probed_surface_heights;
  float _min_surface_height;
  float _max_surface_height;

  static constexpr xy_pos_t PROBE_TO_NOZZLE_XY_OFFSET = {-25.25, 0};
  static constexpr float DABBING_ELEVATION_ABOVE_SURFACE_HEIGHT = 0.35;
  static constexpr float APPROACH_FROM_Z_OFFSET = 1;
  static constexpr float CRUISING_ALTITUDE_SURFACE_HEIGHT_OFFSET = 4;
  static constexpr float STAINING_EXTRUSION_MULTIPLIER = 9.0;
  static constexpr float RETRACTION_MM = 0.0/STAINING_EXTRUSION_MULTIPLIER;
  static constexpr float NON_BOUNDARY_DAB_EXTRA_NOZZLE_DEPTH = 0.25;
  static constexpr float MINIMUM_EXTRUSION_MM = 0.05;

  void go_to_cruising_altitude() const {
    go_to_z(_max_surface_height + CRUISING_ALTITUDE_SURFACE_HEIGHT_OFFSET);
  }
  
  void extrude_stain(const float extrusion_mm) const {
    const float adjusted_extrusion_mm = extrusion_mm > 0.01 ? std::max(MINIMUM_EXTRUSION_MM, extrusion_mm) : 0;
    unscaled_e_move(STAINING_EXTRUSION_MULTIPLIER * adjusted_extrusion_mm, FEEDRATE_EXTRUDE_MM_S);
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
  
  xy_pos_t calculate_xy_using_corner_and_offset(const xy_pos_t position) const {
    const xy_pos_t p = _upper_left_corner + PROBE_TO_NOZZLE_XY_OFFSET + position;
    SERIAL_ECHOLNPGM("Corner (", _upper_left_corner.x, ",", _upper_left_corner.y, "), offset (", PROBE_TO_NOZZLE_XY_OFFSET.x, ",", PROBE_TO_NOZZLE_XY_OFFSET.y, "), for position of (", position.x, ",", position.y, ") calculated final position of (", p.x, ",", p.y, ")");
    return p;
  }
  
  float interpolate_surface_height(const xy_pos_t position, const float surface_offset_from_zero_height) const {
    //SERIAL_ECHOLNPGM("interpolating surface height for position of (", position.x, ", ", position.y, ") and surface offset of ", surface_offset_from_zero_height);
    float sum_of_inverse_squared_distances = 0;
    float total_surface_height = 0;
    //SERIAL_ECHOLNPGM("testing ", _all_elevations_probed_surface_heights.size(), " entries.");
    for (auto iter = _all_elevations_probed_surface_heights.begin(); iter != _all_elevations_probed_surface_heights.end(); ++iter) {
      //SERIAL_ECHOLNPGM("testing an entry in the probed surface heights");
      const std::vector<std::pair<xy_pos_t, float>> & this_elevations_probed_surface_heights = iter->second;
      //SERIAL_ECHOLNPGM("entry with surface offset ", iter->first, " has ", this_elevations_probed_surface_heights.size(), " points");
      if (std::fabs(surface_offset_from_zero_height - iter->first) < 0.1) {
        //SERIAL_ECHOLNPGM("entry has the right surface offset of ", surface_offset_from_zero_height);
        //SERIAL_ECHOLNPGM("entry has ", this_elevations_probed_surface_heights.size(), " points");
        for (unsigned int i = 0; i < this_elevations_probed_surface_heights.size(); ++i) {
          float diff_x = this_elevations_probed_surface_heights[i].first.x - position.x;
          float diff_y = this_elevations_probed_surface_heights[i].first.y - position.y;
          float squared_distance = diff_x * diff_x + diff_y * diff_y;
          // If we're super close to a point, just use its surface level.
          if (squared_distance < 4) {
            return this_elevations_probed_surface_heights[i].second;
          }
          float inverse = 1/squared_distance;
          sum_of_inverse_squared_distances += inverse;
          total_surface_height += this_elevations_probed_surface_heights[i].second * inverse;
        }
        float interpolated_surface_height = total_surface_height / sum_of_inverse_squared_distances;
        if (interpolated_surface_height > _max_surface_height + .01 || interpolated_surface_height < _min_surface_height - .01) {
          SERIAL_ECHOLNPGM("Calculated in invalid interpolated surface height of ", interpolated_surface_height, " when max is ", _max_surface_height, " and min is ", _min_surface_height);
          gcode.dwell(500000000);
        }
        SERIAL_ECHOLNPGM("For position (", position.x, ",", position.y, "), interpolated a surface height of ", interpolated_surface_height);
        return interpolated_surface_height;
      }
    }
  }
  
  float calculate_z(const xy_pos_t position, const float z, const float surface_offset_from_zero_height) const {
    const float interpolated_surface_height = interpolate_surface_height(position, surface_offset_from_zero_height);
    const float total_z = interpolated_surface_height + PROBE_TO_NOZZLE_Z_OFFSET + z;
    SERIAL_ECHOLNPGM("For position (", position.x, ",", position.y, "), calculated total z of ", total_z, " from interpolated surface height of ", interpolated_surface_height, ", offset of ", PROBE_TO_NOZZLE_Z_OFFSET, ", and z of ", z);
    return total_z;
  }
  
  void go_to_xy_from_upper_left_corner(const xy_pos_t position) const {
    go_to_xy(calculate_xy_using_corner_and_offset(position));
  }

  void go_to_xy_z_from_upper_left_corner(const xy_pos_t position, const float z, const float surface_offset_from_zero_height) const {
    go_to_xy_z(calculate_xy_using_corner_and_offset(position), calculate_z(position, z, surface_offset_from_zero_height));
  }

  void go_to_z_from_surface(const xy_pos_t position, const float z, const float surface_offset_from_zero_height) const {
    const float surface_z = calculate_z(position, z, surface_offset_from_zero_height);
    go_to_z(surface_z);
  }

public:
  Dabber(const xy_pos_t upper_left_corner, 
         const float surface_height,
         const std::map<double, std::vector<std::pair<xy_pos_t, float>>> all_elevations_probed_surface_heights) :
    _upper_left_corner(upper_left_corner), 
    _surface_height(surface_height), 
    _all_elevations_probed_surface_heights(all_elevations_probed_surface_heights) {
    _min_surface_height = 1000;
    _max_surface_height = 0;
    for(auto iter = _all_elevations_probed_surface_heights.begin(); iter != _all_elevations_probed_surface_heights.end(); ++iter) {
      for (unsigned int i = 0; i < iter->second.size(); ++i) {
        _min_surface_height = std::min(_min_surface_height, iter->second[i].second);
        _max_surface_height = std::max(_max_surface_height, iter->second[i].second);
      }
    }
  }

  void dab(const xy_pos_t position, const float extrusion_mm, const float surface_offset_from_zero_height, const xy_pos_t post_dab_scoot, const xy_pos_t approach_from, const bool is_a_boundary_dab) const {
    //SERIAL_ECHOLNPGM("Dabbing with position of (", position.x, ", ", position.y, "), surface offset of ", surface_offset_from_zero_height, ", post dab scoot of (", post_dab_scoot.x, ",", post_dab_scoot.y, "), approach from of (", approach_from.x, ",", approach_from.y, ")");
    // Make sure we're at cruising altitude
    go_to_cruising_altitude();
    // Even if approach_from is the zero vector, this works
    const xy_pos_t possible_offsetted_position = position + approach_from;
    //SERIAL_ECHOLNPGM("Moving to possible offsetted position of (", possible_offsetted_position.x, ", ", possible_offsetted_position.y, ")");
    //display_message_and_wait_for_button_push("Moving to offset");
    go_to_xy_from_upper_left_corner(possible_offsetted_position);
    // Extrude the stain for this dab
    unretract_and_extrude_stain(extrusion_mm);
    // Lower and do the actual dab
    const float embossing_extra_depth = is_a_boundary_dab ? 0 : NON_BOUNDARY_DAB_EXTRA_NOZZLE_DEPTH;
    //const float dabbing_height = DABBING_ELEVATION_ABOVE_SURFACE_HEIGHT + surface_offset_from_zero_height - embossing_extra_depth;
    const float dabbing_height = DABBING_ELEVATION_ABOVE_SURFACE_HEIGHT - embossing_extra_depth;
    if (!vector_is_zero(approach_from)) {
      const float total_z_offset = dabbing_height + APPROACH_FROM_Z_OFFSET + vector_magnitude(approach_from);
      //SERIAL_ECHOLNPGM("Preparing to approach at (", position.x, ", ", position.y, "), offset of (", dabbing_height, " + ", APPROACH_FROM_Z_OFFSET, " + ", vector_magnitude(approach_from), ") = ", total_z_offset);
      //display_message_and_wait_for_button_push("Preparing approach");
      go_to_z_from_surface(position, total_z_offset, surface_offset_from_zero_height);
      //SERIAL_ECHOLNPGM("Approaching to (", position.x, ", ", position.y, "), offset of (", dabbing_height, " + ", APPROACH_FROM_Z_OFFSET, ") = ", dabbing_height + APPROACH_FROM_Z_OFFSET);
      //display_message_and_wait_for_button_push("Approaching");
      go_to_xy_z_from_upper_left_corner(position, dabbing_height + APPROACH_FROM_Z_OFFSET, surface_offset_from_zero_height);
    }
    //SERIAL_ECHOLNPGM("At (", position.x, ", ", position.y, "), dropping to dabbing_height of ", dabbing_height);
    //display_message_and_wait_for_button_push("Dabbing");
    go_to_z_from_surface(position, dabbing_height, surface_offset_from_zero_height);
    //gcode.dwell(5000000);
    if (!vector_is_zero(post_dab_scoot)) {
      const xy_pos_t scooted_position = position + post_dab_scoot;
      //SERIAL_ECHOLNPGM("Post dab scooting to (", scooted_position.x, ", ", scooted_position.y, "), dropping to dabbing_height of ", dabbing_height);
      //display_message_and_wait_for_button_push("Scooting");
      go_to_xy_from_upper_left_corner(scooted_position);
      gcode.dwell(500);
      go_to_xy_from_upper_left_corner(position);
    }
    go_to_cruising_altitude();
    retract();
  }
};

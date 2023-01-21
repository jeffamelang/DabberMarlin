/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#include "../gcode.h"

#include "../../module/endstops.h"
#include "../../module/planner.h"
#include "../../module/stepper.h" // for various
#include "../../module/probe.h"

#include "dab_side_base_right.h"
#include "stain_utilities.h"
#include <map>
#include <vector>
#include "../../core/millis_t.h"

#if HAS_MULTI_HOTEND
        #include "../../module/tool_change.h"
#endif

#if HAS_LEVELING
        #include "../../feature/bedlevel/bedlevel.h"
#endif

#if ENABLED(BD_SENSOR)
        #include "../../feature/bedlevel/bdl/bdl.h"
#endif

#if ENABLED(SENSORLESS_HOMING)
        #include "../../feature/tmc_util.h"
#endif

#include "../../module/probe.h"

#if ENABLED(BLTOUCH)
        #include "../../feature/bltouch.h"
#endif

#include "../../lcd/marlinui.h"

#if ENABLED(EXTENSIBLE_UI)
        #include "../../lcd/extui/ui_api.h"
#elif ENABLED(DWIN_CREALITY_LCD)
        #include "../../lcd/e3v2/creality/dwin.h"
#elif ENABLED(DWIN_LCD_PROUI)
        #include "../../lcd/e3v2/proui/dwin.h"
#endif

#if ENABLED(LASER_FEATURE)
        #include "../../feature/spindle_laser.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../core/debug_out.h"

#if ENABLED(QUICK_HOME)
      
        static void quick_home_xy() {
      
          // Pretend the current position is 0,0
          current_position.set(0.0, 0.0);
          sync_plan_position();
      
          const int x_axis_home_dir = TOOL_X_HOME_DIR(active_extruder);
      
          // Use a higher diagonal feedrate so axes move at homing speed
          const float minfr = _MIN(homing_feedrate(X_AXIS), homing_feedrate(Y_AXIS)),
                      fr_mm_s = HYPOT(minfr, minfr);
      
          #if ENABLED(SENSORLESS_HOMING)
                  sensorless_t stealth_states {
                    NUM_AXIS_LIST(
                      TERN0(X_SENSORLESS, tmc_enable_stallguard(stepperX)),
                      TERN0(Y_SENSORLESS, tmc_enable_stallguard(stepperY)),
                      false, false, false, false
                    )
                    , TERN0(X2_SENSORLESS, tmc_enable_stallguard(stepperX2))
                    , TERN0(Y2_SENSORLESS, tmc_enable_stallguard(stepperY2))
                  };
          #endif
      
          do_blocking_move_to_xy(1.5 * max_length(X_AXIS) * x_axis_home_dir, 1.5 * max_length(Y_AXIS) * Y_HOME_DIR, fr_mm_s);
      
          endstops.validate_homing_move();
      
          current_position.set(0.0, 0.0);
      
          #if ENABLED(SENSORLESS_HOMING) && DISABLED(ENDSTOPS_ALWAYS_ON_DEFAULT)
                  TERN_(X_SENSORLESS, tmc_disable_stallguard(stepperX, stealth_states.x));
                  TERN_(X2_SENSORLESS, tmc_disable_stallguard(stepperX2, stealth_states.x2));
                  TERN_(Y_SENSORLESS, tmc_disable_stallguard(stepperY, stealth_states.y));
                  TERN_(Y2_SENSORLESS, tmc_disable_stallguard(stepperY2, stealth_states.y2));
          #endif
        }
      
#endif // QUICK_HOME

#if ENABLED(Z_SAFE_HOMING)
      
        inline void home_z_safely() {
          DEBUG_SECTION(log_G28, "home_z_safely", DEBUGGING(LEVELING));
      
          // Disallow Z homing if X or Y homing is needed
          if (homing_needed_error(_BV(X_AXIS) | _BV(Y_AXIS))) return;
      
          sync_plan_position();
      
          /**
           * Move the Z probe (or just the nozzle) to the safe homing point
           * (Z is already at the right height)
           */
          constexpr xy_float_t safe_homing_xy = { Z_SAFE_HOMING_X_POINT, Z_SAFE_HOMING_Y_POINT };
          #if HAS_HOME_OFFSET
                  xy_float_t okay_homing_xy = safe_homing_xy;
                  okay_homing_xy -= home_offset;
          #else
                  constexpr xy_float_t okay_homing_xy = safe_homing_xy;
          #endif
      
          destination.set(okay_homing_xy, current_position.z);
      
          TERN_(HOMING_Z_WITH_PROBE, destination -= probe.offset_xy);
      
          if (position_is_reachable(destination)) {
      
            if (DEBUGGING(LEVELING)) DEBUG_POS("home_z_safely", destination);
      
            // Free the active extruder for movement
            TERN_(DUAL_X_CARRIAGE, idex_set_parked(false));
      
            TERN_(SENSORLESS_HOMING, safe_delay(500)); // Short delay needed to settle
      
            do_blocking_move_to_xy(destination);
            homeaxis(Z_AXIS);
          }
          else {
            LCD_MESSAGE(MSG_ZPROBE_OUT);
            SERIAL_ECHO_MSG(STR_ZPROBE_OUT_SER);
          }
        }
      
#endif // Z_SAFE_HOMING

#if ENABLED(IMPROVE_HOMING_RELIABILITY)
      
        motion_state_t begin_slow_homing() {
          motion_state_t motion_state{0};
          motion_state.acceleration.set(planner.settings.max_acceleration_mm_per_s2[X_AXIS],
                                       planner.settings.max_acceleration_mm_per_s2[Y_AXIS]
                                       OPTARG(DELTA, planner.settings.max_acceleration_mm_per_s2[Z_AXIS])
                                     );
          planner.settings.max_acceleration_mm_per_s2[X_AXIS] = 100;
          planner.settings.max_acceleration_mm_per_s2[Y_AXIS] = 100;
          TERN_(DELTA, planner.settings.max_acceleration_mm_per_s2[Z_AXIS] = 100);
          #if HAS_CLASSIC_JERK
                  motion_state.jerk_state = planner.max_jerk;
                  planner.max_jerk.set(0, 0 OPTARG(DELTA, 0));
          #endif
          planner.refresh_acceleration_rates();
          return motion_state;
        }
      
        void end_slow_homing(const motion_state_t &motion_state) {
          planner.settings.max_acceleration_mm_per_s2[X_AXIS] = motion_state.acceleration.x;
          planner.settings.max_acceleration_mm_per_s2[Y_AXIS] = motion_state.acceleration.y;
          TERN_(DELTA, planner.settings.max_acceleration_mm_per_s2[Z_AXIS] = motion_state.acceleration.z);
          TERN_(HAS_CLASSIC_JERK, planner.max_jerk = motion_state.jerk_state);
          planner.refresh_acceleration_rates();
        }
      
#endif // IMPROVE_HOMING_RELIABILITY

/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 *  L<bool>   Force leveling state ON (if possible) or OFF after homing (Requires RESTORE_LEVELING_AFTER_G28 or ENABLE_LEVELING_AFTER_G28)
 *  O         Home only if the position is not known and trusted
 *  R<linear> Raise by n mm/inches before homing
 *
 * Cartesian/SCARA parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 */
void GcodeSuite::G28() {
  DEBUG_SECTION(log_G28, "G28", DEBUGGING(LEVELING));
  if (DEBUGGING(LEVELING)) log_machine_info();

  TERN_(BD_SENSOR, bdl.config_state = 0);

  /**
   * Set the laser power to false to stop the planner from processing the current power setting.
   */
  #if ENABLED(LASER_FEATURE)
          planner.laser_inline.status.isPowered = false;
  #endif

  #if ENABLED(DUAL_X_CARRIAGE)
          bool IDEX_saved_duplication_state = extruder_duplication_enabled;
          DualXMode IDEX_saved_mode = dual_x_carriage_mode;
  #endif

  #if ENABLED(MARLIN_DEV_MODE)
          if (parser.seen_test('S')) {
            LOOP_NUM_AXES(a) set_axis_is_at_home((AxisEnum)a);
            sync_plan_position();
            SERIAL_ECHOLNPGM("Simulated Homing");
            report_current_position();
            return;
          }
  #endif

  // Home (O)nly if position is unknown
  if (!axes_should_home() && parser.seen_test('O')) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("> homing not needed, skip");
    return;
  }

  #if ENABLED(FULL_REPORT_TO_HOST_FEATURE)
          const M_StateEnum old_grblstate = M_State_grbl;
          set_and_report_grblstate(M_HOMING);
  #endif

  TERN_(HAS_DWIN_E3V2_BASIC, DWIN_HomingStart());
  TERN_(EXTENSIBLE_UI, ExtUI::onHomingStart());

  planner.synchronize();          // Wait for planner moves to finish!

  SET_SOFT_ENDSTOP_LOOSE(false);  // Reset a leftover 'loose' motion state

  // Disable the leveling matrix before homing
  #if CAN_SET_LEVELING_AFTER_G28
          const bool leveling_restore_state = parser.boolval('L', TERN1(RESTORE_LEVELING_AFTER_G28, planner.leveling_active));
  #endif

  // Cancel any prior G29 session
  TERN_(PROBE_MANUALLY, g29_in_progress = false);

  // Disable leveling before homing
  TERN_(HAS_LEVELING, set_bed_leveling_enabled(false));

  // Reset to the XY plane
  TERN_(CNC_WORKSPACE_PLANES, workspace_plane = PLANE_XY);

  // Count this command as movement / activity
  reset_stepper_timeout();

  #define HAS_CURRENT_HOME(N) (defined(N##_CURRENT_HOME) && N##_CURRENT_HOME != N##_CURRENT)
  #if HAS_CURRENT_HOME(X) || HAS_CURRENT_HOME(X2) || HAS_CURRENT_HOME(Y) || HAS_CURRENT_HOME(Y2) || (ENABLED(DELTA) && HAS_CURRENT_HOME(Z)) || HAS_CURRENT_HOME(I) || HAS_CURRENT_HOME(J) || HAS_CURRENT_HOME(K) || HAS_CURRENT_HOME(U) || HAS_CURRENT_HOME(V) || HAS_CURRENT_HOME(W)
          #define HAS_HOMING_CURRENT 1
  #endif

  #if HAS_HOMING_CURRENT
          auto debug_current = [](FSTR_P const s, const int16_t a, const int16_t b) {
            DEBUG_ECHOF(s); DEBUG_ECHOLNPGM(" current: ", a, " -> ", b);
          };
          #if HAS_CURRENT_HOME(X)
                  const int16_t tmc_save_current_X = stepperX.getMilliamps();
                  stepperX.rms_current(X_CURRENT_HOME);
                  if (DEBUGGING(LEVELING)) debug_current(F(STR_X), tmc_save_current_X, X_CURRENT_HOME);
          #endif
          #if HAS_CURRENT_HOME(X2)
                  const int16_t tmc_save_current_X2 = stepperX2.getMilliamps();
                  stepperX2.rms_current(X2_CURRENT_HOME);
                  if (DEBUGGING(LEVELING)) debug_current(F(STR_X2), tmc_save_current_X2, X2_CURRENT_HOME);
          #endif
          #if HAS_CURRENT_HOME(Y)
                  const int16_t tmc_save_current_Y = stepperY.getMilliamps();
                  stepperY.rms_current(Y_CURRENT_HOME);
                  if (DEBUGGING(LEVELING)) debug_current(F(STR_Y), tmc_save_current_Y, Y_CURRENT_HOME);
          #endif
          #if HAS_CURRENT_HOME(Y2)
                  const int16_t tmc_save_current_Y2 = stepperY2.getMilliamps();
                  stepperY2.rms_current(Y2_CURRENT_HOME);
                  if (DEBUGGING(LEVELING)) debug_current(F(STR_Y2), tmc_save_current_Y2, Y2_CURRENT_HOME);
          #endif
          #if HAS_CURRENT_HOME(Z) && ENABLED(DELTA)
                  const int16_t tmc_save_current_Z = stepperZ.getMilliamps();
                  stepperZ.rms_current(Z_CURRENT_HOME);
                  if (DEBUGGING(LEVELING)) debug_current(F(STR_Z), tmc_save_current_Z, Z_CURRENT_HOME);
          #endif
          #if HAS_CURRENT_HOME(I)
                  const int16_t tmc_save_current_I = stepperI.getMilliamps();
                  stepperI.rms_current(I_CURRENT_HOME);
                  if (DEBUGGING(LEVELING)) debug_current(F(STR_I), tmc_save_current_I, I_CURRENT_HOME);
          #endif
          #if HAS_CURRENT_HOME(J)
                  const int16_t tmc_save_current_J = stepperJ.getMilliamps();
                  stepperJ.rms_current(J_CURRENT_HOME);
                  if (DEBUGGING(LEVELING)) debug_current(F(STR_J), tmc_save_current_J, J_CURRENT_HOME);
          #endif
          #if HAS_CURRENT_HOME(K)
                  const int16_t tmc_save_current_K = stepperK.getMilliamps();
                  stepperK.rms_current(K_CURRENT_HOME);
                  if (DEBUGGING(LEVELING)) debug_current(F(STR_K), tmc_save_current_K, K_CURRENT_HOME);
          #endif
          #if HAS_CURRENT_HOME(U)
                  const int16_t tmc_save_current_U = stepperU.getMilliamps();
                  stepperU.rms_current(U_CURRENT_HOME);
                  if (DEBUGGING(LEVELING)) debug_current(F(STR_U), tmc_save_current_U, U_CURRENT_HOME);
          #endif
          #if HAS_CURRENT_HOME(V)
                  const int16_t tmc_save_current_V = stepperV.getMilliamps();
                  stepperV.rms_current(V_CURRENT_HOME);
                  if (DEBUGGING(LEVELING)) debug_current(F(STR_V), tmc_save_current_V, V_CURRENT_HOME);
          #endif
          #if HAS_CURRENT_HOME(W)
                  const int16_t tmc_save_current_W = stepperW.getMilliamps();
                  stepperW.rms_current(W_CURRENT_HOME);
                  if (DEBUGGING(LEVELING)) debug_current(F(STR_W), tmc_save_current_W, W_CURRENT_HOME);
          #endif
          #if SENSORLESS_STALLGUARD_DELAY
                  safe_delay(SENSORLESS_STALLGUARD_DELAY); // Short delay needed to settle
          #endif
  #endif

  #if ENABLED(IMPROVE_HOMING_RELIABILITY)
          motion_state_t saved_motion_state = begin_slow_homing();
  #endif

  // Always home with tool 0 active
  #if HAS_MULTI_HOTEND
          #if DISABLED(DELTA) || ENABLED(DELTA_HOME_TO_SAFE_ZONE)
                  const uint8_t old_tool_index = active_extruder;
          #endif
          // PARKING_EXTRUDER homing requires different handling of movement / solenoid activation, depending on the side of homing
          #if ENABLED(PARKING_EXTRUDER)
                  const bool pe_final_change_must_unpark = parking_extruder_unpark_after_homing(old_tool_index, X_HOME_DIR + 1 == old_tool_index * 2);
          #endif
          tool_change(0, true);
  #endif

  TERN_(HAS_DUPLICATION_MODE, set_duplication_enabled(false));

  remember_feedrate_scaling_off();

  endstops.enable(true); // Enable endstops for next homing move

  #if ENABLED(DELTA)
      
          constexpr bool doZ = true; // for NANODLP_Z_SYNC if your DLP is on a DELTA
      
          home_delta();
      
          TERN_(IMPROVE_HOMING_RELIABILITY, end_slow_homing(saved_motion_state));
      
  #elif ENABLED(AXEL_TPARA)
      
          constexpr bool doZ = true; // for NANODLP_Z_SYNC if your DLP is on a TPARA
      
          home_TPARA();
      
  #else
      
          #define _UNSAFE(A) (homeZ && TERN0(Z_SAFE_HOMING, axes_should_home(_BV(A##_AXIS))))
      
          const bool homeZ = TERN0(HAS_Z_AXIS, parser.seen_test('Z')),
                     NUM_AXIS_LIST(              // Other axes should be homed before Z safe-homing
                       needX = _UNSAFE(X), needY = _UNSAFE(Y), needZ = false, // UNUSED
                       needI = _UNSAFE(I), needJ = _UNSAFE(J), needK = _UNSAFE(K),
                       needU = _UNSAFE(U), needV = _UNSAFE(V), needW = _UNSAFE(W)
                     ),
                     NUM_AXIS_LIST(              // Home each axis if needed or flagged
                       homeX = needX || parser.seen_test('X'),
                       homeY = needY || parser.seen_test('Y'),
                       homeZZ = homeZ,
                       homeI = needI || parser.seen_test(AXIS4_NAME), homeJ = needJ || parser.seen_test(AXIS5_NAME),
                       homeK = needK || parser.seen_test(AXIS6_NAME), homeU = needU || parser.seen_test(AXIS7_NAME),
                       homeV = needV || parser.seen_test(AXIS8_NAME), homeW = needW || parser.seen_test(AXIS9_NAME)
                     ),
                     home_all = NUM_AXIS_GANG(   // Home-all if all or none are flagged
                          homeX == homeX, && homeY == homeX, && homeZ == homeX,
                       && homeI == homeX, && homeJ == homeX, && homeK == homeX,
                       && homeU == homeX, && homeV == homeX, && homeW == homeX
                     ),
                     NUM_AXIS_LIST(
                       doX = home_all || homeX, doY = home_all || homeY, doZ = home_all || homeZ,
                       doI = home_all || homeI, doJ = home_all || homeJ, doK = home_all || homeK,
                       doU = home_all || homeU, doV = home_all || homeV, doW = home_all || homeW
                     );
      
          #if HAS_Z_AXIS
                  UNUSED(needZ); UNUSED(homeZZ);
          #else
                  constexpr bool doZ = false;
          #endif
      
          TERN_(HOME_Z_FIRST, if (doZ) homeaxis(Z_AXIS));
      
          const bool seenR = parser.seenval('R');
          const float z_homing_height = seenR ? parser.value_linear_units() : Z_HOMING_HEIGHT;
      
          if (z_homing_height && (seenR || NUM_AXIS_GANG(doX, || doY, || TERN0(Z_SAFE_HOMING, doZ), || doI, || doJ, || doK, || doU, || doV, || doW))) {
            // Raise Z before homing any other axes and z is not already high enough (never lower z)
            if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Raise Z (before homing) by ", z_homing_height);
            do_z_clearance(z_homing_height);
            TERN_(BLTOUCH, bltouch.init());
          }
      
          // Diagonal move first if both are homing
          TERN_(QUICK_HOME, if (doX && doY) quick_home_xy());
      
          // Home Y (before X)
          if (ENABLED(HOME_Y_BEFORE_X) && (doY || TERN0(CODEPENDENT_XY_HOMING, doX)))
            homeaxis(Y_AXIS);
      
          // Home X
          if (doX || (doY && ENABLED(CODEPENDENT_XY_HOMING) && DISABLED(HOME_Y_BEFORE_X))) {
      
            #if ENABLED(DUAL_X_CARRIAGE)
            
                    // Always home the 2nd (right) extruder first
                    active_extruder = 1;
                    homeaxis(X_AXIS);
            
                    // Remember this extruder's position for later tool change
                    inactive_extruder_x = current_position.x;
            
                    // Home the 1st (left) extruder
                    active_extruder = 0;
                    homeaxis(X_AXIS);
            
                    // Consider the active extruder to be in its "parked" position
                    idex_set_parked();
            
            #else
            
                    homeaxis(X_AXIS);
            
            #endif
          }
      
          #if BOTH(FOAMCUTTER_XYUV, HAS_I_AXIS)
                  // Home I (after X)
                  if (doI) homeaxis(I_AXIS);
          #endif
      
          // Home Y (after X)
          if (DISABLED(HOME_Y_BEFORE_X) && doY)
            homeaxis(Y_AXIS);
      
          #if BOTH(FOAMCUTTER_XYUV, HAS_J_AXIS)
                  // Home J (after Y)
                  if (doJ) homeaxis(J_AXIS);
          #endif
      
          TERN_(IMPROVE_HOMING_RELIABILITY, end_slow_homing(saved_motion_state));
      
          #if ENABLED(FOAMCUTTER_XYUV)
                  // skip homing of unused Z axis for foamcutters
                  if (doZ) set_axis_is_at_home(Z_AXIS);
          #else
                  // Home Z last if homing towards the bed
                  #if HAS_Z_AXIS && DISABLED(HOME_Z_FIRST)
                          if (doZ) {
                            #if EITHER(Z_MULTI_ENDSTOPS, Z_STEPPER_AUTO_ALIGN)
                                    stepper.set_all_z_lock(false);
                                    stepper.set_separate_multi_axis(false);
                            #endif
                  
                            #if ENABLED(Z_SAFE_HOMING)
                                    if (TERN1(POWER_LOSS_RECOVERY, !parser.seen_test('H'))) home_z_safely(); else homeaxis(Z_AXIS);
                            #else
                                    homeaxis(Z_AXIS);
                            #endif
                            probe.move_z_after_homing();
                          }
                  #endif
            
                  SECONDARY_AXIS_CODE(
                    if (doI) homeaxis(I_AXIS),
                    if (doJ) homeaxis(J_AXIS),
                    if (doK) homeaxis(K_AXIS),
                    if (doU) homeaxis(U_AXIS),
                    if (doV) homeaxis(V_AXIS),
                    if (doW) homeaxis(W_AXIS)
                  );
          #endif
      
          sync_plan_position();
      
  #endif

  /**
   * Preserve DXC mode across a G28 for IDEX printers in DXC_DUPLICATION_MODE.
   * This is important because it lets a user use the LCD Panel to set an IDEX Duplication mode, and
   * then print a standard GCode file that contains a single print that does a G28 and has no other
   * IDEX specific commands in it.
   */
  #if ENABLED(DUAL_X_CARRIAGE)
      
          if (idex_is_duplicating()) {
      
            TERN_(IMPROVE_HOMING_RELIABILITY, saved_motion_state = begin_slow_homing());
      
            // Always home the 2nd (right) extruder first
            active_extruder = 1;
            homeaxis(X_AXIS);
      
            // Remember this extruder's position for later tool change
            inactive_extruder_x = current_position.x;
      
            // Home the 1st (left) extruder
            active_extruder = 0;
            homeaxis(X_AXIS);
      
            // Consider the active extruder to be parked
            idex_set_parked();
      
            dual_x_carriage_mode = IDEX_saved_mode;
            set_duplication_enabled(IDEX_saved_duplication_state);
      
            TERN_(IMPROVE_HOMING_RELIABILITY, end_slow_homing(saved_motion_state));
          }
      
  #endif // DUAL_X_CARRIAGE

  endstops.not_homing();

  // Clear endstop state for polled stallGuard endstops
  TERN_(SPI_ENDSTOPS, endstops.clear_endstop_state());

  // Move to a height where we can use the full xy-area
  TERN_(DELTA_HOME_TO_SAFE_ZONE, do_blocking_move_to_z(delta_clip_start_height));

  TERN_(CAN_SET_LEVELING_AFTER_G28, if (leveling_restore_state) set_bed_leveling_enabled());

  restore_feedrate_and_scaling();

  // Restore the active tool after homing
  #if HAS_MULTI_HOTEND && (DISABLED(DELTA) || ENABLED(DELTA_HOME_TO_SAFE_ZONE))
          tool_change(old_tool_index, TERN(PARKING_EXTRUDER, !pe_final_change_must_unpark, DISABLED(DUAL_X_CARRIAGE)));   // Do move if one of these
  #endif

  #if HAS_HOMING_CURRENT
          if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Restore driver current...");
          #if HAS_CURRENT_HOME(X)
                  stepperX.rms_current(tmc_save_current_X);
          #endif
          #if HAS_CURRENT_HOME(X2)
                  stepperX2.rms_current(tmc_save_current_X2);
          #endif
          #if HAS_CURRENT_HOME(Y)
                  stepperY.rms_current(tmc_save_current_Y);
          #endif
          #if HAS_CURRENT_HOME(Y2)
                  stepperY2.rms_current(tmc_save_current_Y2);
          #endif
          #if HAS_CURRENT_HOME(Z) && ENABLED(DELTA)
                  stepperZ.rms_current(tmc_save_current_Z);
          #endif
          #if HAS_CURRENT_HOME(I)
                  stepperI.rms_current(tmc_save_current_I);
          #endif
          #if HAS_CURRENT_HOME(J)
                  stepperJ.rms_current(tmc_save_current_J);
          #endif
          #if HAS_CURRENT_HOME(K)
                  stepperK.rms_current(tmc_save_current_K);
          #endif
          #if HAS_CURRENT_HOME(U)
                  stepperU.rms_current(tmc_save_current_U);
          #endif
          #if HAS_CURRENT_HOME(V)
                  stepperV.rms_current(tmc_save_current_V);
          #endif
          #if HAS_CURRENT_HOME(W)
                  stepperW.rms_current(tmc_save_current_W);
          #endif
          #if SENSORLESS_STALLGUARD_DELAY
                  safe_delay(SENSORLESS_STALLGUARD_DELAY); // Short delay needed to settle
          #endif
  #endif // HAS_HOMING_CURRENT

  ui.refresh();

  TERN_(HAS_DWIN_E3V2_BASIC, DWIN_HomingDone());
  TERN_(EXTENSIBLE_UI, ExtUI::onHomingDone());

  report_current_position();

  if (ENABLED(NANODLP_Z_SYNC) && (doZ || ENABLED(NANODLP_ALL_AXIS)))
    SERIAL_ECHOLNPGM(STR_Z_MOVE_COMP);

  TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(old_grblstate));

}

    /* Side indexes:
    0. Base left (dragon)
    1. Base right (wolf)
    2. Base back
    3. Base front
    4. Base bottom
    5. Lid left (dragon)
    6. Lid right (wolf)
    7. Lid back
    8. Lid front
    9. Lid top
    */
enum Side { UNKNOWN, BASE_LEFT, BASE_RIGHT, BASE_BACK, BASE_FRONT, BASE_BOTTOM, LID_LEFT, LID_RIGHT, LID_BACK, LID_FRONT, LID_TOP};

Side identify_side_from_surface_height(const double surface_height) {
  const std::map<Side, float> expected_surface_heights =
    { 
      {BASE_FRONT, 147.3},
      {BASE_RIGHT, 149.8}
     };
  const double tolerance = 0.5;
  for (auto e : expected_surface_heights) {
    if ((fabs(surface_height - e.second)) < tolerance) {
      return e.first;
    }
  }
  return UNKNOWN;
}

float find_surface_height(const xy_pos_t & position, const float stop_height) {
  go_to_xy(position);
  return probe.probe_at_point(position, PROBE_PT_NONE, /*verbose_level=*/2, /*probe_relative=*/false, /*sanity_check=*/false, stop_height, /*number_of_probes=*/1);
}

float find_surface_height(const xy_pos_t & position) {
  const float height_at_which_slot_is_empty = 128;
  return find_surface_height(position, height_at_which_slot_is_empty);
}

float probe_at_point_to_height(const xy_pos_t & position, const float stop_height) {
  go_to_xy(position);
  return probe.probe_at_point(position, PROBE_PT_NONE, /*verbose_level=*/0, /*probe_relative=*/false, /*sanity_check=*/false, stop_height, /*number_of_probes=*/1);
}

// bottom left is the first quadrant, then goes clockwise around
xy_pos_t get_probing_location(const int quadrant, const int subquadrant) {
  const xy_pos_t bottom_left_quadrant_probing_location = {47.0, 80.0};
  const float quadrant_offsets[2] = {205.5, 205.0};
  const float subquadrant_offsets[2] = {98.2, 100.3};
  xy_pos_t probing_location = bottom_left_quadrant_probing_location;
  if (quadrant == 1 || quadrant == 2) {
    probing_location[1] += quadrant_offsets[1];
  }
  if (quadrant == 2 || quadrant == 3) {
    probing_location[0] += quadrant_offsets[0];
  }
  if (subquadrant == 1 || subquadrant == 2) {
    probing_location[1] += subquadrant_offsets[1];
  }
  if (subquadrant == 2 || subquadrant == 3) {
    probing_location[0] += subquadrant_offsets[0];
  }
  return probing_location;
  /*
  const xy_pos_t probing_locations[4][4] = 
  {
    {{25.0, 89.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}},
    {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}},
    {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}},
    {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}}
    };
    */
}

xy_pos_t multiply(const float scalar, const xy_pos_t vector) {
    return (xy_pos_t) {scalar * vector.x, scalar * vector.y};
}

enum ProbeResult { NO_SURFACE, SURFACE, SURFACE_BUT_GOT_STUCK };

ProbeResult probe_for_edge_at_point(const xy_pos_t & location, const float cruising_altitude, const float edge_found_height) {
  do_blocking_move_to_xy_z(location, cruising_altitude);
  millis_t start = millis();
  const float probed_height = probe_at_point_to_height(location, edge_found_height);
  millis_t stop = millis();
  go_to_z(cruising_altitude);
  float elapsedMillis = stop - start;
  SERIAL_ECHOLNPGM("Probing took ", elapsedMillis, " millis, got height of ", probed_height);
  bool stuck = elapsedMillis > 3700;
  if (stuck) {
    return ProbeResult::SURFACE_BUT_GOT_STUCK;
  }
  if (isnan(probed_height)) {
    return ProbeResult::NO_SURFACE;
  }
  return ProbeResult::SURFACE;
}

xy_pos_t find_edge_with_hint(const xy_pos_t & starting_location, const xy_pos_t & probing_direction, const float surface_height, const float extra_depth) {
  SERIAL_ECHOLNPGM("Finding edge with hint for start of ", starting_location.x, ",", starting_location.y);
  const float cruising_altitude = surface_height + 2.0;
  const float edge_found_height = surface_height - 0.5 - extra_depth;

  const float step_size = 0.1;
  float edge_offset = 0;
  int step = 0;
  int iteration_number = 0;
  int max_number_of_iterations = 20;
  while (iteration_number < max_number_of_iterations) {
    go_to_z(cruising_altitude);
    const xy_pos_t probing_location = starting_location + multiply(step * step_size, probing_direction);
    const ProbeResult probe_result = probe_for_edge_at_point(probing_location, cruising_altitude, edge_found_height);
    if (step == 0) {
      if (probe_result == ProbeResult::SURFACE || probe_result == ProbeResult::SURFACE_BUT_GOT_STUCK) {
        step = 1;
      } else {
        step = -1;
      }
    } else if (step > 0) {
      if (probe_result == ProbeResult::NO_SURFACE) {
        edge_offset = step * step_size;
        break;
      }
      SERIAL_ECHOLNPGM("with hint, nudging up");
      step += 1;
    } else if (step < 0) {
      if (probe_result == ProbeResult::SURFACE || probe_result == ProbeResult::SURFACE_BUT_GOT_STUCK) {
        edge_offset = (step + 1) * step_size;
        break;
      }
      SERIAL_ECHOLNPGM("with hint, nudging down");
      step -= 1;
    }
    ++iteration_number;
  }
  if (iteration_number == max_number_of_iterations) {
    SERIAL_ECHOLNPGM("ERROR ran out of iterations in find_edge_with_hint, returning NANs");
    return {NAN, NAN};
  }
  go_to_z(cruising_altitude);
  go_to_xy(starting_location);
  probe.deploy();
  probe.stow();
  const xy_pos_t edge_location = starting_location + multiply(edge_offset, probing_direction);
  SERIAL_ECHOLNPGM("Returning with final offset of ", edge_offset, " and edge location of (", edge_location.x, ",", edge_location.y, ")");
  return edge_location;
}

xy_pos_t find_edge_without_hint(const xy_pos_t & starting_location, const xy_pos_t & probing_direction, const float surface_height, const float extra_depth) {
  SERIAL_ECHOLNPGM("Finding edge WITHOUT hint for start of ", starting_location.x, ",", starting_location.y);
  const float cruising_altitude = surface_height + 2.0;
  const float edge_found_height = surface_height - 0.5 - extra_depth;

  // First, make sure that the starting location sees the surface.
  const ProbeResult starting_location_probe_result = probe_for_edge_at_point(starting_location, cruising_altitude, edge_found_height);
  go_to_z(cruising_altitude);
  if (!(starting_location_probe_result == ProbeResult::SURFACE || starting_location_probe_result == ProbeResult::SURFACE_BUT_GOT_STUCK)) {
    SERIAL_ECHOLNPGM("Didn't find the surface at the starting location");
    return {NAN, NAN};
  }
  const float step_sizes[4] = { 0.8, 0.4, 0.2, 0.1 };
  const int number_of_step_sizes = sizeof(step_sizes)/sizeof(step_sizes[0]);
  // current_offset is always assumed to be on the surface
  float current_offset = 0;
  const float MAX_ALLOWABLE_OFFSET = 8;
  float known_no_surface_offset = MAX_ALLOWABLE_OFFSET;
  for (int i = 0; i < number_of_step_sizes; ++i) {
    const float step_size = step_sizes[i];
    //SERIAL_ECHOLNPGM("current offset of ", current_offset, ", starting step size of ", step_size);
    bool stopped_seeing_surface = false;
    while (!stopped_seeing_surface) {
      if (current_offset > MAX_ALLOWABLE_OFFSET) {
        return {NAN, NAN};
      }
      const float test_offset = current_offset + step_size;
      if (test_offset > known_no_surface_offset - .01) {
        //SERIAL_ECHOLNPGM("Not testing offset of ", test_offset, " because we already know it doesn't see the surface");
        stopped_seeing_surface = true;
      } else {
        const xy_pos_t test_location = starting_location + multiply(test_offset, probing_direction);
        //SERIAL_ECHOLNPGM("Trying test offset of ", test_offset, " and diffs of ", test_location.x - starting_location.x, ",", test_location.y - starting_location.y, " for start of ", starting_location.x, ",", starting_location.y);
        const ProbeResult probe_result = probe_for_edge_at_point(test_location, cruising_altitude, edge_found_height);
        // Get the probe to a safe height
        go_to_z(cruising_altitude);
        if (probe_result == ProbeResult::SURFACE_BUT_GOT_STUCK) {
          //SERIAL_ECHOLNPGM("Trying to fix probe at ", test_offset);
          // Try to fix it
          probe.deploy();
          probe.stow();
        } 
        if (probe_result == ProbeResult::NO_SURFACE) {
          //SERIAL_ECHOLNPGM("Found no surface at ", test_offset);
          known_no_surface_offset = std::min(known_no_surface_offset, test_offset);
          stopped_seeing_surface = true;
        } else {
          //SERIAL_ECHOLNPGM("Found the surface at ", test_offset);
          current_offset = test_offset;
        }
      }
    }
  }
  go_to_z(cruising_altitude);
  go_to_xy(starting_location);
  probe.deploy();
  probe.stow();
  const float final_offset = current_offset + step_sizes[3];
  const xy_pos_t edge_location = starting_location + multiply(final_offset, probing_direction);
  SERIAL_ECHOLNPGM("Returning with final offset of ", final_offset, " and edge location of (", edge_location.x, ",", edge_location.y, ")");
  return edge_location;
}

std::vector<std::pair<xy_pos_t, float>> probe_leveling_points(const xy_pos_t & upper_left_corner, const Side side, const float surface_height) {
  const float cruising_altitude = surface_height + 2;
  const std::map<Side, std::vector<xy_pos_t>> MESH_LEVELING_POINTS = 
    { 
// 33 19,42,86
// 64 15,48,86
// 99 38, 86
      {BASE_RIGHT, {
        {3.4, -74.6},
        {3.4, -51.6},
        {3.4, -7.6},
        {34.4, -78.6},
        {34.4, -45.6},
        {34.4, -7.6},
        {69.4, -55.6},
        {69.4, -7.6}
      }}
    };
  const std::vector<xy_pos_t> leveling_points = MESH_LEVELING_POINTS.find(side)->second;
  std::vector<std::pair<xy_pos_t, float>> probed_points;
  SERIAL_ECHOLNPGM("Probing mesh leveling points");
  go_to_z(cruising_altitude);
  for (const xy_pos_t & unoffsetted_probing_location : leveling_points) {
    const xy_pos_t probing_location = upper_left_corner + unoffsetted_probing_location;
    const float surface_height = find_surface_height(probing_location);
    SERIAL_ECHOLNPGM("For probing location (", probing_location.x, ",", probing_location.y, ") found height of ", surface_height);
    probed_points.push_back(std::pair(unoffsetted_probing_location, surface_height));
    go_to_z(cruising_altitude);
  }
  return probed_points;
}

xy_pos_t find_upper_left_corner(const xy_pos_t & probing_location, const Side side, const float surface_height, const xy_pos_t upper_left_corner_hint) {
  const std::map<Side, xy_pos_t> PROBING_LOCATION_TO_TOP_EDGE_INITIAL_OFFSETS = 
    { 
      {BASE_FRONT, {18, 12.5}},
      {BASE_RIGHT, {10, 9.5}} 
    };
  const std::map<Side, xy_pos_t> PROBING_LOCATION_TO_LEFT_EDGE_INITIAL_OFFSETS =
    { 
      {BASE_FRONT, {-21.5, -35}},
      {BASE_RIGHT, {-21.5, -15}} 
    };
  const std::map<Side, float> EXTRA_DEPTH_LEFT_SIDE =
    { 
      {BASE_FRONT, 0.},
      {BASE_RIGHT, 0.}
     };
  const std::map<Side, float> EXTRA_DEPTH_TOP_SIDE =
    { 
      {BASE_FRONT, 3.},
      {BASE_RIGHT, 0.} 
      };
  // Sometimes we can't reach the top left corner, such as for the base right.
  // For that, we probe what we can, and use a hard-code offset from the 
  // corner that we can probe to the actual corner.
  const std::map<Side, xy_pos_t> AFTER_PROBING_CORNER_FIND_OFFSET =
    { 
      {BASE_FRONT, {0.0, 0.0}},
      {BASE_RIGHT, {0.0, 3.0}} 
    };
  SERIAL_ECHOLNPGM("Finding top edge, offset is (", PROBING_LOCATION_TO_TOP_EDGE_INITIAL_OFFSETS.find(side)->second.x, ",", PROBING_LOCATION_TO_TOP_EDGE_INITIAL_OFFSETS.find(side)->second.y, ")");
  xy_pos_t top_edge = (xy_pos_t) {NAN, NAN};
  const xy_pos_t top_edge_starting_location = probing_location + PROBING_LOCATION_TO_TOP_EDGE_INITIAL_OFFSETS.find(side)->second;
  if (!isnan(upper_left_corner_hint.y)) {
    SERIAL_ECHOLNPGM("Finding top edge with hint");
    const xy_pos_t top_edge_starting_location_with_hint = {top_edge_starting_location.x, upper_left_corner_hint.y};
    top_edge = find_edge_with_hint(top_edge_starting_location_with_hint, {0., 1.}, surface_height, EXTRA_DEPTH_TOP_SIDE.find(side)->second);
  } else {
    SERIAL_ECHOLNPGM("Finding top edge WITHOUT hint");
    top_edge = find_edge_without_hint(top_edge_starting_location, {0., 1.}, surface_height, EXTRA_DEPTH_TOP_SIDE.find(side)->second);
  }
  xy_pos_t left_edge = (xy_pos_t) {NAN, NAN};
  const xy_pos_t left_edge_starting_location = probing_location + PROBING_LOCATION_TO_LEFT_EDGE_INITIAL_OFFSETS.find(side)->second;
  if (!isnan(upper_left_corner_hint.x)) {
    SERIAL_ECHOLNPGM("Finding left edge with hint");
    const xy_pos_t left_edge_starting_location_with_hint = {upper_left_corner_hint.x, left_edge_starting_location.y};
    left_edge = find_edge_with_hint(left_edge_starting_location_with_hint, {-1., 0.}, surface_height, EXTRA_DEPTH_LEFT_SIDE.find(side)->second);
  } else {
    SERIAL_ECHOLNPGM("Finding left edge WITHOUT hint");
    left_edge = find_edge_without_hint(left_edge_starting_location, {-1., 0.}, surface_height, EXTRA_DEPTH_LEFT_SIDE.find(side)->second);
  }
  if (isnan(top_edge.x) || isnan(top_edge.y) || isnan(left_edge.x) || isnan(left_edge.y)) {
    SERIAL_ECHOLNPGM("Couldn't find upper left corner");
    return (xy_pos_t) {NAN, NAN};
  }
  const xy_pos_t probed_corner = {left_edge.x, top_edge.y};
  SERIAL_ECHOLNPGM("Found probed corner of (", probed_corner.x, ",", probed_corner.y, ")");
  const xy_pos_t probe_offset = {0.5, -0.3};
  const xy_pos_t upper_left_corner = (xy_pos_t){left_edge.x, top_edge.y} + probe_offset + AFTER_PROBING_CORNER_FIND_OFFSET.find(side)->second;
  SERIAL_ECHOLNPGM("With probe and hard-coded offsets, returning upper left corner of (", upper_left_corner.x, ",", upper_left_corner.y, ")");
  go_to_xy(upper_left_corner);
  return upper_left_corner;
}

/*
bottom left probing position is x=25, y=89

auto home
move up to safe height
slot empty height = something

for each quadrant
  for each probing position
    move to probing position
    move up until bltouch triggers or hit slot empty height
    if bltouch triggered, use height to determine type of side
    store slot height
  make sure that all four slots registered either nothing or the same side.
  for each slot that registered the side
    find probable corner
      go to probing position
      use offsets for that type of side
      probe every 0.1 or something and sink down to slot height - 1 until sink all the way. 
      if sink all the way on first try, abandon slot.
    top left corner is the probable corner plus a side-specific offset.
    run the stainSide function for that side, passing it the top left corner
    */
void GcodeSuite::M1399() { 
  G28();
  const double cruising_altitude = 160;

  const int number_of_quadrants = 1;
  const int number_of_subquadrants = 4;

  std::vector<Side> quadrant_sides(number_of_quadrants, Side::UNKNOWN);
  std::vector<std::vector<float>> surface_heights(number_of_quadrants, std::vector<float>(number_of_subquadrants, NAN));
  std::vector<std::vector<xy_pos_t>> upper_left_corners(number_of_quadrants, std::vector<xy_pos_t>(number_of_subquadrants, {NAN, NAN}));
  std::vector<std::vector<std::vector<std::pair<xy_pos_t, float>>>> surface_probing_points(number_of_quadrants, std::vector<std::vector<std::pair<xy_pos_t, float>>>(number_of_subquadrants));
  for (int quadrant = 0; quadrant < number_of_quadrants; ++quadrant) {
    // Try to identify all sides in this quadrant
    for (int subquadrant = 0; subquadrant < number_of_subquadrants; ++subquadrant) {
      // Make sure we're at a safe altitude to move around without running into things.
      //SERIAL_ECHOLNPGM("(", quadrant, ":", subquadrant, "), Doing a blocking move to cruising altitude of ", cruising_altitude);
      go_to_z(cruising_altitude);
      // Determine the probing location and probe
      const xy_pos_t probing_location = get_probing_location(quadrant, subquadrant);
      //SERIAL_ECHOLNPGM("(", quadrant, ":", subquadrant, "), Finding surface height at probing_location of (", probing_location.x, ", ", probing_location.y, ")");
      const float surface_height = find_surface_height(probing_location);
      SERIAL_ECHOPGM("(", quadrant, ":", subquadrant, "), Found surface height of");
      if (isnan(surface_height)) {
        SERIAL_ECHOPGM(" NO SIDE");
      } else {
        SERIAL_ECHOPGM(" ", surface_height);
      }
      SERIAL_EOL();
      surface_heights[quadrant][subquadrant] = surface_height;
    }
    // Make sure we're at a safe altitude to move around without running into things.
    go_to_z(cruising_altitude);

    // Make sure that all subquadrants identify the same side or no side
    float probed_height_total = 0;
    int number_of_non_empty_sides = 0;
    for (int subquadrant = 0; subquadrant < number_of_subquadrants; ++subquadrant) {
      const float probed_height = surface_heights[quadrant][subquadrant];
      if (!isnan(probed_height)) {
        ++number_of_non_empty_sides;
        probed_height_total += probed_height;
      }
    }
    if (probed_height_total < 0.01) {
      SERIAL_ECHOLNPGM("Quadrant ", quadrant, ", it seems like there are no boxes, so we're skipping the quadrant.");
      continue;
    }
    const float average_surface_height = probed_height_total / number_of_non_empty_sides;
    const Side quadrant_side = identify_side_from_surface_height(average_surface_height);
    quadrant_sides[quadrant] = quadrant_side;
    if (quadrant_side == UNKNOWN) {
      SERIAL_ECHOLNPGM("Could not identify side from average surface height of ", average_surface_height, ", skipping the quadrant.");
      continue;
    }
    SERIAL_ECHOLNPGM("Quadrant ", quadrant, ", found quadrant side of ", quadrant_side);

    // Now, gather data about each side in the quadrant.
    std::vector<xy_pos_t> subquadrant_upper_left_corner_hints(4, {NAN, NAN});
    for (int subquadrant = 0; subquadrant < number_of_subquadrants; ++subquadrant) {
      // Skip this side if we didn't find anything.
      if (isnan(surface_heights[quadrant][subquadrant])) {
        // There is no side here; move to the next one
        continue;
      }
      const float surface_height = surface_heights[quadrant][subquadrant];

      // By this time, we've identified a side, and we're almost ready to 
      // start dabbing. First, we have to find the upper left corner. 
      const xy_pos_t probing_location = get_probing_location(quadrant, subquadrant);
      SERIAL_ECHOLNPGM("\n(", quadrant, ":", subquadrant, "), ==================== Looking for upper left corner ==================\n");
      const xy_pos_t upper_left_corner = find_upper_left_corner(probing_location, quadrant_side, surface_height, subquadrant_upper_left_corner_hints[subquadrant]);
      if (subquadrant == 0) {
        subquadrant_upper_left_corner_hints[1].x = upper_left_corner.x;
        subquadrant_upper_left_corner_hints[3].y = upper_left_corner.y;
      }
      if (subquadrant == 1) {
        subquadrant_upper_left_corner_hints[2].y = upper_left_corner.y;
      }
      if (subquadrant == 2) {
        subquadrant_upper_left_corner_hints[3].x = upper_left_corner.x;
      }
      upper_left_corners[quadrant][subquadrant] = upper_left_corner;
      //xy_pos_t upper_left_corner = {30.0000,93.8000};

      if (isnan(upper_left_corner.x) || isnan(upper_left_corner.y)) {
        SERIAL_ECHOLNPGM("(", quadrant, ":", subquadrant, "), Could not find the upper left corner of this face, skipping.");
        continue;
      } 

      SERIAL_ECHOLNPGM("(", quadrant, ":", subquadrant, "), SUCCESSFULLY found upper left corner at (", upper_left_corner.x, ",", upper_left_corner.y, ") and surface height of ", surface_height, ", probing leveling points.");
      surface_probing_points[quadrant][subquadrant] = probe_leveling_points(upper_left_corner, quadrant_side, surface_height);
      SERIAL_ECHOLNPGM("(", quadrant, ":", subquadrant, "), SUCCESSFULLY probed leveling points, proceeding to dab with corner (", upper_left_corner.x, ",", upper_left_corner.y, ") and surface height of ", surface_height);
    }
  }
  SERIAL_ECHOLNPGM("\n\n----------------------------------");
  SERIAL_ECHOLNPGM("Done with probing, moving on to dabbing");
  SERIAL_ECHOLNPGM("----------------------------------\n");
  // Now that we've done all of the measuring for the sides, start dabbing.
  for (int quadrant = 0; quadrant < number_of_quadrants; ++quadrant) {
    const Side quadrant_side = quadrant_sides[quadrant];
    if (quadrant_side == UNKNOWN) {
      SERIAL_ECHOLNPGM("We weren't able to determine a side type for quadrant ", quadrant, ", so we're skipping it.");
      continue;
    }
    for (int subquadrant = 0; subquadrant < number_of_subquadrants; ++subquadrant) {
      const float surface_height = surface_heights[quadrant][subquadrant];
      const xy_pos_t upper_left_corner = upper_left_corners[quadrant][subquadrant];
      const std::vector<std::pair<xy_pos_t, float>> & probing_points = surface_probing_points[quadrant][subquadrant];
      SERIAL_ECHOLNPGM("(", quadrant, ":", subquadrant, "), Dabbing with corner (", upper_left_corner.x, ",", upper_left_corner.y, ") and surface height of ", surface_height , " and ", probing_points.size(), " probing points");
      // Skip this side if we don't have all the information
      if (isnan(surface_height) || isnan(upper_left_corner.x) || isnan(upper_left_corner.y) || probing_points.size() == 0) {
        SERIAL_ECHOLNPGM("(", quadrant, ":", subquadrant, "), we seem to be missing some data, so we're skipping it.");
        // There is no side here; move to the next one
        continue;
      }
      // Now that we know the side and we've found the upper left corner, we call the dabbing routine specific to that side.
      const Dabber* dabber = new Dabber(upper_left_corner, surface_height, probing_points);
      if (quadrant_side == BASE_RIGHT) {
        SERIAL_ECHOLNPGM("Starting to dab a base right");
        dab_side_base_right(dabber);
      }
      delete dabber;
    }
  }
  SERIAL_ECHOLNPGM("\n\n----------------------------------");
  SERIAL_ECHOLNPGM("Done with dabbing! Dropping the bed.");
  SERIAL_ECHOLNPGM("----------------------------------\n");
  go_to_z(400);
}

void GcodeSuite::M1099() { 
  G28();
  const float cruising_altitude = 80;
  const xy_pos_t primary_probing_locations[4] = 
  {
    {41, 15},
    {41, 370},
    {377, 370},
    {377, 15},
  };
  const int number_of_primary_probing_locations = sizeof(primary_probing_locations)/sizeof(primary_probing_locations[0]);
  float primary_probing_location_heights[4] = {0., 0., 0., 0.};
  for (int i = 0; i < number_of_primary_probing_locations; ++i) {
    const xy_pos_t probing_location = primary_probing_locations[i];
    go_to_z(cruising_altitude);
    const float surface_height = find_surface_height(probing_location, 3);
    primary_probing_location_heights[i] = surface_height;
    SERIAL_ECHOLNPGM("Probing location ", i, " at (", probing_location.x, ",", probing_location.y, ") has height of ", surface_height);
  }
  go_to_z(cruising_altitude);
  float min_height = 100;
  float max_height = 0;
  for (int i = 0; i < number_of_primary_probing_locations; ++i) {
    const float surface_height = primary_probing_location_heights[i];
    min_height = min(min_height, surface_height);
    max_height = max(max_height, surface_height);
  }
  if ((max_height - min_height) < 0.25) {
    SERIAL_ECHOLNPGM("Bed is level, reporting secondary deviances");
    const float average_height = (max_height + min_height)/2.;
    const xy_pos_t secondary_probing_locations[12] = 
    {
      {41, 165},
      {41, 220},
      {171, 15},
      {171, 165},
      {171, 220},
      {171, 370},
      {248, 15},
      {248, 165},
      {248, 220},
      {248, 370},
      {377, 165},
      {377, 220},
    };
    const int number_of_secondary_probing_locations = sizeof(secondary_probing_locations)/sizeof(secondary_probing_locations[0]);
    for (int i = 0; i < number_of_secondary_probing_locations; ++i) {
      const xy_pos_t probing_location = secondary_probing_locations[i];
      go_to_z(cruising_altitude);
      const float surface_height = find_surface_height(probing_location, 3);
      SERIAL_ECHOLNPGM("Probing location ", i, " at (", probing_location.x, ",", probing_location.y, ") has deviance of ", surface_height - average_height);
    }
    go_to_z(cruising_altitude);
  } else {
    SERIAL_ECHOLNPGM("Bed is NOT level, adjust it");
    SERIAL_ECHOLNPGM("Screws start from bottom left and go clockwise around");
    const float height_per_rotation = 0.5;
    for (int i = 0; i < number_of_primary_probing_locations; ++i) {
      const float height_difference = primary_probing_location_heights[i] - min_height;
      SERIAL_ECHOLNPGM("Turn screw ", i, " ", height_difference/height_per_rotation, " turns clockwise");
    }
  }
}

// Purging
void GcodeSuite::M1199() {
  const float feedrate_extrude_mm_s = 70;
  const float purging_mm = 4000;
  unscaled_e_move(purging_mm, feedrate_extrude_mm_s);
}


// Priming
void GcodeSuite::M1299() {
  const float feedrate_extrude_mm_s = 70;
  const float purging_mm = 30;
  unscaled_e_move(purging_mm, feedrate_extrude_mm_s);
}
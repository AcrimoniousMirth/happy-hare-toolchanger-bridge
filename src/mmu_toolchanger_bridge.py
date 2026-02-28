# Happy Hare MMU Software - Toolchanger Bridge
#
# Goal: Provide dynamic extruder and sensor switching for multi-toolhead setups.
#
# Architecture:
#   T0  -> extruder  (single filament, gate-fed by HH via long bowden to toolhead 0)
#   T1+ -> extruder1 (MMU multi-material toolhead)
#
# On each toolhead switch, this bridge updates:
#   1. mmu.extruder_name and mmu_extruder_stepper references
#   2. The extruder stepper on gear_rail extra endstops (for rapid stop on homing)
#   3. HH's live sensor dict (all_sensors) to reflect the active toolhead's sensors
#   4. mmu.bowden_homing_max and mmu.gate_homing_max to match the active toolhead's
#      bowden tube length (T0 has a much longer path than T1-T5)
#
# Sensor lookup uses a numeric suffix derived from the extruder name:
#   extruder  -> suffix 0 -> mmu_extruder_0, mmu_toolhead_0, mmu_tension_0
#   extruder1 -> suffix 1 -> mmu_extruder_1, mmu_toolhead_1, mmu_tension_1
#
# These should match existing [filament_switch_sensor] sections in your config.
#
# Config:
#   [mmu_toolchanger_bridge]
#   sensor_prefix: mmu        # prefix for sensor lookup (default: mmu)
#   t0_bowden_max: 2000       # mm: bowden + gate homing ceiling for T0 path (default: 2000)
#   t0_extruder: extruder     # the Klipper extruder name for T0 (default: extruder)
#
# Startup auto-apply:
#   At klippy:connect, if the currently active gate maps to T0 (mmu.extruder_name
#   matches t0_extruder), bridge settings are applied automatically so that preload
#   triggered by the pre-gate sensor works immediately without a manual toolchange.
#
# Bowden length for T0:
#   Uncalibrated: gate_homing_max and bowden_homing_max are set to t0_bowden_max.
#   Calibrated:   HH reads mmu.bowden_lengths[gate] from mmu_vars.cfg automatically
#                 (key: 'mmu_calibration_bowden_lengths'). gate_homing_max is still
#                 set to t0_bowden_max to allow the preload homing move to reach
#                 mmu_extruder_N in one pass.
#
# Copyright (C) 2024  Antigravity / Google Deepmind
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

class MmuToolchangerBridge:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')

        # Prefix used when looking up per-toolhead sensors.
        # With default 'mmu', looks for: mmu_toolhead_N, mmu_extruder_N, mmu_tension_N
        self.sensor_prefix = config.get('sensor_prefix', 'mmu')

        # The Klipper extruder name that corresponds to T0 (the non-MMU toolhead).
        # Bridge applies t0_bowden_max and T0 sensors whenever this extruder is active.
        self.t0_extruder = config.get('t0_extruder', 'extruder')

        # Maximum homing search distance for the T0 path (gate + bowden combined).
        # Applied to BOTH gate_homing_max and bowden_homing_max when T0 is active.
        # Once gate 0 is calibrated via MMU_CALIBRATE_BOWDEN, the stored value in
        # mmu_vars.cfg is used for the bowden move; gate_homing_max still uses this
        # ceiling so preload can reach mmu_extruder_0 in a single homing pass.
        self.t0_bowden_max = config.getfloat('t0_bowden_max', 2000., above=0.)

        # Saved HH defaults (captured at klippy:connect)
        self._orig_bowden_homing_max = None
        self._orig_gate_homing_max = None

        self.printer.register_event_handler('klippy:connect', self._handle_connect)

        self.gcode.register_command(
            'SET_MMU_EXTRUDER', self.cmd_SET_MMU_EXTRUDER,
            desc="Dynamically switch Happy Hare's active extruder and sensors"
        )

    # -------------------------------------------------------------------------

    def _handle_connect(self):
        """Save HH defaults and auto-apply T0 mode if T0 is already active at boot."""
        mmu = self.printer.lookup_object('mmu', None)
        if mmu is None:
            return

        # Save originals so we can restore them when T1+ is active
        self._orig_bowden_homing_max = mmu.bowden_homing_max
        self._orig_gate_homing_max   = getattr(mmu, 'gate_homing_max', None)

        logging.info(
            "MMU Toolchanger Bridge: defaults captured — "
            "bowden_homing_max=%.1fmm, gate_homing_max=%s, T0 max=%.1fmm"
            % (self._orig_bowden_homing_max,
               str(self._orig_gate_homing_max),
               self.t0_bowden_max))

        # Auto-apply T0 settings if HH already has extruder (T0) as the configured
        # extruder at startup, so that automatic preload triggered by the pre-gate
        # sensor works without needing a manual toolchange first.
        if mmu.extruder_name == self.t0_extruder:
            logging.info(
                "MMU Toolchanger Bridge: startup extruder is '%s' — "
                "auto-applying T0 bridge settings" % self.t0_extruder)
            self._apply_settings(mmu, self.t0_extruder)

    # -------------------------------------------------------------------------

    def _get_suffix(self, extruder_name):
        """Derive numeric suffix from extruder name.
        'extruder' -> '0',  'extruder1' -> '1',  'extruder2' -> '2', etc.
        """
        return extruder_name.replace('extruder', '') or '0'

    def _lookup_sensor(self, name):
        """Look up a filament_switch_sensor by name, returning None if absent."""
        return self.printer.lookup_object(
            "filament_switch_sensor %s" % name, None)

    def _apply_settings(self, mmu, extruder_name):
        """Core logic: update extruder, endstops, homing distances, and sensors."""

        # --- 1. Resolve the target extruder ---
        printer_extruder = self.printer.lookup_object(extruder_name, None)
        if printer_extruder is None:
            raise self.printer.command_error(
                "Could not find extruder '%s'" % extruder_name)
        if not hasattr(printer_extruder, 'extruder_stepper'):
            raise self.printer.command_error(
                "Extruder '%s' has no stepper attached" % extruder_name)

        new_stepper = printer_extruder.extruder_stepper

        # --- 2. Update MMU extruder name and stepper references ---
        mmu.extruder_name = extruder_name
        mmu.mmu_toolhead.mmu_extruder_stepper = new_stepper
        mmu.mmu_extruder_stepper = new_stepper

        # --- 3. Update extruder stepper on homing-related gear rail endstops ---
        if mmu.mmu_machine.homing_extruder:
            new_mcu_stepper = getattr(new_stepper, 'stepper', None)
            if new_mcu_stepper is None and hasattr(new_stepper, 'get_steppers'):
                steppers = new_stepper.get_steppers()
                new_mcu_stepper = steppers[0] if steppers else None

            if new_mcu_stepper:
                extruder_endstop_names = {
                    mmu.SENSOR_TOOLHEAD,
                    mmu.SENSOR_COMPRESSION,
                    mmu.SENSOR_TENSION,
                }
                gear_stepper_ids = {
                    id(s) for s in mmu.gear_rail.steppers
                    if hasattr(s, 'get_name')
                }
                for (mcu_endstop, name) in mmu.gear_rail.extra_endstops:
                    if name in extruder_endstop_names:
                        if hasattr(mcu_endstop, 'steppers'):
                            mcu_endstop.steppers = [
                                s for s in mcu_endstop.steppers
                                if id(s) in gear_stepper_ids
                            ] + [new_mcu_stepper]
                        elif hasattr(mcu_endstop, 'add_stepper'):
                            try:
                                mcu_endstop.add_stepper(new_mcu_stepper)
                            except Exception:
                                pass

        # --- 4. Set homing distance limits for the active toolhead ---
        # T0 has a long bowden from the MMU gate to toolhead 0's extruder sensor.
        # Both bowden_homing_max (used for full load) and gate_homing_max (used for
        # preload) must be set large enough to cover the entire path in one pass.
        is_t0 = (extruder_name == self.t0_extruder)

        new_bowden_max = self.t0_bowden_max if is_t0 else self._orig_bowden_homing_max
        new_gate_max   = self.t0_bowden_max if is_t0 else self._orig_gate_homing_max

        if self._orig_bowden_homing_max is not None:
            mmu.bowden_homing_max = new_bowden_max

        if self._orig_gate_homing_max is not None and hasattr(mmu, 'gate_homing_max'):
            mmu.gate_homing_max = new_gate_max

        gate = mmu.gate_selected if mmu.gate_selected >= 0 else 0
        calibrated = (mmu.bowden_lengths[gate]
                      if hasattr(mmu, 'bowden_lengths') and gate < len(mmu.bowden_lengths)
                      else -1)
        if calibrated > 0:
            mmu.log_info(
                "MMU: Homing max set to gate=%.1fmm bowden=%.1fmm for '%s' "
                "(calibrated gate %d length: %.1fmm)"
                % (new_gate_max, new_bowden_max, extruder_name, gate, calibrated))
        else:
            mmu.log_info(
                "MMU: Homing max set to gate=%.1fmm bowden=%.1fmm for '%s' "
                "(gate %d uncalibrated — run MMU_CALIBRATE_BOWDEN after gear calibration)"
                % (new_gate_max, new_bowden_max, extruder_name, gate))

        # --- 5. Swap sensors in HH's live all_sensors dict ---
        suffix = self._get_suffix(extruder_name)
        p = self.sensor_prefix  # e.g. 'mmu'

        sensor_map = {
            mmu.SENSOR_EXTRUDER_ENTRY: "%s_extruder_%s" % (p, suffix),
            mmu.SENSOR_TOOLHEAD:       "%s_toolhead_%s" % (p, suffix),
            mmu.SENSOR_TENSION:        "%s_tension_%s"  % (p, suffix),
            # For T0, mmu_extruder_0 is both the extruder entry and the gate
            # homing target (no separate mmu_gate sensor in the T0 filament path).
            mmu.SENSOR_GATE:           "%s_extruder_%s" % (p, suffix),
        }

        for hh_key, sensor_name in sensor_map.items():
            sensor_obj = self._lookup_sensor(sensor_name)
            if sensor_obj is not None:
                mmu.sensor_manager.all_sensors[hh_key] = sensor_obj
            else:
                mmu.sensor_manager.all_sensors.pop(hh_key, None)

        # Refresh HH's viewable_sensors for the current gate/UI
        mmu.sensor_manager.reset_active_gate(mmu.gate)

        mmu.log_info(
            "MMU: Active extruder switched to '%s' (sensor suffix: %s)"
            % (extruder_name, suffix))

    # -------------------------------------------------------------------------

    def cmd_SET_MMU_EXTRUDER(self, gcmd):
        mmu = self.printer.lookup_object('mmu', None)
        if mmu is None:
            gcmd.respond_info("MMU object not found")
            return

        extruder_name = gcmd.get('EXTRUDER', 'extruder')

        try:
            self._apply_settings(mmu, extruder_name)
        except Exception as e:
            import traceback
            error_msg = "Failed to switch extruder: %s\n%s" % (str(e), traceback.format_exc())
            logging.error("MMU Toolchanger Bridge: %s" % error_msg)
            raise self.printer.command_error(
                "Failed to switch extruder to '%s'. Check mmu.log/klippy.log for details."
                % extruder_name)

def load_config(config):
    return MmuToolchangerBridge(config)

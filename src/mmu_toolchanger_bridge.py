# Happy Hare MMU Software - Toolchanger Bridge
#
# Goal: Provide dynamic extruder and sensor switching for multi-toolhead setups.
#
# Architecture:
#   T0  -> extruder  (single filament, gate-fed by HH)
#   T1+ -> extruder1 (MMU multi-material toolhead)
#
# On each toolhead switch, this bridge updates:
#   1. mmu.extruder_name and mmu_extruder_stepper references
#   2. The extruder stepper on gear_rail extra endstops (for rapid stop on homing)
#   3. HH's live sensor dict (all_sensors) to reflect the active toolhead's sensors
#
# Sensor lookup uses a numeric suffix derived from the extruder name:
#   extruder  -> suffix 0 -> mmu_extruder_0, mmu_toolhead_0, mmu_tension_0
#   extruder1 -> suffix 1 -> mmu_extruder_1, mmu_toolhead_1, mmu_tension_1
#
# These should match existing [filament_switch_sensor] sections in your config.
# No changes to mmu_hardware.cfg or Tool*.cfg needed.
#
# Config:
#   [mmu_toolchanger_bridge]
#   sensor_prefix: mmu   # prefix for sensor names (default: mmu)
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

        self.gcode.register_command(
            'SET_MMU_EXTRUDER', self.cmd_SET_MMU_EXTRUDER,
            desc="Dynamically switch Happy Hare's active extruder and sensors"
        )

    def _get_suffix(self, extruder_name):
        """Derive numeric suffix from extruder name.
        'extruder' -> '0',  'extruder1' -> '1',  'extruder2' -> '2', etc.
        """
        return extruder_name.replace('extruder', '') or '0'

    def _lookup_sensor(self, name):
        """Look up a filament_switch_sensor by name, returning None if absent."""
        return self.printer.lookup_object(
            "filament_switch_sensor %s" % name, None)

    def cmd_SET_MMU_EXTRUDER(self, gcmd):
        mmu = self.printer.lookup_object('mmu', None)
        if mmu is None:
            gcmd.respond_info("MMU object not found")
            return

        extruder_name = gcmd.get('EXTRUDER', 'extruder')

        try:
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
            # The toolhead, compression, and tension extra endstops need to stop
            # the active extruder stepper rapidly during homing moves.
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
                                # Keep gear steppers, replace extruder stepper
                                mcu_endstop.steppers = [
                                    s for s in mcu_endstop.steppers
                                    if id(s) in gear_stepper_ids
                                ] + [new_mcu_stepper]
                            elif hasattr(mcu_endstop, 'add_stepper'):
                                try:
                                    mcu_endstop.add_stepper(new_mcu_stepper)
                                except Exception:
                                    pass

            # --- 4. Swap sensors in HH's live all_sensors dict ---
            # Derive suffix: 'extruder' -> '0', 'extruder1' -> '1', etc.
            suffix = self._get_suffix(extruder_name)
            p = self.sensor_prefix  # e.g. 'mmu'

            # Map of HH sensor key -> filament_switch_sensor name to look up
            sensor_map = {
                mmu.SENSOR_EXTRUDER_ENTRY: "%s_extruder_%s" % (p, suffix),
                mmu.SENSOR_TOOLHEAD:       "%s_toolhead_%s" % (p, suffix),
                mmu.SENSOR_TENSION:        "%s_tension_%s"  % (p, suffix),
            }

            for hh_key, sensor_name in sensor_map.items():
                sensor_obj = self._lookup_sensor(sensor_name)
                if sensor_obj is not None:
                    mmu.sensor_manager.all_sensors[hh_key] = sensor_obj
                else:
                    # Sensor not defined for this toolhead — remove the slot so
                    # HH doesn't try to use it
                    mmu.sensor_manager.all_sensors.pop(hh_key, None)

            # Refresh HH's viewable_sensors for the current gate/UI
            mmu.sensor_manager.reset_active_gate(mmu.gate)

            mmu.log_info(
                "MMU: Active extruder switched to '%s' (sensor suffix: %s)"
                % (extruder_name, suffix))

        except Exception as e:
            import traceback
            error_msg = "Failed to switch extruder: %s\n%s" % (str(e), traceback.format_exc())
            logging.error("MMU Toolchanger Bridge: %s" % error_msg)
            raise self.printer.command_error(
                "Failed to switch extruder to '%s'. Check mmu.log/klippy.log for details."
                % extruder_name)

def load_config(config):
    return MmuToolchangerBridge(config)

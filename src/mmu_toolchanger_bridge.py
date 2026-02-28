# Happy Hare MMU Software - Toolchanger Bridge
#
# Goal: Provide dynamic extruder and sensor switching for multi-toolhead setups.
#
# Architecture:
#   T0  -> extruder  (single filament, gate-fed by HH, has its own sensors)
#   T1+ -> extruder1 (MMU multi-material toolhead, sensors configured in [mmu_sensors])
#
# On each toolhead switch, this bridge updates:
#   1. mmu.extruder_name and mmu_extruder_stepper references
#   2. The extruder stepper on gear_rail extra endstops (for rapid stop on homing)
#   3. HH's live sensor dict (all_sensors) to reflect the active toolhead's sensors
#
# Sensor naming convention for non-MMU toolheads (e.g. T0 using 'extruder'):
#   [filament_switch_sensor extruder_toolhead_sensor]  -> toolhead sensor for T0
#   [filament_switch_sensor extruder_extruder_sensor]  -> extruder entry sensor for T0
#
# Copyright (C) 2024  Antigravity / Google Deepmind
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

class MmuToolchangerBridge:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')

        # Which extruder HH is natively configured for (its sensors live in [mmu_sensors]).
        # All other extruders are "non-MMU" and need per-toolhead sensor lookup.
        self.mmu_extruder_name = config.get('mmu_extruder', 'extruder1')

        # Snapshot of HH's original sensors, captured at klippy:connect.
        # Used to restore all_sensors when switching back to the MMU toolhead.
        self._mmu_sensor_snapshot = {}

        self.printer.register_event_handler('klippy:connect', self._handle_connect)

        self.gcode.register_command(
            'SET_MMU_EXTRUDER', self.cmd_SET_MMU_EXTRUDER,
            desc="Dynamically switch Happy Hare's active extruder and sensors"
        )

    def _handle_connect(self):
        """Snapshot the HH sensor state after all objects are ready."""
        mmu = self.printer.lookup_object('mmu', None)
        if mmu is None:
            return
        snapshot_keys = [
            mmu.SENSOR_EXTRUDER_ENTRY,
            mmu.SENSOR_TOOLHEAD,
            mmu.SENSOR_TENSION,
            mmu.SENSOR_COMPRESSION,
        ]
        for key in snapshot_keys:
            sensor = mmu.sensor_manager.all_sensors.get(key)
            if sensor is not None:
                self._mmu_sensor_snapshot[key] = sensor
        logging.info("MMU Toolchanger Bridge: Snapshotted %d MMU sensors for '%s'"
                     % (len(self._mmu_sensor_snapshot), self.mmu_extruder_name))

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
            # The toolhead, compression, and tension extra endstops have an extruder
            # stepper added so that homing moves stop the extruder rapidly. We need
            # to keep this pointing at the correct (now-active) extruder.
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
                    gear_stepper_set = {
                        id(s) for s in mmu.gear_rail.steppers
                        if hasattr(s, 'get_name')
                    }
                    for (mcu_endstop, name) in mmu.gear_rail.extra_endstops:
                        if name in extruder_endstop_names:
                            if hasattr(mcu_endstop, 'steppers'):
                                # Keep gear steppers; replace extruder stepper
                                mcu_endstop.steppers = [
                                    s for s in mcu_endstop.steppers
                                    if id(s) in gear_stepper_set
                                ] + [new_mcu_stepper]
                            elif hasattr(mcu_endstop, 'add_stepper'):
                                try:
                                    mcu_endstop.add_stepper(new_mcu_stepper)
                                except Exception:
                                    pass

            # --- 4. Swap sensors in HH's live all_sensors dict ---
            if extruder_name == self.mmu_extruder_name:
                # Switching back to the MMU toolhead: restore original HH sensors
                for key, sensor in self._mmu_sensor_snapshot.items():
                    mmu.sensor_manager.all_sensors[key] = sensor
            else:
                # Switching to a non-MMU toolhead (e.g. T0 on 'extruder'):
                # Look up per-toolhead sensors by convention:
                #   filament_switch_sensor {extruder_name}_toolhead_sensor
                #   filament_switch_sensor {extruder_name}_extruder_sensor
                th_sensor = self.printer.lookup_object(
                    "filament_switch_sensor %s_toolhead_sensor" % extruder_name, None)
                ex_sensor = self.printer.lookup_object(
                    "filament_switch_sensor %s_extruder_sensor" % extruder_name, None)

                if th_sensor:
                    mmu.sensor_manager.all_sensors[mmu.SENSOR_TOOLHEAD] = th_sensor
                else:
                    mmu.sensor_manager.all_sensors.pop(mmu.SENSOR_TOOLHEAD, None)

                if ex_sensor:
                    mmu.sensor_manager.all_sensors[mmu.SENSOR_EXTRUDER_ENTRY] = ex_sensor
                else:
                    mmu.sensor_manager.all_sensors.pop(mmu.SENSOR_EXTRUDER_ENTRY, None)

                # Sync feedback sensors (tension/compression) are on the MMU side only;
                # remove them so HH doesn't expect them on this toolhead.
                mmu.sensor_manager.all_sensors.pop(mmu.SENSOR_TENSION, None)
                mmu.sensor_manager.all_sensors.pop(mmu.SENSOR_COMPRESSION, None)

            # Refresh HH's viewable_sensors for the current gate/UI
            mmu.sensor_manager.reset_active_gate(mmu.gate)

            mmu.log_info("MMU: Active extruder switched to '%s'" % extruder_name)

        except Exception as e:
            import traceback
            error_msg = "Failed to switch extruder: %s\n%s" % (str(e), traceback.format_exc())
            logging.error("MMU Toolchanger Bridge: %s" % error_msg)
            raise self.printer.command_error(
                "Failed to switch extruder to '%s'. Check mmu.log/klippy.log for details."
                % extruder_name)

def load_config(config):
    return MmuToolchangerBridge(config)

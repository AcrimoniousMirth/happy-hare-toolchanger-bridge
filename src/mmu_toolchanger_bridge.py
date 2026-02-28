# Happy Hare MMU Software - Toolchanger Bridge
#
# Goal: Provide dynamic extruder and sensor switching for multi-toolhead setups
#
# Copyright (C) 2024  Antigravity / Google Deepmind
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

class MmuToolchangerBridge:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        
        # Register GCODE commands
        self.gcode.register_command('SET_MMU_EXTRUDER', self.cmd_SET_MMU_EXTRUDER, 
                                   desc="Dynamically switch Happy Hare's active extruder")

    def cmd_SET_MMU_EXTRUDER(self, gcmd):
        mmu = self.printer.lookup_object('mmu', None)
        if mmu is None:
            gcmd.respond_info("MMU object not found")
            return

        extruder_name = gcmd.get('EXTRUDER', 'extruder')
        
        # 1. Update mmu.extruder_name
        mmu.extruder_name = extruder_name
        
        # 2. Update mmu_toolhead.mmu_extruder_stepper
        try:
            # Happy Hare uses MmuExtruderStepper to wrap the actual stepper
            try:
                from . import mmu_machine
            except ImportError:
                import mmu_machine
            MmuExtruderStepper = mmu_machine.MmuExtruderStepper
            
            # Get the exact config section for the target extruder
            extruder_config = self.printer.lookup_object('configfile').get_section(extruder_name)
            
            new_mmu_stepper = MmuExtruderStepper(extruder_config, mmu.gear_rail)
            mmu.mmu_toolhead.mmu_extruder_stepper = new_mmu_stepper
            mmu.mmu_extruder_stepper = new_mmu_stepper
            
            # 3. Synchronize gear rail endstops if necessary
            # The sensor manager holds references to steppers for endstop stopping
            for endstop in mmu.gear_rail.get_endstops():
                name = endstop.get_name()
                if name in [mmu.SENSOR_TOOLHEAD, mmu.SENSOR_EXTRUDER_ENTRY, mmu.SENSOR_COMPRESSION, mmu.SENSOR_TENSION]:
                    # Update the stepper associated with this endstop
                    # This ensures rapid stopping on synced homing for the new extruder
                    endstop.steppers = [new_mmu_stepper.stepper]
            
            # 4. Swap sensors in sensor_manager.all_sensors
            # We expect sensors named mmu_extruder_N, mmu_toolhead_N, and mmu_tension_N to exist
            # Suffix is 0 for 'extruder', 1 for 'extruder1', etc.
            suffix = extruder_name.replace('extruder', '') or '0'
            
            ext_sensor = self.printer.lookup_object('filament_switch_sensor mmu_extruder_' + suffix, None)
            th_sensor = self.printer.lookup_object('filament_switch_sensor mmu_toolhead_' + suffix, None)
            tension_sensor = self.printer.lookup_object('filament_switch_sensor mmu_tension_' + suffix, None)
            
            if ext_sensor:
                mmu.sensor_manager.all_sensors[mmu.SENSOR_EXTRUDER_ENTRY] = ext_sensor
            if th_sensor:
                mmu.sensor_manager.all_sensors[mmu.SENSOR_TOOLHEAD] = th_sensor
            if tension_sensor:
                mmu.sensor_manager.all_sensors[mmu.SENSOR_TENSION] = tension_sensor

            # Refresh UI/Active sensors
            mmu.sensor_manager.reset_active_gate(mmu.gate)
            
            mmu.log_info("MMU: Active extruder dynamically switched to '%s' (suffix %s)" % (extruder_name, suffix))
        except Exception as e:
            import traceback
            error_msg = "Failed to switch extruder: %s\n%s" % (str(e), traceback.format_exc())
            logging.error("MMU Toolchanger Bridge: %s" % error_msg)
            # Use self.printer.command_error for nice reporting in Mainsail/Fluidd
            raise self.printer.command_error("Failed to switch extruder to '%s'. Check mmu.log/klippy.log for details." % extruder_name)

def load_config(config):
    return MmuToolchangerBridge(config)

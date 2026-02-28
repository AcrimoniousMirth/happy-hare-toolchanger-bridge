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

class BridgeMockEndstop:
    def __init__(self, reactor):
        self.reactor = reactor
        self._pin = "mock"
    def add_stepper(self, stepper):
        pass
    def get_steppers(self):
        return []
    def query_endstop(self, print_time):
        return 0 # Not triggered
    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered):
        # We return an uncompleted completion object so it doesn't trigger immediately
        return self.reactor.completion()
    def home_wait(self, home_end_time):
        return home_end_time

class MmuToolchangerBridge:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
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

        # T0 Loading specific settings (Default to HH global parameters)
        mmu_config = config.getsection('mmu')
        self.t0_slow_zone_mm = config.getfloat('t0_slow_zone_mm', 
                                              mmu_config.getfloat('extruder_homing_buffer', 30.))
        self.t0_load_attempts = config.getint('t0_load_attempts', 1, minval=1)
        self.t0_fast_speed = config.getfloat('t0_fast_speed', 
                                             mmu_config.getfloat('gear_from_buffer_speed', 150.))
        self.t0_slow_speed = config.getfloat('t0_slow_speed', 
                                             mmu_config.getfloat('gear_homing_speed', 20.))
        self.t0_pre_gate_to_gate_distance = config.getfloat('t0_pre_gate_to_gate_distance', 0.)
        self.t0_gate_homing_max = config.getfloat('t0_gate_homing_max', 350.)
        self.t0_gate_preload_homing_max = config.getfloat('t0_gate_preload_homing_max', 350.)
        self.t0_gate_parking_distance = config.getfloat('t0_gate_parking_distance', 0.)
        # Saved HH defaults (captured at klippy:connect)
        self._orig_settings = {}
        
        # Per-extruder state tracking to allow multi-tool coexistence.
        # Maps extruder_name -> filament_pos
        self._extruder_pos_map = {}

        self.printer.register_event_handler('klippy:connect', self._handle_connect)

        self.gcode.register_command(
            'SET_MMU_EXTRUDER', self.cmd_SET_MMU_EXTRUDER,
            desc="Dynamically switch Happy Hare's active extruder and sensors"
        )
        self.gcode.register_command(
            'DUMP_MMU_BRIDGE', self.cmd_DUMP_MMU_BRIDGE,
            desc="Diagnostic command to dump bridge state"
        )

    # -------------------------------------------------------------------------

    def _handle_connect(self):
        """Save HH defaults and auto-apply T0 mode if T0 is already active at boot."""
        mmu = self.printer.lookup_object('mmu', None)
        if mmu is None:
            return

        # 1. Register all per-toolhead sensors as HH Gear Rail endstops.
        # HH only automatically registers standard sensor names (e.g. mmu_extruder).
        # We must manually register mmu_extruder_0, mmu_toolhead_0, etc. so that
        # the gear motor can stop on them during homing/calibration.
        p = self.sensor_prefix
        sensor_manager = mmu.sensor_manager
        
        # We'll look for sensors matching our toolhead pattern
        # (e.g. mmu_extruder_0, mmu_toolhead_0, mmu_extruder_1, etc.)
        for name, sensor in sensor_manager.all_sensors.items():
            if name.startswith("%s_" % p) and any(x in name for x in ["extruder_", "toolhead_", "pre_gate_", "gear_", "gate_"]):
                # Ensure it's in gear_rail extra_endstops so we can relay to it
                if name not in mmu.gear_rail.get_extra_endstop_names():
                    sensor_pin = None
                    if hasattr(sensor, 'runout_helper') and sensor.runout_helper.switch_pin:
                        sensor_pin = sensor.runout_helper.switch_pin
                    elif hasattr(sensor, '_pin'):
                        sensor_pin = sensor._pin
                    
                    if sensor_pin:
                        try:
                            mmu.gear_rail.add_extra_endstop(sensor_pin, name)
                            logging.info("MMU Toolchanger Bridge: Registered %s as gear rail endstop" % name)
                        except Exception as e:
                            logging.warning("MMU Toolchanger Bridge: Failed to register %s: %s" % (name, str(e)))

        # 2. Ensure generic HH endstop names exist in gear_rail so they can be swapped.
        # If the MMU doesn't have an extruder sensor, "extruder" won't be in the list,
        # which causes MMU_CALIBRATE_BOWDEN to fail. We add a 'mock' if missing.
        # We also need to add unit-prefixed versions (e.g. unit_1_extruder) because
        # HH looks those up in multi-unit configurations (T1-T5).
        # We use our MockEndstop to satisfy HH's logging/debug logic.
        mock_es = BridgeMockEndstop(self.reactor)
        generic_names = [mmu.SENSOR_GATE, mmu.SENSOR_EXTRUDER_ENTRY, mmu.SENSOR_TOOLHEAD]
        for name in generic_names:
            names_to_check = [name]
            if mmu.mmu_machine.num_units > 1:
                # Add unit prefixes for all possible units (0 to num_units-1)
                for i in range(mmu.mmu_machine.num_units):
                    names_to_check.append(sensor_manager.get_unit_sensor_name(name, i))

            for n in names_to_check:
                if n not in mmu.gear_rail.get_extra_endstop_names():
                    mmu.gear_rail.add_extra_endstop("mock", n, register=True, mcu_endstop=mock_es)

        # 3. Save originals so we can restore them when T1+ is active
        self._orig_settings = {
            'bowden_homing_max':      mmu.bowden_homing_max,
            'gate_homing_max':        getattr(mmu, 'gate_homing_max', None),
            'gear_from_buffer_speed': getattr(mmu, 'gear_from_buffer_speed', 100.),
            'gear_homing_speed':      getattr(mmu, 'gear_homing_speed', 20.),
            'extruder_homing_buffer': getattr(mmu, 'extruder_homing_buffer', 0.),
            'extruder_homing_endstop': getattr(mmu, 'extruder_homing_endstop', 'mmu_extruder'),
            'gate_homing_endstop':    getattr(mmu, 'gate_homing_endstop', 'mmu_gate'),
            'extruder_force_homing':  getattr(mmu, 'extruder_force_homing', False),
            'gate_load_retries':      getattr(mmu, 'gate_load_retries', 2),
            'preload_attempts':       getattr(mmu, 'preload_attempts', 2),
            'gate_preload_homing_max': getattr(mmu, 'gate_preload_homing_max', 350.),
            'gate_parking_distance':  getattr(mmu, 'gate_parking_distance', 0.),
            'gate_preload_parking_distance': getattr(mmu, 'gate_preload_parking_distance', -10.),
            'extra_endstops':         list(mmu.gear_rail.extra_endstops) # copy
        }

        logging.info("MMU Toolchanger Bridge: HH defaults captured")

        # 3. Auto-apply T0 settings if HH already has extruder (T0) as the configured
        # extruder at startup.
        if mmu.extruder_name == self.t0_extruder:
            self._apply_settings(mmu, self.t0_extruder)

    # -------------------------------------------------------------------------

    def _get_suffix(self, extruder_name):
        """Derive numeric suffix from extruder name.
        'extruder' -> '0',  'extruder1' -> '1',  'extruder2' -> '2', etc.
        """
        return extruder_name.replace('extruder', '') or '0'

    def _lookup_sensor_wrapper(self, mmu, name):
        """Look up a sensor object in HH manager, else look up in Klipper."""
        if name in mmu.sensor_manager.all_sensors:
            return mmu.sensor_manager.all_sensors[name]
        return self.printer.lookup_object("filament_switch_sensor %s" % name, None)

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
        # Before switching, save the filament position for the outgoing extruder.
        if mmu.extruder_name:
            self._extruder_pos_map[mmu.extruder_name] = mmu.filament_pos

        mmu.extruder_name = extruder_name
        mmu.mmu_toolhead.mmu_extruder_stepper = new_stepper
        mmu.mmu_extruder_stepper = new_stepper

        # Restore the filament position for the incoming extruder (default to UNLOADED)
        mmu.filament_pos = self._extruder_pos_map.get(extruder_name, mmu.FILAMENT_POS_UNLOADED)

        is_t0 = (extruder_name == self.t0_extruder)
        suffix = self._get_suffix(extruder_name)
        p = self.sensor_prefix # e.g. 'mmu'

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
                gear_stepper_ids = {id(s) for s in mmu.gear_rail.steppers if hasattr(s, 'get_name')}
                
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

        # --- 4. Apply T0 specific load/homing settings ---
        if is_t0:
            # Relay strategy:
            # 1. Map Gate sensor to Pre-Gate (instant pick-up detection)
            # 2. Map Extruder Entry to T0's Toolhead sensor
            # 3. Use high speed for bowden move
            # 4. Use slow zone for final homing to toolhead sensor
            
            mmu.bowden_homing_max = self.t0_bowden_max
            if hasattr(mmu, 'gate_homing_max'):
                mmu.gate_homing_max = self.t0_gate_homing_max
            
            mmu.gate_preload_homing_max = self.t0_gate_preload_homing_max
            mmu.gate_parking_distance = self.t0_gate_parking_distance
            
            mmu.gear_from_buffer_speed = self.t0_fast_speed
            mmu.gear_homing_speed = self.t0_slow_speed
            mmu.extruder_homing_buffer = self.t0_slow_zone_mm
            mmu.extruder_homing_endstop = mmu.SENSOR_EXTRUDER_ENTRY
            mmu.gate_homing_endstop = mmu.SENSOR_GATE
            mmu.extruder_force_homing = True
            mmu.gate_load_retries = self.t0_load_attempts
            mmu.preload_attempts = self.t0_load_attempts
            mmu.gate_preload_parking_distance = 0.0

            # Sensor relay swaps in gear_rail.extra_endstops
            # We map generic HH names (including unit-prefixed ones) to the
            # physical sensors active for this toolhead.
            unit_val = int(suffix)
            
            p_ext = "%s_extruder_%s" % (p, suffix)
            p_th  = "%s_toolhead_%s" % (p, suffix)
            p_gt  = "%s_pre_gate_%s" % (p, suffix) if is_t0 else None
            
            new_endstops = []
            registered_es = mmu.gear_rail.extra_endstops
            
            # Find physical endstops for target sensors
            ext_es = next((e[0] for e in registered_es if e[1] == p_ext), None)
            th_es  = next((e[0] for e in registered_es if e[1] == p_th), None)
            gt_es  = next((e[0] for e in registered_es if e[1] == p_gt), None) if p_gt else None
            
            for es, name in registered_es:
                # Expected HH lookup names for this unit
                hh_ext = mmu.sensor_manager.get_unit_sensor_name(mmu.SENSOR_EXTRUDER_ENTRY, unit_val)
                hh_th  = mmu.sensor_manager.get_unit_sensor_name(mmu.SENSOR_TOOLHEAD, unit_val)
                hh_gt  = mmu.sensor_manager.get_unit_sensor_name(mmu.SENSOR_GATE, unit_val)
                
                if name in [hh_ext, mmu.SENSOR_EXTRUDER_ENTRY] and ext_es:
                    new_endstops.append((ext_es, name))
                    logging.info("MMU Toolchanger Bridge: Relayed %s to %s" % (name, p_ext))
                elif name in [hh_th, mmu.SENSOR_TOOLHEAD] and th_es:
                    new_endstops.append((th_es, name))
                    logging.info("MMU Toolchanger Bridge: Relayed %s to %s" % (name, p_th))
                elif name in [hh_gt, mmu.SENSOR_GATE] and gt_es:
                    new_endstops.append((gt_es, name))
                    logging.info("MMU Toolchanger Bridge: Relayed %s to %s" % (name, p_gt))
                else:
                    new_endstops.append((es, name))
            mmu.gear_rail.extra_endstops = new_endstops
        else:
            # Restore defaults for T1-T5
            if self._orig_settings:
                mmu.bowden_homing_max = self._orig_settings['bowden_homing_max']
                if hasattr(mmu, 'gate_homing_max'):
                    mmu.gate_homing_max = self._orig_settings['gate_homing_max']
                mmu.gear_from_buffer_speed = self._orig_settings['gear_from_buffer_speed']
                mmu.gear_homing_speed = self._orig_settings['gear_homing_speed']
                mmu.extruder_homing_buffer = self._orig_settings['extruder_homing_buffer']
                mmu.extruder_homing_endstop = self._orig_settings['extruder_homing_endstop']
                mmu.gate_homing_endstop = self._orig_settings['gate_homing_endstop']
                mmu.extruder_force_homing = self._orig_settings['extruder_force_homing']
                mmu.gate_load_retries = self._orig_settings['gate_load_retries']
                mmu.preload_attempts = self._orig_settings['preload_attempts']
                if 'gate_preload_homing_max' in self._orig_settings:
                    mmu.gate_preload_homing_max = self._orig_settings['gate_preload_homing_max']
                if 'gate_parking_distance' in self._orig_settings:
                    mmu.gate_parking_distance = self._orig_settings['gate_parking_distance']
                mmu.gate_preload_parking_distance = self._orig_settings['gate_preload_parking_distance']
                mmu.gear_rail.extra_endstops = list(self._orig_settings['extra_endstops'])

        # --- 5. Swap sensors in HH's live all_sensors dict for UI/Logic ---
        sensor_map = {
            mmu.SENSOR_EXTRUDER_ENTRY: "%s_extruder_%s" % (p, suffix),
            mmu.SENSOR_TOOLHEAD:       "%s_toolhead_%s" % (p, suffix),
            mmu.SENSOR_TENSION:        "%s_tension_%s"  % (p, suffix),
            mmu.SENSOR_GATE:           "%s_gate_0"  % p if is_t0 else None,
        }

        # Sensor Deception for T0 Preload:
        # Happy Hare checks for gear and gate sensors using prefix + _gate. 
        # For Gate 0, it looks for 'mmu_gear_0'. We only map gear to pre_gate.
        if is_t0:
            sensor_map.update({
                "%s_0" % mmu.SENSOR_GEAR_PREFIX: "%s_pre_gate_0" % p,
            })

        for hh_key, sensor_name in sensor_map.items():
            if sensor_name:
                sensor_obj = self._lookup_sensor_wrapper(mmu, sensor_name)
                if sensor_obj is not None:
                    mmu.sensor_manager.all_sensors[hh_key] = sensor_obj
                else:
                    mmu.sensor_manager.all_sensors.pop(hh_key, None)
            elif not is_t0: # Restore T1+ gate sensor
                pass # Already handled by reset_active_gate below if we didn't touch it

        # Refresh HH's sensor manager unit cache to recognize updated mappings
        mmu.sensor_manager.reset_active_unit(mmu.unit_selected)
        mmu.sensor_manager.reset_active_gate(mmu.gate_selected)

        mmu.log_info(
            "MMU: Active extruder switched to '%s' (sensor suffix: %s)"
            % (extruder_name, suffix))
        
        # Diagnostic: Log sensor states
        for hh_name in [mmu.SENSOR_EXTRUDER_ENTRY, mmu.SENSOR_GATE, mmu.SENSOR_TOOLHEAD]:
            s = self._lookup_sensor_wrapper(mmu, hh_name)
            if s:
                helper = getattr(s, 'runout_helper', None)
                state = "unknown"
                if helper:
                    state = "DETECTED" if helper.filament_present else "EMPTY"
                actual_name = getattr(s, 'name', 'unknown')
                logging.info("MMU Toolchanger Bridge: Sensor status '%s' -> %s (%s)" % (hh_name, actual_name, state))

    # -------------------------------------------------------------------------

    def cmd_DUMP_MMU_BRIDGE(self, gcmd):
        mmu = self.printer.lookup_object('mmu', None)
        if mmu is None:
            gcmd.respond_info("MMU object not found")
            return

        gcmd.respond_info("--- MMU Bridge State ---")
        gcmd.respond_info("Extruder: %s, Pos: %s" % (mmu.extruder_name, mmu.filament_pos))
        
        gcmd.respond_info("Gear Rail Extra Endstops:")
        for es, name in mmu.gear_rail.extra_endstops:
            mcu_name = es.get_mcu().get_name() if hasattr(es, 'get_mcu') else es.__class__.__name__
            pin = getattr(es, '_pin', 'unknown')
            # Check triggered state if possible
            trig = "unknown"
            if hasattr(es, 'query_endstop'):
                trig = "TRIGGERED" if es.query_endstop(self.reactor.monotonic()) else "OPEN"
            gcmd.respond_info("  %s -> pin:%s, mcu:%s [%s]" % (name, pin, mcu_name, trig))

        gcmd.respond_info("All Sensors (Manager Keys):")
        for name in sorted(mmu.sensor_manager.all_sensors.keys()):
            s = mmu.sensor_manager.all_sensors[name]
            helper = getattr(s, 'runout_helper', None)
            state = "unknown"
            if helper:
                state = "DETECTED" if helper.filament_present else "EMPTY"
            gcmd.respond_info("  %s -> %s" % (name, state))

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

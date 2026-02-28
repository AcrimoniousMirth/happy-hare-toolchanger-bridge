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
import re

class BridgeProxyEndstop:
    def __init__(self, mmu, native_sensor, name):
        self.mmu = mmu
        self.printer = mmu.printer
        self.reactor = mmu.printer.get_reactor()
        self.sensor = native_sensor
        self.name = name
        self._pin = "proxy"
        self.completion = None

    def add_stepper(self, stepper):
        pass

    def get_steppers(self):
        # We must return a list with at least one stepper so HomingMove doesn't ignore us.
        # We return the Gear Rail's steppers.
        return self.mmu.gear_rail.steppers

    def get_mcu(self):
        # Return the stepper's MCU to satisfy some checks
        if self.mmu.gear_rail.steppers:
            return self.mmu.gear_rail.steppers[0].get_mcu()
        return None

    def query_endstop(self, print_time):
        # Return state from native sensor. Note: filament_switch_sensor.get_status() returns a dict.
        return 1 if self.sensor.get_status(0).get('filament_detected') else 0

    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered):
        self.completion = self.reactor.completion()
        self._check_sensor(triggered)
        return self.completion

    def _check_sensor(self, triggered):
        # If the sensor matches our target 'triggered' state, complete.
        status = self.sensor.get_status(0)
        current_state = 1 if status.get('filament_detected') else 0
        logging.info("MMU Bridge Proxy [%s]: State=%d, Target=%d, Raw=%s" % (self.name, current_state, triggered, str(status)))
        if bool(current_state) == bool(triggered):
            self.completion.complete(True)
        else:
            # Poll every 10ms. Preload/Homing is slow so this is fine.
            self.reactor.register_timer(lambda e: self._check_sensor(triggered), self.reactor.monotonic() + 0.01)
        return self.reactor.NEVER

    def home_wait(self, home_end_time):
        self.completion.wait()
        return home_end_time

class BridgeProxySensor:
    def __init__(self, name, native_sensor):
        self.name = name
        self.sensor = native_sensor
        # Happy Hare logic expects a runout_helper with filament_present and sensor_enabled
        # We wrap the native sensor's status
        class ProxyHelper:
            def __init__(self, sensor):
                self.sensor = sensor
                self.sensor_enabled = True
                self.runout_suspended = False
                self._pin = getattr(sensor, '_pin', 'proxy')
            
            @property
            def switch_pin(self):
                return self._pin

            @property
            def filament_present(self):
                return self.sensor.get_status(0).get('filament_detected', False)

            def enable_button_feedback(self, enable):
                pass
            
            def enable_runout(self, enable):
                self.sensor_enabled = enable
        self.runout_helper = ProxyHelper(native_sensor)

    def get_status(self, eventtime):
        return self.sensor.get_status(eventtime)

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
        self.t0_gate_homing_max = config.getfloat('t0_gate_homing_max',
                                 config.getfloat('t0_gate_preload_homing_max', 350.))
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
        """Register a slightly delayed callback to ensure HH has fully initialized."""
        self.reactor.register_timer(self._delayed_connect, self.reactor.monotonic() + 0.5)

    def _delayed_connect(self, eventtime):
        """Save HH defaults and auto-apply T0 mode if T0 is already active at boot."""
        mmu = self.printer.lookup_object('mmu', None)
        if mmu is None:
            logging.info("MMU Toolchanger Bridge: MMU object not found, skipping sensor registration")
            return self.reactor.NEVER

        sensor_manager = mmu.sensor_manager
        p = self.sensor_prefix

        # 1. Proactively discover all native Klipper sensors that should be bridged.
        # We look for [filament_switch_sensor mmu_X] and [mmu_sensor mmu_X].
        configfile = self.printer.lookup_object('configfile')
        config = configfile.get_status(None).get('config', configfile.get_status(None).get('settings', {}))
        
        found_sensors = []
        for section in config.keys():
            match = re.match(r'^(filament_switch_sensor|mmu_sensor) (%s_.*)$' % p, section)
            if match:
                name = match.group(2)
                found_sensors.append((section, name))

        for section, name in found_sensors:
            sensor_obj = self.printer.lookup_object(section, None)
            if not sensor_obj:
                continue

            # Determine sensor type and unit index if applicable
            # Patterns: mmu_gate_0, mmu_pre_gate_0, mmu_extruder_0, mmu_toolhead_0, etc.
            match = re.search(r'^(.*)_(\d+)$', name)
            sensor_type = match.group(1) if match else name
            i = int(match.group(2)) if match else None

            # 1. Register as a Proxy Endstop in HH's Gear Rail so hmove/calibration works.
            # We add it if HH hasn't already registered a real hardware endstop with this name.
            if name not in mmu.gear_rail.get_extra_endstop_names():
                proxy_es = BridgeProxyEndstop(mmu, sensor_obj, name)
                mmu.gear_rail.add_extra_endstop("mock", name, register=True, mcu_endstop=proxy_es)
                logging.info("MMU Toolchanger Bridge: Registered %s as gear rail ProxyEndstop" % name)

            # 2. Register as a Proxy Sensor in HH's Sensor Manager for UI/Logic.
            if name not in sensor_manager.all_sensors:
                proxy_s = BridgeProxySensor(name, sensor_obj)
                sensor_manager.all_sensors[name] = proxy_s
                logging.info("MMU Toolchanger Bridge: Registered %s as proxy Sensor object" % name)

            # 3. Special Case: Map unit-specific gate sensor to HH's 'gear' endstop name.
            # HH preloads by homing to 'mmu_gear_N'. In T0 mode, we want this to be the gate sensor.
            # We map 'mmu_gear_N' (Endstop) to the same proxy as 'mmu_gate_N'.
            if i is not None and "gate" in sensor_type and "pre_gate" not in sensor_type:
                gear_name = "%s_%d" % (mmu.SENSOR_GEAR_PREFIX, i)
                if gear_name not in mmu.gear_rail.get_extra_endstop_names():
                    proxy_es = BridgeProxyEndstop(mmu, sensor_obj, gear_name)
                    mmu.gear_rail.add_extra_endstop("mock", gear_name, register=True, mcu_endstop=proxy_es)
                if gear_name not in sensor_manager.all_sensors:
                    sensor_manager.all_sensors[gear_name] = BridgeProxySensor(gear_name, sensor_obj)

        # 2. Ensure generic HH endstop names (mmu_gate, etc.) have Mock fallbacks if missing.
        mock_es = BridgeMockEndstop(self.reactor)
        generic_names = [mmu.SENSOR_GATE, mmu.SENSOR_EXTRUDER_ENTRY, mmu.SENSOR_TOOLHEAD]
        for base_name in generic_names:
            names_to_check = [base_name]
            for i in range(mmu.mmu_machine.num_units):
                names_to_check.append(sensor_manager.get_unit_sensor_name(base_name, i))
                names_to_check.append("%s_%d" % (mmu.SENSOR_GEAR_PREFIX, i))

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

        return self.reactor.NEVER

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
            # 4. Use slow zone for final homing to toolhead sensor

            # Homing ceilings to match the long active toolhead path.
            mmu.bowden_homing_max = self.t0_bowden_max
            if hasattr(mmu, 'gate_homing_max'):
                # For T0, the "gate" sensor is at the toolhead, so "homing to gate" 
                # (especially on unload) must cover the full bowden length.
                mmu.gate_homing_max = self.t0_bowden_max
            
            # NOTE: We keep t0_gate_homing_max (from config) for specialized uses 
            # if we ever decide to split them, but for now we need full length here.
            
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
            p_gear = "%s_gate_%s" % (p, suffix) if is_t0 else None
            
            new_endstops = []
            registered_es = mmu.gear_rail.extra_endstops
            
            # Find physical endstops for target sensors
            ext_es = next((e[0] for e in registered_es if e[1] == p_ext), None)
            th_es  = next((e[0] for e in registered_es if e[1] == p_th), None)
            gt_es  = next((e[0] for e in registered_es if e[1] == p_gt), None) if p_gt else None
            gear_es = next((e[0] for e in registered_es if e[1] == p_gear), None) if p_gear else None
            
            # Map HH target names to the physical endstops we want to relay them to
            hh_ext = mmu.sensor_manager.get_unit_sensor_name(mmu.SENSOR_EXTRUDER_ENTRY, unit_val)
            hh_th  = mmu.sensor_manager.get_unit_sensor_name(mmu.SENSOR_TOOLHEAD, unit_val)
            hh_gt  = mmu.sensor_manager.get_unit_sensor_name(mmu.SENSOR_GATE, unit_val)
            hh_gear = "%s_%s" % (mmu.SENSOR_GEAR_PREFIX, suffix)
            
            relay_map = {}
            if ext_es:
                relay_map[hh_ext] = ext_es
                relay_map[mmu.SENSOR_EXTRUDER_ENTRY] = ext_es
            if th_es:
                relay_map[hh_th] = th_es
                relay_map[mmu.SENSOR_TOOLHEAD] = th_es
            if gt_es:
                relay_map[hh_gt] = gt_es
                relay_map[mmu.SENSOR_GATE] = gt_es
            if gear_es:
                relay_map[hh_gear] = gear_es
            
            for es, name in registered_es:
                if name in relay_map:
                    new_endstops.append((relay_map[name], name))
                    logging.info("MMU Toolchanger Bridge: Relayed %s" % name)
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
            mmu.SENSOR_GATE:           "%s_gate_%s"     % (p, suffix),
        }

        # T0 Preload Sensor Logic:
        # Happy Hare native behavior: user inserts to 'gate', HH preloads by homing to 'gear'.
        # For T0, user inserts filament at 'mmu_pre_gate_0' (so it acts as HH's 'gate').
        # Preload moves the filament until it reaches 'mmu_gate_0'.
        # We map HH's 'mmu_gate' and 'mmu_gear_0' names to 'mmu_gate_0' so it stops.
        if is_t0:
            gear_key = "%s_0" % mmu.SENSOR_GEAR_PREFIX
            gate_key = mmu.sensor_manager.get_unit_sensor_name(mmu.SENSOR_GATE, 0)
            sensor_map.update({
                gear_key: "%s_gate_0" % p,
                gate_key: "%s_gate_0" % p,
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
            state = "unknown"
            if hasattr(es, 'query_endstop'):
                state = "TRIGGERED" if es.query_endstop(0) else "OPEN"
            gcmd.respond_info("  %s -> pin:%s, mcu:%s [%s]" % (name, pin, mcu_name, state))

        gcmd.respond_info("All Sensors (Manager View):")
        for name in sorted(mmu.sensor_manager.all_sensors.keys()):
            s = mmu.sensor_manager.all_sensors[name]
            helper = getattr(s, 'runout_helper', None)
            detected = False
            if helper:
                detected = helper.filament_present
            
            # Find associated endstop to show pin
            es_info = "no_endstop"
            for es, es_name in mmu.gear_rail.extra_endstops:
                if es_name == name:
                    mcu_name = es.get_mcu().get_name() if hasattr(es, 'get_mcu') else es.__class__.__name__
                    pin = getattr(es, '_pin', 'unknown')
                    state = "unknown"
                    if hasattr(es, 'query_endstop'):
                        state = "TRIGGERED" if es.query_endstop(0) else "OPEN"
                    es_info = "pin:%s, mcu:%s [%s]" % (pin, mcu_name, state)
                    break
            
            gcmd.respond_info("  %s -> %s (%s)" % (name, "DETECTED" if detected else "EMPTY", es_info))

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

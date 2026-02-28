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
#   4. mmu.bowden_homing_max and mmu.gate_homing_max to match the toolhead sensors.
#   5. mmu.gate_homing_endstop/extruder_homing_endstop set to SENSOR_GATE/SENSOR_EXTRUDER.
#
#   Physical Sensor Chain (T0 Mode):
#   [Pre-Gate] --- (t0_gate_preload_max) ---> [Gate_0] --- (t0_bowden_max) ---> [Extruder_0]
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
    def __init__(self, mmu, proxy_sensor, name):
        self.mmu = mmu
        self.proxy_sensor = proxy_sensor # This is a BridgeProxySensor instance
        self.name = name
        self.sensor = proxy_sensor.sensor # Real Klipper sensor object
        # Reference the BridgeProxySensor's ProxyHelper
        self.runout_helper = getattr(proxy_sensor, 'runout_helper', None)
        self.reactor = mmu.printer.get_reactor()
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
        # Use our helper's logical state (handles inversion)
        try:
            return 1 if self.proxy_sensor.runout_helper.filament_present else 0
        except Exception:
            return 0

    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered):
        self.completion = self.reactor.completion()
        logging.info("MMU Bridge Proxy [%s]: Homing started (Target Triggered=%d)" % (self.name, triggered))
        self._check_sensor(triggered)
        return self.completion

    def _check_sensor(self, triggered):
        # If the sensor matches our target 'triggered' state, complete.
        try:
            current_state = bool(self.proxy_sensor.runout_helper.filament_present)
            if current_state == bool(triggered):
                if not self.completion.completed():
                    self.completion.complete(0)
                return self.reactor.NEVER
        except Exception:
            pass

        # Check again in 10ms
        self.reactor.register_timer(self.reactor.monotonic() + 0.01,
                                    lambda et: self._check_sensor(triggered))
        return self.reactor.NEVER

    def home_wait(self, home_end_time):
        self.completion.wait()
        return home_end_time

class ProxyHelper:
    def __init__(self, sensor):
        self.sensor = sensor
        self.sensor_enabled = True
        self.runout_suspended = False
        self._pin = getattr(sensor, '_pin', 'proxy')
        # Try to find the real Klipper runout helper
        self.native_helper = getattr(sensor, 'runout_helper', None)
    
    @property
    def switch_pin(self):
        return self._pin

    @property
    def filament_present(self):
        # Prefer the logical status from Klipper's runout_helper (handles '!')
        if self.native_helper and hasattr(self.native_helper, 'filament_present'):
            return self.native_helper.filament_present
        # Fallback to the raw status if no helper
        try:
            if hasattr(self.sensor, 'get_status'):
                return self.sensor.get_status(0).get('filament_detected', False)
        except Exception:
            pass
        return False

    def enable_button_feedback(self, enable):
        if self.native_helper:
            if hasattr(self.native_helper, 'enable_button_feedback'):
                self.native_helper.enable_button_feedback(enable)
            elif hasattr(self.native_helper, 'button_handler_suspended'):
                self.native_helper.button_handler_suspended = not enable
    
    def enable_runout(self, enable):
        self.sensor_enabled = enable
        if self.native_helper:
            if hasattr(self.native_helper, 'set_enabled'):
                self.native_helper.set_enabled(enable)
            if hasattr(self.native_helper, 'enable_runout'):
                self.native_helper.enable_runout(enable)
            if hasattr(self.native_helper, 'runout_suspended'):
                self.native_helper.runout_suspended = not enable

    # Fallbacks for HH MmuRunoutHelper specific methods
    def set_enabled(self, enable):
        self.enable_runout(enable)
    def enable_clog_detection(self, enable):
        pass
    def enable_tangle_detection(self, enable):
        pass
    def reset_counts(self):
        pass

class BridgeProxySensor:
    def __init__(self, name, native_sensor):
        self.name = name
        self.sensor = native_sensor
        # Happy Hare logic expects a runout_helper with filament_present and sensor_enabled
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
        self.gcode.register_command(
            'SCAN_MMU_PINS', self.cmd_SCAN_MMU_PINS,
            desc="Scan all MMB physical STP pins"
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
        
        # 1. Discover sensors already known to Happy Hare.
        # This is the most stable way to find native pre-gate and gate sensors.
        for name in list(sensor_manager.all_sensors.keys()):
            sensor_obj = sensor_manager.all_sensors[name]
            if isinstance(sensor_obj, BridgeProxySensor):
                continue

            # Wrap it
            proxy_s = BridgeProxySensor(name, sensor_obj)
            sensor_manager.all_sensors[name] = proxy_s
            logging.info("MMU Toolchanger Bridge: Wrapped existing sensor '%s' in proxy" % name)

        # 2. Proactively discover any toolhead/extruder sensors that might be missing
        # but are defined in the Klipper configuration.
        p = self.sensor_prefix
        search_names = [
            'toolhead', 'extruder', 'filament_tension', 'mmu_gate', 'mmu_extruder_entry'
        ]
        # Extend with unit-specific ones and prefixed ones
        candidate_bases = []
        for sn in search_names:
            candidate_bases.append(sn)
            candidate_bases.append("%s_%s" % (p, sn))
        
        found_sections = []
        all_objects = self.printer.lookup_objects()
        for obj_name in all_objects:
            if not isinstance(obj_name, str):
                continue
            if obj_name.startswith('filament_switch_sensor ') or obj_name.startswith('mmu_sensor '):
                section, name = obj_name.split(' ', 1)
                # Check if it matches one of our expected sensor names
                # e.g. mmu_toolhead_0, mmu_extruder_1, etc.
                if any(name.startswith(base) for base in candidate_bases):
                    found_sections.append((obj_name, name))

        for section, name in found_sections:
            if name in sensor_manager.all_sensors and isinstance(sensor_manager.all_sensors[name], BridgeProxySensor):
                continue
            
            sensor_obj = self.printer.lookup_object(section, None)
            if not sensor_obj:
                continue

            proxy_s = BridgeProxySensor(name, sensor_obj)
            sensor_manager.all_sensors[name] = proxy_s
            logging.info("MMU Toolchanger Bridge: Found and wrapped external sensor '%s'" % name)

            # Determine sensor type and unit index if applicable
            # Patterns: mmu_gate_0, mmu_pre_gate_0, mmu_extruder_0, mmu_toolhead_0, etc.
            match = re.search(r'^(.*)_(\d+)$', name)
            sensor_type = match.group(1) if match else name
            i = int(match.group(2)) if match else None

            # 1. Register as a Proxy Endstop in HH's Gear Rail so hmove/calibration works.
            # We add it if HH hasn't already registered a real hardware endstop with this name.
            if name not in mmu.gear_rail.get_extra_endstop_names():
                proxy_es = BridgeProxyEndstop(mmu, proxy_s, name) # Pass the BridgeProxySensor instance
                mmu.gear_rail.add_extra_endstop("mock", name, register=True, mcu_endstop=proxy_es)
                logging.info("MMU Toolchanger Bridge: Registered %s as gear rail ProxyEndstop" % name)

            # 3. Special Case: Map unit-specific gate sensor to HH's 'gear' endstop name.
            # HH preloads by homing to 'mmu_gear_N'. In T0 mode, we want this to be the gate sensor.
            # We map 'mmu_gear_N' (Endstop) to the same proxy as 'mmu_gate_N'.
            if i is not None and "gate" in sensor_type and "pre_gate" not in sensor_type:
                gear_name = "%s_%d" % (mmu.SENSOR_GEAR_PREFIX, i)
                if gear_name not in mmu.gear_rail.get_extra_endstop_names():
                    proxy_es = BridgeProxyEndstop(mmu, proxy_s, gear_name) # Pass the BridgeProxySensor instance
                    mmu.gear_rail.add_extra_endstop("mock", gear_name, register=True, mcu_endstop=proxy_es)
                if gear_name not in sensor_manager.all_sensors:
                    sensor_manager.all_sensors[gear_name] = proxy_s # Use the same proxy_s for the gear sensor

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
            
            p_th   = "%s_toolhead_%s" % (p, suffix)
            p_ext  = "%s_extruder_%s" % (p, suffix)
            p_gate = "%s_gate_%s"     % (p, suffix) # e.g. mmu_gate_0
            p_pre  = "%s_pre_gate_%s" % (p, suffix) # e.g. mmu_pre_gate_0
            
            new_endstops = []
            registered_es = mmu.gear_rail.extra_endstops
            
            # Find physical endstops for target sensors
            ext_es  = next((e[0] for e in registered_es if e[1] == p_ext), None)
            th_es   = next((e[0] for e in registered_es if e[1] == p_th), None)
            gate_es = next((e[0] for e in registered_es if e[1] == p_gate), None)
            pre_es  = next((e[0] for e in registered_es if e[1] == p_pre), None)
            
            # Map HH target names to the physical endstops we want to relay them to
            hh_ext  = mmu.sensor_manager.get_unit_sensor_name(mmu.SENSOR_EXTRUDER_ENTRY, unit_val)
            hh_th   = mmu.sensor_manager.get_unit_sensor_name(mmu.SENSOR_TOOLHEAD, unit_val)
            hh_gt   = mmu.sensor_manager.get_unit_sensor_name(mmu.SENSOR_GATE, unit_val)
            hh_gear = "%s_%s" % (mmu.SENSOR_GEAR_PREFIX, suffix)
            
            relay_map = {}
            if ext_es:
                relay_map[hh_ext] = ext_es
                relay_map[mmu.SENSOR_EXTRUDER_ENTRY] = ext_es
            if th_es:
                relay_map[hh_th] = th_es
                relay_map[mmu.SENSOR_TOOLHEAD] = th_es
            if gate_es:
                relay_map[hh_gt] = gate_es
                relay_map[mmu.SENSOR_GATE] = gate_es
                relay_map[hh_gear] = gate_es # HH homes to 'gear' for preload
            
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
            elif not is_t0: # Restore T1+ gate sensor
                pass # Already handled by reset_active_gate below if we didn't touch it

        # 2. Update Happy Hare's active unit sensors
        try:
            can_reset = True
            for name, s in mmu.sensor_manager.all_sensors.items():
                if name.startswith("unit_"):
                    if not hasattr(s, 'runout_helper') or not hasattr(s.runout_helper, 'enable_button_feedback'):
                        logging.info("MMU Toolchanger Bridge: Delaying HH sensor reset (Unit sensor '%s' not ready)" % name)
                        can_reset = False
                        break
            
            if can_reset:
                mmu.sensor_manager.reset_active_unit(mmu.unit_selected)
            else:
                logging.warning("MMU Toolchanger Bridge: Skipping mmu.sensor_manager.reset_active_unit (uninitialized sensors)")
        except Exception as e:
            logging.error("MMU Toolchanger Bridge: Error during sensor reset: %s" % str(e))
        mmu.sensor_manager.reset_active_gate(mmu.gate_selected)

        # Global Pre-gate Monitoring:
        # Ensure that ALL pre-gate sensors are enabled regardless of the active unit.
        # Happy Hare 3.0+ handles the insertion logic, but we must make sure Klipper
        # is actually listening to the pins.
        for name, sensor in mmu.sensor_manager.all_sensors.items():
            if mmu.SENSOR_PRE_GATE_PREFIX in name:
                helper = getattr(sensor, 'runout_helper', None)
                if helper:
                    helper.enable_runout(True)
                    logging.info("MMU Toolchanger Bridge: Global monitor ENABLING %s" % name)

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
            try:
                mcu_name = es.get_mcu().get_name() if hasattr(es, 'get_mcu') else es.__class__.__name__
                pin = getattr(es, '_pin', 'unknown')
                state = "unknown"
                if hasattr(es, 'query_endstop'):
                    state = "TRIGGERED" if es.query_endstop(0) else "OPEN"
                gcmd.respond_info("  %s -> pin:%s, mcu:%s [%s]" % (name, pin, mcu_name, state))
            except Exception as e:
                gcmd.respond_info("  %s -> Error: %s" % (name, str(e)))

        gcmd.respond_info("All Sensors (Manager View):")
        for name in sorted(mmu.sensor_manager.all_sensors.keys()):
            s = mmu.sensor_manager.all_sensors[name]
            logical_state = "unknown"
            raw_state = "unknown"
            pin_info = "proxy"
            
            try:
                # 1. Get Logical State
                target_info = ""
                if hasattr(s, 'filament_present'):
                    logical_state = "DETECTED" if s.filament_present else "EMPTY"
                    # If it's a proxy, show the target
                    inner = getattr(s, 'sensor', None)
                    if inner:
                         target_info = " [proxies: %s]" % getattr(inner, 'name', 'unknown')
                else:
                    helper = getattr(s, 'runout_helper', None)
                    if helper and hasattr(helper, 'filament_present'):
                        logical_state = "DETECTED" if helper.filament_present else "EMPTY"
                
                # 2. Get Raw State
                sensor_obj = getattr(s, 'sensor', None)
                if sensor_obj is None:
                    # Try to find the Klipper object directly (native HH sensors)
                    possible_names = []
                    # Try with prefix
                    klipper_name = getattr(s, 'name', name)
                    possible_names.append(klipper_name)
                    if not klipper_name.startswith('filament_switch_sensor'):
                        possible_names.append('filament_switch_sensor ' + klipper_name)
                    if not klipper_name.startswith('mmu_sensor'):
                        possible_names.append('mmu_sensor ' + klipper_name)
                    
                    for kn in possible_names:
                        sensor_obj = self.printer.lookup_object(kn, None)
                        if sensor_obj:
                            break
                
                if sensor_obj and hasattr(sensor_obj, 'get_status'):
                    try:
                        status = sensor_obj.get_status(0)
                        raw_state = "DETECTED" if status.get('filament_detected') else "EMPTY"
                        raw_class = sensor_obj.__class__.__name__
                        raw_info = "%s [%s]" % (raw_state, raw_class)
                    except:
                        raw_info = "Error getting status"
                elif sensor_obj:
                    raw_info = "No get_status [%s]" % sensor_obj.__class__.__name__
                else:
                    raw_info = "Object Not Found"
                
                # 3. Get Pin Info from extra endstops
                for es, es_name in mmu.gear_rail.extra_endstops:
                    if es_name == name:
                        pin_info = getattr(es, '_pin', 'unknown')
                        break
                
                gcmd.respond_info("  %s%s -> Log:%s, Raw:%s (pin:%s)" % (name, target_info, logical_state, raw_info, pin_info))
            except Exception as e:
                gcmd.respond_info("  %s -> Error: %s" % (name, str(e)))

    # -------------------------------------------------------------------------

    def cmd_SCAN_MMU_PINS(self, gcmd):
        mmu = self.printer.lookup_object('mmu', None)
        if not mmu:
            gcmd.respond_info("MMU object not found")
            return

        gcmd.respond_info("--- MMU Hardware Pin Scan ---")
        gcmd.respond_info("Legend: Log=Logical State, Raw=Native Klipper State")
        
        # STP Pin Mapping for BTT MMB v1.1
        stp_pins = {
            "STP1": "PA3", "STP2": "PA4", "STP3": "PB9", "STP4": "PB8",
            "STP5": "PC15", "STP6": "PC13", "STP7": "PC14", "STP8": "PB12",
            "STP9": "PB11", "STP10": "PB10", "STP11": "PB2"
        }
        
        mcus = ["MMB_0-2", "MMB_3-5"]
        all_sensors = mmu.sensor_manager.all_sensors
        
        # Find which sensors are on which pins
        pin_to_sensor = {}
        for name, s in all_sensors.items():
            # Look for pin in extra endstops
            for es, es_name in mmu.gear_rail.extra_endstops:
                if es_name == name:
                    full_pin = getattr(es, '_pin', 'unknown')
                    pin_to_sensor[full_pin] = name

        for mcu_name in mcus:
            gcmd.respond_info("MCU: %s" % mcu_name)
            for stp_id in range(1, 12):
                stp = "STP%d" % stp_id
                pin = stp_pins[stp]
                full_pin = "%s:%s" % (mcu_name, pin)
                sensor_name = pin_to_sensor.get(full_pin, "No Config")
                
                status_str = "HIDDEN"
                log_str = "?"
                
                # Try to find the Klipper object for this specific pin
                found_obj = None
                for obj_name in self.printer.lookup_objects():
                    if not isinstance(obj_name, str): continue
                    if 'filament_switch_sensor' in obj_name or 'mmu_sensor' in obj_name:
                        obj = self.printer.lookup_object(obj_name)
                        obj_pin = getattr(obj, '_pin', '')
                        # Handle ^ ! mappings
                        if pin in obj_pin and mcu_name in obj_pin:
                            found_obj = obj
                            if sensor_name == "No Config":
                                sensor_name = obj_name.split(' ')[-1]
                            break
                
                if found_obj and hasattr(found_obj, 'get_status'):
                    try:
                        raw_state = found_obj.get_status(0).get('filament_detected', False)
                        status_str = "DETECTED" if raw_state else "EMPTY"
                        
                        # Check logical state if we have a mapping
                        if sensor_name in all_sensors:
                            s = all_sensors[sensor_name]
                            if hasattr(s, 'filament_present'):
                                log_str = "DET" if s.filament_present else "MT"
                            else:
                                h = getattr(s, 'runout_helper', None)
                                if h: log_str = "DET" if h.filament_present else "MT"
                    except:
                        status_str = "ERR"
                
                gcmd.respond_info("  %s (%s) -> Log:%s, Raw:%s [%s]" % 
                                 (stp, pin, log_str, status_str, sensor_name))
                                 
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

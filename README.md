# Happy Hare Toolchanger Bridge

This is an add-on for [Happy Hare](https://github.com/moggieuk/Happy-Hare) that enables dynamic extruder and sensor switching for multi-toolhead setups (e.g., Klipper-Toolchanger).

## Description

Happy Hare natively assumes a single physical extruder. This bridge allows you to dynamically switch which extruder and toolhead sensors Happy Hare uses at runtime. This is essential for setups where different tools reside on different physical toolheads with their own extruders and sensors.

## Installation

### Automatic installation

The module can be installed into an existing Klipper installation with the included install script:

```bash
cd ~
git clone https://github.com/AcrimoniousMirth/happy-hare-toolchanger-bridge.git # <--- Update to your actual repo URL
cd happy-hare-toolchanger-bridge
./install.sh
```

### Manual installation

1.  Clone the repository to your home directory.
2.  Link the extension:
    ```bash
    ln -sf ~/happy-hare-toolchanger-bridge/src/mmu_toolchanger_bridge.py ~/klipper/klippy/extras/mmu_toolchanger_bridge.py
    ```
3.  Restart Klipper.

---

## Configuration

### 1. Enable the Bridge
Add the following to your `printer.cfg` (or an included file like `mmu.cfg`):

```ini
[mmu_toolchanger_bridge]
```

### 2. Include Coordination Macros
Include the provided macros to handle the switching logic during toolchanges:

```ini
[include ~/happy-hare-toolchanger-bridge/config/mmu_toolchanger_logic.cfg]
```

### 3. Usage
The bridge provides the `SET_MMU_EXTRUDER` command:

```gcode
SET_MMU_EXTRUDER EXTRUDER=extruder1
```

This is automatically handled by the overridden `SELECT_TOOL` macro included in the logic config.

---

## How it Works

The bridge intercepts toolchange requests and:
1.  Updates Happy Hare's internal `extruder_name`.
2.  Re-initializes the `MmuExtruderStepper` wrapper to point to the active physical extruder.
3.  Dynamically re-routes `gear_rail` endstops to the active extruder stepper for synchronized homing/loading.
4.  Swaps sensor objects in Happy Hare's `sensor_manager` to match the active toolhead.

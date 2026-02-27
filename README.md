# Happy Hare Toolchanger Bridge

This is an add-on for [Happy Hare](https://github.com/moggieuk/Happy-Hare) that enables dynamic extruder and sensor switching for multi-toolhead setups (e.g., Klipper-Toolchanger).

## Description

Happy Hare natively assumes a single physical extruder. This bridge allows you to dynamically switch which extruder and toolhead sensors Happy Hare uses at runtime. This is essential for setups where different tools reside on different physical toolheads with their own extruders and sensors.

## Installation

### Automatic installation

The module can be installed into an existing Klipper installation with the included install script:

```bash
cd ~
git clone https://github.com/AcrimoniousMirth/happy-hare-toolchanger-bridge.git
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

### 2. Coordination Macros
The `install.sh` script automatically links the coordination macros into your printer's configuration directory. Include them in your `printer.cfg` (or `mmu.cfg`):

```ini
[include mmu_toolchanger_logic.cfg]
```

### 3. Example Configurations
Full examples for hardware mapping and main MMU integration are provided in the `examples/` directory of this repository for reference.

---

## How it Works

The bridge intercepts toolchange requests and:
1.  Updates Happy Hare's internal `extruder_name`.
2.  Re-initializes the `MmuExtruderStepper` wrapper to point to the active physical extruder.
3.  Dynamically re-routes `gear_rail` endstops to the active extruder stepper for synchronized homing/loading.
4.  Swaps sensor objects in Happy Hare's `sensor_manager` to match the active toolhead.

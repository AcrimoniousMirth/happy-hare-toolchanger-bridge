#!/bin/bash
# Happy Hare Toolchanger Bridge - Install Script
set -e

KLIPPER_PATH="${HOME}/klipper"
KLIPPER_SERVICE_NAME=klipper
MOONRAKER_CONFIG_DIR="${HOME}/printer_data/config"

# Fall back to old directory for configuration as default
if [ ! -d "${MOONRAKER_CONFIG_DIR}" ]; then
    echo "\"$MOONRAKER_CONFIG_DIR\" does not exist. Falling back to "${HOME}/klipper_config" as default."
    MOONRAKER_CONFIG_DIR="${HOME}/klipper_config"
fi

usage(){ echo "Usage: $0 [-k <klipper path>] [-s <klipper service name>] [-c <configuration path>] [-u]" 1>&2; exit 1; }
# Parse command line arguments
while getopts "k:s:c:uh" arg; do
    case $arg in
        k) KLIPPER_PATH=$OPTARG;;
        s) KLIPPER_SERVICE_NAME=$OPTARG;;
        c) MOONRAKER_CONFIG_DIR=$OPTARG;;
        u) UNINSTALL=1;;
        h) usage;;
    esac
done

SRCDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

check_klipper()
{
    if [ "$(sudo systemctl list-units --full -all -t service --no-legend | grep -F "$KLIPPER_SERVICE_NAME.service")" ]; then
        echo "Klipper service found with name "$KLIPPER_SERVICE_NAME"."
    else
        echo "[ERROR] Klipper service with name "$KLIPPER_SERVICE_NAME" not found."
        exit -1
    fi
}

check_folders()
{
    if [ ! -d "$KLIPPER_PATH/klippy/extras/" ]; then
        echo "[ERROR] Klipper installation not found in directory \"$KLIPPER_PATH\"."
        exit -1
    fi
    echo "Klipper installation found at $KLIPPER_PATH"

    if [ ! -f "${MOONRAKER_CONFIG_DIR}/moonraker.conf" ]; then
        echo "[ERROR] Moonraker configuration not found in directory \"$MOONRAKER_CONFIG_DIR\"."
        exit -1
    fi
    echo "Moonraker configuration found at $MOONRAKER_CONFIG_DIR"
}

link_extension()
{
    echo -n "Linking extension to Klipper... "
    ln -sf "${SRCDIR}/src/mmu_toolchanger_bridge.py" "${KLIPPER_PATH}/klippy/extras/mmu_toolchanger_bridge.py"
    echo "[OK]"
}

restart_moonraker()
{
    echo -n "Restarting Moonraker... "
    sudo systemctl restart moonraker
    echo "[OK]"
}

add_updater()
{
    echo -e -n "Adding/Updating manager in moonraker.conf... "

    update_section=$(grep -c '\[update_manager happy-hare-toolchanger-bridge\]' ${MOONRAKER_CONFIG_DIR}/moonraker.conf || true)
    if [ "${update_section}" -eq 0 ]; then
        echo -e "\n" >> ${MOONRAKER_CONFIG_DIR}/moonraker.conf
        echo "[update_manager happy-hare-toolchanger-bridge]" >> ${MOONRAKER_CONFIG_DIR}/moonraker.conf
        echo "type: git_repo" >> ${MOONRAKER_CONFIG_DIR}/moonraker.conf
        echo "path: ${SRCDIR}" >> ${MOONRAKER_CONFIG_DIR}/moonraker.conf
        echo "origin: https://github.com/AcrimoniousMirth/happy-hare-toolchanger-bridge.git" >> ${MOONRAKER_CONFIG_DIR}/moonraker.conf
        echo "primary_branch: main" >> ${MOONRAKER_CONFIG_DIR}/moonraker.conf
        echo "managed_services: klipper" >> ${MOONRAKER_CONFIG_DIR}/moonraker.conf
        echo -e "\n" >> ${MOONRAKER_CONFIG_DIR}/moonraker.conf
        echo "[OK]"
        restart_moonraker
    else
        # Update path in case it changed
        sed -i "s|path:.*happy-hare-toolchanger-bridge.*|path: ${SRCDIR}|" ${MOONRAKER_CONFIG_DIR}/moonraker.conf
        echo "[UPDATED PATH]"
        restart_moonraker
    fi
}

start_klipper()
{
    echo -n "Starting Klipper... "
    sudo systemctl start $KLIPPER_SERVICE_NAME
    echo "[OK]"
}

stop_klipper()
{
    echo -n "Stopping Klipper... "
    sudo systemctl stop $KLIPPER_SERVICE_NAME
    echo "[OK]"
}

uninstall()
{
    if [ -f "${KLIPPER_PATH}/klippy/extras/mmu_toolchanger_bridge.py" ]; then
        echo -n "Uninstalling... "
        rm -f "${KLIPPER_PATH}/klippy/extras/mmu_toolchanger_bridge.py"
        echo "[OK]"
        echo "You can now remove the [update_manager happy-hare-toolchanger-bridge] section in moonraker.conf."
    else
        echo "mmu_toolchanger_bridge.py not found in \"${KLIPPER_PATH}/klippy/extras/\". Is it installed?"
        echo "[FAILED]"
    fi
}

verify_ready()
{
    if [ "$EUID" -eq 0 ]; then
        echo "[ERROR] This script must not run as root. Exiting."
        exit -1
    fi
}

# Run steps
verify_ready
check_klipper
check_folders
stop_klipper
if [ ! $UNINSTALL ]; then
    link_extension
    add_updater
else
    uninstall
fi
start_klipper

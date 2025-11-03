if [ -n "$SUDO_USER" ] || [ -n "$SUDO_UID" ]; then
    echo "This script was executed with sudo."
    echo "Use './autorun.sh' instead of 'sudo ./autorun.sh'"
    echo "Exiting..."
    exit 1
fi

whoami && pulseaudio --start && sleep 1 && XDG_RUNTIME_DIR=/run/user/$(id -u) ~/robot-code/ugv_server/ugv-env/bin/python ~/robot-code/ugv_server/WebSocketClient.py
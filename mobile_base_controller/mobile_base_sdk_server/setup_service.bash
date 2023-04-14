# Generate reachy_mobile_base.service
# Put this service file in ~/.config/systemd/user

mkdir -p $HOME/.config/systemd/user
SCRIPT=$(realpath "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

tee $HOME/.config/systemd/user/reachy_mobile_base.service <<EOF
[Unit]
Description=Mobile base SDK server service

[Service]
SyslogIdentifier=reachy_mobile_base
ExecStartPre=/bin/sleep 10
ExecStart=/usr/bin/bash $SCRIPTPATH/launch_mobile_base.bash
KillSignal=SIGKILL

[Install]
WantedBy=default.target
EOF

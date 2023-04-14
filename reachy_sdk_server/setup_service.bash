SCRIPT=$(realpath "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

tee reachy_sdk_server.service <<EOF
[Unit]
Description=Reachy SDK server service
[Service]
SyslogIdentifier=reachy_sdk_server
ExecStartPre=/bin/sleep 2
ExecStart=/usr/bin/bash $SCRIPTPATH/launch.bash
KillSignal=SIGKILL

[Install]
WantedBy=default.target
EOF

mkdir -p $HOME/.config/systemd/user

mv reachy_sdk_server.service $HOME/.config/systemd/user

echo ""
echo "reachy_sdk_server.service is now setup."

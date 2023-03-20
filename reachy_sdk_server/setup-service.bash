tee reachy_sdk_server.service <<EOF
[Unit]
Description=Reachy SDK server service
[Service]
ExecStart=/usr/bin/bash $PWD/launch.bash
[Install]
WantedBy=default.target
EOF

mkdir -p $HOME/.config/systemd/user

mv reachy_sdk_server.service $HOME/.config/systemd/user

echo ""
echo "reachy_sdk_server.service is now setup."
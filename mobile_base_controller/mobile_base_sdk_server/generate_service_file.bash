# Generate reachy_mobile_base.service
# Put this service file in ~/.config/systemd/user

tee reachy_mobile_base.service <<EOF
[Unit]
Description=Mobile base SDK server service

[Service]
ExecStartPre=/bin/sleep 10
ExecStart=/usr/bin/bash $PWD/launch_mobile_base.bash
Environment="PATH=$PATH:$(dirname $(which reachy-identify-model))"

[Install]
WantedBy=default.target
EOF

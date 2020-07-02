#!/bin/bash

sudo -S chmod 777 /dev/ttyS1 << EOF
1
EOF

sudo -S chmod 777 /dev/ttyS2 << EOF
1
EOF

sudo -S chmod 777 /dev/ttyS3 << EOF
1
EOF

sudo -S chmod 777 /dev/ttyS4 << EOF
1
EOF

sudo -S chmod 777 /dev/ttyS5 << EOF
1
EOF


sudo -S chmod 777 /dev/ttyS6 << EOF
1
EOF

cd /home/sweet/Desktop/robot/robot

sudo -S chmod +x robot << EOF
1
EOF
sudo -S chmod +x ../tracking/tracking << EOF
1
EOF
sudo -S chmod +x ../track_cmd/track_cmd << EOF
1
EOF

gnome-terminal --window --title='robot' -- ./robot
cd ../tracking
gnome-terminal --window --title='tracking' -- ./tracking
# gnome-terminal --window --title='tracking_single' -- ../tracking_single/tracking_single
cd ../track_cmd
gnome-terminal --window --title='track_cmd' -- ./track_cmd


exit 0

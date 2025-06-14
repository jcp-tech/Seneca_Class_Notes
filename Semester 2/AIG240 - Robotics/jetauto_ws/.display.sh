#!/bin/bash
monitors=$(xrandr --listactivemonitors |grep 'Monitors' |grep -o [0-9].*)
sleep 2
if [ $monitors -eq 0 ]; then
    xrandr --fb 1920x1080
fi

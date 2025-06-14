#!/bin/bash
sudo cp *.service /etc/systemd/system/
cd /etc/systemd/system/
sudo systemctl enable clear_log.service jupyter.service start_app_node.service voltage_detect.service expand_rootfs.service 

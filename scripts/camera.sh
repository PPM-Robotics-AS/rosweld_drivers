#!/bin/bash
cd $(rospack find rosweld_drivers)
python -m src.drivers.ip_camera --url $1 --topic $2 --user $3 --password $4 --max_fps $5
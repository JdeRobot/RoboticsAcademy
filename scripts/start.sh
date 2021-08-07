#!/bin/bash
rm -rf instructions.json manager.py
cp /RoboticsAcademy/scripts/instructions.json /instructions.json
cp /RoboticsAcademy/scripts/manager.py /manager.py
cp /RoboticsAcademy/scripts/radi-entrypoint.sh /radi-entrypoint.sh
source radi-entrypoint.sh
python3 RoboticsAcademy/manage.py runserver 0.0.0.0:8000 &
python3.8 manager.py

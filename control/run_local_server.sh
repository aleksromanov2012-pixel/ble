#!/bin/bash
cd /Users/VladislavOzerin/metalinspector
export INSPECTOR_AUTH=0
export INSPECTOR_ROBOT_TOKEN=a896062ed82a5b33a9d8ff5c163056f9
exec /Library/Frameworks/Python.framework/Versions/3.11/bin/python3 -m uvicorn inspector.server.app:app --host 0.0.0.0 --port 8712

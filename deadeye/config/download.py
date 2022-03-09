#!/usr/bin/env python

import datetime
import json
import threading

from networktables import NetworkTables

NT_SERVER = "10.27.67.2"
DEADEYE_CAMERA = "A0"

cond = threading.Condition()
notified = [False]


def connectionListener(connected, info):
    # print(info, "; Connected=%s" % connected)
    with cond:
        notified[0] = True
        cond.notify()


NetworkTables.initialize(server=NT_SERVER)
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    if not notified[0]:
        cond.wait()

nti = NetworkTables.getDefault()
table = nti.getTable(f"/Deadeye/{DEADEYE_CAMERA[0]}/{DEADEYE_CAMERA[1]}")
pipeline = json.loads(table.getString("Pipeline", "{}"))
capture = json.loads(table.getString("Capture", "{}"))
config = {
    "camera": DEADEYE_CAMERA,
    "server": NT_SERVER,
    "date": datetime.datetime.now().isoformat(),
    "capture": capture,
    "pipeline": pipeline,
}
print(json.dumps(config, indent=2))

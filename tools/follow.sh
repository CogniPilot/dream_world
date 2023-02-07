#!/bin/bash
gz service -s /gui/follow/pgain --reqtype gz.msgs.Double --reptype gz.msgs.Boolean --timeout 2000 --req 'data: 1'
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 2000 --req 'x: -1, y: 0, z: 0.5'

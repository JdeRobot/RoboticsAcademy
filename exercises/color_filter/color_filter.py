#!/usr/bin/python3.5
#
#  Copyright (C) 1997-2016 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Alberto Martin Florido <almartinflorido@gmail.com>
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#       Carlos Awadallah Estevez <carlosawadallah@gmail.com>
#

import sys
import yaml

from Camera.threadcamera import ThreadCamera
from MyAlgorithm import MyAlgorithm
from gui.threadGUI import ThreadGUI
from gui.GUI import MainWindow
from PyQt5.QtWidgets import QApplication

import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)

def selectVideoSource(cfg):
    """
    @param cfg: configuration
    @return cam: selected camera
    @raise SystemExit in case of unsupported video source
    """
    source = cfg['Introrob']['Source']
    if source.lower() == 'local':
        from Camera.local_camera import Camera
        cam_idx = cfg['Introrob']['Local']['DeviceNo']
        print('  Chosen source: local camera (index %d)' % (cam_idx))
        cam = Camera(cam_idx)
    elif source.lower() == 'video':
        from Camera.local_video import Camera
        video_path = cfg['Introrob']['Video']['Path']
        print('  Chosen source: local video (%s)' % (video_path))
        cam = Camera(video_path)
    elif source.lower() == 'stream':
        # comm already prints the source technology (ICE/ROS)
        import comm
        import config
        cfg = config.load(sys.argv[1])
        jdrc = comm.init(cfg, 'Introrob')
        proxy = jdrc.getCameraClient('Introrob.Stream')
        from Camera.stream_camera import Camera
        cam = Camera(proxy)
    else:
        raise SystemExit(('%s not supported! Supported source: Local, Video, Stream') % (source))

    return cam


def readConfig():
    try:
        with open(sys.argv[1], 'r') as stream:
            return yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)
        raise SystemExit('Error: Cannot read/parse YML file. Check YAML syntax.')
    except:
        raise SystemExit('\n\tUsage: python2 color_filter.py color_filter_conf.yml\n')


if __name__ == '__main__':

    cfg = readConfig()
    camera = selectVideoSource(cfg)

    # Threading the camera...
    t_cam = ThreadCamera(camera)
    t_cam.start()

    algorithm=MyAlgorithm(camera)


    app = QApplication(sys.argv)
    frame = MainWindow()
    frame.setCamera(camera)
    frame.setAlgorithm(algorithm)
    frame.show()

    t2 = ThreadGUI(frame)  
    t2.daemon=True
    t2.start()
    
    sys.exit(app.exec_()) 

#! /usr/bin/env python
# -*- coding: utf-8 -*-
""" A script for reading rosbag data. """

import rosbag
import time

'''
for (topic, msg, t) in bag.read_messages():
    print("topic::" + str(topic) + "msg::" + str(msg) + "t::" + str(t))

for (topic, msg, t) in bag.read_messages():
    print (topic, msg, t)
'''


if __name__ == '__main__':
    bag = rosbag.Bag('2018-08-24-12-48-46.bag')
    for (topic,msg,t) in bag.read_messages():
        #print(msg)
        try:
            x = str(msg).split('x: ')[1].split()[0]
            y = str(msg).split('y: ')[1].split()[0]
            print(x, y)
        except IndexError:
            pass
        '''
        secs = int(str(msg).split('pose:', 2)[0].split('nsecs: ')[1].split()[0])*0.00000001
        print(secs)

        pos_x = str(msg).split('position')
        print(pos_x)
        if len(pos_x) > 1:
            pos_x = pos_x[1].split('x: ')[1].split()[0]
            #print(pos_x)

        pos_y = str(msg).split('position')
        if len(pos_x) > 1:
            pos_y = pos_y[1].split('y: ')[1].split()[0]
            #print(pos_y)

        '''
        time.sleep(1.0)

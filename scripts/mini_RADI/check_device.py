import stat
import os
import sys

def check_device(device_path):
    try:
        return stat.S_ISCHR(os.lstat(device_path)[stat.ST_MODE])
    except:
        return False

def usage():
    print("usage: python3 check_device.py < device path >\n\t use this program to check if a given device exists in a system")
    sys.exit(1)

if (len(sys.argv) != 2):
    usage()

device_path = sys.argv[1]
print(check_device(device_path))
if (check_device(device_path)):
    sys.exit(0)
else:
    sys.exit(1)

import sys
import argparse

sys.path.insert(0, "/RoboticsApplicationManager")

from robotics_application_manager.manager.manager import Manager

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "host", type=str, help="Host to listen to  (0.0.0.0 or all hosts)"
    )
    parser.add_argument("port", type=int, help="Port to listen to")
    args = parser.parse_args()

    RAM = Manager(args.host, args.port)
    RAM.start()

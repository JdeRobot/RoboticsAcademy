import jderobot
import json
from Target import Target

class Parser:
    def __init__(self,filename):
        with open(filename) as data_file:
            self.data = json.load(data_file)

    def getTargets(self):
        targets = []
        for t in self.data["targets"]:
            targets.append(Target(t["name"],jderobot.Pose3DData(t["x"],t["y"],0,0,0,0,0,0),False,False))
        return targets





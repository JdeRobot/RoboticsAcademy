import subprocess

class Light:
    def __init__(self):
        pass
    
    def set_cmd_on(self):
        
        try:
            subprocess.run('''gz topic -p /gazebo/default/light/modify -m  "name:'iris::flashlight::light_source::lamp' range:30.0"''', shell = True,check= True)
        except:
            pass
    
    def set_cmd_off(self):
        
        try:
            subprocess.run('''gz topic -p /gazebo/default/light/modify -m  "name:'iris::flashlight::light_source::lamp' range:0.0"''', shell = True,check= True)
        except:
            pass

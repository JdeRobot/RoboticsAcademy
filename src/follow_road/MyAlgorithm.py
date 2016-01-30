from sensors import sensor

class MyAlgorithm():
    def __init__(self, sensor):
        self.sensor = sensor

    def execute(self):
        input_image = self.sensor.getImage()
        if input_image != None:
            self.sensor.setColorImage(input_image)
            '''
            If you want show a thresold image (black and white image)
            self.sensor.setThresoldImage(bk_image)
            '''

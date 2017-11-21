# Práctica follow_line

El objetivo de esta práctica es realizar un control reactivo PID capaz de seguir la línea pintada en el circuito de carreras.

## ¿Cómo ejecutar?
Para lanzar la infraestructura de esta práctica hay que lanzar primero el simulador con el escenario oportuno:

gazebo simpleCircuit.world

Despues hay que ejecutar la aplicación académica, que ya incorporará tu código

python2 ./follow_line.py followLineF1.cfg 

## ¿Cómo realizar la práctica?

Para realizar la práctica hay que editar el fichero MyAlgorithms.py e insertar en él tu código, que dota de inteligencia al coche autónomo.

### ¿Dónde insertar el código?
[MyAlgorithm.py](MyAlgorithm.py#L74)
```
    def execute(self):
        #GETTING THE IMAGES
        imageLeft = self.sensor.getImageLeft()
        imageRight = self.sensor.getImageRight()

        # Add your code here
        print "Runing"

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        #self.sensor.setV(10)
        #self.sensor.setW(5)

        #SHOW THE FILTERED IMAGE ON THE GUI
        self.setRightImageFiltered(imageRight)
        self.setLeftImageFiltered(imageLeft)
```

### API
* cameraL.getImage() - para obtener la imagen izquierda del par estéreo
* motors.setV() - para establecer la velocidad lineal
* motors.setW() - para establecer la velocidad angular
* self.setRightImageFiltered() - permite visualizar una imagen de depuración o con información relevante. Debe ser una imagen en formato RGB (Tip: np.dstack())


## Video demostrativo
https://www.youtube.com/watch?v=eNuSQN9egpA

* *Codigo base realizado por Alberto Martín (@almartinflorido)*
* *Código de la práctica realizado por Francisco Rivas (@chanfr)*
* *Modelos y mundos de Gazebo realizados por Francisco Pérez (@fqez)*

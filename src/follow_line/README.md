# Follow Line
* [Go to English instructions (soon?)](#english)
* [Ir a las instrucciones en Español](#spanish)

<a name="spanish"/>
# Práctica follow_line

El objetivo de esta práctica es realizar un control reactivo PID
capaz de seguir la línea pintada en el circuito.

La solución puede integrar uno o varios de los siguientes niveles
de dificultad, así como cualquier otro que a uno se le ocurra:
* Inicialización sobre la línea y nunca salirse
* Implementar búsqueda de la línea
  * Inicializalización sin ver la línea
* Robustez ante discontinuidades en la línea
* Ir lo más rápido posible


## Cómo ejecutar
Se ha preparado un script para lanzar la práctica. Para abordar
máquinas poco potentes, no lanza la interfaz de gazebo por defecto.
* Ejecución sin ver el mundo: `./run_it.sh`
* Ejecución viendo el mundo: `./run_it.sh GUI`
Para simplificar el cierre del entorno, basta con cerrar la(s)
ventana(s) de follow_line. *Ctrl+C dará problemas*.

Exiten dos mundos a elegir, uno simple (simpleCircuit), y otro 
completo más pesado (circuit).
Se puede elegir cual de ellos usar en [run_it.sh](run_it.sh#L7-L8)

Si tenéis una máquina muy lenta, o usáis el circuito completo, deberéis
aumentar el tiempo de espera de gazebo en la [línea 11](run_it.sh#L11)


## Cómo realizar la práctica
Para realizar la práctica se debe editar el fichero MyAlgorithms.py e
insertar la lógica de control.

### Dónde insertar el código
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
* self.setRightImageFiltered() - permite visualizar una imagen de 
  depuración o con información relevante.
  Debe ser una imagen en formato RGB (Tip: np.dstack())


## Video demostrativo
http://jderobot.org/store/varribas/uploads/curso/videos/follow_line_1.mp4


## Atribuciones
* *Copyright (C) 2016 [CC-BY-4.0](https://creativecommons.org/licenses/by/4.0/) Victor Arribas (@varhub)*

* *Codigo base realizado por Alberto Martín (@almartinflorido)*
* *Código de la práctica realizado por Francisco Rivas (@chanfr)*
* *Modelos y mundos de Gazebo realizados por Francisco Pérez (@fqez)*

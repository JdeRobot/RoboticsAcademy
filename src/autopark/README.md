# Autopark

# Práctica Autopark

El objetivo de esta práctica consiste en implementar la lógica de un algoritmo de navegación para una vehículo automático. El vehículo deberá encontrar una plaza de aparcamiento y estacionar de forma adecuada.


## Cómo ejecutar
Para lanzar el ejemplo, sigue los siguientes pasos:
* Ejecución sin ver el mundo: `gzserver autopark.world`
* Ejecución viendo el mundo: `gazebo autopark.world`
* Ejecución de la práctica y de la interfaz de usuario: `python2 autopark.py --Ice.Config=autopark.cfg`
* Ejecución del evaluador automático: `python2 referee.py --Ice.Config=autopark.cfg`

Para simplificar el cierre del entorno, basta con cerrar la(s) ventana(s) de Autopark. Ctrl+C dará problemas.

## Cómo realizar la práctica
Para realizar la práctica se debe editar el fichero MyAlgorithm.py e insertar en él la lógica de control.

### Dónde insertar el código
[MyAlgorithm.py](MyAlgorithm.py#L74)
```
    def execute(self):

        # Add your code here
        print "Runing"

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        #self.motors.sendV(10)
        #self.motors.sendW(5)
        # TODO
        
```


### API
* pose3d.getX() - para obtener la posición del robot
* pose3d.getY() - para obtener la posición del
* pose3d.getYaw() - para otener la orientación del robot
* laser.getLaserData() - Permite obtener los datos del sensor láser, que se compone de 180 pares de valores (0-180º, distancia en milímetros).
* motors.sendW() - para establecer la velocidad angular
* motors.sendV() - para establecer la velocidad lineal

Para este ejemplo, se ha de conseguir que taxi aparque correctamente en la plaza de aparcamiento libre. La aplicación del evaluador automático (referee) medirá diferentes parámetros (tiempo que tarda el taxi en aparcar, número de choques con otros coches, distancia a los vehículos), y en función de estos, realizará la calificación del algoritmo de solución.

## Conversion de tipos
### Laser
```
    laser_data = self.laser.getLaserData()

    def parse_laser_data(laser_data):
        laser = []
        for i in range(laser_data.numLaser):
            dist = laser_data.distanceData[i]/1000.0
            angle = math.radians(i)
            laser += [(dist, angle)]
         return laser
```

```
        laser_vectorized = []
        for d,a in laser:
            # (4.2.1) laser into GUI reference system
            x = d * math.cos(a) * -1
            y = d * math.sin(a) * -1
            v = (x,y)
            laser_vectorized += [v]

        laser_mean = np.mean(laser_vectorized, axis=0)
```

## Video demostrativo
https://www.youtube.com/watch?v=2SYEb3DyWEE


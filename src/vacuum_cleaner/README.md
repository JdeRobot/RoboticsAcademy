# Vacuum Cleaner

# Práctica Vacuum Cleaner

El objetivo de esta práctica consiste en implementar la lógica de un algoritmo de navegación para una aspiradora autónoma. El principal objetivo será recorrer la mayor superficie de una casa mediante el algoritmo programado.


## Cómo ejecutar
Para lanzar el ejemplo, sigue los siguientes pasos:
* Ejecución sin ver el mundo: `gzserver Vacuum.world`
* Ejecución viendo el mundo: `gazebo Vacuum.world`
* Ejecución de la práctica y de la interfaz de usuario: `python2 vacuumCleaner.py --Ice.Config=vacuumCleaner.cfg`
* Ejecución del evaluador automático: `python2 referee.py --Ice.Config=vacuumCleaner.cfg`
* Ejecución de la gráfica de la derivada del porcentaje: `python2 graphicPercentaje.py --Ice.Config=vacuumCleaner.cfg`

Para simplificar el cierre del entorno, basta con cerrar la(s) ventana(s) de VacuumCleaner. Ctrl+C dará problemas.

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
* pose3d.getYaw() - para otener la orientación del robot
* bumper.getBumperData().state - para establecer si el robot ha chocado o no. Devuelve un 1 si el robot colisiona y un 0 si no ha chocado.
* laser.getLaserData() - Permite obtener los datos del sensor láser, que se compone de 180 pares de valores (0-180º, distancia en milímetros). 
* motors.sendV() - para establecer la velocidad lineal
* motors.sendW() - para establecer la velocidad angular

Para este ejemplo, se ha de conseguir que la aspiradora recorra el mayor porcentaje posible de la casa. La aplicación del evaluador automático (referee) medirá el porcentaje recorrido, y en función de este porcentaje, realizará la calificación del algoritmo de solución.

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
https://www.youtube.com/watch?v=ThTXrqTDJ_A


# Obstacle Avoidance
* [Go to English instructions (not planned)](#english)
* [Ir a las instrucciones en Español](#spanish)

<a name="spanish"/>
# Práctica obstacle_avoidance

El objetivo de esta práctica es implementar la lógica del algoritmo
de navegación VFF.

La navegación mediante VFF (Virtual Force Field), consiste en que:
- Cada objeto en el entorno genera una fuerza repulsora hacia el robot
- El destino genera una fuerza atractora en el robot.

Con ello se consigue que el robot vaya hacia el objetivo distanciándose
de los obstáculos, de modo que su dirección sea la suma vectorial de
todas las fuerzas.

La solución puede integrar uno o varios de los siguientes niveles
de dificultad, así como cualquier otro que a uno se le ocurra:
* Ir lo más rápido posible
* Escoger el camino más seguro
* Obstáculos en movimiento
* Robustez ante situaciones de indecisión (suma vectorial nula)


## Cómo ejecutar
Se ha preparado un script para lanzar la práctica. Para abordar
máquinas poco potentes, no lanza la interfaz de gazebo por defecto.
* Ejecución sin ver el mundo: `./run_it.sh`
* Ejecución viendo el mundo: `./run_it.sh GUI`
Para simplificar el cierre del entorno, basta con cerrar la(s)
ventana(s) de obstacle_avoidance. *Ctrl+C dará problemas*.


Si tenéis una máquina muy lenta, o usáis el circuito completo, deberéis
aumentar el tiempo de espera de gazebo en la [línea 10](run_it.sh#L10)


## Cómo realizar la práctica
Para realizar la práctica se debe editar el fichero MyAlgorithms.py e
insertar la lógica de control.

### Dónde insertar el código
[MyAlgorithm.py](MyAlgorithm.py#L49)
```
    def execute(self):
        self.currentTarget=self.getNextTarget()
        self.targetx = self.currentTarget.getPose().x
        self.targety = self.currentTarget.getPose().y

        # TODO
```

### API
* sensor.getRobotX() - para lbtener la posición del robot
* sensor.getRobotY() - para lbtener la posición del robot
* sensor.getRobotTheta() - para obtener la orientacion del robot con 
  respecto al mapa
* sensor.getLaserData() - para obtener los datos del sensor laser
  se compone de 180 pares de valores: (0-180º distancia en milimetos)
* sensor.setV() - para establecer la velocidad lineal
* sensor.setW() - para establecer la velocidad angular


### API propia
Para simplificar se ofrece la implementación de los puntos de control.
Para emplearlo solo se deben realizar dos acciones:
1. Obtener el siguiente punto:
   `self.currentTarget=self.getNextTarget()`
2. Marcarlo como visitado cuando sea preciso:
   `self.currentTarget.setReached(True)`


## Conversion de tipos
### Laser
```
    laser_data = self.sensor.getLaserData()

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

### Sistema de coordenadas
```
    def absolutas2relativas(x, y, rx, ry, rt):
        # Convert to relatives
        dx = x - rx
        dy = y - ry

        # Rotate with current angle
        x = dx*math.cos(-rt) - dy*math.sin(-rt)
        y = dx*math.sin(-rt) + dy*math.cos(-rt)

	return x,y
```


## Depuración
La interfaz gráfica permite visualziar cada uno de los vectores de
fuerzas calculados. Para ello se deberá dar valor a las siguientes
variables:
```
        # Car direction
        self.carx = 0.0
        self.cary = 0.0

        # Obstacles direction
        self.obsx = 0.0
        self.obsy = 0.0

        # Average direction
        self.avgx = 0.0
        self.avgy = 0.0
```

Así como al destino que tenemos asignado:
```
        # Current target
        self.targetx = 0.0
        self.targety = 0.0
```

## Video demostrativo


## Atribuciones
* *Copyright (C) 2016 [CC-BY-4.0](https://creativecommons.org/licenses/by/4.0/) Victor Arribas (@varhub)*

* *Codigo base realizado por Alberto Martín (@almartinflorido)*
* *Código de la práctica realizado por Eduardo Perdices*
* *Modelos y mundos de Gazebo realizados por Francisco Pérez (@fqez)*


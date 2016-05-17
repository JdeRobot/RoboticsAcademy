# Global Navigation
* [Go to English instructions (not planned)](#english)
* [Ir a las instrucciones en Español](#spanish)

<a name="spanish"/>
# Práctica Global Navigation

El objetivo de esta práctica consiste en implementar la lógica de un algoritmo de Gradient Path Planning (GPP).

La navegación global mediante GPP, consiste en:

- Seleccionado un destino, el algoritmo de GPP se encarga de encontrar el camino más corto hacia éste, evitando, en el caso de esta práctica, todo lo que no sea carretera.
- Una vez se ha seleccionado el camino, hay que implementar en el robot la lógica necesaria para que siga este camino y llegue al objetivo.

Con esto, se consigue que el robot vaya al destino marcado de forma autónoma y por el camino más corto.

La solución puede integrar uno o varios de los siguientes niveles
de dificultad, así como cualquier otro que a uno se le ocurra:
* Llegar al objetivo.
* Optimizar la forma de buscar el camino más corto.
* Llegar lo más rápido posible al destino.


## Cómo ejecutar
Se ha preparado un script para lanzar la práctica. Para abordar
máquinas poco potentes, no lanza la interfaz de gazebo por defecto.
* Ejecución sin ver el mundo: `./run_it.sh`
* Ejecución viendo el mundo: `./run_it.sh GUI`

Si tenéis una máquina muy lenta, o usáis el circuito completo, deberéis
aumentar el tiempo de espera de gazebo en la [línea 11](run_it.sh#L11)

En caso de querer ejecutarla sin script se debe:
- Lanzar Gazebo con o sin GUI: 
    gazebo cityLarge.world     #Con GUI
    gzserver cityLarge.world   #Sin GUI

- Ejecutar la práctica indicándole cual es el archivo de configuración para el mapa:
    python main.py --mapConfig=taxiMap.conf --Ice.Config=teleTaxi.cfg

Además, la práctica puede lanzarse sin lanzar Gazebo. Para esto, simpemente hay que ejecutarla sin el argumento --Ice.Config=teleTaxi.cfg, aunque entonces no se podrá obtener información de los sensores del vehículo, pero se podrá trabajar con la imagen para calcular el campo sin necesidad de tener Gazebo abierto.

## Cómo realizar la práctica
Para realizar la práctica se debe editar el fichero MyAlgorithms.py e insertar en él toda la funcionalidad.

### Dónde insertar el código
Hay dos partes en las que se debe insertar el código:

- La parte relativa a encontrar el camino más corto debe situarse dentro de la función generatePath, que se ejecuta sólo cuando se pulsa el botón en la GUI[MyAlgorithm.py](MyAlgorithm.py#L17)

```
    def generatePath(self):
        print "LOOKING FOR SHORTER PATH"
        mapIm = self.grid.getMap()      
        dest = self.grid.getDestiny()   
        gridPos = self.grid.getPose()

        # TODO
```

- El código relaticvo a llegar al destino, va en la función execute, que se ejecuta periódicamente.[MyAlgorithm.py](MyAlgorithm.py#L29)

```
    def execute(self):
        # Add your code here
        print "GOING TO DESTINATION"
```

### API
* sensor.getRobotX() - para otener la posición del robot
* sensor.getRobotY() - para otener la posición del robot
* sensor.getRobotTheta() - para obtener la orientacion del robot con respecto al mapa
* sensor.setV() - para establecer la velocidad lineal
* sensor.setW() - para establecer la velocidad angular


### API propia
Este componente, además de relacionarse con el mundo, tiene que relacionarse con el mapa. Para simplificar esto, se tiene un objeto grid con las siguientes funciones:
* grid.getMap() - devuelve la imagen del mapa que se está mostrando. La imagen devuelta será una imagen de 3 canales con valores 0 y 255, donde 0 representa los obstaculos y 255 la carretera. Aunque la imagen tenga 3 canales, para la práctica servirá con usar uno sólo.
* grid.getDestiny() - devuelve el destino seleccionado mediante la GUI como una tupla (x,y).
* grid.getPose() - devuelve la posición respecto al mapa, no respecto al mundo, también como una tupla (x,y).
* gird.showGrid() - crea una ventana en la que representa los valores del campo que se le han asignado a la rejilla. Los valores más pequeños tendrán un color más cercano a negro, y se irán haciendo más claros a medida que se trate de valores más grandes. Para la representación se hace una copia del grid y se normalizan sus valores para que queden entre 0 y 1, y se representa después con cv2.imshow().

La clase Grid proporciona además una rejilla, sobre la que ir apuntanto la distancia al destino en ella. Los valores de esta rejilla son del tipo float. El tamaño y las posiciones de la rejilla coinciden con el de la imagen. Para interactuar con ella:
* grid.getVal(x, y) - devuelve el valor en esa posición del grid. 
* grid.setVal(x, y, val) - establece el valor val en la posición indicada.

Esta clase ofrece también otra rejilla sobre la que deberá apuntarse el camino una vez encontrado. Los puntos con valor 0 serán ignorados, los valores superiores serán considerados camino. Las funciones para interactuar son:
* grid.setPathVal(x,y, val) - establece el valor val en la posición indicada.
* grid.getPathVal(x,y) - devuelve el valor de la posición indicada.
* grid.setPathFinded() - establece que se ha encontrado el camino para que empiece a pintarse. 

Por último, el objeto grid también ofrece funciones para pasar de coordenadasdel mundo a coordenadas del mapa y viceversa:
* gridToWorld(gridX, gridY) - recibe las componentes x e y de las coordenadas del mapa y devuelve una tupla con las coordenadas equivalentes en el mundo: (worldX, worldY)
* worldToGrid(worldX, worldY) - recibe las componentes x e y de las coordenadas del mundo y devulve una tupla con las coordenadas equivalentes en el mapa: (gridX, gridY)


## Video demostrativo


## Atribuciones
* *Copyright (C) 2016 CC-BY-4.0 Samuel Rey (@reysam93)
Texto original creado por Victor Arribas (@varhub) adaptado para cubrir las necesidades de esta práctica. Copyright (C) 2016 CC-BY-4.0 Victor Arribas*

* *Codigo base realizado por Alberto Martín (@almartinflorido)*
* *Código de la práctica y  mundo realizado por Samuel Rey*

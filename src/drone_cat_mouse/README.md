# Cat and mouse

# Práctica drone_cat_mouse

El objetivo de esta práctica es programar un comportamiento 
autónomo para un drone que simule el juego del gato y el ratón.

## Cómo ejecutar
Para lanzar el ejemplo, sigue los siguientes pasos:

* Ejecución sin ver el mundo: `gzserver ardrone-trees-simple.world`
* Ejecución viendo el mundo: `gazebo ardrone-trees-simple.world`
* Ejecución del gato: `python2 cat.py cat_conf.cfg`
* Ejecución del ratón: `./qX_mouse qX.cfg` 
       Donde X será el ratón a ejecutar (q1, q2, etc)
* Ejecución del árbitro: `python2 referee.py referee.cfg`

Para simplificar el cierre del entorno, basta con cerrar la(s)
ventana(s) de bump_and_go. *Ctrl+C dará problemas*.

## Cómo realizar la práctica
Para realizar la práctica se debe editar el fichero MyAlgorithms.py e
insertar la lógica de control.

### Dónde insertar el código
[MyAlgorithm.py](MyAlgorithm.py#L58)
```
    def execute(self):
        # Add your code here
        tmp = self.navdata.getNavdata()
        if tmp is not None:
            print ("State: " +str(tmp.state))
            print ("Altitude: " +str(tmp.altd))
            print ("Vehicle: " +str(tmp.vehicle))
            print ("Battery %: " +str(tmp.batteryPercent))
```

### API
* cmdvel.sendCMDVel(self,vx,vy,vz,ax,ay,az): envía comandos de velocidad linear y angular al drone.
* pose.getX(), pose.getY(), pose.getZ(): devuelve los valores de posición del drone en el espacio.
* pose.getRoll(), pose.getPitch(), pose.getYaw(): devuelve los valores de rotacin del drone en el espacio.
* camera.getImage(): devuelve la imagen captada por la cámara activa del drone (frontal o ventral).
* extra.toggleCam(): cambia la cámara activa del drone (ventral o frontal).
* extra.takeOff(): despega el drone.
* extra.land(): aterriza el drone.

Para este ejemplo, se ha de conseguir que el drone negro (gato) programado por el alumno,
siga al drone rojo (ratón) que ya está preprogramado y tiene una trayectoria aleatoria lo
más cerca posible sin llegar a colisionar. La aplicación referee (árbitro en inglés) medirá 
la distancia que hay entre ambos drones y con ello asignará una puntuación. Cuanto más tiempo 
se pase cerca del ratón en el tiempo, más puntuación se obtendrá.


## Video demostrativo del campeonato de 2016
https://youtu.be/Hd2nhOx1tqI?t=8m30s

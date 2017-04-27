# 3D Recosntruction

# Práctica 3d_reconstruction

## Cómo ejecutar
Para lanzar el ejemplo, sigue los siguientes pasos:

* Ejecución sin ver el mundo: `gzserver reconstruccion3D.world`
* Ejecución viendo el mundo: `gazebo reconstruccion3D.world`
* Ejecución del visor 3D: `3DViewer 3DViewer.cfg` 
* Ejecución del ejemplo: `python2 3d_reconstruction.py 3d_reconstruction.cfg`

Para simplificar el cierre del entorno, basta con cerrar la(s)
ventana(s) de 3d_reconstruction. *Ctrl+C dará problemas*.

## Cómo realizar la práctica
Para realizar la práctica se debe editar el fichero MyAlgorithms.py e
insertar la lógica del algoritmo.

### Dónde insertar el código
[MyAlgorithm.py](MyAlgorithm.py#L74)
```
    def execute(self):
        #GETTING THE IMAGES
        imageLeft = self.sensor.getImageLeft()
        imageRight = self.sensor.getImageRight()

        if self.done:
            return

        # Add your code here
        # pointIn=np.array([502,21,1])
        # pointInOpt=self.camLeftP.graficToOptical(pointIn)
        # point3d=self.camLeftP.backproject(pointInOpt)
        # projected1 = self.camRightP.project(point3d)
        # print (self.camRightP.opticalToGrafic(projected1))
        ...
```

### API
* motors.setV() - para establecer la velocidad lineal
* motors.setW() - para establecer la velocidad angular
* motors.sendVelocities() - para enviar las velocidades previamente establecidas
* motors.sendV() - envía velocidad lineal al robot
* motors.sendW() - envía velocidad angular al robot
* self.setLeftImageFiltered(imageRight) - muestra la imagen filtrada en el frame izquierdo del GUI
* self.setRightImageFiltered(imageRight) - muestra la imagen filtrada en el frame derecho del GUI4
* PLOT 3D data on the viewer:
   ```
   point=np.array([1, 1, 1])
   self.sensor.drawPoint(point,(255,255,255))
   ```

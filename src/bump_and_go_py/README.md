# Bump and go

# Práctica bump_and_go

El objetivo de esta práctica es programar un comportamiento 
choca gira básico mediante una máquina de estados finita. 


## Cómo ejecutar
Para lanzar el ejemplo, sigue los siguientes pasos:

* Ejecución sin ver el mundo: `gzserver kobuki-simple.world`
* Ejecución viendo el mundo: `gazebo kobuki-simple.world`
* Ejecución del ejemplo: `python3 bump_and_go.py bumpGo.cfg`

Para simplificar el cierre del entorno, basta con cerrar la(s)
ventana(s) de bump_and_go. *Ctrl+C dará problemas*.

## Cómo realizar la práctica
Para realizar la práctica se debe editar el fichero MyAlgorithms.py e
insertar la lógica de control.

### Dónde insertar el código
[MyAlgorithm.py](MyAlgorithm.py#L74)
```
    def execute(self):

        # Add your code here
        print "Runing"

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        #self.sensor.sendV(10)
        #self.sensor.sendW(5)
```

### API
* motors.setV() - para establecer la velocidad lineal
* motors.setW() - para establecer la velocidad angular
* motors.sendVelocities() - para enviar las velocidades previamente establecidas
* motors.sendV() - envía velocidad lineal al robot
* motors.sendW() - envía velocidad angular al robot
* laser.getLaserData() - recoge los datos enviados por el laser en un array

### API Propia

Se ofrece la implementación de una máquina de estados finita y determinista. El alumno podrá definir
sus propios autómatas del modo que más le convenga. El autómata deberá ser creado en la clase gui/gui.py

### Dónde insertar el código
[gui/gui.py](gui.py#L396)
```
if __name__ == '__main__':
  
      import sys

      machine = Machine(3)
      machine.setStateName(0, 'Forward') 
      machine.setStateName(1, 'Backward')
      machine.setStateName(2, 'Turn')
      machine.addTransition(0, 1,'close')
      machine.addTransition(1, 2,'time')
      machine.addTransition(2, 0,'time2')
```

#### Máquina de estados
```
      Machine(n) - Crea una máquina con "n" estados 
      machine.addState(name) - Añade un estado a la máquina con nombre "name"
      machine.addTransition(orig, fin, name) - Añade una transición a la máquina con origen "orig" (numero de estado), 
            destino "fin" (numero de estado) y nombre "name"
      machine.getState(n) - Devuelve el estado "n" por número de estado o por nombre de estado
      machine.getStates() - Devuelve todos los estados de la máquina
      machine.getTransitions() - Devuleve las transiciones de la máquina
      machine.getActiveTransition() - Devuelve la transición activa de la máquina.
      machine.setStateActive(n, flag) - Marca el estado "n" como activo si "flag" es True, lo marca como inactivo si "flag" es False
      machine.setTransitionActive(n, flag) - Análogo al anterior pero para las transiciones
      machine.setStateName(n, name) - Pone al estado "n" el nombre "name"
      machine.isStateActive(n) - Determina si el estado "n" está activo
      machine.deactivateAll() - Desactiva todos los estados y transiciones.
      
```

Para este ejemplo ya se facilita una máquina de tres estados:  FORWARD, BACKWARD y TURN. Con las funciones: isStateActive(),
setTransitionActive() y setStateActive() se puede resolver este ejemplo por completo.

NOTA: La función setTransitionActive() tiene un carácter meramente visual.

## Video demostrativo
https://www.youtube.com/watch?v=HRjXf2GzK70

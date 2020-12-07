# Entrega: OpenRAVE
Este repositorio contiene la generación de laberinto con un robot en OpenRAVE, moviendo el robot desde el origen hasta la meta. Este desplazamiento se consigue mediante el uso de un plugin de openRAVE desarrollado en python para la resolución de este ejercicio.

Autor: **Pedro Arias Pérez**

Link: [pariaspe/openrave-playground](https://github.com/pariaspe/openrave-playground)


## Índice
- [1. Descripción](#1-descripción)
- [2. Estructura de Carpetas](#2-estructura-de-carpetas)
- [3. Base](#3-base)
- [4. Extras](#4-extras)
    - [4.1. Extra 1](#extra-1-prueba)

---

## 1. Descripción
Para la práctica se han realizado los siguientes hitos:

- **Base**:
    1. TODO
    
- **Extra**:
    1. Prueba: TODO

## 2. Estructura de carpetas
El esquema de organización del repositorio es el siguiente:
```
.
```

## 3. Base
Tras finalizar la instalación se calculan las dimensiones del mapa:

```bash
parias@parias-msi:~/repos/gazebo-tools$ python3
Python 3.6.9 (default, Oct  8 2020, 12:12:24)
[GCC 8.4.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> x = len("arias")*2
>>> y = len("perez")*2
>>> print(x, y)
10 10
>>>
```

Dadas estas dimensiones `(10x10)` se ha construído el siguiente mapa en formato csv `(map1.csv)`:

```bash
parias@parias-msi:~/repos/openrave-tools$ cat assets/map1.csv
1,1,1,1,1,1,1,1,1,1
1,0,0,0,0,0,0,0,0,1
1,0,0,0,0,0,0,0,0,1
1,0,0,0,0,0,0,0,0,1
1,1,1,1,0,0,0,0,0,1
1,1,1,1,0,0,0,0,0,1
1,0,0,0,0,0,0,0,0,1
1,0,0,0,0,0,0,0,0,1
1,0,0,0,0,0,0,0,0,1
1,1,1,1,1,1,1,1,1,1
```

Se ha elegido un mapa muy sencillo con una resolución de un metro por caracter. El inicio será la posición `[1,1]` (fila 2, columna 2), mientras que la meta será en la posición `[8,8]` (fila 9, columna 9).

Utilizando `openrave-map-from-csv.py` se genera el mundo de openRAVE:

```bash
parias@parias-msi:~/repos/openrave-tools$ ./openrave-map-from-csv.py 
[[1. 1. 1. 1. 1. 1. 1. 1. 1. 1.]
 [1. 0. 0. 0. 0. 0. 0. 0. 0. 1.]
 [1. 0. 0. 0. 0. 0. 0. 0. 0. 1.]
 [1. 0. 0. 0. 0. 0. 0. 0. 0. 1.]
 [1. 1. 1. 1. 0. 0. 0. 0. 0. 1.]
 [1. 1. 1. 1. 0. 0. 0. 0. 0. 1.]
 [1. 0. 0. 0. 0. 0. 0. 0. 0. 1.]
 [1. 0. 0. 0. 0. 0. 0. 0. 0. 1.]
 [1. 0. 0. 0. 0. 0. 0. 0. 0. 1.]
 [1. 1. 1. 1. 1. 1. 1. 1. 1. 1.]]
('lines = X =', 10)
('columns = Y =', 10)
```

![map.kinbody.xml](/doc/map1.png)


Tras generar el mapa se crea el entorno (`map.env.xml`) donde se incluiran el mapa y el robot. El robot en cuestion (`ecro`) se incluye con una posición inicial de [1.6, 1.75] sobre el mapa.

```xml
<Environment>
	<camtrans>18.821295 16.280615 52.393200</camtrans>
	<camrotationaxis>-0.999964 0.000198 -0.008424 176.466456</camrotationaxis>
	<camfocal>52.445621</camfocal>
	<physicsengine type="ode">
		<odeproperties>
			<friction>.5</friction>
			<gravity>0 0 -9.8</gravity>
			<selfcollision>1</selfcollision>
			<erp>.5</erp>
			<cfm>.000001</cfm>
			<dcontactapprox>1</dcontactapprox>
		</odeproperties>
	</physicsengine>

	<Robot name="ecroSim" file="ecro/ecro.robot.xml">
		<RotationAxis>0 0 1 90</RotationAxis>
		<translation>1.6 1.75 0.01</translation>
	</Robot>

	<KinBody name="map" file="map.kinbody.xml"/>
</Environment>
```

![map.env.xml](/doc/env.png)

Tras tener el entorno listo se crea el siguiente código en python que se encarga de controlar al robot para que llegue a su destino. La parte principal del código son los tres bucles infinitos que se pueden observar entre las lineas 28 y 68. El primer bucle se encarga de que el robot alcance un punto intermedio situado en `[1.6, 8]`. El segundo bucle es el que produce el giro del robot para que encare la meta. Finalmente, el tercer bucle se encarga de alcanzar la meta de forma similar al primero.

```python
#!/usr/bin/env python

from openravepy import *
import time
import numpy as np
from math import sqrt, atan2

GOAL = [8, 8]
RATE = 1


env = Environment() # create openrave environment
env.SetViewer('qtosg') # attach viewer (optional)

try:
    env.Load('/home/yo/repos/openrave-playground/tools/map.env.xml') # load a simple scene
    robot = env.GetRobots()[0] # get the first robot
    print("Using robot: ", robot.GetName())
    robot.SetController(RaveCreateController(env,'idealvelocitycontroller'),range(robot.GetDOF()),0)
    control = robot.GetController()

    env.StartSimulation(timestep=0.001)

    time.sleep(2)  # wait to viewer

    # THE ROBOT WILL GO [1.6, 8], TURN TO THE GOAL AND THEN MOVE FORWARD

    print("\nGOING TO POINT 1 [1.6, 8]")
    while True:
        x, y, _ = np.round(robot.GetTransform()[:3, -1,].T, 2)  # getting pose from ecro

        dx = 1.6 - x
        dy = 8.0 - y
        d = sqrt(dx**2 + dy**2)  # distance to point
        control.SetDesired([d, d, d, d])  # set vel
        print(d)
        if abs(d) <= 0.1:
            break  # point reached
        time.sleep(1/RATE)  # spin

    print("\nTURNING")
    while True:
        x, y, _ = np.round(robot.GetTransform()[:3, -1,].T, 2)  # getting pose from ecro
        dx = GOAL[0] - x
        dy = GOAL[1] - y

        goal_heading = atan2(dy, dx)  # heading tp goal
        current_heading = np.round(axisAngleFromRotationMatrix(robot.GetTransform()[:3, :3]), 3)[-1]  # getting heading from ecro
        vel = current_heading - goal_heading
        vel *= 2  # double vel
        print(vel)
        control.SetDesired([vel, -vel, vel, -vel])  # turning
        if abs(vel) <= 0.01:
            break  # heading reached
        time.sleep(1/RATE)  # spin

    print("\nGOING TO POINT GOAL")
    while True:
        x, y, _ = np.round(robot.GetTransform()[:3, -1,].T, 2)  # getting pose from ecro

        dx = GOAL[0] - x
        dy = GOAL[1] - y
        d = sqrt(dx**2 + dy**2)  # distance to point
        control.SetDesired([d, d, d, d])  # set vel
        print(d)
        if abs(d) <= 0.1:
            break  # point reached
        time.sleep(1/RATE)  # spin

    print("\nGOAL REACHED!!\n")
    print("BYE!")

    time.sleep(1)

finally:
    env.Destroy()

```

## 4. Extras
### Extra 1: Vídeo parte base

Se muestra en vídeo el resultado de la ejecución de la parte base.

 [![OpenRAVE Base](http://img.youtube.com/vi/cw2RJPpvA7c/0.jpg)](http://www.youtube.com/watch?v=cw2RJPpvA7c)

### Extra 2: EcroWrapper

El segundo extra ofrece una capa de abstracción sobre el robot `ecro`. Esta capa sobre el robot ofrece una serie de métodos que permiten trabajar de forma más sencilla y con órdenes más elaboradas que si trabajasemos directamente sobre el robot. Estos métodos se recogen en la clase `EcroWrapper` que se muestra a continuación.

```python
class EcroWrapper:
    def __init__(self, env, ecro):
        self.robot = ecro
        self.robot.SetController(RaveCreateController(env,'idealvelocitycontroller'),range(self.robot.GetDOF()),0)
        self.control = self.robot.GetController()

    def get_pos(self):
        H_0_robot = self.robot.GetTransform()
        return np.round(H_0_robot[:3, -1,].T, 2)

    def get_rot(self):
        H_0_robot = self.robot.GetTransform()
        return H_0_robot[:3, :3]

    def get_angles(self):
        return np.round(axisAngleFromRotationMatrix(self.get_rot()), 3)

    def get_heading(self):
        return self.get_angles()[-1]

    def set_linear_vel(self, vel):
        if vel > MAX_VEL:
            vel = MAX_VEL
        self.control.SetDesired(4*[vel])

    def set_angular_vel(self, vel):
        if vel > MAX_VEL:
            vel = MAX_VEL
        self.control.SetDesired([vel, -vel, vel, -vel])

    def set_cmd_pos(self, pos, block=False):
        if block:
            print("\nHEADING TO ({0}, {1})".format(str(pos[0]), str(pos[1])))
            dif = 1  # init value, just to enter in while
            while abs(dif) > ANG_TOL:
                x, y, _ = self.get_pos()

                dx = pos[0] - x
                dy = pos[1] - y

                goal_heading = atan2(dy, dx)

                dif = self.get_heading() - goal_heading
                self.set_angular_vel(dif*4)

                time.sleep(1/RATE)

            print("GOING TO ({0}, {1})".format(str(pos[0]), str(pos[1])))
            dist = 1
            while abs(dist) > LIN_TOL:
                x, y, _ = self.get_pos()

                dx = pos[0] - x
                dy = pos[1] - y
                dist = sqrt(dx**2 + dy**2)
                self.set_linear_vel(dist*4)

                time.sleep(1/RATE)

            return True
        else:
            pass
```

Entre las acciones que recoge el `Wrapper` esta el acceso a información del robot como la posición o la orientación, un control en velocidad (tanto lineal como angular) y un control en posición bloqueante.

### Extra 3:
El tercer extra incluye la integración de un planificador de movimientos A\*. El planificador que se utiliza es el realizado para la asignatura de *Introducción a la Planificación de Robots* [1](https://github.com/pariaspe/graph-searching/tree/main/extra/astar). Los archivos necesarios se copían a este repositorio para evitar dependencias. El código es exactamente el mismo salvo por unas constantes de ruta que se modifican para poder encontrar el mapa utilizado.

El plafinicador se ejecuta en un proceso a parte y su salida se parsea para extraer la ruta a seguir. Previo a su ejecución, la ruta se optimiza levemente para reducir el numero de órdenes a enviar al robot.

```python
def get_route(map, start, end):
    map_inflated = inflate_map(map)
    process = Popen(["python3", "tools/astar.py", "-m", map_inflated, "-s", str(start[0]), str(start[1]), "-e", str(end[0]), str(end[1])], stdout=PIPE, stderr=PIPE)
    stdout, stderr =  process.communicate()

    output =stdout.split('\n')[:-12]
    i = -1
    while True:
        if output[i] == "%%%%%%%%%%%%%%%%%%%":
            break
        i -= 1
    output = output[i+1:]
    steps = [end]
    for l in output:
        steps.insert(0, [int(l[13]), int(l[19])])
    return optimize_route(steps)
```
En la línea 2 de cógido, se puede observar como el mapa se *infla* antes de calcular la ruta. Esto se explica en el siguiente apartado.

### Extra 4: Inflated maps
El cuarto extra agranda (*infla*) los obstaculos del mapa para simular el volumen del robot. Esto es necesario para que la ruta que calcule el planificador sea realizable por el robot. Los aspectos del código se pueden observar los métodos `inflate_map` y `inflate_squares` de `extra.py`. A modo ilustrativo, se muestra el mapa *inflado* que se ha utilizado.

```txt
1,1,1,1,1,1,1,1,1,1
1,1,1,1,1,1,1,1,1,1
1,1,0,0,0,0,0,0,1,1
1,1,1,1,0,0,0,0,1,1
1,1,1,1,1,0,0,0,1,1
1,1,1,1,1,0,0,0,1,1
1,1,1,1,0,0,0,0,1,1
1,1,0,0,0,0,0,0,1,1
1,1,1,1,1,1,1,1,1,1
1,1,1,1,1,1,1,1,1,1
```


### Extra 5: Vídeo parte extra
Se muestra en vídeo el resultado de la ejecución de la parte extra.

 [![OpenRAVE Extra](http://img.youtube.com/vi/cw2RJPpvA7c/0.jpg)](http://www.youtube.com/watch?v=cw2RJPpvA7c)
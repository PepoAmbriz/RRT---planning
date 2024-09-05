# RRT-planning

Algoritmo RRT para un robot tipo automóvil.

- José Francisco Ambriz Gutiérrez.


Librerías necesarias
Versión de Python: 3.10.11

1 import glfw
2 from OpenGL . GL import *
3 import pygame
4 import numpy as np
5 import math
6 import random
7 import time

Puedes correr el código una vez instaladas las dependencias necesarias, el código genera mapas aleatorios en donde se ejecuta el RRT. Una vez ejecutado el script presiona ’espacio’ para iniciar la expansión.

Si el mapa generado no se puede resolver mejor vuelve a correr el script ya que el algoritmo nunca encontrará solución (puedes hacerlo cerrando la ventana o bien tecleando ctrl + c en la terminal de ejecución).

**** Parámetros modificables.

- Puedes cambiar las posiciónes del robot y del goal a tu antojo (linea 15).

- Si lo deseas puedes utilizar el mapa predeterminado comentando la linea ’obstacles = random_map()’ (linea 62).

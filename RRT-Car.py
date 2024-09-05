# RRT básico para el car-like robot
# José Francisco Ambriz Gutiérrez

import glfw
from OpenGL.GL import *
import pygame
import numpy as np
import math
import random
import time

width = 1300
height = 1300

### Posiciones iniciales y finales del robot ###
robot = (-500, -450, 0)
L=30
goal = (500, 500, math.pi/2)
goal_radius = 50  # Radio del goal


R = L/2
epsilon = 10  # Tamaño del paso hasta donde agregamos un nuevo nodo
v = 1  # Velocidad del robot	
max_turn_angle = math.pi / 4  # Máximo ángulo de giro del robot

# Lista de obstáculos rectangulares con 2 puntos opuestos por obstáculo
obstacles_border = [((-700, -700), (-600, 700)),
                ((-700, -700), (700, -600)),
                ((600, -700), (700, 700)),
                ((-700, 600), (700, 700))]

obsatcles=      [((-400, -100), (-300, 400)),
                ((-400, -400), (100, -300)),
                ((300, -400), (400, 100)),
                ((-100, 300), (400, 400)),
                #((-100, -100), (0, 100)),
                ((-100, -100), (100, 0)),
                ((0, -100), (100, 100))]
obstacles = obstacles_border + obsatcles
def random_map():
    obstacles = []
    num_obstacles = random.randint(3, 15)  # Número aleatorio de obstáculos

    for _ in range(num_obstacles):
        # Primer punto del obstáculo
        x1 = random.uniform(-width / 2 + 100, width / 2 - 100)
        y1 = random.uniform(-height / 2 + 100, height / 2 - 100)

        max_offset_x = min(150, width / 2 - x1 )  # Limitar el tamaño máximo en x
        max_offset_y = min(150, height / 2 - y1 )  # Limitar el tamaño máximo en y

        x2 = random.uniform(x1 + 50, x1 + max_offset_x)
        y2 = random.uniform(y1 + 50, y1 + max_offset_y)
        
        obstacles.append(((x1, y1), (x2, y2)))
    
    obstacles = obstacles_border + obstacles
    return obstacles
# Comentar si queremos el mapa original cool
obstacles = random_map()

#Colores y flag
iniciar = False
green = (0.0, 1.0, 0.0, 1)
red = (1.0, 0.0, 0.0, 1.0)
blue = (0.0, 0.0, 1.0, 1)
obstacle_color = (0.0, 0.0, 0.0, 1)
polygon_color = (1, 0, 0, 0.3)

### INICIA FUNCIONES DE DIBUJO ###

#Funcion para inicializar la ventana de OpenGL
def myInit():
    if not glfw.init():
        raise Exception("No se pudo inicializar GLFW")
    window = glfw.create_window(width, height, "RRT - Carrito 2D", None, None)
    if not window:
        glfw.terminate()
        raise Exception("No se pudo crear la ventana")
    glfw.make_context_current(window)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glClearColor(1.0, 1.0, 1.0, 1.0)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glOrtho(-width / 2, width / 2, -height / 2, height / 2, -1, 1)
    glMatrixMode(GL_MODELVIEW)

#Para leer teclas del teclado
def key_callback(window, key, scancode, action, mods):
    global iniciar
    if key == glfw.KEY_SPACE and action == glfw.PRESS:
        iniciar = not iniciar

# Para hacer circulos
def draw_circle(x, y, radius, segments=30, color=(0.0, 0.0, 0.0, 1.0)):
    glBegin(GL_TRIANGLE_FAN)
    glColor4f(*color)
    glVertex2f(x, y)
    for i in range(segments + 1):
        angle = 2.0 * np.pi * float(i) / segments
        glVertex2f(x + radius * np.cos(angle), y + radius * np.sin(angle))
    glEnd()

#Función para dibujar flecha orientada en point=(x,y,theta) con un tamaño de L y grosor w 
def draw_arrow(point, L, w=4, color=(0.0, 0.0, 1.0, 1.0)):
    x, y, theta = point
    x1 = x + L * math.cos(theta)
    y1 = y + L * math.sin(theta)
    glColor4f(*color)
    glLineWidth(w)
    glBegin(GL_LINES)
    glVertex2f(x, y)
    glVertex2f(x1, y1)
    glEnd()
    glBegin(GL_TRIANGLES)
    glVertex2f(x1, y1)
    glVertex2f(x1 - 10 * math.cos(theta + math.pi / 6), y1 - 10 * math.sin(theta + math.pi / 6))
    glVertex2f(x1 - 10 * math.cos(theta - math.pi / 6), y1 - 10 * math.sin(theta - math.pi / 6))
    glEnd()

#Función para dibujar un obstáculo cuadrado con 2 puntos opuestos por obstáculo
def draw_obstacle(obstacle, obstacle_color):
    x1, y1, x2, y2 = obstacle[0][0], obstacle[0][1], obstacle[1][0], obstacle[1][1]
    glColor4f(*obstacle_color)
    glBegin(GL_QUADS)
    glVertex2f(x1, y1)
    glVertex2f(x2, y1)
    glVertex2f(x2, y2)
    glVertex2f(x1, y2)
    glEnd()

#Funcion para dibujar un carrito con el eje trasero en (x,y) y orientado en theta
def draw_car(x, y, theta, L):
    rad = theta 
    # Precalcular los valores trigonométricos
    cos_theta = np.cos(rad)
    sin_theta = np.sin(rad)
    # Definir las dimensiones del coche
    width = L *0.9
    height = L/2
    corners = [
        (-width, -height),
        (width, -height),
        (width, height),
        (-width, height)
    ]
    # Aplicar la transf a cada esquina
    rotated_translated_corners = []
    for cx, cy in corners:
        new_x = x + (cx * cos_theta - cy * sin_theta)
        new_y = y + (cx * sin_theta + cy * cos_theta)
        rotated_translated_corners.append((new_x, new_y))
    # Dibujar el cuerpo del coche
    glColor4f(0.5, 0.7, 1.0, 1.0)
    glBegin(GL_QUADS)
    for corner in rotated_translated_corners:
        glVertex2f(corner[0], corner[1])
    glEnd()

    # Definir las posiciones relativas de las ruedas
    wheel_width = L / 3.0
    wheel_height = L / 4.0
    wheel_offsets = [
        (-width * 0.9, -height * 1.1),
        (width * 0.9, -height * 1.1),
        (-width * 0.9, height * 1.1),
        (width * 0.9, height * 1.1)
    ]

    # Dibujar las ruedas del coche
    glColor4f(0.0, 0.0, 0.0, 0.7)
    for offset_x, offset_y in wheel_offsets:
        wheel_corners = [
            (offset_x - wheel_width / 2, offset_y - wheel_height / 2),
            (offset_x + wheel_width / 2, offset_y - wheel_height / 2),
            (offset_x + wheel_width / 2, offset_y + wheel_height / 2),
            (offset_x - wheel_width / 2, offset_y + wheel_height / 2)
        ]
        glBegin(GL_QUADS)
        for wx, wy in wheel_corners:
            # Aplicar la rotación y translación a las esquinas de la rueda
            new_wx = x + (wx * cos_theta - wy * sin_theta)
            new_wy = y + (wx * sin_theta + wy * cos_theta)
            glVertex2f(new_wx, new_wy)
        glEnd()
    

#Funcion para hacer lineas ente puntos
def draw_line(point1, point2, color=(0.0, 0.0, 0.0, 1.0), thickness=1.5):
    glColor4f(*color)
    glLineWidth(thickness)
    glBegin(GL_LINES)
    glVertex2f(point1[0], point1[1])
    glVertex2f(point2[0], point2[1])
    glEnd()

#Iterar sobre el arrray de obstáculos y dibujarlo
def draw_obstacle_map():
    for obstacle in obstacles:
        draw_obstacle(obstacle, obstacle_color)


#### TERMINA FUNCIONES DE DIBUJO ####

#Coordenadas del mundo a coordenadas de pixel
def world_to_pixel(x, y, window_width, window_height):
    pixel_x = int(x + (window_width * 0.5))
    pixel_y = int(y + (window_height * 0.5))
    return pixel_x, pixel_y

#Funcion para leer un pixel
def read_pixel_color(x1, y1):
    x,y= world_to_pixel(x1, y1, width, height)
    glFlush()
    data = glReadPixels(x, y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE)
    # Accede a los componentes de color directamente en el resultado de glReadPixels
    red = data[0] / 255.0
    green = data[1] / 255.0
    blue = data[2] /255.0
    alpha = data[3] / 255.0 
    #print(red, green, blue, alpha)
    return (red, green, blue, alpha)

# Función para verificar si un punto (x, y) está dentro de algún obstáculo usando colores
def is_inside_obstacle(x, y):
        if (read_pixel_color(x,y) != (1,1,1,1) and read_pixel_color(x,y) != red and read_pixel_color(x,y) != (0,0,0,0) and read_pixel_color(x,y) != blue):
            return True
        else:
            return False
#Función que toma 20 puntos en la circunferencia de un radio R y un centro (x,y) y verifica si alguno de ellos está dentro de un obstáculo
def is_inside_obstacle_circle(x, y, R):
    R = R + 10
    for i in range(20):
        x1 = x + R * math.cos(i * 2 * math.pi / 20)
        y1 = y + R * math.sin(i * 2 * math.pi / 20)
        if is_inside_obstacle(x1, y1):
            return True
    return False

#Metrica de distancia entre dos puntos
def distance(point1, point2):
    alpha= min(abs(point1[2]-point2[2]), 2*math.pi-abs(point1[2]-point2[2]))
    return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2 + alpha**2)

#Para dar un paso de un punto a otro usamos el modelo de car-like robot e integracion de euler
#xp = x + v * cos(theta) * dt
#yp = y + v * sin(theta) * dt
#thetap = theta + omega * dt

def generate_random_controls(num_controls=50):
    controls = []
    for _ in range(num_controls):
        # Genera una velocidad lineal y angular aleatorias dentro de los límites del robot
        v_random = random.choice([v, -v])
        phi_random = random.uniform(-max_turn_angle, max_turn_angle)
        controls.append((v_random, phi_random))
    return controls

def apply_control(p1, control, dt):
    x, y, theta = p1
    v, phi = control
    #print(v, phi)
    # Actualiza la posición del robot utilizando el modelo cinemático
    theta_new = theta + (v / L) * math.tan(phi) * dt
    x_new = x + v * math.cos(theta) * dt
    y_new = y + v * math.sin(theta) * dt

    theta_new = math.atan2(math.sin(theta_new), math.cos(theta_new))
    
    return (x_new, y_new, theta_new)

def step_from_to(p1, p2, epsilon=30):
    best_control = None
    min_distance = float('inf')
    best_new_point = None

    # Genera controles aleatorios
    controls = generate_random_controls()

    for control in controls:
        dt = epsilon / abs(control[0])  # Ajusta dt en función de la velocidad
        new_point = apply_control(p1, control, dt)
        dist = distance(new_point, p2)
        
        if dist < min_distance and not is_inside_obstacle_circle(new_point[0], new_point[1], R):
            min_distance = dist
            best_control = control
            best_new_point = new_point

    return best_new_point, best_control

#Funcion para encontrar el nodo más cercano a un punto en una lista de nodos
def encontrar_nodo_mas_cercano(padres, punto):
    if not padres:
        return None, float('inf')

    nodo_cercano = None
    distancia_minima = float('inf')

    for padre in padres:
        dist = distance(padre, punto)
        if dist < distancia_minima:
            distancia_minima = dist
            nodo_cercano = padre

    return nodo_cercano, distancia_minima


def main():
    global iniciar, robot, goal, obstacles, R, L,goal_radius, epsilon

    myInit()
    glfw.set_key_callback(glfw.get_current_context(), key_callback)
    pygame.init()
    font = pygame.font.Font(None, 36)

    # Listas de padres e hijos
    padres = [(robot, None)]  # El nodo inicial no tiene control asociado
    hijos = []
    flag = 0

    while not glfw.window_should_close(glfw.get_current_context()):
        glClear(GL_COLOR_BUFFER_BIT)

        draw_obstacle_map()
    
        if iniciar:
            flag = 0
            # Generamos un punto aleatorio de manera uniforme en el mapa y lo graficamos
            punto = (random.uniform(-width / 2 +20, width / 2  -20), random.uniform(-height / 2  +20, height / 2  -20), random.uniform(0, 2*math.pi))
            draw_circle(punto[0], punto[1], 5, color=blue)

            # Calculamos el nodo más cercano al punto aleatorio generado
            nodo_cercano, _ = encontrar_nodo_mas_cercano([p[0] for p in padres], punto)
            # Calculamos el nuevo nodo con el paso
            nuevo_nodo, control = step_from_to(nodo_cercano, punto, epsilon)

            # Si el nuevo nodo es válido, lo agregamos al árbol
            if nuevo_nodo:
                padres.append((nuevo_nodo, control))
                hijos.append(nodo_cercano)
                # Dibujamos la línea entre el nodo cercano y el nuevo nodo en color azul
                draw_line(nodo_cercano, nuevo_nodo, blue)
                # Si el nuevo nodo está a una distancia menor a 25 del goal, terminamos
                if distance(nuevo_nodo, goal) < goal_radius:
                    iniciar = False


        # Dibujamos el árbol
        for padre, hijo in zip(hijos, padres[1:]):
            draw_line(padre, hijo[0], blue)

        # Dibujamos el robot
        draw_car(robot[0], robot[1], robot[2], L)
        draw_arrow(robot, L)
        # Dibujamos el goal
        draw_circle(goal[0], goal[1], goal_radius, color=(1, 0, 0, 0.2))
        draw_arrow(goal, L)

        # Si el árbol ha terminado, trazamos la ruta desde el último nodo hasta el nodo inicial
        if not iniciar and len(padres) > 1:
            # Empezamos desde el último nodo
            nodo_actual = padres[-1][0]
            ruta_controles = []
            ruta_nodos = [nodo_actual]
            # Mientras no lleguemos al nodo inicial
            while nodo_actual != robot:
                # Buscamos el índice del nodo actual en la lista padres
                indice_actual = [p[0] for p in padres].index(nodo_actual)
                # Encontramos el padre del nodo actual
                padre = hijos[indice_actual - 1]
                # Vamos guardando la ruta
                ruta_nodos.append(padre)
                # Actualizamos el nodo actual para que sea su padre
                nodo_actual = padre

                # Guardamos el control asociado al nodo actual
                ruta_controles.append(padres[indice_actual][1])

            # Invertimos la lista de controles para aplicar en el orden correcto
            ruta_controles.reverse()
            flag = 1

        if flag == 1:
            for control in ruta_controles:
                #Dibujamos la ruta en rojo

                dt = epsilon / abs(control[0])
                robot = apply_control(robot, control, dt)
                #Borrar el robot anterior
                glClear(GL_COLOR_BUFFER_BIT)
                draw_car(robot[0], robot[1], robot[2], L)
                draw_arrow(robot, L)
                #Dibujar el arbol
                for padre, hijo in zip(hijos, padres[1:]):
                    draw_line(padre, hijo[0], blue)
                #Dibujar ruta en rojo
                for i in range(len(ruta_nodos) - 1):
                    draw_line(ruta_nodos[i], ruta_nodos[i + 1], red, 2)

                #Dibujar obstaculos
                draw_obstacle_map()
                #Dibujar goal
                draw_circle(goal[0], goal[1], goal_radius, color=(1, 0, 0, 0.2))
                draw_arrow(goal, L)
                glfw.swap_buffers(glfw.get_current_context())
                glfw.poll_events()
                time.sleep(0.1)  # Pequeño retraso para ver el movimiento del robot


        glfw.swap_buffers(glfw.get_current_context())
        glfw.poll_events() 

    glfw.terminate()


# ...

if __name__ == "__main__":
    main()

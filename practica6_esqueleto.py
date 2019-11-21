#! /usr/bin/python

# 6ta Practica Laboratorio
# Complementos Matematicos I

import argparse
import matplotlib.pyplot as plt
import numpy as np
import random
import math

class LayoutGraph:

    def __init__(self, grafo, iters, refresh, c1, c2, temp, verbose=False):
        '''
        Parametros de layout:
        iters: cantidad de iteraciones a realizar
        refresh: Numero de iteraciones entre actualizaciones de pantalla.
        0 -> se grafica solo al final.
        c1: constante usada para calcular la repulsion entre nodos
        c2: constante usada para calcular la atraccion de aristas
        '''

        # Guardo el grafo
        self.grafo = grafo

        # Inicializo estado
        self.posiciones = {}
        self.accum = {}

        # Guardo opciones
        self.iters = iters
        self.refresh = refresh
        self.c1 = c1
        self.c2 = c2
        self.temp = temp
        self.verbose = verbose
        self.gravity = 0.002

        # Tamaño del gráfico
        self.size = (1000, 1000)

    def layout(self):
        '''
        Aplica el algoritmo de Fruchtermann-Reingold para obtener (y mostrar)
        un layout
        '''

        # Hago que se muestre el gráfico completo
        axes = plt.gca()
        axes.set_xlim([0, self.size[0]])
        axes.set_ylim([0, self.size[1]])

        # Posiciones aleatorias para cada vértice
        for v in self.grafo[0]:
            self.posiciones[v] = (random.randint(0,self.size[0]), random.randint(0,self.size[1]))

        plt.ion()

        for i in range(self.iters):
            if self.verbose:
                print("Iteración n°", i)
            step(self)
            if self.refresh !=0 and i % self.refresh == 0:
                refrescar(self)
                plt.draw()
                plt.pause(0.0001)
                plt.clf()
            if i == self.iters - 1:
                refrescar(self)
            if self.verbose and i != self.iters - 1:
                print("=============================")
        
        plt.ioff()
        plt.show()

def refrescar(self):
    for v in self.grafo[0]:
        plt.scatter(self.posiciones[v][0], self.posiciones[v][1])
    for e in self.grafo[1]:
        v0 = e[0]
        v1 = e[1]
        plt.plot([self.posiciones[v0][0], self.posiciones[v1][0]], [self.posiciones[v0][1], self.posiciones[v1][1]])   

def step(self):
    initialize_accumulators(self)
    compute_attraction_forces(self)
    compute_repulsion_forces(self)
    compute_gravity_forces(self)
    update_positions(self)
    update_temperature(self)

def initialize_accumulators(self):
    for v in self.grafo[0]:
        self.accum[v] = (0,0)

def compute_attraction_forces(self):
    if(self.verbose):
        print("Calculando fuerzas de atraccion...")
    for e in self.grafo[1]:
        v0 = self.posiciones[e[0]]
        v1 = self.posiciones[e[1]]
        dist = math.sqrt((v0[0] - v1[0])**2 + (v0[1] - v1[1])**2)
        mod_fa = (dist**2)/(self.c2*math.sqrt((self.size[0] * self.size[1])/len(self.grafo[0])))
        fx = (mod_fa*(v1[0] - v0[0]))/dist
        fy = (mod_fa*(v1[1] - v0[1]))/dist
        self.accum[e[0]] = (self.accum[e[0]][0] + fx, self.accum[e[0]][1] + fy)
        self.accum[e[1]] = (self.accum[e[1]][0] - fx, self.accum[e[1]][1] - fy)
        if(self.verbose):
            print("Fuerza de atracción entre " + e[0] + " y " + e[1] + ": (" + str(fx) + " , " + str(fy) + ")")

def compute_repulsion_forces(self):
    if self.verbose:
        print("Calculando fuerzas de repulsión...")
    for n0 in self.grafo[0]:
        for n1 in self.grafo[0]:
            if n0 != n1:
                v0 = self.posiciones[n0]
                v1 = self.posiciones[n1]
                dist = math.sqrt((v0[0] - v1[0])**2 + (v0[1] - v1[1])**2)
                if dist < 0.005:
                    fx = random.random()
                    fy = random.random()
                else:
                    mod_fa = ((self.c1*math.sqrt((self.size[0] * self.size[1])/len(self.grafo[0])))**2)/dist
                    fx = (mod_fa*(v1[0] - v0[0]))/dist
                    fy = (mod_fa*(v1[1] - v0[1]))/dist
                self.accum[n0] = (self.accum[n0][0] - fx, self.accum[n0][1] - fy)
                self.accum[n1] = (self.accum[n1][0] + fx, self.accum[n1][1] + fy)
                if(self.verbose):
                    print("Fuerza de repulsión entre " + n0 + " y " + n1 + ": (" + str(fx) + " , " + str(fy) + ")")

def compute_gravity_forces(self):
    for n0 in self.grafo[0]:
            v0 = self.posiciones[n0]
            v1 = (self.size[0]/2,self.size[1]/2)
            mod_fa = self.gravity
            dist = math.sqrt((v0[0] - v1[0])**2 + (v0[1] - v1[1])**2)
            fx = (mod_fa*(v1[0] - v0[0]))/dist
            fy = (mod_fa*(v1[1] - v0[1]))/dist
            self.accum[n0] = (self.accum[n0][0] + fx, self.accum[n0][1] + fy)
            if self.verbose:
                print("Fuerza de gravedad para " + n0 + ": (" + str(fx) + " , " + str(fy) + ")")

def update_positions(self):
    if self.verbose:
        print("Calculando posiciones...")
    for v in self.grafo[0]:
        mod_accum = math.sqrt((self.accum[v][0] ** 2) + (self.accum[v][1] ** 2))
        if mod_accum > self.temp:
            c = self.temp/mod_accum
            self.accum[v] = (self.accum[v][0]*c,self.accum[v][1]*c)
        px = self.posiciones[v][0] + self.accum[v][0]
        py = self.posiciones[v][1] + self.accum[v][1]
        self.posiciones[v] = (px,py)
        if self.verbose:
            print("Posición de " + v + ": " + str(self.posiciones[v]))

def update_temperature(self):
    self.temp = 0.95 * self.temp
    if self.verbose:
        print("Temperatura: " + str(self.temp))

def lee_grafo_archivo(nombre):
   vertices = []
   aristas = []
   grafo = (vertices, aristas)
   cantVertices = 0
   with open(nombre, 'r') as archivo:
       nroLinea = 1
       cantVertices = int(archivo.readline())
       for line in archivo.readlines():
           if nroLinea <= cantVertices:
               vertices.append(line[:len(line)-1:]) 
           else:
               line = line.split()
               aristas.append((line[0], line[1]))
           nroLinea += 1
   return grafo   

def main():
    # Definimos los argumentos de linea de comando que aceptamos
    parser = argparse.ArgumentParser()

    # Verbosidad, opcional, False por defecto
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Muestra mas informacion al correr el programa'
    )
    # Cantidad de iteraciones, opcional, 50 por defecto
    parser.add_argument(
        '--iters',
        type=int,
        help='Cantidad de iteraciones a efectuar',
        default=50
    )
    # Cada cuantas iteraciones se refresca el gráfico, opcional, 0 por defecto
    parser.add_argument(
        '--refresh',
        type=int,
        help='Frecuencia de refresco del gráfico',
        default=0
    )
    # Temperatura inicial
    parser.add_argument(
        '--temp',
        type=float,
        help='Temperatura inicial',
        default=100.0
    )
    # Archivo del cual leer el grafo
    parser.add_argument(
        'file_name',
        help='Archivo del cual leer el grafo a dibujar'
    )

    args = parser.parse_args()

    # Creamos nuestro objeto LayoutGraph
    layout_gr = LayoutGraph(
        grafo=lee_grafo_archivo(args.file_name),
        iters=args.iters,
        refresh=args.refresh,
        c1=0.1,
        c2=5,
        temp=args.temp,
        verbose=args.verbose
        )

    # Ejecutamos el layout
    layout_gr.layout()
    return


if __name__ == '__main__':
    main()

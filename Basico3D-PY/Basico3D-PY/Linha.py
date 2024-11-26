# ************************************************
#   Linha.py
#   Define a classe Linha
#   Autor: Márcio Sarroglia Pinho
#       pinho@pucrs.br
# ************************************************

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from Ponto import Ponto

from random import randint as rand

""" Classe Linha """
class Linha:
    def __init__(self, minx: float = 0, miny: float = 0, maxx: float = 0, maxy: float = 0, x1: float = 0, y1: float = 0, x2: float = 0, y2: float = 0):
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    """ Gera uma linha com tamanho 'tamMax' dentro de um limite 'limite'. Armazena os valores nas variaveis x1,x2,y1,y2."""
    def geraLinha(self, limite: int, tamMax: int):
        self.x1 = (rand(0, limite)*10) / 10.0
        self.y1 = (rand(0, limite)*10) / 10.0

        deltaX = rand(0, limite) / limite
        deltaY = rand(0, limite) / limite

        if (rand(0, 1) % 2):
            self.x2 = self.x1 + deltaX * tamMax
        else:
            self.x2 = self.x1 - deltaX * tamMax

        if (rand(0, 2) % 2):
            self.y2 = self.y1 + deltaY * tamMax
        else:
            self.y2 = self.y1 - deltaY * tamMax

    def geraLinha(self, x1: float, y1: float, x2: float, y2: float):
        self.x1 = x1
        self.y1 = y1

        self.x2 = x2
        self.y2 = y2
        #print("Gera Linha")

    
    """ Desenha a linha na tela atual """
    def desenhaLinha(self):
        glBegin(GL_LINES)
        
        glVertex2f(self.x1, self.y1)
        glVertex2f(self.x2, self.y2)

        glEnd()

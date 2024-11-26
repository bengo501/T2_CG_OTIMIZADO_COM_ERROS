# ***********************************************************************************
#   ExibePoligonos.py
#       Autor: Márcio Sarroglia Pinho
#       pinho@pucrs.br
#   Este programa exibe um polígono em OpenGL
#   Para construir este programa, foi utilizada a biblioteca PyOpenGL, disponível em
#   http://pyopengl.sourceforge.net/documentation/index.html
#
#   Sugere-se consultar também as páginas listadas
#   a seguir:
#   http://bazaar.launchpad.net/~mcfletch/pyopengl-demo/trunk/view/head:/PyOpenGL-Demo/NeHe/lesson1.py
#   http://pyopengl.sourceforge.net/documentation/manual-3.0/index.html#GLUT
#
#   No caso de usar no MacOS, pode ser necessário alterar o arquivo ctypesloader.py,
#   conforme a descrição que está nestes links:
#   https://stackoverflow.com/questions/63475461/unable-to-import-opengl-gl-in-python-on-macos
#   https://stackoverflow.com/questions/6819661/python-location-on-mac-osx
#   Veja o arquivo Patch.rtf, armazenado na mesma pasta deste fonte.
# ***********************************************************************************

import os

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

from Ponto import *

PontoInstanciado  = Ponto()

def calcula_ponto(p):
        ponto_novo = []
        matriz_gl = glGetFloatv(GL_MODELVIEW_MATRIX)
        #print (matriz_gl)
        for i in range(4):
            ponto_novo.append(
                                matriz_gl[0][i] * p.x +
                                matriz_gl[1][i] * p.y +
                                matriz_gl[2][i] * p.z +
                                matriz_gl[3][i]
                            )
        #print ("Ponto Novo:", ponto_novo)
       
        return ponto_novo[0], ponto_novo[1], ponto_novo[2]

w,h= 500,500
def square():
    glBegin(GL_QUADS)
    glVertex2f(100, 100)
    glVertex2f(200, 100)
    glVertex2f(200, 200)
    glVertex2f(100, 200)
    glEnd()

def iterate():
    glViewport(0, 0, 500, 500)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glOrtho(0.0, 500, 0.0, 500, 0.0, 1.0)
    glMatrixMode (GL_MODELVIEW)
    glLoadIdentity()

def showScreen():
    print ("showScreen")
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    iterate()
    glColor3f(1.0, 0.0, 3.0)
    square()
    P = calcula_ponto(Point(1,1))
    print ("Ponto Instanciado Antes do Translate: ", P);
    glTranslatef(5,5,0)
    P = calcula_ponto(Point(1,1))
    print ("Ponto Instanciado DEPOIS do Translate: ", P);
    glutSwapBuffers()


# The function called whenever a key is pressed. Note the use of Python tuples to pass in: (key, x, y)
#ESCAPE = '\033'
ESCAPE = b'\x1b'

def getKey(*args):
    
    print (args)
    # If escape is pressed, kill everything.
    if args[0] == b'q':
        os._exit(0)
    if args[0] == ESCAPE:
        os._exit(0)
    if args[0] == b'a':
        glutPostRedisplay()


glutInit(sys.argv)
#glutInit()
glutInitDisplayMode(GLUT_RGBA)
glutInitWindowSize(500, 500)
glutInitWindowPosition(0, 0)
wind = glutCreateWindow("OpenGL Coding Practice")
glutDisplayFunc(showScreen)
#glutIdleFunc(showScreen)
glutKeyboardFunc(getKey)

try:
	glutMainLoop()
except SystemExit:
	pass

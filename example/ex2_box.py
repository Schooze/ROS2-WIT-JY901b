import serial
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
"""
Download semua library yang dibutuhkan dengan menjalankan perintah berikut:

pip install pyserial pygame PyOpenGL

"""

# Inisialisasi serial
jy_sensor = serial.Serial(port="/dev/ttyUSB0", baudrate="9600", timeout=1)

def reverse_angle(angle):
    """ Membalik nilai sudut dari 0-360 menjadi 360-0. """
    return 360 - angle

# Fungsi menggambar kotak 3D dengan warna berbeda di tiap sisinya
def draw_colored_cube():
    vertices = [
        [1, 1, -1], [1, -1, -1], [-1, -1, -1], [-1, 1, -1],  # Belakang
        [1, 1, 1], [1, -1, 1], [-1, -1, 1], [-1, 1, 1]       # Depan
    ]

    faces = [
        (0, 1, 2, 3),  # Belakang (merah)
        (4, 5, 6, 7),  # Depan (hijau)
        (0, 4, 7, 3),  # Kiri (biru)
        (1, 5, 6, 2),  # Kanan (kuning)
        (3, 7, 6, 2),  # Bawah (ungu)
        (0, 4, 5, 1)   # Atas (cyan)
    ]

    colors = [
        (1, 0, 0),  # Merah
        (0, 1, 0),  # Hijau
        (0, 0, 1),  # Biru
        (1, 1, 0),  # Kuning
        (1, 0, 1),  # Ungu
        (0, 1, 1)   # Cyan
    ]

    glBegin(GL_QUADS)
    for i, face in enumerate(faces):
        glColor3fv(colors[i])  # Atur warna
        for vertex in face:
            glVertex3fv(vertices[vertex])
    glEnd()

# Fungsi menggambar outline merah di pinggir kotak
def draw_red_outline():
    vertices = [
        [1, 1, -1], [1, -1, -1], [-1, -1, -1], [-1, 1, -1],  # Belakang
        [1, 1, 1], [1, -1, 1], [-1, -1, 1], [-1, 1, 1]       # Depan
    ]

    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7)
    ]

    glColor3f(1, 0, 0)  # Warna merah
    glLineWidth(3)  # Ketebalan garis
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

# Fungsi menggabungkan byte untuk mendapatkan nilai sudut
def combine_bytes(DataH, DataL):
    Data = (DataH << 8) | DataL
    return Data

# Inisialisasi Pygame dan OpenGL
pygame.init()
display = (800, 600)
pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
glTranslatef(0.0, 0.0, -5)

roll, pitch, yaw = 0, 0, 0  # Variabel sudut awal

# Tunggu inisialisasi sensor
while True:
    data = jy_sensor.read(size=1)
    if data == b'\x55':
        print("Sensor connected!")
        jy_sensor.read(size=10)
        break

# Loop utama
running = True
while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
    
    data = jy_sensor.read(size=11)
    if len(data) == 11 and data[1] == 83:  # Jika paket data adalah sudut (Angle)
        roll = combine_bytes(data[3], data[2]) / 32768 * 180
        pitch = combine_bytes(data[5], data[4]) / 32768 * 180
        yaw = combine_bytes(data[7], data[6]) / 32768 * 180

    print(f"Angle output: {roll:.3f}, {pitch:.3f}, {yaw:.3f} degrees")
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glPushMatrix()

    glRotatef(pitch, 1, 0, 0)   # Rotasi terhadap sumbu X
    glRotatef(yaw, 0, 1, 0)     # Rotasi terhadap sumbu Y
    glRotatef(reverse_angle(roll), 0, 0, 1)  # Rotasi terhadap sumbu Z

    draw_colored_cube()  # Gambar kubus dengan warna
    draw_red_outline()   # Tambahkan outline merah
    
    glPopMatrix()
    pygame.display.flip()
    pygame.time.wait(10)

pygame.quit()
jy_sensor.close()

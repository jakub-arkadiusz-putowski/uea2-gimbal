import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import time

# Konfiguracja portu
ser = serial.Serial('/dev/ttyUSB0', 250000, timeout=0.01)
ser.flushInput()

# Uproszczony cylinder
def create_cylinder(resolution=15):
    theta = np.linspace(0, 2*np.pi, resolution)
    z = np.linspace(-1, 1, 2)
    theta_grid, z_grid = np.meshgrid(theta, z)
    x = 0.5 * np.cos(theta_grid)
    y = 0.5 * np.sin(theta_grid)
    return x, y, z_grid

# Inicjalizacja wykresu
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
#ax.set_axis_off()  # Wyłącz osie dla lepszej wydajności

x, y, z = create_cylinder()
cylinder = ax.plot_surface(x, y, z, color='cyan', alpha=0.8)

last_update = time.time()
fps = 0

def update(frame):
    global last_update, fps
    
    # Odczyt danych
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line.count(',') == 1:
                roll, pitch = map(float, line.split(','))
                
                # Obliczanie rotacji (zoptymalizowane)
                roll_rad = np.radians(roll)
                pitch_rad = np.radians(pitch)
                
                cos_r, sin_r = np.cos(roll_rad), np.sin(roll_rad)
                cos_p, sin_p = np.cos(pitch_rad), np.sin(pitch_rad)
                
                # Transformacja współrzędnych
                x_new =  x * cos_p + z * sin_p
                y_new =  y * cos_r - ( -x * sin_p + z * cos_p ) * sin_r
                z_new =  y * sin_r + ( -x * sin_p + z * cos_p ) * cos_r
                
                # Aktualizacja cylindra
                ax.collections.clear()
                ax.plot_surface(x_new, y_new, z_new, color='cyan', alpha=0.8)
                
                # Pomiar FPS
                now = time.time()
                fps = 0.9*fps + 0.1/(now - last_update)
                last_update = now
                ax.set_title(f'MPU6050 Orientation - FPS: {fps:.1f}')
                
        except:
            pass
    
    return cylinder,

ani = FuncAnimation(fig, update, interval=1, blit=False)
plt.tight_layout()
plt.show()
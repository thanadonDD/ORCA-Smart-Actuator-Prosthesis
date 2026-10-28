import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

def get_transform(x, y, z, r, p, yw):
    # แปลงองศาเป็นเรเดียน
    r, p, yw = np.radians([r, p, yw])
    # Rotation Matrices
    Rx = np.array([[1,0,0,0],[0,np.cos(r),-np.sin(r),0],[0,np.sin(r),np.cos(r),0],[0,0,0,1]])
    Ry = np.array([[np.cos(p),0,np.sin(p),0],[0,1,0,0],[-np.sin(p),0,np.cos(p),0],[0,0,0,1]])
    Rz = np.array([[np.cos(yw),-np.sin(yw),0,0],[np.sin(yw),np.cos(yw),0,0],[0,0,1,0],[0,0,0,1]])
    T = np.array([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]])
    return T @ Rz @ Ry @ Rx

def draw_frame(ax, M, label):
    origin = M @ np.array([0, 0, 0, 1])
    colors = ['r', 'g', 'b'] # X, Y, Z
    for i, c in enumerate(colors):
        vec = np.zeros(4); vec[i] = 1.0; vec[3] = 1
        target = M @ vec
        ax.quiver(origin[0], origin[1], origin[2], 
                  target[0]-origin[0], target[1]-origin[1], target[2]-origin[2], 
                  color=c, linewidth=2, alpha=0.8)
    ax.text(origin[0], origin[1], origin[2], label, fontweight='bold')

fig = plt.figure(figsize=(15, 10))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(bottom=0.35)

# ข้อมูล Sliders: (Name, Initial_X, Initial_Color)
frames_data = [
    {'name': 'Frame A', 'init_x': -2, 'color': 'red'},
    {'name': 'Frame B', 'init_x': 0, 'color': 'green'},
    {'name': 'Frame C', 'init_x': 2, 'color': 'blue'}
]

sliders = {}
for i, data in enumerate(frames_data):
    # สร้างพื้นที่ Slider สำหรับแต่ละ Frame (x, y, z, roll, pitch, yaw)
    # วาง Layout ให้เป็น 3 คอลัมน์
    base_x = 0.05 + (i * 0.32)
    sliders[data['name']] = {
        'x': Slider(plt.axes([base_x, 0.25, 0.2, 0.015]), f"{data['name']} X", -5, 5, valinit=data['init_x']),
        'y': Slider(plt.axes([base_x, 0.22, 0.2, 0.015]), f"Y", -5, 5, valinit=0),
        'z': Slider(plt.axes([base_x, 0.19, 0.2, 0.015]), f"Z", -5, 5, valinit=0),
        'r': Slider(plt.axes([base_x, 0.14, 0.2, 0.015]), f"Roll", -180, 180, valinit=0),
        'p': Slider(plt.axes([base_x, 0.11, 0.2, 0.015]), f"Pitch", -180, 180, valinit=0),
        'yw': Slider(plt.axes([base_x, 0.08, 0.2, 0.015]), f"Yaw", -180, 180, valinit=0),
    }

def update(val):
    ax.clear()
    ax.set_xlim([-7, 7]); ax.set_ylim([-7, 7]); ax.set_zlim([-7, 7])
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    
    # วาด World Frame (จุดหมุนนิ่ง)
    draw_frame(ax, np.eye(4), "WORLD")
    
    # วาดแต่ละ Frame จากค่าใน Slider
    for name, s in sliders.items():
        M = get_transform(s['x'].val, s['y'].val, s['z'].val, 
                          s['r'].val, s['p'].val, s['yw'].val)
        draw_frame(ax, M, name)

# ลงทะเบียน Event
for f in sliders.values():
    for s in f.values():
        s.on_changed(update)

update(0)
plt.show()
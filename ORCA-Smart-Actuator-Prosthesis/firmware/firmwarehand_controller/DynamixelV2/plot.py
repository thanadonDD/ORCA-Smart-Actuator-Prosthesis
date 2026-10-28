import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ==========================================
# 1. ข้อมูลดิบ (Original Input - LEFT HAND)
# ==========================================
# ตำแหน่งข้อมือ (Wrist XYZ)
wrist_pos_L = np.array([-0.02576517, 0.00967418, 0.17999464])

# ข้อมูลของแต่ละนิ้ว (Contact = ปลายนิ้ว, Target = ผิววัตถุ)
fingers_L = {
    "Index": {
        "contact": np.array([0.01670554, -0.02331157, 0.08671286]),
        "target":  np.array([0.00285968, 0.00625214, 0.09167317])
    },
    "Middle": {
        "contact": np.array([0.02904432, 0.01963918, 0.10011816]),
        "target":  np.array([-0.00713751, 0.00943947, 0.09743631])
    },
    "Ring": {
        "contact": np.array([0.0178429, 0.0407541, 0.09542907]),
        "target":  np.array([0.0021026, 0.01041877, 0.0903803])
    },
    "Thumb": {
        "contact": np.array([-0.04732369, 0.00978366, 0.09575395]),
        "target":  np.array([-0.00555157, 0.01051588, 0.09462238])
    }
}

# ==========================================
# 2. ฟังก์ชันแปลงเป็นมือขวา (FLIP Y-AXIS)
# ==========================================
def convert_to_right_hand(wrist, fingers_dict):
    # 1. แปลง Wrist (กลับค่า Y)
    wrist_R = wrist.copy()
    wrist_R[1] = -wrist_R[1] 
    
    # 2. แปลงนิ้วแต่ละนิ้ว
    fingers_R = {}
    for name, data in fingers_dict.items():
        fingers_R[name] = {
            # กลับค่า Y ของจุด Contact
            "contact": np.array([data["contact"][0], -data["contact"][1], data["contact"][2]]),
            # กลับค่า Y ของจุด Target
            "target":  np.array([data["target"][0],  -data["target"][1],  data["target"][2]])
        }
    return wrist_R, fingers_R

# --- 🔥 เรียกใช้งานฟังก์ชันแปลงตรงนี้ 🔥 ---
wrist_pos, fingers = convert_to_right_hand(wrist_pos_L, fingers_L)

print("✅ Converted to Right Hand Coordinates (Y-Axis Flipped)")

# ==========================================
# 3. การสร้างกราฟ 3D (Plotting RIGHT HAND)
# ==========================================
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 3.1 พล็อตตำแหน่งข้อมือ
ax.scatter(wrist_pos[0], wrist_pos[1], wrist_pos[2], c='black', s=150, marker='s', label='Wrist (ข้อมือ)')

# 3.2 วนลูปพล็อตแต่ละนิ้ว
colors = ['blue', 'green', 'purple', 'cyan'] # สีเส้นเชื่อมข้อมือ
i = 0

for name, data in fingers.items():
    c = data["contact"]
    t = data["target"]
    
    # พล็อตจุดสัมผัส (Contact Point) - สีแดง
    ax.scatter(c[0], c[1], c[2], c='red', s=60, marker='o', label='Contact (ปลายนิ้ว)' if i==0 else "")
    
    # พล็อตเป้าหมาย (Target Point) - สีเทา/กากบาท
    ax.scatter(t[0], t[1], t[2], c='gray', s=40, marker='x', label='Target (ผิววัตถุ)' if i==0 else "")
    
    # ลากเส้นจาก ข้อมือ -> ปลายนิ้ว (แสดงโครงสร้างมือ)
    ax.plot([wrist_pos[0], c[0]], 
            [wrist_pos[1], c[1]], 
            [wrist_pos[2], c[2]], 
            color=colors[i], alpha=0.5, linewidth=1)
            
    # ลากเส้นจาก ปลายนิ้ว -> เป้าหมาย (แสดงระยะสปริง) - เส้นประสีส้ม
    ax.plot([c[0], t[0]], 
            [c[1], t[1]], 
            [c[2], t[2]], 
            color='orange', linestyle='--', linewidth=2, label='Spring Force' if i==0 else "")

    # ใส่ชื่อนิ้วกำกับ
    ax.text(c[0], c[1], c[2], f"  {name}", fontsize=10)
    
    i += 1

# ==========================================
# 4. การตกแต่งกราฟ (Settings)
# ==========================================
ax.set_xlabel('X Axis (m)')
ax.set_ylabel('Y Axis (m)')
ax.set_zlabel('Z Axis (m)')
ax.set_title('Robotic Hand Grasp Visualization (Right Hand Converted)')

# ปรับสเกลแกนให้เท่ากัน (Equal Aspect Ratio)
all_points = np.vstack([wrist_pos] + [d['contact'] for d in fingers.values()] + [d['target'] for d in fingers.values()])
max_range = np.array([all_points[:,0].max()-all_points[:,0].min(), 
                      all_points[:,1].max()-all_points[:,1].min(), 
                      all_points[:,2].max()-all_points[:,2].min()]).max() / 2.0

mid_x = (all_points[:,0].max()+all_points[:,0].min()) * 0.5
mid_y = (all_points[:,1].max()+all_points[:,1].min()) * 0.5
mid_z = (all_points[:,2].max()+all_points[:,2].min()) * 0.5

ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

ax.legend()
plt.show()
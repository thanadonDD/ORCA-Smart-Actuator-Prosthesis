import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

# ==========================================
# 🛠️ Helper Functions: 4x4 Homogeneous Transforms
# ==========================================
def make_transform(R_mat, t_vec):
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = t_vec
    return T

def rot_z(theta_rad):
    return make_transform(R.from_euler('z', theta_rad, degrees=False).as_matrix(), [0,0,0])

def rot_y(theta_rad):
    return make_transform(R.from_euler('y', theta_rad, degrees=False).as_matrix(), [0,0,0])

def trans(x, y, z):
    return make_transform(np.eye(3), [x,y,z])

# ==========================================
# 🧠 โมดูลที่ 1: พิมพ์เขียวของ "นิ้ว" (The Finger Class)
# ==========================================
class Finger:
    def __init__(self, name, base_pos, link_lengths, base_rot_matrix=None):
        self.name = name
        self.link_lengths = link_lengths
        
        # เก็บเป็น Local Frame สัมพัทธ์กับ T_palm
        self.base_pos_local = np.array(base_pos)
        self.base_rot_local = np.eye(3) if base_rot_matrix is None else np.array(base_rot_matrix)
            
        # สร้าง T_base_local (คงที่เสมอเพราะนิ้วติดกับฝ่ามือ)
        self.T_base_local = make_transform(self.base_rot_local, self.base_pos_local)

    def get_local_target(self, global_target, T_palm):
        # คำนวณ Global Pose ของฐานนิ้ว: T_palm @ T_base_local
        T_base_global = T_palm @ self.T_base_local
        T_inv = np.linalg.inv(T_base_global)

        p_global = np.append(global_target, 1) # ทำเป็น Homogeneous Vector (x,y,z,1)
        p_local = T_inv @ p_global

        return p_local[:3]

    def solve_ik(self, global_target, T_palm):
        local_target = self.get_local_target(global_target, T_palm)
        x, y, z = local_target
        L1, L2, L3 = self.link_lengths

        # ==========================================
        # 🔒 ตะแกรงร่อน Joint Limits (อิงจาก ORCA Hand)
        # ==========================================
        if self.name.lower() == 'thumb':
            lim_yaw = np.radians(70)          # ABD: กาง/หุบ 45°
            lim_p1_min, lim_p1_max = np.radians(-20), np.radians(115) # MCP: แอ่น 20°, งอ 115°
            lim_p2_min, lim_p2_max = np.radians(-20), np.radians(100) # IP:  แอ่น 20°, งอ 100°
        else: # นิ้วชี้ กลาง นาง ก้อย (Fingers 2 to 5)
            lim_yaw = np.radians(30)          # ABD: กาง/หุบ 30°
            lim_p1_min, lim_p1_max = np.radians(-90), np.radians(110) # MCP: แอ่น 20°, งอ 110°
            lim_p2_min, lim_p2_max = np.radians(-90), np.radians(130) # PIP: แอ่น 20°, งอ 130°

        # ==========================================
        # 🧠 เริ่มคำนวณ IK
        # ==========================================
        # 1. Yaw (หมุนซ้ายขวาที่กระดูกท่อนแรก L1)
        theta1 = np.arctan2(y, x)
        
        # 🔥 ล็อคคอแกน Yaw ไม่ให้นิ้วหักออกข้างเกิน Limit!
        theta1 = np.clip(theta1, -lim_yaw, lim_yaw)

        # 2. คลี่ 3 มิติออกมาเป็นระนาบ 2 มิติ
        r_xy = np.sqrt(x**2 + y**2)
        r_prime = r_xy - L1 
        z_prime = z
        D = np.sqrt(r_prime**2 + z_prime**2)

        # 3. คำนวณ Pitch 1 และ 2
        if D > (L2 + L3):
            # กรณีไกลเกินเอื้อม
            theta2 = np.arctan2(z_prime, r_prime)
            theta3 = 0.0
        else:
            cos_t3 = (D**2 - L2**2 - L3**2) / (2 * L2 * L3)
            cos_t3 = np.clip(cos_t3, -1.0, 1.0)
            
            # ใช้ลบหน้าสมการเพื่อให้ข้อต่อโก่งเป็นสะพาน
            theta3 = np.arctan2(-np.sqrt(1 - cos_t3**2), cos_t3)
            
            alpha = np.arctan2(z_prime, r_prime)
            beta = np.arctan2(L3 * np.sin(theta3), L2 + L3 * np.cos(theta3))
            theta2 = alpha - beta

        # 🔥 ล็อคคอแกน Pitch 1 (โคนนิ้ว) และ Pitch 2 (ข้อกลางนิ้ว)
        theta2 = np.clip(theta2, lim_p1_min, lim_p1_max)
        theta3 = np.clip(theta3, lim_p2_min, lim_p2_max)

        self.current_angles = [theta1, theta2, theta3]
        return self.current_angles
    
    def get_fk_points(self, T_palm):
        if not hasattr(self, 'current_angles'):
            self.current_angles = [0.0, 0.0, 0.0]

        t1, t2, t3 = self.current_angles
        L1, L2, L3 = self.link_lengths

        # ✔️ โครงสร้าง FK ที่ถูกต้องตามเส้นสีน้ำตาลที่คุณวาด
        T_base = T_palm @ self.T_base_local
        
        # Joint 1: หมุน Yaw แล้วพุ่งราบไปตาม L1 
        T_p1 = T_base @ rot_z(t1) @ trans(L1, 0, 0)
        
        # Joint 2: ยก Pitch ที่ปลาย L1 แล้วทะยานขึ้นไปตาม L2 
        T_p2 = T_p1 @ rot_y(-t2) @ trans(L2, 0, 0)
        
        # Joint 3: งอ Pitch ที่ปลาย L2 แล้วชี้ลงไปตาม L3 ทิ่มเข้าหาดาวเป้าหมาย
        T_p3 = T_p2 @ rot_y(-t3) @ trans(L3, 0, 0)

        p0 = T_base[:3, 3]
        p1 = T_p1[:3, 3]
        p2 = T_p2[:3, 3]
        p3 = T_p3[:3, 3]

        return [p0, p1, p2, p3]

# ==========================================
# 🧠 โมดูลที่ 2: ประกอบร่างเป็น "มือ" (The RobotHand Class)
# ==========================================
class RobotHand:
    def __init__(self):
        print("🛠️ กำลังสร้างโครงกระดูกมือหุ่นยนต์ (ระบบ Auto-Align)...")
        
        # T_palm ของแท้: หมุนข้อมือตะแคง 90 องศา (ทวนเข็ม)
        self.T_palm = make_transform(R.from_euler('z', 90, degrees=True).as_matrix(), [0,0,0])

        # 🪄 ฟังก์ชันเวทมนตร์: จัดกระดูกให้เชื่อมต่อกับเส้นประอัตโนมัติ
        def auto_align_base(base_pos, finger_name):
            # 1. บังคับให้แกน X ภายในของนิ้ว พุ่งตรงไปตามความยาวของเส้นประ
            vec = np.array(base_pos)
            x_axis = vec / np.linalg.norm(vec)
            
            # 2. กำหนด "ทิศอ้างอิง" เพื่อให้นิ้วรู้ว่าต้องกำมือลงทางไหน
            if finger_name.lower() == 'thumb':
                # นิ้วโป้งจะพับกวาดขวางหน้ามือ (ชี้เฉียงๆ ไปทางแกน +X โลก)
                bend_ref = np.array([1, -0.5, 0]) 
            else:
                # นิ้วชี้-ก้อย จะพับกำลงมาหาฝ่ามือตรงๆ (ชี้ลงแกน -Y โลก)
                bend_ref = np.array([0, -1, 0])   
                
            bend_ref = bend_ref / np.linalg.norm(bend_ref)
            
            # 3. คำนวณแกน Y และ Z ให้ตั้งฉากกันแบบ 3D (Cross Product)
            y_axis = np.cross(bend_ref, x_axis)
            if np.linalg.norm(y_axis) < 1e-6: # ป้องกันบั๊กกรณีแกนบังเอิญขนานกัน
                y_axis = np.array([1, 0, 0])
            y_axis = y_axis / np.linalg.norm(y_axis)
            
            z_axis = np.cross(x_axis, y_axis)
            
            # ประกอบร่างเป็น Rotation Matrix 3x3 
            return np.column_stack((x_axis, y_axis, z_axis))

        # 📌 พิกัดฐานนิ้วของคุณ (ใช้ตัวเลขเดิมเป๊ะ)
        thumb_pos = [-31.97, 13.6, 31.35]
        index_pos = [-26.09, -7.8, 63.49]
        middle_pos= [2, -10.51, 67.93]
        ring_pos  = [19.52, -7.4, 59.15]
        pinky_pos = [40.59, -1.65, 46.39]

        # 🚀 ปักหมุดนิ้วทั้งหมด โดยใช้ฟังก์ชัน Auto-Align จัดกระดูกให้เลย!
        self.thumb = Finger("Thumb", base_pos=thumb_pos, link_lengths=[33.5, 33, 29.67], base_rot_matrix=auto_align_base(thumb_pos, 'thumb'))
        self.index = Finger("Index", base_pos=index_pos, link_lengths=[20, 38, 43.38], base_rot_matrix=auto_align_base(index_pos, 'index'))
        self.middle= Finger("Middle", base_pos=middle_pos, link_lengths=[20, 46, 42.83], base_rot_matrix=auto_align_base(middle_pos, 'middle'))
        self.ring  = Finger("Ring",   base_pos=ring_pos,   link_lengths=[20, 46, 43.24], base_rot_matrix=auto_align_base(ring_pos, 'ring'))
        self.pinky = Finger("Pinky",  base_pos=pinky_pos,  link_lengths=[20, 33, 41.79], base_rot_matrix=auto_align_base(pinky_pos, 'pinky'))

    def grab_object(self, targets_dict):
        print("\n🎯 กำลังเคลื่อนที่นิ้วไปยังเป้าหมายแต่ละจุด...")
        if 'thumb' in targets_dict: self.thumb.solve_ik(targets_dict['thumb'], self.T_palm)
        if 'index' in targets_dict: self.index.solve_ik(targets_dict['index'], self.T_palm)
        if 'middle' in targets_dict: self.middle.solve_ik(targets_dict['middle'], self.T_palm)
        if 'ring' in targets_dict: self.ring.solve_ik(targets_dict['ring'], self.T_palm)
        if 'pinky' in targets_dict: self.pinky.solve_ik(targets_dict['pinky'], self.T_palm)

    def print_servo_angles(self):
        print("\n🤖 === สรุปองศาข้อต่อสำหรับสั่งงาน Servo Motor ===")
        # รวมนิ้วทั้งหมดที่มีในระบบ
        fingers = [self.thumb, self.index, self.middle, self.ring, self.pinky]
        
        for finger in fingers:
            if hasattr(finger, 'current_angles'):
                # แปลงจาก Radian เป็น Degree
                t1, t2, t3 = np.degrees(finger.current_angles)
                
                # จัดฟอร์แมตให้อ่านง่าย
                print(f"👉 [{finger.name:6s}] | "
                      f"Theta1 (กาง/หุบ): {t1:>6.1f}° | "
                      f"Theta2 (โคนนิ้ว): {t2:>6.1f}° | "
                      f"Theta3 (ข้อกลาง): {t3:>6.1f}°")
        print("==================================================")


def rotate_targets_z(targets_dict, angle_degrees):
    print(f"🔄 กำลังหมุนเป้าหมายไป {angle_degrees} องศา ทวนเข็มนาฬิกา...")
    angle_rad = np.radians(angle_degrees)
    rot_matrix = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad), 0],
        [np.sin(angle_rad),  np.cos(angle_rad), 0],
        [0,                  0,                 1]
    ])
    
    rotated_targets = {}
    for name, coords in targets_dict.items():
        rotated_targets[name] = np.dot(rot_matrix, coords)
    return rotated_targets

# ==========================================
# 🎨 โมดูลที่ 3: ระบบสร้างภาพ 3 มิติ (The Visualizer)
# ==========================================
def visualize_robot_hand(hand, targets_dict):
    print("\n📊 กำลังสร้างภาพ 3 มิติ...")
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # ดึงพิกัด T_palm ของจริงมาวาด
    px, py, pz = hand.T_palm[:3, 3]
    ax.scatter(px, py, pz, color='black', s=150, label='Palm Center (Wrist)')
    
    for name, target_xyz in targets_dict.items():
        ax.scatter(target_xyz[0], target_xyz[1], target_xyz[2], color='gold', s=200, marker='*', label=f'{name.capitalize()} Target')
    
    fingers = [hand.thumb, hand.index, hand.middle, hand.ring, hand.pinky]
    colors = ['red', 'blue', 'green', 'purple', 'orange']
    
    for finger, color in zip(fingers, colors):
        # 🔑 หาตำแหน่งฐานนิ้วใน Global Frame
        T_base_global = hand.T_palm @ finger.T_base_local
        bx, by, bz = T_base_global[:3, 3]
        
        # ลากเส้นจาก Palm Center -> ข้อมือนิ้ว (Base)
        ax.plot([px, bx], [py, by], [pz, bz], color='gray', linestyle='--', alpha=0.6)
        
        # ดึง FK มาวาด (โยน T_palm ไปให้มัน)
        fk_points = finger.get_fk_points(hand.T_palm)
        
        xs = [p[0] for p in fk_points]
        ys = [p[1] for p in fk_points]
        zs = [p[2] for p in fk_points]
        
        ax.plot(xs, ys, zs, color=color, linewidth=5, marker='o', markersize=7, label=f'{finger.name}')
        
        finger_key = finger.name.lower()
        if finger_key in targets_dict:
            tx, ty, tz = targets_dict[finger_key]
            ax.plot([xs[-1], tx], [ys[-1], ty], [zs[-1], tz], color=color, linestyle=':', alpha=0.5)

    ax.set_xlabel('X Axis (mm)')
    ax.set_ylabel('Y Axis (mm)')
    ax.set_zlabel('Z Axis (mm)')
    ax.set_title('Robot Hand - Homogeneous Transforms Framework')
    
    ax.legend(loc='center left', bbox_to_anchor=(1.05, 0.5), fontsize='small')
    ax.set_xlim([-70, 70]) 
    ax.set_ylim([-50, 70]) 
    ax.set_zlim([0, 150])  
    plt.tight_layout()
    plt.show()

# ==========================================
# 🚀 ระบบสั่งการ (Execution block)
# ==========================================
my_robot_hand = RobotHand()

factor = 1.6 #1.2
f_So = 0.02  #0.05
# ขวดน้ำ
algorithm_targets = {                 
    'index':  np.array([ 0.01670554-f_So, -0.02331157,  0.08671286]) * 1000 * factor,
    'middle': np.array([ 0.02904432-f_So,  0.01963918,  0.10011816]) * 1000 * factor,
    'ring':   np.array([ 0.0178429-f_So ,  0.0407541,  0.09542907]) * 1000 * factor,
    'thumb':  np.array([-0.04732369,  0.00978366,  0.09575395]) * 1000 
}

# กล้วย
# algorithm_targets = {
#     'index':  np.array([ 0.01646557 - f_So, -0.02965779,  0.09378032]) * 1000 * factor,
#     'middle': np.array([ 0.02846135 - f_So,  0.01502560,  0.10755376]) * 1000 * factor,
#     'ring':   np.array([ 0.00933816 - f_So,  0.04048842,  0.10845652]) * 1000 * factor,
#     'thumb':  np.array([-0.06452259,         0.00348648,  0.10520257]) * 1000   # นิ้วโป้งผมใส่ factor ให้ด้วยเผื่อคุณอยากสเกลพร้อมกันหมด
# }

# แก้วเต่าบิน ยังใช้ไม่ได้
# algorithm_targets = {
#     'index':  np.array([ 0.01541344 - f_So, -0.02770524,  0.07130268]) * 1000 * factor,
#     'middle': np.array([ 0.03433154 - f_So, -0.00293812,  0.08593015]) * 1000 * factor,
#     'ring':   np.array([ 0.01781725 - f_So,  0.03586029,  0.07660327]) * 1000 * factor,
#     'thumb':  np.array([-0.04749663,         0.00886341,  0.07745987]) * 1000   # นิ้วโป้งรวม factor ให้สเกลพร้อมกัน
# }

# เหรียญ
# algorithm_targets = {
#     'index':  np.array([ 0.04786394 - f_So, -0.03774016,  0.01161203]) * 1000 * factor,
#     'middle': np.array([ 0.01640301 - f_So,  0.00188233,  0.01395212]) * 1000 * factor,
#     'ring':   np.array([ 0.00852209 - f_So,  0.03501816,  0.01195771]) * 1000 * factor,
#     'thumb':  np.array([-0.06525861,        -0.00239154,  0.01025603]) * 1000 * factor
# }

angle_to_rotate = 0
rotated_targets = rotate_targets_z(algorithm_targets, angle_to_rotate)

my_robot_hand.grab_object(rotated_targets)
my_robot_hand.print_servo_angles()

visualize_robot_hand(my_robot_hand, rotated_targets)


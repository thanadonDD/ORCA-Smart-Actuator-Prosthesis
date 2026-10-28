import numpy as np
from scipy.spatial.transform import Rotation as R

# 1. ใส่ตัวเลขที่คุณวัดได้จาก SolidWorks ลงไป (หน่วยเป็นองศา)
roll_x = 10.86   # บิดเข้าฝ่ามือ
pitch_y = 11.55 # แกน Y พับกี่องศา?
yaw_z = 10.89    # แกน Z กางกี่องศา?

# 2. ให้สมองซิลิคอน (scipy) สร้างเมทริกซ์ 3x3 ให้เราอัตโนมัติ!
# 'xyz' หมายถึง ลำดับการหมุนรอบแกน x, y, z
r = R.from_euler('xyz', [roll_x, pitch_y, yaw_z], degrees=True)

# 3. ได้เมทริกซ์ที่รวมการเอียงทั้ง 3 แกนเรียบร้อยแล้ว!
thumb_rotation = r.as_matrix()

print("นี่คือเมทริกซ์ของนิ้วโป้งคุณ:\n", thumb_rotation)


# pinky
# roll_x = 10.84   # บิดเข้าฝ่ามือ
# pitch_y = 11.55 # แกน Y พับกี่องศา?
# yaw_z = 10.89    # แกน Z กางกี่องศา?

# ring
# roll_x = 3.11   # บิดเข้าฝ่ามือ
# pitch_y = 3.07 # แกน Y พับกี่องศา?
# yaw_z = 10.3    # แกน Z กางกี่องศา?
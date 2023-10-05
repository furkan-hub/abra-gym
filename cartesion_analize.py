import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json

# JSON dosyasını oku
with open('cartesian_locs.json', 'r') as file:
    data = json.load(file)

# Boş bir 3D grafik oluştur
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Her bir İHA çifti için verileri işle
for item in data:
    uav_nums = item['uav_num']
    x = item['x']
    y = item['y']
    z = item['z']

    # İHA'nın koordinatlarını 3D uzayda çiz
    ax.scatter(x, y, z, label=f'UAV {uav_nums[0]} - UAV {uav_nums[1]}')

# Eksen etiketleri
ax.set_xlabel('X Koordinatı')
ax.set_ylabel('Y Koordinatı')
ax.set_zlabel('Z Koordinatı')

# Grafik başlığı ve etiketleri
plt.title('İHA Koordinat Farkları 3D Görselleştirme')
plt.legend()

# Grafik gösterimi
plt.show()

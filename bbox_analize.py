import matplotlib.pyplot as plt
import json

# JSON dosyasını oku
with open('bbox.json', 'r') as file:
    data = json.load(file)

# Boş bir grafik oluştur
plt.figure(figsize=(10, 6))

# Her bir bounding box için verileri çiz
for item in data:
    uav = item['uav']
    bbox = item['bbox']
    x, y, width, height = bbox

    # Bounding box çizimi
    plt.gca().add_patch(plt.Rectangle((x, y), width, height, fill=None, color='r', linewidth=2))

    # Bounding box etiketi
    plt.text(x, y, f'UAV {uav}', fontsize=12, color='r', verticalalignment='bottom')

# Eksenleri ayarla
plt.xlim(-1.5, 1.5)
plt.ylim(-1.5, 1.5)
plt.gca().set_aspect('equal', adjustable='box')

# Grafik başlığı ve etiketleri
plt.title('UAV Bounding Box Analizi')
plt.xlabel('X Koordinatı')
plt.ylabel('Y Koordinatı')

# Grafik gösterimi
plt.grid()
plt.show()

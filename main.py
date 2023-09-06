import tkinter as tk
import threading
import time
from dronekit import *
from time import *
import subprocess
import random
import datetime
import json
import math

# Bu tool 2024 yılında abra iha takımının yapay zeka pilotunu eğitmek için oluşturulmuş bir eğitim alanıdır
# Bu script birden fazla ardupilot sitl oluşturarak yapay bir savaşan iha simulasyonu yapmakttadır.
# ABRA UAV  
# Fighter UAV Comptation

#region Variables
sitl = [] #bütün stil için vehicle objelerini tutar plane0,plane1,plane2...
connection_is_ok = False #bağlantının tamamlandığını belirtir
telems = [] #telemetrileri tutar telem0 = sitl0 telemetrisi demektir. stil0 ise plane0 temsil etmektedir.
gps_time = {}
mavproxy_check = False#mavproxy açılsın mı açılmasın mı
counter = 0#mavroxy butonu için değişken

#waypointler
waypoints= [Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 40.22931650,29.00920630,100.000000),
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 40.23301880,29.00596620,100.000000),
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 40.23306790,29.00807980,100.000000),
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 40.23210960,29.00356290,100.000000),
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 40.23217510,29.00605200,100.000000),
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 40.23210140,29.00800470,100.000000),
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 40.23116760,29.00348780,100.000000),
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 40.23122500,29.00604130,100.000000),
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 40.23118400,29.00806900,100.000000)]

#gui için renk tanımlamaları
GRAY = "#383737"
RED = "#FF0000"
LIGHT_GRAY = "#787878"
DAHALIGHT_GRAY = "#828282"
GREEN = "#00ff1a"

#endregion

def loop():# telemetrilerin oluşturuluması ve json dosyasına yazılması
    global telems

    while True:
        #update
        

        if connection_is_ok == True:

            for i in range(len(sitl)):

                #gps zamanı için lisenner mavlink mesajlarını dinliyor(bu listernerlar için connect fonksiyonunda waitready fonksiyonu true olmalıdır)
                @(sitl[i]).on_message('SYSTEM_TIME')
                def listener(self, name, message):
                    global gps_time
                    time_ms = message.time_unix_usec / 1000000  # Unix zamanını saniyeye dönüştürün
            
                    # Unix zamanını bir tarih ve saat nesnesine çevirin
                    dtime = datetime.datetime.fromtimestamp(time_ms)
            
                    # Tarih ve saat bilgisini alın
                    year = dtime.year
                    month = dtime.month
                    day = dtime.day
                    hour = dtime.hour
                    minute = dtime.minute
                    second = dtime.second
                    microsecond = dtime.microsecond
            
                    # Zaman bilgisini bir sözlükte saklayın
                    gps_time = {
                        "saat": hour,
                        "dakika": minute,
                        "saniye": second,
                        "milisaniye": microsecond // 1000  # Mikrosaniyeyi milisaniyeye dönüştürün
                    }

                    #print(gps_time)
                    
                telem = {#telemetri paketlerini oluştur
                    "takim_numarasi": i+1,
                    "iha_enlem": float(sitl[i].location.global_frame.lat),
                    "iha_boylam": float(sitl[i].location.global_frame.lon),
                    "iha_irtifa": float(sitl[i].location.global_frame.alt),
                    "iha_dikilme": float((sitl[i].attitude.pitch)*(180 / math.pi)),
                    "iha_yonelme": float((sitl[i].attitude.yaw)*(180 / math.pi)),
                    "iha_yatis": float((sitl[i].attitude.roll)*(180 / math.pi)),
                    "iha_hiz": float(sitl[i].groundspeed),
                    "iha_batarya": int(sitl[i].battery.level),
                    "iha_otonom": 1,
                    "iha_kilitlenme": 0,
                    "hedef_merkez_X": 0,
                    "hedef_merkez_Y": 0,
                    "hedef_genislik": 0,
                    "hedef_yukseklik": 0,
                    "gps_saati":gps_time,
                }
                telems.append(telem)
               

            #print(str(telems))
            #print(str(list(telems)))
            
            #json_telem = json.dumps(telems)
            with open("telems.json", "w") as dosya:#telemetri paketlerini json olarak kaydet
                json.dump(telems, dosya,indent=4)

            #print(json_telem)
            telems.clear()
            sleep(1)

def btn_start():# simulasyonu başlatma 

    global sitl, connection_is_ok
    #text boxlardan bilgileri al
    sitl_count = sitl_text_box.get(1.0, "end-1c").strip()
    sitl_speed = speed_text_box.get(1.0, "end-1c").strip()
    sitl = []

    #text boxtan aldığı verilere göre konsola sitl başlatmak için gerekli komutları git
    #)-n simulasyon sayısı, -L home possition, --no-mavproxy mavproxy iptal et, --speed up simulasyon hızı
    if not sitl_count.isdigit() and not sitl_speed.isdigit():
        print("Please enter a integer number recomend 1-10")
    else:
        print(sitl_count)
        sitl_count = int(sitl_count)
        print("stil count is "+str(sitl_count)+" simulation speed is "+sitl_speed+" initializing...")
        
        if mavproxy_check == False:
            command = "sim_vehicle.py -v ArduPlane --speedup " +sitl_speed + " -n"+str(sitl_count)+" -L 40.2293165,29.0092063 --no-mavproxy"
        else:
            command = "sim_vehicle.py -v ArduPlane --speedup " +sitl_speed + " -n"+str(sitl_count)+" -L 40.2293165,29.0092063 --console --map"


        subprocess.Popen(command, shell=True, stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, stdin=subprocess.PIPE)
        sleep(5)
        print("ok.")

        #tüm simulasyonlar için dronekit ile bağlantı objesi oluştur
        for i in range(sitl_count):
            if mavproxy_check == False:
                print("plane "+str(i)+" 127.0.0.1:"+str(5760+i*10)+" is connecting...")
                plane = connect("tcp:127.0.0.1:"+str(5760+i*10), wait_ready=False)
            else:
                print("plane "+str(i)+" 127.0.0.1:"+str(5762+i*10)+" is connecting...")
                plane = connect("tcp:127.0.0.1:"+str(5762+i*10), wait_ready=False)
            print("plane "+str(i)+" is connected.")
            sitl.append(plane)
            sleep(2)
        print("Connected all planes... have a god flight !")
        connection_is_ok = True

def btn_close():#henüz bir işe yaramıyor
    print("Closing all connecitons...")

def btn_takeoff():#tüm araçları arm edip belirlü bir irtifaya kaldırır
    if connection_is_ok == True:
        print("Arming for all planes !")
        for i in range(len(sitl)):
            sitl[i].armed = True
            sleep(1)
            sitl[i].mode = ("TAKEOFF")
        print("take-off for all planes !")

    else:
        print("There is no plane to take off.")

def btn_auto():# tüm araçları auto moda alır
    if connection_is_ok == True:
        print("auto mode for all planes !")
        for i in range(len(sitl)):
            sitl[i].mode = ("AUTO")
    else:
        print("There is no plane.")

def btn_fbwa():#tüm araçları fbwa moduna alır
    if connection_is_ok == True:
        print("fbwa mode for all planes !")
        for i in range(len(sitl)):
            sitl[i].mode = ("FBWA")
    else:
        print("There is no plane.")

def btn_choose_auto():#seçilen aracı auto moda geçirir
    
    if connection_is_ok == True:
     
        choosen_sitl = choose_text_box.get(1.0, "end-1c").strip()
        print("fbwa mode for plane: "+str(choosen_sitl))
        sitl[choosen_sitl].mode = ("AUTO")
    else:
        print("There is no plane.")

def btn_choose_fbwa():#seçilen aracı fbwa monuda geçirir

    if connection_is_ok == True:

        choosen_sitl = choose_text_box.get(1.0, "end-1c").strip()
        print("auto mode for plane: "+str(choosen_sitl))
        sitl[choosen_sitl].mode = ("FBWA")
    else:
        print("There is no plane.")

def btn_choose_takeoff():#belirli bir aracı arm edip take off moduna geçirir

    if connection_is_ok == True:

        choosen_sitl = choose_text_box.get(1.0, "end-1c").strip()
        print("Take off for plane: "+str(choosen_sitl))
        sitl[choosen_sitl].mode = ("TAKEOFF")
    else:
        print("There is no plane.")


def mavproxy_check_but():#mavproxy etkinleştirilsin mi
    global mavproxy_check, counter
    counter += 1
    if counter == 1:
        mavproxy_check = True

        print("mavroxy on")

    if counter == 2:
        mavproxy_check = False
        counter = 0
        print("mavroxy off")

def random_waypoints():#tüm araçlara belirli bir alan içindee rastgele yeni görev yazar.
    if connection_is_ok == True:

    
        for i in range(len(sitl)):
            
            selected_items = random.sample(waypoints, 4)#rastgele 4 waypoint seç
            
            sitl[i].commands.clear()
            sitl[i].commands.upload()
    
            sleep(0.5)
    
            sitl[i].commands.add(selected_items[0])
            sitl[i].commands.add(selected_items[1])
            sitl[i].commands.add(selected_items[2])
            sitl[i].commands.add(selected_items[3])
            #sitl[i].commands.add(Command(0, 0, 0,mavutil.mavlink.MAV_CMD_DO_JUMP,0, 0, 0, 0,1,-1,0,0,0,0))
            sitl[i].commands.upload()#seçilen waypointleri yükle
    else:
        print("There is no plane.")

def loc_calc():#tüm araçlar için kamerayı simule eder ve kilitlenme dörtgenlerini oluşturur
    pass

#region gui
# Arayüzü oluştur
root = tk.Tk()
root.geometry("640x480")
root.title("ABRA UAV GYM")

# Pencere arkaplan rengini siyah yap
root.configure(bg="black")

#threading
background_thread = threading.Thread(target=loop)
background_thread.daemon = True

# Thread'i başlat
background_thread.start()

param_label = tk.Label(root, bg=GRAY, width=13, height=15)
param_label.place(x=5, y=5)

#start button
but_start = tk.Button(root, text="Start", command=btn_start,bg="green")
but_start.place(x=120, y=5)

#stop button
but_close = tk.Button(root, text="Close", command=btn_close,bg="red")
but_close.place(x=170, y=5)

#take off button
but_takeoff = tk.Button(root, text="Take off", command=btn_takeoff,bg="yellow")
but_takeoff.place(x=10, y=250)

#waypoint change button
but_random_mission = tk.Button(root, text="waypoints", command=random_waypoints,bg="orange")
but_random_mission.place(x=120, y=50)

#mode auto button
but_auto = tk.Button(root, text="auto", command=btn_auto,bg="yellow")
but_auto.place(x=80, y=250)

#mode FBWA button
but_auto = tk.Button(root, text="fbwa", command=btn_auto,bg="yellow")
but_auto.place(x=140, y=250)

#mode auto button(choosen)
but_auto = tk.Button(root, text="auto", command=btn_auto,bg="yellow")
but_auto.place(x=560, y=40)

#mode FBWA button(choosen)
but_auto = tk.Button(root, text="fbwa", command=btn_auto,bg="yellow")
but_auto.place(x=600, y=40)

#text
sitl_text_box = tk.Text(root,
                         height=1,
                         width=5)
sitl_text_box.place(x=10, y=50)

speed_text_box = tk.Text(root,
                         height=1,
                         width=5)
speed_text_box.place(x=10, y=120)

choose_text_box = tk.Text(root,
                         height=1,
                         width=5)
choose_text_box.place(x=500, y=40)

checkbox = tk.Checkbutton(root, text="Mavproxy",command=mavproxy_check_but)
checkbox.place(x=10, y=200)


stilcount_label = tk.Label(root, text="Sitl Count")
stilcount_label.place(x=10, y=20)

stilspeed_label = tk.Label(root, text="Sitl speed")
stilspeed_label.place(x=10, y=90)

choose_label = tk.Label(root, text="Enter a sitl number (1-n)")
choose_label.place(x=500, y=5)

# Ana döngüyü başlat
root.mainloop()
#endregion
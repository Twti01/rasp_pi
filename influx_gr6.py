########################################## Importiere Abhängigkeiten ############################################################

import influxdb
import datetime
import time
import threading
import RPi.GPIO as GPIO
import dht11
from light_sensor import LightSensor
from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_gps_v3 import BrickletGPSV3
from tinkerforge.brick_imu_v2 import BrickIMUV2
import math
import paho.mqtt.client as mqtt
import string
import random
import json

########################################## Klasse für Filterung von Bewegung-Daten #########################################################

# Kalman-Filter-Klasse
class KalmanFilter:
    def __init__(self, process_noise=1e-5, measurement_noise=1e-2, error_covariance=1.0, initial_estimate=0.0):
        self.q = process_noise  # Prozessrauschen
        self.r = measurement_noise  # Messrauschen
        self.p = error_covariance  # Fehlerkovarianz
        self.x = initial_estimate  # Anfangsschätzung des Zustands
        self.k = 0  # Kalman-Gewinn

    def update(self, measurement):
        # Zeit-Update (Predict)
        self.p = self.p + self.q

        # Mess-Update (Correct)
        self.k = self.p / (self.p + self.r)
        self.x = self.x + self.k * (measurement - self.x)
        self.p = (1 - self.k) * self.p

        return self.x

########################################## RPI-Sensoren einbinden und festlegen #################################################
#Verbindung zum GPS-Brick
HOST = "localhost"
PORT = 4223
UID_GPS = "21LD"  #über BrickViewer finden

# Verbindung für IMU Host und Port bleibt gleich
UID_IMU = "62fgA2"  # UID IMU Brick 2.0

# initialize GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


# Verbindung zu GPS und IMU aufbauen
ipcon = IPConnection()
gps = BrickletGPSV3(UID_GPS, ipcon)
imu = BrickIMUV2(UID_IMU, ipcon)

# read data using pin 4 (BCM)
instance = dht11.DHT11(pin = 4)
########################################## Datenbank erstellen ##################################################################

db = influxdb.InfluxDBClient(host="127.0.0.1", port=8086, username="GR6", password="gruppe6")

print("connection to influxdb done!")

# Überprüfen und Erstellen der Datenbank, falls nicht vorhanden
db_name = "abschluss_pi"
databases = db.get_list_database()
if not any(db['name'] == db_name for db in databases):
    db.create_database(db_name)
    print(f"Database {db_name} created")
else:
    print(f"Database {db_name} already exists")
db.switch_database("abschluss_pi")

print("DB aktiv")

########################################## Messwerte initializieren und anpassen ################################################


# Variablen für GPS und IMU 
latitude = 0.0
longitude = 0.0
acceleration_x = 0.0
acceleration_y = 0.0
acceleration_z = 0.0
angular_velocity_x = 0.0
angular_velocity_y = 0.0
angular_velocity_z = 0.0
gps_lock = threading.Lock()

# Checken, ob Daten für GPS/IMU vorhanden sind
gps_data_valid = False
imu_data_valid = False

# Kalman-Filter auf IMU-Daten anwenden
kf_acc_x = KalmanFilter()
kf_acc_y = KalmanFilter()
kf_acc_z = KalmanFilter()
kf_gyro_x = KalmanFilter()
kf_gyro_y = KalmanFilter()
kf_gyro_z = KalmanFilter()

# Callback Funktion für IMU Daten
# https://www.tinkerforge.com/de/doc/Software/Bricks/IMUV2_Brick_Python.html#imu-v2-brick-python-api
def cb_acceleration(x, y, z):
    global acceleration_x, acceleration_y, acceleration_z, imu_data_valid
    acceleration_x = round(kf_acc_x.update(x / 100.0), 4)  # IMU-Brick Ausgabe m/s² in m/s²
    acceleration_y = round(kf_acc_y.update(y / 100.0), 4)
    acceleration_z = round(kf_acc_z.update(z / 100.0), 4)
    imu_data_valid = True

def cb_angular_velocity(x, y, z):
    global angular_velocity_x, angular_velocity_y, angular_velocity_z, imu_data_valid
    angular_velocity_x = round(kf_gyro_x.update(x / 16), 4)  # konvertiere in °/s
    angular_velocity_y = round(kf_gyro_y.update(y / 16), 4)
    angular_velocity_z = round(kf_gyro_z.update(z / 16), 4)
    imu_data_valid = True

# Callback-Funktion für GPS-Daten
def cb_coordinates(lat, ns, lon, ew):
    global latitude, longitude, gps_data_valid
    with gps_lock:
        latitude = round(lat / 1000000.0, 4)
        longitude = round(lon / 1000000.0, 4)
        gps_data_valid = True

########################################## MQTT-Verbindung aufbauen ##################################################################

random_name = "".join(random.choices(string.ascii_letters + string.digits, k=10))
client = mqtt.Client(client_id=random_name , clean_session=True)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("connection successful")
    elif rc == 1:
        print("connection refused - incorrect protocol version")
    elif rc == 2:
        print("connection refused - invalid client identifier")
    elif rc == 3:
        print("connection refused - server unavailable")
    elif rc == 4:
        print("connection refused - username or/and password incorrect")
    elif rc == 5:
        print("connection refused - not authorised")
    else:
        print(f"connection refused: {rc}")
        
def on_disconnect(client, userdata, rc):
    print(f"disconnected from server with result code: {rc}")

client.on_connect = on_connect 
client.on_disconnect = on_disconnect

def publish_data(topic, data):
    client.publish(topic, json.dumps(data))

client.connect("mqtt-dashboard.com", 1883)
client.loop_start()
########################################### Messwerte erfassen, wenn vorhanden und Anpassung ####################################

def worker(sensor_name, sensor_type, interval):
    while True:
        sensor = LightSensor()
        result = instance.read()
        light = sensor.readLight()
        
        #Lesen bis Werte erhalten werden
        while not result.is_valid() or light is None or light == 0:  
            result = instance.read()
            temp = result.temperature
            humid = result.humidity
            light = sensor.readLight()
        temp = result.temperature
        humid = result.humidity


        #Falls GPS-Daten nicht passen -> Eintragen in DB überspringen
        if not gps_data_valid:
            time.sleep(interval)
            continue

        #Falls IMU-Daten nicht passen -> Eintragen in DB überspringen
        if not imu_data_valid:
            time.sleep(interval)
            continue


        #Berechnung für Gesamtbeschleunigung
        resulting_acceleration = math.sqrt(acceleration_x**2 + acceleration_y**2 + acceleration_z**2)
        #Berechnung für gesamte Winkelgeschwindigkeit
        resulting_angular_velocity = math.sqrt(angular_velocity_x**2 + angular_velocity_y**2 + angular_velocity_z**2)


        with gps_lock:
            lat = latitude
            lon = longitude

        #Werte auf Maximal 4 Dezimalstellen 
        temp = round(temp, 4)
        humid = round(humid, 4)
        light = round(light, 4)
########################################## Datensatz für influx und in mqtt schreiben/publishen #################################### 
        
        #influx
        data = {
            "measurement": "umwelt",
            "time": datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ"),
            "tags": {
                "owner": "Gruppe 6",
                "name": sensor_name,
                "type": sensor_type,
            },
            "fields": {
                "temp": float(temp),
                "humid": float(humid),
                "light": float(light),
            }
        }
        data2 = {
            "measurement": "koor",
            "time": datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ"),
            "tags": {
                "owner": "Gruppe 6",
                "name": sensor_name,
                "type": sensor_type,
            },
            "fields": {
                "lat": float(lat),
                "lon": float(lon)
            }
        }
        data3 = {
            "measurement": "bewegung",
            "time": datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ"),
            "tags": {
                "owner": "Gruppe 6",
                "name": sensor_name,
                "type": sensor_type,
            },
            "fields": {
                "acc_x": float(acceleration_x),
                "acc_y": float(acceleration_y),
                "acc_z": float(acceleration_z),
                "resulting_acceleration": float(resulting_acceleration),
                "gyro_x": float(angular_velocity_x),
                "gyro_y": float(angular_velocity_y),
                "gyro_z": float(angular_velocity_z),
                "resulting_angular_velocity": float(resulting_angular_velocity),
            }
        }
        print(data, data2, data3)
        if not db.write_points([data]):
            print("error write points!")
        if not db.write_points([data2]):
            print("error write points!")
        if not db.write_points([data3]):
            print("error write points!")
    
       #Hochladen nach Datenbankgruppen über MQTT
        client.publish("RPi/GR6/env", json.dumps({"temp": temp, "humid": humid, "light": light}))
        client.publish("RPi/GR6/gps", json.dumps({"latitude": lat, "longitude": lon}))
        client.publish("RPi/GR6/imu", json.dumps({
            "acc_x": acceleration_x,
            "acc_y": acceleration_y,
            "acc_z": acceleration_z,
            "resulting_acceleration": resulting_acceleration,
            "gyro_x": angular_velocity_x,
            "gyro_y": angular_velocity_y,
            "gyro_z": angular_velocity_z,
            "resulting_angular_velocity": resulting_angular_velocity
        }))



        time.sleep(interval)


########################################## Callbacks und Schreiben in influxdb ##################################################

if __name__ == "__main__":
    try:
        ipcon.connect(HOST, PORT)  # Verbinde mit dem Brick Daemon
        time.sleep(2)  # Warte kurz, um sicherzustellen, dass die Verbindung stabil ist

        # Registriere Callback für GPS-Daten
        gps.register_callback(gps.CALLBACK_COORDINATES, cb_coordinates)
        gps.set_coordinates_callback_period(1000)

        # Registriere Callback für IMU-Daten
        imu.register_callback(imu.CALLBACK_ACCELERATION, cb_acceleration)
        imu.register_callback(imu.CALLBACK_ANGULAR_VELOCITY, cb_angular_velocity)
        imu.set_acceleration_period(1000)
        imu.set_angular_velocity_period(1000)

        #Messwerte in DB eintragen -> über worker-Funktion
        threading.Thread(target=worker, args=("Multisensor", "dht11, light, gps3.0, imu2.0", 5)).start()
        print("Lets go")
        while True:
            time.sleep(1)
    except Exception as e:
        print("Fehler:", e)
    finally:
        ipcon.disconnect()
        GPIO.cleanup()

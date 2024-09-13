# -----------------------------------------------
# Import Dependencies 
# -----------------------------------------------

from flask import Flask, jsonify, request, abort
import influxdb, secrets, requests
import functools
import json

# -----------------------------------------------
# initialize Flask and database of influx
# -----------------------------------------------

app = Flask(__name__)

host = "192.168.178.118"
port = 8086
user = "linnart"
password = "linnart"
database = "abschluss_pi"

client = influxdb.InfluxDBClient(host="127.0.0.1", port=port, username=user, password=password)
client.switch_database(database)

with open("api-keys.json", "r") as f:
    key = json.load(f)

# -----------------------------------------------
# decorated function for adding api-key
# -----------------------------------------------

def require_api_key(view_function):
    @functools.wraps(view_function)
    def decorated_function(*args, **kwargs):
        api_key = request.headers.get('X-API-Key')
        if api_key and api_key in key:
            return view_function(*args, **kwargs)
        else:
            abort(401)  # Unauthorized
    return decorated_function

# ----------------------------------------------
# URL Requests
# ----------------------------------------------

#Bewegung
@app.route("/bewegung", methods=['GET'])
@app.route("/bewegung/<q>", methods=['GET'])
@require_api_key
def movement(q=None):
    measures = []
    query = "SELECT * FROM bewegung"
    extract = client.query(query)
    points = extract.get_points(measurement="bewegung")
    if extract is not None:
        for point in points:
            bewegung_content = {"time": point.get('time'),
                             "acc_x": point.get('acc_x'),
                             "acc_y": point.get('acc_y'),
                             "acc_z": point.get('acc_z'),
                             "gyro_x": point.get('gyro_x'),
                             "gyro_y": point.get('gyro_y'),
                             "gyro_z": point.get('gyro_z'),
                             "resulting_acceleration": point.get('resulting_acceleration'),
                             "resulting_angular_velocity": point.get('resulting_angular_velocity')
                             }
            if q == None or q not in bewegung_content:
                measures.append(bewegung_content)

            elif q in bewegung_content:
                measures.append({q: bewegung_content[q]})
            else: 
                continue

    return jsonify(measures), 200

#Location
@app.route("/position", methods=["GET"])
@app.route("/position/<q>", methods=["GET"])
@require_api_key
def position(q=None):
    measures = []
    query = "SELECT * FROM koor"
    extract = client.query(query)
    points = extract.get_points(measurement="koor")
    if extract is not None:
        for point in points:
            position_content = {"lat": point.get('lat'),
                             "lon": point.get('lon')
                             }

            if q == None or q not in position_content:
                measures.append(position_content)

            elif q in position_content:
                measures.append({q: position_content[q]})
            else: 
                continue
    return jsonify(measures), 200

#Enviornment
@app.route("/env", methods=["GET"])
@app.route("/env/<q>", methods=["GET"])
@require_api_key
def env(q=None):
    measures = []
    query = "SELECT * FROM umwelt"
    extract = client.query(query)
    points = extract.get_points(measurement="umwelt")
    if extract is not None:
        for point in points:
            env_content = {"humid": point.get('humid'),
                             "temp": point.get('temp'),
                             "light": point.get('light')
                             }
            if q == None or q not in env_content:
                measures.append(env_content)

            elif q in env_content:
                measures.append({q: env_content[q]})
            else: 
                continue
    return jsonify(measures), 200

#start server 
if __name__ == '__main__':
    app.run(host=host, port=8000, debug=True)

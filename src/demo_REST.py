#demo_REST.py

import requests

url = 'http://127.0.0.1:8080/~/server'

headers = {'X-M2M-Origin': 'admin:admin', 'Content-type': 'application/json;ty=2'}

data = {
 "m2m: ae": {
 "api": "app-sensor",
 "rr": "faux",
 "lbl": ["Type / capteur", "Catégorie / température", "Lieu / domicile"],
 "rn": "MY_SENSOR"
 }
}

response = requests.post(url, data=json.dumps(data), headers=headers)

from http.server import BaseHTTPRequestHandler, HTTPServer
import urllib.parse
import json
import paho.mqtt.client as mqtt

# MQTT ASETUKSET
MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
MQTT_TOPIC = "kyykkaiot"

mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)


# HTTP PALVELIN
class MyHandler(BaseHTTPRequestHandler):

    def do_POST(self):
        if self.path != "/api/endpoint":
            self.send_error(404)
            return

        length = int(self.headers.get('Content-Length', 0))

        raw = self.rfile.read(length).decode()

        parsed = dict(urllib.parse.parse_qsl(raw))

        mapped = {}

        if "p" in parsed:
            mapped["hr"] = parsed["p"]

        if "a" in parsed:
            if float(parsed["a"]) > 95:
                print("Lähetä kiihtyvyys",parsed["a"])
                mapped["acceleration"] = float(parsed["a"]) / 100.0
            else:
                print("Älä lähetä kiihtyvyys, arvo liian pieni:", parsed["a"])

        if "v" in parsed:
            mapped["alcohol"] = parsed["v"]

        print("Muunnettu JSON-data:", mapped)


        # LÄHETÄ MQTT (JSON)
        mqtt_payload = json.dumps(mapped)
        mqtt_client.publish(MQTT_TOPIC, mqtt_payload)
        print("Lähetetty MQTT:ään:", mqtt_payload)

        print("-------------------------")

        # Vastataan Arduinolle
        self.send_response(200)
        self.send_header("Content-type", "text/plain")
        self.end_headers()
        self.wfile.write(b"OK")


# KÄYNNISTÄ PALVELIN
def run():
    server = HTTPServer(("0.0.0.0", 5000), MyHandler)
    print("HTTP + MQTT gateway käynnissä portissa 5000")
    server.serve_forever()


if __name__ == "__main__":
    run()
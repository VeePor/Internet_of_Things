import random
from paho.mqtt import client as mqtt_client

MQTT_BROKER = 'broker.hivemq.com'
MQTT_PORT = 1883
MQTT_TOPIC = "kyykkaiot"
client_id = f'python-kyykka-{random.randint(0, 1000)}'

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Yhdistetty MQTT-brokeriin!")
        client.subscribe(MQTT_TOPIC)
        print(f"Tilattu MQTT-topic: {MQTT_TOPIC}")
    else:
        print("Yhdistämisessä MQTT-brokeriin tapahtui virhe, koodi:", rc)

def on_message(client, userdata, msg):
    print(f"Vastaanotettu MQTT-viesti: {msg.topic} -> {msg.payload.decode()}")

client = mqtt_client.Client(
    client_id = client_id,
    callback_api_version=mqtt_client.CallbackAPIVersion.VERSION1
)

client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_BROKER, MQTT_PORT)
client.loop_forever()
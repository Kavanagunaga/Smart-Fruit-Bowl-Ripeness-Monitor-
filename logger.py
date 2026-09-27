import csv
import json
import os
import paho.mqtt.client as mqtt

MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883

TOPICS = [
    ("bme680/data", 1),
    ("bme680/decision", 1),
]

DATA_CSV = "bme680_data.csv"
DECISION_CSV = "bme680_decision_v3.csv"

data_file = open(DATA_CSV, "a", newline="", encoding="utf-8")
decision_file = open(DECISION_CSV, "a", newline="", encoding="utf-8")

data_writer = csv.writer(data_file)
decision_writer = csv.writer(decision_file)

if os.path.getsize(DATA_CSV) == 0:
    data_writer.writerow([
        "timestamp",
        "ist",
        "temp",
        "hum",
        "gas"
    ])
    data_file.flush()

if os.path.getsize(DECISION_CSV) == 0:
    decision_writer.writerow([
    "timestamp",
    "ist",
    "samples",
    "gas_drop_pct",
    "hum_rise",
    "condition"
    ])
    decision_file.flush()


def on_connect(client, userdata, flags, rc):
    print(f"Connected to broker, rc={rc}")
    client.subscribe(TOPICS)
    print("Subscribed to:")
    for topic, qos in TOPICS:
        print(f"  - {topic} (qos={qos})")


def on_message(client, userdata, msg):
    try:
        print("RECEIVED:", msg.topic)
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)

        print(f"\nTopic: {msg.topic}")
        print(data)

        if msg.topic == "bme680/data":
            data_writer.writerow([
                data.get("timestamp"),
                data.get("ist"),
                data.get("temp"),
                data.get("hum"),
                data.get("gas")
            ])
            data_file.flush()

        elif msg.topic == "bme680/decision":
            decision_writer.writerow([
                data.get("timestamp"),
                data.get("ist"),
                data.get("samples"),
                data.get("gas_drop_pct"),
                data.get("hum_rise"),
                data.get("condition")
            ])
            decision_file.flush()

        else:
            print(f"Unhandled topic: {msg.topic}")

    except json.JSONDecodeError:
        print("Invalid JSON received:")
        print(msg.payload.decode("utf-8", errors="ignore"))
    except Exception as e:
        print(f"Error processing message from {msg.topic}: {e}")


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)

client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_BROKER, MQTT_PORT, 60)

try:
    client.loop_forever()
except KeyboardInterrupt:
    print("\nStopping subscriber...")
finally:
    data_file.close()
    decision_file.close()
    client.disconnect()
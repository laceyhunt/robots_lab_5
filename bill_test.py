import mqtt as mqtt_module
import time

client = mqtt_module.connect_mqtt()
mqtt_module.subscribe(client, mqtt_module.cluster_topic)
client.loop_start()
mqtt_module.message_recieved = False
while not mqtt_module.message_recieved:
    time.sleep(0.2)

parsed_msg = mqtt_module.message_i_got
cluster = parsed_msg["cluster_flag"]
if cluster != 'none':
    print("Cluster detected, processing...")
    coordinates = parsed_msg["coordinate"]
    print(f"Coordinates received: {coordinates}")    
mqtt_module.publish(client, mqtt_module.package_json([0,0,0,0], "none"), mqtt_module.cluster_topic)
mqtt_module.subscribe(client, mqtt_module.coord_topic)

import mqtt
"""  
   USAGE:
   
   
   # Initialize client
   client = connect_mqtt()
   client.loop_start()

   # Publish position
   msg=package_json(rand_pos, True)
   publish(client, msg)
   
   # Wait for DJ to go to and send new handoff coordinates
   message_recieved = False
   subscribe(client)
   while not message_recieved:
      time.sleep(0.2)

   # Disconnect MQTT
   client.loop_stop()
   client.disconnect()
"""

def main():
   client = mqtt.connect_mqtt()
   client.loop_start()
   
   coords=[
      [1,2,3,4],
      [0,0,0,0]
   ]
   
   # Publish position
   msg=mqtt.package_json(coords, cluster_flag="bill")
   mqtt.publish(client,msg, mqtt.cluster_topic)
   
   mqtt.message_recieved = False
   mqtt.subscribe(client, mqtt.cluster_topic)
   while not mqtt.message_recieved:
      mqtt.time.sleep(0.2)
   print(mqtt.message_i_got)
      
   # Disconnect MQTT
   client.loop_stop()
   client.disconnect()
   
main()
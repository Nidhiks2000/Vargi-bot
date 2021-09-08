from multiprocessing.dummy import Pool
import time
import requests

import sys
import paho.mqtt.client as mqtt    #import the client1
import time

class print_colour:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


# -----------------  MQTT SUB -------------------
def iot_func_callback_sub(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

def mqtt_subscribe_thread_start(arg_callback_func, arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_qos):
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1) # wait
        # mqtt_client.loop_forever() # starts a blocking infinite loop
        mqtt_client.loop_start()    # starts a new thread
        return 0
    except:
        return -1


# -----------------  MQTT PUB -------------------
def mqtt_publish(arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos):
    try:        
        mqtt_client = mqtt.Client("mqtt_pub")
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.loop_start()

        print("Publishing message to topic", arg_mqtt_topic)
        mqtt_client.publish(arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos)
        time.sleep(0.1) # wait
        mqtt_client.loop_stop() #stop the loop
        return 0
    except:
        return -1
#----------------PUSHING TO GOOGLE SHEETS--------------------------

        #variadic function to push data to google sheets
#function to push to google sheets
def push_to_google_sheets(*args):
    

    
 

    parameters ={"id":"Sheet1","TeamID":"VB_1299","UniqueID":"MnOpqRsT","SKU":args[0][0],"Item":args[0][1],"Priority": args[0][2],"Storage Number":args[0][3],"Cost": args[0][4],"Quantity":args[0][5]} 


    URL = "https://script.google.com/macros/s/AKfycbyuc519rp6UbeLF7lE8wyPT32MlU6zkllLapdqMhzBEohqH33u_RLA/exec"
    response = requests.get(URL, params=parameters)

    print(response.content)

def push_to_google_sheets_incoming_order(*args):

    parameters = {"id":"Sheet1","TeamID":"VB_1299","UniqueID":"MnOpqRsT","OrderID":args[0][0],"Order date and time":args[0][1],"city": args[0][2],"item":args[0][3],"priority": args[0][4],"Orderquantity":args[0][5],"latitude": args[0][6] ,"Longitude": args[0][7] ,"cost":args[0][8]} 

    URL = "https://script.google.com/macros/s/AKfycbyuc519rp6UbeLF7lE8wyPT32MlU6zkllLapdqMhzBEohqH33u_RLA/exec"

    print(response.content)



#!/usr/bin/python
import time
import RPi.GPIO as GPIO
import time
import Adafruit_CharLCD as LCD
import Adafruit_DHT
import os,sys
import picamera
from urllib.parse import urlparse
import paho.mqtt.client as paho
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

'''
define pin for lcd
'''
# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005
delay = 1



# Define GPIO pins
led_pin = 13                                    #Led pin connected to GPIO 13
buzzer_pin = 16                                 #Buzzer pin connected to GPIO 16
motion_sensor_pin= 20                           #PIR sensor pin connected to GPIO 20
motor_pin1=21                                   #First output pin of motor connected to GPIO 21
motor_pin2=4                                    #Second output pin of motor connected to GPIO 4

#Define GPIO to LCD mapping
lcd1 =12
lcd2 =7
lcd3 =8
lcd4 =25
lcd5 =24
lcd6 =23


lcd = LCD.Adafruit_CharLCD(lcd1,lcd2,lcd3,lcd4,lcd5,lcd6,0,16,2) #initializing LCD


#defining pins as output or input
GPIO.setup(led_pin, GPIO.OUT)                     #setting led_pin as output pin
GPIO.setup(buzzer_pin, GPIO.OUT)                  #setting buzzer_pin as output pin
GPIO.setup(motion_sensor_pin, GPIO.IN)            #setting motion_sensor_pin as output pin
GPIO.setup(motor_pin1, GPIO.OUT)                  #setting motor_pin1 as output pin
GPIO.setup(motor_pin2, GPIO.OUT)                  #setting motor_pin2 as output pin
P = GPIO.PWM(led_pin,100)                         #Defining PWM


initial = time.time();

while True:                                       # in while loop,PIR sensor continously sensing 
    val=GPIO.input(motion_sensor_pin)
    print(val)
    if val==1:
        GPIO.output(buzzer_pin,GPIO.LOW)          #if there is no object in front of PIR sensor(PIR sensor inputout value= 1) , the buzzer will be off)
    else:
        GPIO.output(buzzer_pin,GPIO.HIGH)         #if there is an object is sensed by  PIR sensor(PIR sensor input value= 0) , the buzzer will be On)
    final = time.time();
    if ((int)(final - initial) == 8):             #Exiting the loop after 10 seconds
        break
    
def on_connect(self, mosq, obj, rc):              #connecting to mqtt client
    self.subscribe("led", 0)
        
def on_message(mosq, obj, msg):                   #this function is called when a button is pressed on mqtt dashboard
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
    if(msg.payload == b"led_ON"):                 #if the button pressed is Light On , mqtt client will send the payload "led_ON"
        lcd.clear()                               #clearing the screen of LCD
        lcd.message("LED ON")                     #printing message on LCD "LED ON"
        for x in range (100):                     #defining x in range of 100 for pwm (x =(0 to 100))
            P.start(x)                            #triggering PWM 
            time.sleep(0.1)                       #sleep for 0.1 seconds
             
    elif(msg.payload== b"led_OFF"):               #if the button pressed is Light Off , mqtt client will send the payload "led_OFF"
        lcd.clear()                               #clearing the screen of LCD
        lcd.message("LED OFF")                    #printing message on LCD "LED OFF"
            for x in range (100):                 #defining x in range of 100 for pwm (x =(100 to 0))
              P.start(100-x)                      #triggering PWM
              time.sleep(0.1)                     #sleep for 0.1 seconds
        
        
    elif(msg.payload== b"motor_OFF"):             #if the button pressed is Fan Off , mqtt client will send the payload "motor_OFF"
        lcd.clear()                               #clearing the screen of LCD
        lcd.message("MOTOR OFF")                  #printing message on LCD "MOTOR OFF"
        GPIO.output(motor_pin1,GPIO.LOW)          # making both output pins of motor low, to stop the motor 
        GPIO.output(motor_pin2,GPIO.LOW)    

    elif(msg.payload== b"temp_sensing"):          #if the button pressed is Check temprature , mqtt client will send the payload "temp_sensing"
        humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT11,2)  #checking the temperature and humidity with Adafruit DHT library function
        lcd.clear()                               #clearing the screen of LCD
        lcd.message("Temp: "+str(temperature) + "C\nHumidity: " +str(humidity))    #printing temrature and humidity on LCD
        time.sleep(8)                             #sleep for 8 seconds
        if(temperature >= 20):                    #if temprature is greater then 20 degree celcius , then turn on the motor
            lcd.clear()                           #clearing the screen of LCD
            lcd.message("MOTOR ON")               #printing message on LCD "MOTOR ON"
            GPIO.output(motor_pin1,GPIO.HIGH)     #making one outpin as high and other output pin of motor as low to turn on the motor 
            GPIO.output(motor_pin2,GPIO.LOW)
         
    elif(msg.payload== b"camera_start"):          #if the button pressed is camera On , mqtt client will send the payload "camera_start"
        lcd.clear()                               #clearing the screen of LCD
        lcd.message("Camera Sensing")             #printing message on LCD "Camera Sensing"
        with picamera.PiCamera() as camera:       #using picamera from PICamera library to access the camera
            camera.start_preview()                #triggering the camera
            time.sleep(10)                        #sleep for 10 seconds
            camera.stop_preview()                 #stop the camera
            lcd.clear()                           #clearing the screen of LCD
            lcd.message("Camera Off")             #printing message on LCD "Camera off"
            
    elif(msg.payload== b"camera_off"):            #if the button pressed is camera Off , mqtt client will send the payload "camera_off"
        lcd.clear()                               #clearing the screen of LCD
        lcd.message("Camera off")                 #printing message on LCD "Camera off"
        with picamera.PiCamera() as camera:       #using picamera from PICamera library to access the camera
            camera.stop_preview()                 #stop the camera
        
        
def on_publish(mosq, obj, mid):                   #settings got mqttc client
        print("mid: " + str(mid))

        
def on_subscribe(mosq, obj, mid, granted_qos):    #settings got mqttc client
        print("Subscribed: " + str(mid) + " " + str(granted_qos))

mqttc = paho.Client()                             # mqttc object declaration using paho library 
    # Assign event callbacks
mqttc.on_message = on_message                     # called as callback
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe

url_str = os.environ.get('CLOUDMQTT_URL', 'tcp://broker.emqx.io:1883')     #defining  url to connect to mqttc dashboard
url = urlparse(url_str)
mqttc.connect(url.hostname, url.port)                                      #connecting to mqttc client



lcd.message("WELCOME")                             #displaying "welcome" on LCD screen
time.sleep(0.5)                                    #sleep for 0.5 seconds
delay = 5

while 1:
      # Print out results
    rc = mqttc.loop()                              #continously lisitnging to mqttc client
    time.sleep(0.5)
   
  
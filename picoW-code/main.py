from picozero import pico_led
import time
from retrieve_data import retrieve
from machine import Pin
from hx711 import HX711

print("Starting...")
time.sleep(0.2)

DT1_PIN = Pin(15, Pin.IN)   # DOUT
SCK1_PIN = Pin(14, Pin.OUT) # SCK
DT2_PIN = Pin(0, Pin.IN)   # DOUT
SCK2_PIN = Pin(1, Pin.OUT) # SCK
ZERO_POINT = -25.0
BUFFER = 30.0
TIMES_TO_RUN = 5

def test_connection():
    try:
        timer = 0.2
        for i in range(10):
            pico_led.on()
            time.sleep(timer)
            pico_led.off()
            time.sleep(timer)
            timer -= 0.02
        
        print("Connection successful")
        return 1
    
    except Exception as e:
        print(f"Connection failed: {e}")
        return 0

print("Running main.py")
connection = test_connection()

if connection:
    # for debugging
#     for i in range(TIMES_TO_RUN):
#         retrieve(DT_PIN, SCK_PIN, BUFFER)
    while True:
        retrieve(DT1_PIN, SCK1_PIN, DT2_PIN, SCK2_PIN, BUFFER)
  

from machine import Pin
import time
from hx711 import HX711
from picozero import pico_led

# -----------------------------
# Setup HX711 pins
# -----------------------------
DT_PIN = Pin(15, Pin.IN)   # DOUT
SCK_PIN = Pin(14, Pin.OUT) # SCK
FORWARD_DIR_TEST = "1"
ZERO_POINT = -35.0
BUFFER = 10.0

def detect_move(weight):
    print(weight)
    if weight < (ZERO_POINT - BUFFER):
#         return "-1"
        return "[MOVE_LEFT]"
    
    elif weight > (ZERO_POINT + BUFFER):
#         return "1"
        return "[MOVE_RIGHT]"
    
    else:
        return ""
#         return "0"

def retrieve(dt1_pin, sck1_pin, dt2_pin, sck2_pin, buffer):
    
    hxLR = HX711(sck1_pin, dt1_pin)
    hxLR.set_scale(1000)   # initial scale factor (calibrate later)
#     hxLR.tare()            # tare the scale
    
#     hxFB = HX711(sck2_pin, dt2_pin)
#     hxFB.set_scale(1000)   # initial scale factor (calibrate later)
#     hxFB.tare()            # tare the scale

#   print("Tare done. Start weighing...\n")

    # -----------------------------
    # Main loop
    # -----------------------------
    
    pico_led.on()
    try:
        weightLR = hxLR.get_units()
#             weightFB = hxFB.get_units()
        
#             print("LR Weight: {:.2f} g".format(weightLR))
#             print("FB Weight: {:.2f} g".format(weightFB))
        
        x = detect_move(weightLR)
#             y = detect_move(weightFB, buffer)
        print(x)
#         y = "0"
#         res = x + ',' + y
#         if res != '0,0':
#             print(res)
        

    except Exception as e:
        pico_led.off()
        print(f"Error reading sensor: {e}")


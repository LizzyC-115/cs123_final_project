"""
Used by Raspberry Pi Pico W to read load cell data. This file must be run
manually or automated by main.py on the Pico W.

NOTE: machine and hx711 are libraries in the Raspberry Pi
"""
from machine import Pin, UART
import time
from hx711 import HX711


DT_PIN = Pin(15, Pin.IN)   # DOUT
SCK_PIN = Pin(14, Pin.OUT) # SCK
BUFFER = 5                 # Threshold to send command

def retrieve(dt_pin, sck_pin, buffer):
    hx = HX711(sck_pin, dt_pin)
    hx.set_scale(1000)   # initial scale factor (calibrate later)
    hx.tare()            # tare the scale
    uart = UART(0, baudrate=115200, tx=0, rx=1)
    print("Tare done. Start weighing...\n")
    uart.write("Tare done. Start weighing...\n")

    try: 
        weight = hx.get_units()
        print("Weight: {:.2f} g".format(weight))
        uart.write("Weight: {:.2f} g".format(weight))

        if weight < -buffer:
            print("[TURN LEFT]")
            uart.write("[TURN LEFT]")
        
        elif weight > buffer:
            print("[TURN RIGHT]")
            uart.write("[TURN RIGHT]")
            
        else:
            print("[]")
            uart.write("[]")

    time.sleep(0.2)
            
    except Exception as e:
        print(f"Error reading sensor: {e}")
        uart.write(f"Error reading sensor: {e}")
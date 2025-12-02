"""
Example Raspberry Pi Pico W Controller for Pupper
Upload this to your Pico W using Thonny or similar MicroPython IDE.

This script demonstrates sending commands to Pupper via USB serial.
Customize the command sequences for your use case.
"""

import time

def send_command(command):
    """Send a command to Pupper via USB serial (print to REPL)."""
    print(command)
    time.sleep(0.1)  # Small delay for serial transmission

def demo_basic_movement():
    """Demo: Basic movement commands."""
    print("=== Basic Movement Demo ===")
    send_command("move_forward")
    time.sleep(1)
    send_command("move_forward")
    time.sleep(1)
    send_command("turn_right")
    time.sleep(1)
    send_command("move_forward")
    time.sleep(1)
    send_command("stop")
    time.sleep(2)

def demo_tracking():
    """Demo: Person tracking."""
    print("=== Tracking Demo ===")
    send_command("start_tracking_person")
    time.sleep(10)  # Track for 10 seconds
    send_command("stop")
    time.sleep(2)

def demo_fun_routine():
    """Demo: Fun movement routine."""
    print("=== Fun Routine Demo ===")
    send_command("wiggle")
    time.sleep(6)
    send_command("bark")
    time.sleep(1)
    send_command("bob")
    time.sleep(6)
    send_command("dance")
    time.sleep(13)

def demo_obstacle_navigation():
    """Demo: Navigate around obstacles."""
    print("=== Obstacle Navigation Demo ===")
    # Simple pattern: forward, check, turn, forward
    send_command("move_forward")
    time.sleep(1)
    send_command("move_forward")
    time.sleep(1)
    send_command("turn_left")
    time.sleep(1)
    send_command("move_forward")
    time.sleep(1)
    send_command("turn_right")
    time.sleep(1)
    send_command("move_forward")
    time.sleep(1)
    send_command("stop")

# ===== BUTTON CONTROL EXAMPLE =====
# Uncomment this section if you have buttons wired to GPIO pins

# from machine import Pin
#
# # Setup buttons (adjust pin numbers for your wiring)
# btn_forward = Pin(15, Pin.IN, Pin.PULL_UP)
# btn_backward = Pin(14, Pin.IN, Pin.PULL_UP)
# btn_left = Pin(13, Pin.IN, Pin.PULL_UP)
# btn_right = Pin(12, Pin.IN, Pin.PULL_UP)
# btn_stop = Pin(11, Pin.IN, Pin.PULL_UP)
#
# def button_control_loop():
#     """Control Pupper with buttons."""
#     print("=== Button Control Mode ===")
#     while True:
#         if not btn_forward.value():
#             send_command("move_forward")
#             time.sleep(0.3)
#         elif not btn_backward.value():
#             send_command("move_backward")
#             time.sleep(0.3)
#         elif not btn_left.value():
#             send_command("turn_left")
#             time.sleep(0.3)
#         elif not btn_right.value():
#             send_command("turn_right")
#             time.sleep(0.3)
#         elif not btn_stop.value():
#             send_command("stop")
#             time.sleep(0.3)
#         time.sleep(0.05)

# ===== WIRELESS CONTROL EXAMPLE =====
# Uncomment for WiFi-based remote control

# import network
# import socket
#
# def setup_wifi_ap():
#     """Create WiFi access point for remote control."""
#     ap = network.WLAN(network.AP_IF)
#     ap.active(True)
#     ap.config(essid='PupperControl', password='pupper123')
#     print('WiFi AP started:', ap.ifconfig())
#     return ap
#
# def wifi_control_server():
#     """Simple HTTP server for remote control."""
#     ap = setup_wifi_ap()
#     addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
#     s = socket.socket()
#     s.bind(addr)
#     s.listen(1)
#     print('Listening on', addr)
#
#     while True:
#         cl, addr = s.accept()
#         request = cl.recv(1024).decode()
#
#         # Parse command from HTTP GET request
#         if 'GET /forward' in request:
#             send_command("move_forward")
#         elif 'GET /backward' in request:
#             send_command("move_backward")
#         elif 'GET /left' in request:
#             send_command("turn_left")
#         elif 'GET /right' in request:
#             send_command("turn_right")
#         elif 'GET /stop' in request:
#             send_command("stop")
#
#         # Send response
#         cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
#         cl.send('<h1>Pupper Controller</h1>')
#         cl.send('<a href="/forward">Forward</a><br>')
#         cl.send('<a href="/backward">Backward</a><br>')
#         cl.send('<a href="/left">Left</a><br>')
#         cl.send('<a href="/right">Right</a><br>')
#         cl.send('<a href="/stop">Stop</a>')
#         cl.close()

# ===== MAIN PROGRAM =====

def main():
    """Main program - choose your control method."""

    print("Pupper Pico W Controller Starting...")
    time.sleep(2)

    # Run demo sequences
    # Uncomment the demos you want to run:

    demo_basic_movement()
    time.sleep(3)

    demo_tracking()
    time.sleep(3)

    demo_fun_routine()
    time.sleep(3)

    # demo_obstacle_navigation()
    # time.sleep(3)

    # For button control, uncomment:
    # button_control_loop()

    # For WiFi control, uncomment:
    # wifi_control_server()

    print("Demo complete!")

if __name__ == "__main__":
    main()

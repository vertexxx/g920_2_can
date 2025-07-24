import pygame
import can
import time
import struct
import numpy as np
from scipy.interpolate import interp1d

# Initialize pygame for joystick
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick detected.")
    exit(1)

joystick = pygame.joystick.Joystick(0)
joystick.init()

# Setup CAN FD bus (non-ISO, 500k/2000k baud, FD enabled)
bus = can.interface.Bus(
    interface='vector',
    channel=0,
    fd=True,
    bitrate=500000,
    data_bitrate=2000000,
    app_name='python-can',
    fd_iso=False
)

# --- Settings for stick drift deadzone and curve calculation ---
STICK_DEADZONE = 0.05  # Minimum absolute value to consider movement

CURVE_RADIUS_MIN = 20.0   # Minimum curve radius
CURVE_RADIUS_MAX = 5000.0  # Maximum curve radius

CURVATURE_MIN = 0.0003 # Minimum curvature (1/m) 

def apply_deadzone(val, deadzone=STICK_DEADZONE):
    # Zero out values within the deadzone
    return 0.0 if abs(val) < deadzone else val

def get_stick_values():
    pygame.event.pump()
    # Left stick: axis 0 (X), axis 1 (Y)
    left_x = apply_deadzone(joystick.get_axis(0))
    left_y = apply_deadzone(joystick.get_axis(1))
    # Right stick: axis 2 (X), axis 3 (Y)
    right_x = apply_deadzone(joystick.get_axis(2))
    right_y = apply_deadzone(joystick.get_axis(3))
    return left_x, left_y, right_x, right_y

def scale_axis(val):
    # Scale from -1..1 to 0..255
    return int((val + 1) * 127.5)

def get_trigger_and_buttons():
    pygame.event.pump()
    # Triggers: axis 4 (LT), axis 5 (RT)
    left_trigger = joystick.get_axis(4)
    right_trigger = joystick.get_axis(5)
    # Buttons: 0=A, 1=B, 2=X, 3=Y
    a = joystick.get_button(0)
    b = joystick.get_button(1)
    x = joystick.get_button(2)
    y = joystick.get_button(3)
    return left_trigger, right_trigger, a, b, x, y

def scale_trigger(val):
    # Triggers may range from -1 (released) to 1 (fully pressed)
    return int((val + 1) * 127.5)

curvature_lastvalue = 0.0
def calc_curve_values(left_x, trigger_right, trigger_left):
    # Sum both stick X axes for steering input
    steering = left_x
    damping = (trigger_right+1)/2
    filter = (trigger_left+1)/2
    x = [0,  0.5,   1]  # z.B. Zeitpunkte
    y = [190, 60, 10]  # z.B. Messwerte
    interpolation = interp1d(x, y, kind='linear')
    max_radius = interpolation(damping)
    curve_radius = max_radius * 1/steering
    curvature = 1/curve_radius
    if curve_radius < -1*100000:
        curve_radius = -1*100000
    elif curve_radius > 100000:
        curve_radius = 100000
    #PT1 filter for curvature:
    global curvature_lastvalue
    pt1_min = 1/(3)
    pt1_max = 1/(60)
    alpha = (pt1_max-pt1_min)*filter + pt1_min 

    curvature = alpha * curvature + (1 - alpha) * curvature_lastvalue
    curvature_lastvalue = curvature

    print(f"Steering sum: {steering:.2f} Damping: {damping:.2f} radius: {curve_radius:.0f} alpha: {alpha:.4f}")
    return curve_radius, curvature

while True:
    left_x, left_y, right_x, right_y = get_stick_values()
    # First message: sticks
    data_sticks = [
        scale_axis(left_x),
        scale_axis(left_y),
        scale_axis(right_x),
        scale_axis(right_y)
    ]
    msg_sticks = can.Message(arbitration_id=0x100, data=bytearray(data_sticks), is_extended_id=False)
    try:
        bus.send(msg_sticks)
        #print(f"Sent sticks: {data_sticks}")
    except can.CanError:
        print("Sticks message NOT sent")

    left_trigger, right_trigger, a, b, x, y = get_trigger_and_buttons()
    # Second message: triggers and buttons
    data_trig_btn = [
        scale_trigger(left_trigger),
        scale_trigger(right_trigger),
        a,
        b,
        x,
        y
    ]
    msg_trig_btn = can.Message(arbitration_id=0x101, data=bytearray(data_trig_btn), is_extended_id=False)
    try:
        bus.send(msg_trig_btn)
        #print(f"Sent triggers/buttons: {data_trig_btn}")
    except can.CanError:
        print("Triggers/buttons message NOT sent")

    # Third message: curve radius and curvature (floats, 4 bytes each, little endian)
    curve_radius, curvature = calc_curve_values(left_x, right_trigger, left_trigger)
    # Use little-endian format for two floats (as per DBC: 0|32@1+ and 32|32@1+)
    data_curve = struct.pack('<ff', curve_radius, curvature)
    msg_curve = can.Message(arbitration_id=0x102, data=data_curve, is_extended_id=False)
    try:
        bus.send(msg_curve)
        #print(f"Sent curve: radius={curve_radius:.0f}, curvature={curvature:.4f}")
    except can.CanError:
        print("Curve message NOT sent")

    time.sleep(1/50) #Hz


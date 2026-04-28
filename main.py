from machine import Pin
import time

# Import tests
from tests.Battery_Test_20s import SpeedTest20s
from tests.PID_Attitude_Test import AttitudePIDTest
from tests.Ballast_Test_3_0 import BallastTest30
from tests.Ballast_PID_180_Turn_Test import BallastPID180TurnTest
from tests.mpu_startup_calibration import calibrate_at_startup
# -----------------------
# Hardware Setup
# -----------------------
button = Pin(15, Pin.IN, Pin.PULL_UP)   # Latching switch
led = Pin(14, Pin.OUT)                  # Onboard LED

# -----------------------
# Test Registry
# -----------------------
tests = [
    AttitudePIDTest(),
    BatteryLifeTest(),
    SpeedTest20s(),
    BallastPID180TurnTest()
]

current_index = 0
current_test = None

# -----------------------
# LED Blink Function
# -----------------------
def blink(count, delay=0.2):
    for _ in range(count):
        led.on()
        time.sleep(delay)
        led.off()
        time.sleep(delay)
    time.sleep(0.5)

# -----------------------
# Show Selected Test
# -----------------------
def indicate_test(index):
    # Blink index+1 times
    blink(index + 1)

# -----------------------
# Button State
# -----------------------
def is_pressed():
    return button.value() == 0   # pressed = LOW

# -----------------------
# Main Loop
# -----------------------
print("Running startup MPU calibration...")
startup_cal = calibrate_at_startup()
if startup_cal is None:
    print("Startup MPU calibration failed")
else:
    print("Startup MPU calibration ready")

print("System Ready")

while True:

    # -----------------------
    # IDLE: Select Test
    # -----------------------
    current_test = tests[current_index]
    print("Next Test: ", current_test.name)
    while not is_pressed():
        indicate_test(current_index)

        

    # -----------------------
    # START TEST
    # -----------------------
    
    
    print("Starting:", current_test.name)

    if not current_test.start():
        print("Test failed to start")
        continue
    # Set to next test each cycle
    current_index = (current_index + 1) % len(tests)
    # Solid LED = running
    led.on()

    # -----------------------
    # RUN LOOP (Closed-loop control)
    # -----------------------
    while is_pressed():
        current_test.update()
        time.sleep(0.02)

    # -----------------------
    # ABORT (button released)
    # -----------------------
    print("ABORT triggered")

    current_test.stop()

    led.off()

    # Small delay to prevent bounce issues
    time.sleep(0.5)

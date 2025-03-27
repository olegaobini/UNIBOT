import gpiod
import time
import threading

# GPIO Setup
CHIP = '/dev/gpiochip4'  # Raspberry Pi 5 typically uses gpiochip4 for GPIO pins
PWM_PIN = 18             # GPIO18 (BCM numbering)

# PWM parameters
freq = 100               # Initial frequency (Hz), lower for stable software PWM
duty = 50                # Initial duty cycle (%)

# Open GPIO chip and pin
chip = gpiod.Chip(CHIP)
line = chip.get_line(PWM_PIN)
line.request(consumer='pwm', type=gpiod.LINE_REQ_DIR_OUT)

# PWM thread control
running = True

def pwm_control():
    global freq, duty, running
    while running:
        period = 1.0 / freq
        on_time = period * (duty / 100)
        off_time = period - on_time

        if on_time > 0:
            line.set_value(1)
            time.sleep(on_time)
        if off_time > 0:
            line.set_value(0)
            time.sleep(off_time)

# Start PWM thread
pwm_thread = threading.Thread(target=pwm_control)
pwm_thread.start()

# User input control
print("PWM control started (using gpiod software PWM).")
print("Commands:")
print("  q - increase frequency by 10 Hz")
print("  a - decrease frequency by 10 Hz")
print("  p - increase duty cycle by 5%")
print("  l - decrease duty cycle by 5%")
print("  Ctrl+C - exit")

try:
    while True:
        usr_input = input("Enter command: ")

        if usr_input == 'q':
            freq += 100
            print(f"Frequency increased: {freq} Hz")

        elif usr_input == 'a':
            freq = max(1, freq - 100)
            print(f"Frequency decreased: {freq} Hz")

        elif usr_input == 'p':
            duty = min(100, duty + 5)
            print(f"Duty cycle increased: {duty}%")

        elif usr_input == 'l':
            duty = max(0, duty - 5)
            print(f"Duty cycle decreased: {duty}%")

        else:
            print("Invalid input! Try again.")

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    running = False
    pwm_thread.join()
    line.set_value(0)
    line.release()
    chip.close()


import RPi.GPIO as gpio
import time

btn = 17   # GPIO 0 (Emergency)
led = 12   # GPIO 26 (Y)
led1 = 16  # GPIO 25 (R)

try:
    gpio.setmode(gpio.BCM)
    gpio.setup(btn, gpio.IN)
    gpio.setup(led, gpio.OUT)
    gpio.setup(led1, gpio.OUT)

    while True:
        get_btn = gpio.input(btn)
        if get_btn == gpio.LOW:
            gpio.output(led, gpio.LOW)
            gpio.output(led1, gpio.HIGH)
            print('STOP!')
        else:
            gpio.output(led, gpio.HIGH)
            gpio.output(led1, gpio.LOW)
            print('GO!')

except Exception as e:
    print("Error:", e)
finally:
    gpio.cleanup()


import RPi.GPIO as gpio
import time

ledG = 12   # GPIO 27 (G)
ledY = 26   # GPIO 26 (Y)
ledR = 16  # GPIO 25 (R)

gpio.setmode(gpio.BCM)
#gpio.setup(btn, gpio.IN)
gpio.setup(ledG, gpio.OUT)
gpio.setup(ledR, gpio.OUT)
gpio.setup(ledY, gpio.OUT)
gpio.output(ledG,gpio.HIGH)
gpio.output(ledY,gpio.HIGH)
gpio.output(ledR,gpio.HIGH)

time.sleep(5)

gpio.output(ledG,gpio.HIGH)
gpio.output(ledY,gpio.LOW)
gpio.output(ledR,gpio.LOW)

time.sleep(5)

gpio.output(ledG,gpio.LOW)
gpio.output(ledY,gpio.HIGH)
gpio.output(ledR,gpio.LOW)

time.sleep(5)
gpio.cleanup()
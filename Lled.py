import RPi.GPIO as gpio

led = 26   #GPIO 26(G)
led1 = 12  #GPIO 27(Y)
led2 = 16  #GPIO 25(R)

btn = 17

isstop = False

try:
    gpio.setmode(gpio.BCM)
    gpio.setup(btn, gpio.IN)
    gpio.setup(led, gpio.OUT)
    gpio.setup(led1, gpio.OUT)
    gpio.setup(led2, gpio.OUT)
    gpio.output(led, gpio.HIGH)
    gpio.output(led1, gpio.HIGH)
    gpio.output(led2, gpio.HIGH)
    while True:
        '''get_btn = gpio.input(btn)
        if get_btn == gpio.LOW:
            gpio.output(led1, gpio.LOW)
            gpio.output(led2, gpio.HIGH)
            isstop = True
            print('STOP!')
        else:
            if not isstop:
                gpio.output(led1, gpio.HIGH)
            gpio.output(led2, gpio.LOW)
            print('GO!')'''
        pass

except Exception as e:
    print("Error:", e)
finally:
    gpio.cleanup()

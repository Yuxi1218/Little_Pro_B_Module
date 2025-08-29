#!/usr/bin/env python3
import time
import RPi.GPIO as gpio

led = 26   #GPIO 26(G)
led1 = 12  #GPIO 27(Y)
led2 = 16  #GPIO 25(R)

gpio.setmode(gpio.BCM)
gpio.setup(led,gpio.OUT)
gpio.setup(led1,gpio.OUT)
gpio.setup(led2,gpio.OUT)
try:
    
    while True:        
        gpio.output(led, gpio.HIGH)
        gpio.output(led1, gpio.HIGH) 
        gpio.output(led2, gpio.HIGH)         
        time.sleep(1)
        gpio.output(led, gpio.LOW)
        gpio.output(led1, gpio.LOW)
        gpio.output(led2, gpio.LOW)        
        time.sleep(1)
    
except KeyboardInterrupt:  
    # 當你按下 CTRL+C 中止程式後，所要做的動作
    print("STOP")
  
except:  
    # 其他例外發生的時候，所要做的動作
    print("Other error or exception occurred!" )
  
finally:  
    gpio.cleanup() # 把這段程式碼放在 finally 區域，確保程式中止時能夠執行並清掉GPIO的設定！   

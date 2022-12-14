
import RPi.GPIO as GPIO #GPIOインポート
import time#時間制御インポート

#PIN指定（物理番号指定）
AIN1 = 15
AIN2 = 29
BIN1 = 31
BIN2 = 33

#GPIOのモード
GPIO.setmode(GPIO.BOARD)    #物理ピン番号でGPIOを指定
GPIO.setup(AIN1,GPIO.OUT)   #AIN1番ピンで
GPIO.setup(AIN2,GPIO.OUT)   #AIN2ピンで
GPIO.setup(BIN1,GPIO.OUT)   #BIN1ピンで
GPIO.setup(BIN2,GPIO.OUT)   #BIN2ピンでGPIO出力設定

#周波数設定
Apwm = GPIO.PWM(AIN2,255)     #PWM(pin,Hz)→AIN2番ピンで255Hz
Bpwm = GPIO.PWM(BIN2,255)     #PWM(pin,Hz)→BIN2番ピンで255Hz出力設定

def forward():        #前進
    #右車輪(A1,A2)=(+,0)→正回転
    Apwm.start(0)               #start(duty比)つまり出力0の状態              
    GPIO.output(AIN1,GPIO.HIGH) #AIN1を出力(pin,出力を100％) 
    Apwm.ChangeDutyCycle(0)     #ChangeDutyCycle(duty)→AIN2のduty比０に設定
    #左車輪(BIN1.BIN2)=(+,0)→正回転
    Bpwm.start(0)               
    GPIO.output(BIN1,GPIO.HIGH) #def forward()右車輪と同上
    Bpwm.ChangeDutyCycle(0)     
    
def turn_right():    #右回転
    #右車輪(AIN1,AIN2)=(0,+)→負回転
    Apwm.start(0)                
    GPIO.output(AIN1,GPIO.LOW)  #def forward()右車輪と同上
    Apwm.ChangeDutyCycle(100)   #duty比100％→255Hz（max）出力
    #左車輪(BIN1.BIN2)=(+,0)→正回転
    Bpwm.start(0)                
    GPIO.output(BIN1,GPIO.HIGH) #def forward()右車輪と同上
    Bpwm.ChangeDutyCycle(0)     
    
def turn_left():     #左回転
    #右車輪(AIN1.AIN2)=(+,0)→正回転
    Apwm.start(0)               
    GPIO.output(AIN1,GPIO.HIGH) #def forward()右車輪と同上
    Apwm.ChangeDutyCycle(0)     
    #左車輪(BIN1.BIN2)=(0,+)→負回転
    Bpwm.start(0)               
    GPIO.output(BIN1,GPIO.LOW)  #def forward()右車輪と同上
    Bpwm.ChangeDutyCycle(100)   

def back():          #後進
    #左車輪(AIN1.AIN2)=(0,+)→負回転
    Apwm.start(0)                  #def forward()右車輪と同上
    GPIO.output(AIN1,GPIO.LOW)
    Apwm.ChangeDutyCycle(75)
    #左車輪(BIN1.BIN2)=(+,0)→負回転
    Bpwm.start(0)                  #def forward()右車輪と同上
    GPIO.output(BIN1,GPIO.LOW)
    Bpwm.ChangeDutyCycle(100)
    
def stop():           #停止
    #左車輪(AIN1.AIN2)=(0,0)→停止
    Apwm.start(0)                  #def forward（）右車輪と同上
    GPIO.output(AIN1,GPIO.LOW)
    Apwm.ChangeDutyCycle(0)
    #左車輪(BIN1.BIN2)=(0,0)→停止
    Bpwm.start(0)                  #def forward()右車輪と同上
    GPIO.output(BIN1,GPIO.LOW)
    Bpwm.ChangeDutyCycle(0)
    
    
    
    
def def_test():      #キーボード入力に応じて動作する関数
    while True:
        act = input("動作を入力>>>")
        if act == "f":
            forward()
            time.sleep(2)
            break
        elif act =="l":
            turn_left()
            time.sleep(2)
            Apwm.stop()
            Bpwm.stop()
        elif act =="r":
            turn_right()
            time.sleep(2)
            Apwm.stop()
            Bpwm.stop()
        elif act =="b":
            back()
            time.sleep(2)
            Apwm.stop()
            Bpwm.stop()        
        else:
            print("終了します。")
            GPIO.cleanup()


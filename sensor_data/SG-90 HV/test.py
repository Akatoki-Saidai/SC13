import time
import RPi.GPIO as GPIO
import sys  # 引数取得

# pin number
PIN_servo = 17   # for servo

GPIO.setmode(GPIO.BCM)      # GPIOへアクセスする番号をBCM番号で指定することを宣言
GPIO.setup(PIN_servo, GPIO.OUT)     # 出力ピンに設定

# servo setup
servo = GPIO.PWM(PIN_servo, 50)
servo.start(0)

# サーボの角度を初期化（0degにする、とりあえず）
time.sleep(0.5)
servo.ChangeDutyCycle(7.25)
time.sleep(0.5)

# servo -90deg <= x <= 90deg
# -90deg -> 0.5ms, 90deg -> 2.4ms (from spec. she
now_time=time.time()
# whileで繰り返し処理
    # 標準入力によりサーボの角度を指定
input_num = 5.0


        # サーボを動かす
move_servo_msec = (12-2.5)/180*(int(input_num) + 90) + 2.5 
servo.ChangeDutyCycle(move_servo_msec)
time.sleep(5.0)
    
    

# while文抜けたらサーボストップ
servo.stop()
GPIO.cleanup()

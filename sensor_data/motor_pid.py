import RPi.GPIO as GPIO #GPIOインポート

#pidのプログラム　参考　https://qiita.com/BIG_LARGE_STONE/items/4f8af62b3edc4a03c4a5
#                     https://veresk.hatenablog.com/entry/2019/06/29/192525
#PIN指定
AIN1 = 15
AIN2 = 29
BIN1 = 31
BIN2 = 33

#GPIOのモード
GPIO.setmode(GPIO.BOARD)#物理ピン番号でGPIOを指定
GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(AIN2,GPIO.OUT)
GPIO.setup(BIN1,GPIO.OUT)
GPIO.setup(BIN2,GPIO.OUT)
#周波数設定
a1 = GPIO.PWM(AIN1,255)#255Hz
a2 = GPIO.PWM(AIN2,255)#255Hz
b1 = GPIO.PWM(BIN1,255)
b2 = GPIO.PWM(BIN2,255)
#PWM起動
a1.start(0)#Aenable接続（E）
a2.start(0)#Aphase接続（P）
b1.start(0)
b2.start(0)


def pid_right(): 

    #ここでいう誤差とは (目標値 - 現在の物体の中心)　であり、　入力とはモーターのpwmの値。
    #pid制御についての参考文献　https://controlabo.com/pid-control-introduction/
    #Kx　の値については調節が必要
    M0 = 0.00 #M : 与える操作量(入力)
    M1 =  0.00 #一つ前に与えた操作量(入力)
    goal = 320.00 #目標値 (物体の中心)
    e = 0.00 #偏差(目的値と現在値の差)
    e1 = 0.00 #前回の偏差
    e2 = 0.00 #前々回の偏差
    Kp = 0.1 #誤差が大きいときには入力を大きく、誤差が少ないときには入力を小さくするように入力値を調整  イメージとしては引っ張ったばねが徐々に収束する感じ
    Ki = 0.1 #現在の物体の中心がが目標値を上回るときにはpwmの上昇幅を小さくし、下回るときにはpwmの上昇幅を大きくするように入力を調整  ダンパーがついているイメージ
    Kd = 0.1 #予期していない外部からの影響を和らげる項

    #e(物体の中心の誤差)を使うことで，M(適切なpwmの出力)を出すことが目標

    i = 0

    x_list = []
    y_list = []

    x_list.append(0)
    y_list.append(0.00)

    while(True):

        M1 = M0
        e2 = e1
        e1 = e
        e = goal - y_list[i-1]
        M0 = M1 + Kp * (e-e1) + Ki * e + Kd * ((e-e1) - (e1-e2))
        x_list.append(i)
        y_list.append(M0)
        #print(y_list[i])
        i += 1
        if(0 > M0):
            M0 = 0
        elif(100 < M0):
            M0 = 100

        return y_list(i)


def pid_left():
    #ここでいう誤差とは (目標値 - 現在の物体の中心)　であり、　入力とはモーターのpwmの値。
    #pid制御についての参考文献　https://controlabo.com/pid-control-introduction/
    #Kx　の値については調節が必要
    M0 = 0.00 #M : 与える操作量(入力)
    M1 =  0.00 #一つ前に与えた操作量(入力)
    goal = 320.00 #目標値 
    e = 0.00 #偏差(目的値と現在値の差)
    e1 = 0.00 #前回の偏差
    e2 = 0.00 #前々回の偏差
    Kp = 0.1 #誤差が大きいときには入力を大きく、誤差が少ないときには入力を小さくするように入力値を調整  イメージとしては引っ張ったばねが徐々に収束する感じ
    Ki = 0.1 #現在の物体の中心がが目標値を上回るときにはpwmの上昇幅を小さくし、下回るときにはpwmの上昇幅を大きくするように入力を調整  ダンパーがついているイメージ
    Kd = 0.1 #予期していない外部からの影響を和らげる項
    #e(物体の中心の誤差)を使うことで，M(適切な操作量)を出すことが目標

    i = 0

    x_list = []
    y_list = []

    x_list.append(0)
    y_list.append(0.00)

    while(True):
        M1 = M0
        e2 = e1
        e1 = e
        e = goal - y_list[i-1]
        M0 = M1 + Kp * (e-e1) + Ki * e + Kd * ((e-e1) - (e1-e2))
        x_list.append(i)
        y_list.append(M0)
        #print(y_list[i])

        i += 1
        if(0 > M0):
            M0 = 0
        elif(100 < M0):
            M0 = 100

        return y_list[i]

def stop():
    a1.ChangeDutyCycle(0)
    a2.ChangeDutyCycle(0)
    b1.ChangeDutyCycle(0)
    b2.ChangeDutyCycle(0)


def motor_processing(x):
    if(x != None):
        power_R = (60 + pid_right() - pid_left()) #60は初期値   要調節
        power_L = (60 + pid_left() - pid_right()) #60は初期値   要調節

        a1.ChangeDutyCycle(power_R)
        a2.ChangeDutyCycle(0)
        b1.ChangeDutyCycle(power_L)
        b2.ChangeDutyCycle(0)

    else:#見失ったら回転
        a1.ChangeDutyCycle(70)
        a2.ChangeDutyCycle(0)
        b1.ChangeDutyCycle(0)
        b2.ChangeDutyCycle(0)
            




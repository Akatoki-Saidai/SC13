from Base_UAP import UAP

class Motor(UAP):
    
    def __init__(self):

        #PIN指定
        self.AIN1 = 15
        self.AIN2 = 29
        self.BIN1 = 31
        self.BIN2 = 33

        #GPIOのモード
        #GPIO.setmode(GPIO.BOARD)#物理ピン番号でGPIOを指定
        GPIO.setup(self.AIN1,GPIO.OUT)
        GPIO.setup(self.AIN2,GPIO.OUT)
        GPIO.setup(self.BIN1,GPIO.OUT)
        GPIO.setup(self.BIN2,GPIO.OUT)
        #周波数設定
        self.a1 = GPIO.PWM(self.AIN1,255)#255Hz
        self.a2 = GPIO.PWM(self.AIN2,255)#255Hz
        self.b1 = GPIO.PWM(self.BIN1,255)
        self.b2 = GPIO.PWM(self.BIN2,255)
        #PWM起動
        self.a1.start(0)#Aenable接続（E）
        self.a2.start(0)#Aphase接続（P）
        self.b1.start(0)
        self.b2.start(0)

        self.duty = 50 #duty比　回転速度変更用変数

    def right_forward(self):#E=1 P=0の時
        self.a1.ChangeDutyCycle(self.duty)
        self.a2.ChangeDutyCycle(0)


    def right_back(self):#E=1 P=1の時
        self.a1.ChangeDutyCycle(0)
        self.a2.ChangeDutyCycle(self.duty)

    def right_stop(self):#E=0 P=0?の時
        self.a1.ChangeDutyCycle(0)
        self.a2.ChangeDutyCycle(0)
#----------左モーター関数-----------
    def left_forward(self):
        self.b1.ChangeDutyCycle(self.duty)
        self.b2.ChangeDutyCycle(0)

    def left_back(self):
        self.b1.ChangeDutyCycle(0)
        self.b2.ChangeDutyCycle(self.duty)

    def left_stop(self):
        self.b1.ChangeDutyCycle(0)
        self.b2.ChangeDutyCycle(0)

#-----------動く方向関数---------
    def CCW(self):#前進
        self.right_forward()
        self.left_forward()

    def back(self):#右回転
        self.right_back()
        self.left_forward()

    def forward(self):#左回転
        self.right_forward()
        self.left_back()

    def stop(self):#停止
        self.right_stop()
        self.left_stop()
    
    def CW(self):#後進
        self.right_back()
        self.left_back()
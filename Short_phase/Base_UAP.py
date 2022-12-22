import time
import RPi.GPIO as GPIO

class UAP():

    def __init__(self):

        #------超音波------
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        self.TRIG = 11
        self.ECHO = 13

        #------pid------

        #ここでいう誤差とは (目標値 - 現在の物体の中心)　であり、　入力とはモーターのpwmの値。
        #pid制御についての参考文献　https://controlabo.com/pid-control-introduction/
        #Kx　の値については調節が必要
        self.goal = 320.00 #目標値 (物体の中心)
        self.e = 0.00 #偏差(目的値と現在値の差)
        self.Kp = 0.1 #誤差が大きいときには入力を大きく、誤差が少ないときには入力を小さくするように入力値を調整  イメージとしては引っ張ったばねが徐々に収束する感じ
        self.Ki = 0.1 #現在の物体の中心がが目標値を上回るときにはpwmの上昇幅を小さくし、下回るときにはpwmの上昇幅を大きくするように入力を調整  ダンパーがついているイメージ
        self.Kd = 0.1 #予期していない外部からの影響を和らげる項
        #self.T = 0.01
        #e(物体の中心の誤差)を使うことで，M(適切なpwmの出力)を出すことが目標
        #厳密に項の値を調べないと発散するかも。そのため、値に制限かけるように。


    def reading(self,sensor):
        if sensor == 0:
            GPIO.setup(self.TRIG,GPIO.OUT)
            GPIO.setup(self.ECHO,GPIO.IN)
            GPIO.output(self.TRIG, GPIO.LOW)
            time.sleep(0.3)
         
            GPIO.output(self.TRIG, True)
            time.sleep(0.00001)
            GPIO.output(self.TRIG, False)
 
            while GPIO.input(self.ECHO) == 0:
                signaloff = time.time()
         
            while GPIO.input(self.ECHO) == 1:
                signalon = time.time()
 
                timepassed = signalon - signaloff
                distance = timepassed * 17000
                return distance

            GPIO.cleanup()

        else:
            print ("Incorrect usonic() function varible.")

        
    def pid_right(self,pwm1,e1,e2,x_center): 

        e2 = e1
        e1 = e
        e = self.goal - x_center
        pwm0 = pwm1 + self.Kp * (e-e1) + self.Ki * e - self.Kd * ((e-e1) - (e1-e2))
            

        return pwm0,e,e1


    def pid_left(self,pwm1,e1,e2,x_center):

        e2 = e1
        e1 = e
        e = self.goal - x_center
        pwm0 = pwm1 + self.Kp * (e-e1) + self.Ki * e - self.Kd * ((e-e1) - (e1-e2))

        return pwm0,e,e1
 
#もし上記の式でうまくいかなかったとき用
"""
def pid(self,pwm1,e1,ie,x_center):

    e = self.goal - x_center
    de = (e - e1)/ self.T#偏差/微小な値で傾き
    ie = ie + (e + e1) * self.T/2 #台形近似
    pwm0 = pwm1 + self.Kp * (e-e1) + self.Ki * ie - self.Kd * de

    return pwm0,e
"""

if __name__ == 'main':
    cansat = UAP()
    while True:
        print (cansat.reading(0))
        if(cansat.reading(0) < 30 ):
            break  

import numpy as np
import sys
import cv2
import onnxruntime
from yolox.data.data_augment import preproc as preprocess
from yolox.data.datasets import COCO_CLASSES
from yolox.utils import multiclass_nms, demo_postprocess, vis
import threading
from Base import BaseSR
import RPi.GPIO as GPIO
import time
from multiprocessing import Process,Value
class Short_Range(BaseSR):
    
    def __init__(self):
        
        global e
        global e1
    
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)

        global signaloff
        global signalon
        
        #------超音波------
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        self.TRIG = 11
        self.ECHO = 13
    
        self.x_center = Value("d",320)
        self.x_before = 0
        #------pid------

        #ここでいう誤差とは (目標値 - 現在の物体の中心)　であり、　入力とはモーターのpwmの値。
        #pid制御についての参考文献　https://controlabo.com/pid-control-introduction/
        #Kx　の値については調節が必要
        self.goal_L = 325.0 #目標値 (物体の中心)
        self.goal_R = 315.0
        self.e = 0.00 #偏差(目的値と現在値の差)
        self.Kp = 0.1 #誤差が大きいときには入力を大きく、誤差が少ないときには入力を小さくするように入力値を調整  イメージとしては引っ張ったばねが徐々に収束する感じ
        self.Ki = 0.1 #現在の物体の中心がが目標値を上回るときにはpwmの上昇幅を小さくし、下回るときにはpwmの上昇幅を大きくするように入力を調整  ダンパーがついているイメージ
        self.Kd = 0.1 #予期していない外部からの影響を和らげる項

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
        """
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
        """
        self.pwm_right_init = 0.0#pwmの初期値
        self.e1_right_init = 0.0#前回の偏差の初期値
        self.e2_right_init = 0.0#前々回の偏差の初期値

        self.pwm_left_init = 0.0#pwmの初期値
        self.e1_left_init = 0.0#前回の偏差の初期値
        self.e2_left_init = 0.0#前々回の偏差の初期値
        
        self.cap = cv2.VideoCapture(0)
        self.input_shape = (416, 416)
        self.session = onnxruntime.InferenceSession("yolox_nano_cone_best.onnx")

    def object_detection(self):
    
        while(self.cap.isOpened()):
            # フレームを取得
            ret, frame = self.cap.read()
            origin_img = frame
            img, ratio = preprocess(origin_img, self.input_shape)#モデルのinputが416*416で学習をしたため、カメラの映像のサイズを416*416にリサイズしている
            # 推論＋後処理
            ort_inputs = {self.session.get_inputs()[0].name: img[None, :, :, :]}
            output = self.session.run(None, ort_inputs)
            predictions = demo_postprocess(output[0], self.input_shape, p6=False)[0]
    
            # xyxyへの変換＋NMS
            boxes = predictions[:, :4]
            scores = predictions[:, 4:5] * predictions[:, 5:]#BoundingBoxごとのスコア
            boxes_xyxy = np.ones_like(boxes)
            boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2]/2.
            boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3]/2.
            boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2]/2.
            boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3]/2.
            boxes_xyxy /= ratio
            #boxes_xyxy = np.amax(boxes_xyxy)
            dets = multiclass_nms(boxes_xyxy, scores, nms_thr=0.40, score_thr=0.1)
            if dets is not None:#コーンが映った場合、BoundingBoxが描画される
                final_boxes, final_scores, final_cls_inds = dets[:, :4], dets[:, 4], dets[:, 5]
                #final_boxes　　認識した物体の座標(左上頂点のx,左上頂点のy,右下頂点のx,右下頂点のy)
                # BoundingBoxを描画する場合

                #座標がnumpy配列の一つの要素としてまとめられて渡されるので、区切る。
                x_left_upper = str(final_boxes)
                x_left_upper = x_left_upper.replace('[', '')
                x_left_upper = x_left_upper.replace(']', '')
                x_left_upper = x_left_upper.split()
                self.x_center.value = float(x_left_upper[0]) + float(x_left_upper[2]) / 2
                #print(np.max(final_scores))
                arg = np.argmax(final_scores)#認識した物体の中で、Coneの確率が最も高いのを取り出す。
                #print(final_cls_inds.max())

                #1つだけ認識できるようにした。もし複数物体認識したいのならvisualize.pyの#を外して、インデントを上げる。またmax()を外す。
                inference_img = vis(origin_img, final_boxes[arg], final_scores[arg], final_cls_inds.max(),
                            conf=0.5, class_names=COCO_CLASSES)#final_cls_indsに、.max()してるのは、要素を1つだけvis()に渡したいので、.max()とすることで一つだけ渡している。他のタスクとかではできない。
            cv2.imshow("Frame",frame)
            #self.video.write(frame)name='Thread T2'
            # qキーが押されたら途中終了
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break       

        self.cap.release()
        cv2.destroyAllWindows()


    def motor_pid(self):
        power_R=0
        power_L=0
        
        #周波数設定
        a1 = GPIO.PWM(self.AIN1,255)#255Hz
        a2 = GPIO.PWM(self.AIN2,255)#255Hz
        b1 = GPIO.PWM(self.BIN1,255)
        b2 = GPIO.PWM(self.BIN2,255)
        #PWM起動
        a1.start(0)#Aenable接続（E）
        a2.start(0)#Aphase接続（P）
        b1.start(0)
        b2.start(0)
        #値の初期化
        pwm_right,e1_right,e2_right = self.pwm_right_init,self.e1_right_init,self.e2_right_init
        pwm_left,e1_left,e2_left = self.pwm_left_init,self.e1_left_init,self.e2_left_init

        while True:
            #止まる(物体の近くに接近した際)
            """
            if self.reading(0) < 20:
            
                self.stop()
                self.GPIO.cleanup()
                break
            """
            pwm_right,e1_right,e2_right = self.pid_right(pwm_right,e1_right,e2_right)
            pwm_left,e1_left,e2_left = self.pid_left(pwm_left,e1_left,e2_left)

            print("pid_L:" + str(power_R))
            print("pid_R:" + str(power_L))
            #print(self.x_center)
            if(self.x_center != None):#中心座標が得れたら

                power_R = (30 + pwm_right - pwm_left) #30は初期値   要調節
                power_L = (30 + pwm_left - pwm_right) #30は初期値   要調節
            else:
                print("eee")

            if(power_R > 100):#入力が100を超えたら
                print("i")
                power_R = 100
            if(power_R < 0):#入力が0を下回ったら
                print("u")
                power_R = 0  
            if(power_L > 100):#入力が100を超えたら
                print("e")
                power_L = 100
            if(power_L < 0):#入力が0を下回ったら
                print("o")
                power_L = 0
            """
            if (self.x_center == self.x_before):#見失ったら回転
                print("11")
                self.CCW()
                time.sleep(3)
                continue
            """
            #print("EE")
            a1.ChangeDutyCycle(power_R)
            a2.ChangeDutyCycle(0)
            b1.ChangeDutyCycle(power_L)
            b2.ChangeDutyCycle(0)
            time.sleep(3)
       
            #self.x_before = self.x_center 
    

if __name__ == '__main__':
    cansat = Short_Range()

    thread_object_detection = threading.Thread(target=cansat.object_detection())
    thread_motor = threading.Thread(target=cansat.motor_pid())
    thread_object_detection.start()
    #time.sleep(3)
    #time.sleep(1)
    thread_motor.start()


    

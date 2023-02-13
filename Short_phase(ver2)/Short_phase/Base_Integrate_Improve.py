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
import csv

class Short_Range(BaseSR):
    
    def __init__(self):
        
        global e
        global e1


        # 全体(GPIO)の初期化
        
        GPIO.setwarnings(False)

        #GPIOのモード
        #GPIO.setmode(GPIO.BOARD)#物理ピン番号でGPIOを指定
        GPIO.setmode(GPIO.BOARD)

        #----------------------------------------------------

        # 物体検出の初期化

        # 出力する動画ファイルの設定
        self.fps = 10
        self.size = (640,480)
        self.fourcc = cv2.VideoWriter_fourcc('m','p','4','v')
        self.video_name = 'VIDEO.avi'

        # 共通メモリ
        self.x_center = Value("d",320)
        self.count = Value("i",0) #カメラが終了したらモーターも終了するためのフラグ or 超音波で接近を検知したら、カメラが終了し映像を保存するためのフラグ
        
        # カメラ指定
        self.cap = cv2.VideoCapture(0)

        # 入力画像のサイズ
        self.input_shape = (416, 416)

        # モデルのファイル名
        self.session = onnxruntime.InferenceSession("yolox_nano_cone_best.onnx")

        #----------------------------------------------------

        # 超音波の初期化

        # PIN指定
        self.TRIG = 11
        self.ECHO = 13

        global signaloff
        global signalon

        #-----------------------------------------------------

        # モーターの初期化

        # PIN指定
        self.AIN1 = 15
        self.AIN2 = 29
        self.BIN1 = 31
        self.BIN2 = 33

        # duty比指定(0~100)
        self.duty = 50


    def object_detection(self):

        # 動画ファイルを保存するための形式指定
        video = cv2.VideoWriter(self.video_name, self.fourcc, self.fps, self.size)

        while(self.cap.isOpened()):

            # フレームを取得
    
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) # バッファサイズ1
            ret, frame = self.cap.read() # 映像読み込み
            
            origin_img = frame
            img, ratio = preprocess(origin_img, self.input_shape)# モデルのinputが416*416で学習をしたため、カメラの映像のサイズを416*416にリサイズしている
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
            if dets is not None :#コーンが映った場合、BoundingBoxが描画される
                final_boxes, final_scores, final_cls_inds = dets[:, :4], dets[:, 4], dets[:, 5]
                # final_boxes　　認識した物体の座標(左上頂点のx,左上頂点のy,右下頂点のx,右下頂点のy)

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
            video.write(frame)

            # qキーが押されたら途中終了
            if cv2.waitKey(25) & 0xFF == ord('q') and self.count.value == 1:
                self.count.value = 1
                self.cap.release()
                print("camera finished")
                cv2.destroyAllWindows()
                break       



    def motor_pid(self):
        GPIO.setup(self.AIN1,GPIO.OUT)
        GPIO.setup(self.AIN2,GPIO.OUT)
        GPIO.setup(self.BIN1,GPIO.OUT)
        GPIO.setup(self.BIN2,GPIO.OUT)
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
        """
        with open('hcsr04.csv', 'w') as fh: 
            hcsr04 = csv.writer(fh)
            hcsr04.writerow(['distance'])
        """


        while True:
            print("-----------------------")
            print("center : "+ str(self.x_center.value))
            #zahyouha baai ni yotte kaeru
            
            print("Ultra Sound : " + str(self.reading(0)))
            """
            with open('hcsr04.csv', 'w') as fh: 
                hcsr04 = csv.writer(fh)
                distance_row = [self.reading(0)]
                hcsr04.writerow(distance_row)
            """
            if self.reading(0) < 0.00000001:
            
                a1.ChangeDutyCycle(0)
                a2.ChangeDutyCycle(0)   
                b1.ChangeDutyCycle(0)
                b2.ChangeDutyCycle(0)

                self.count.value = 1
        
                break
            
            if  270 <= self.x_center.value and self.x_center.value < 370 :
                #まっすぐ進み動作(物体が中心近くにいる際)
                print("forward")
                a1.ChangeDutyCycle(self.duty)
                a2.ChangeDutyCycle(0)   
                b1.ChangeDutyCycle(0)
                b2.ChangeDutyCycle(self.duty)
                time.sleep(2)
            elif self.x_center.value < 270 :
                #回転する動作(物体がカメラの中心から左にずれている際)
                print("CCW")
                a1.ChangeDutyCycle(self.duty)
                a2.ChangeDutyCycle(0)   
                b1.ChangeDutyCycle(self.duty)
                b2.ChangeDutyCycle(0)
                time.sleep(2)
            
            elif self.x_center.value >= 370 :
                #回転する動作（物体がカメラの中心から右にずれている際）
                print("CW")
                a1.ChangeDutyCycle(0)
                a2.ChangeDutyCycle(self.duty)   
                b1.ChangeDutyCycle(0)
                b2.ChangeDutyCycle(self.duty)
                time.sleep(2)
            
            else:
                print("nazo")
                
            if self.count.value == 1:
                #fh.close()
                break
       
            #self.x_before = self.x_center 
    

if __name__ == '__main__':
    cansat = Short_Range()

    p1 = Process(target=cansat.object_detection)
    p2 = Process(target=cansat.motor_pid)
    p1.start()
    p2.start()
    GPIO.cleanup()
    p1.join()
    p2.join()


    


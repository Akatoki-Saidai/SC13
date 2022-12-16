import cv2
import numpy as np
import sys
sys.path.append("C:\\Users\\xxxx\\YOLOX")
import onnxruntime
from yolox.data.data_augment import preproc as preprocess
from yolox.data.datasets import COCO_CLASSES
from yolox.utils import multiclass_nms, demo_postprocess, vis
import os
import inspect
import RPi.GPIO as GPIO #GPIOインポート
import time
import motor
import ultrasound 
import threading

class Short_range():
    
    def __init__(self):

        #カメラアクセス
        self.cap = cv2.VideoCapture(0)
        # 画像の読み込み、前処理
        self.input_shape = (416, 416)
        # ONNXセッション
        self.session = onnxruntime.InferenceSession("C:\\Users\\xxxx\\YOLOX\\yolox_nano_cone.onnx")
        # 出力する動画ファイルの設定
        self.fps = 15
        self.size = (640,480)
        self.fourcc = cv2.VideoWriter_fourcc(*'H264')#H264は圧縮フォーマット形式なので、速度を上昇させることを目的で)
        self.video_name = 'VIDEO.avi'
        self.video = cv2.VideoWriter(self.video_name, self.fourcc, self.fps, self.size)




    def object_detection(self):
        global x_center
    
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

                print(np.max(final_scores))
                arg = np.argmax(final_scores)#認識した物体の中で、Coneの確率が最も高いのを取り出す。
                print(final_cls_inds.max())
        
                x_center = (final_boxes[0] + final_boxes[2]) / 2

                #1つだけ認識できるようにした。もし複数物体認識したいのならvisualize.pyの#を外して、インデントを上げる。またmax()を外す。
                inference_img = vis(origin_img, final_boxes[arg], final_scores[arg], final_cls_inds.max(),
                            conf=0.5, class_names=COCO_CLASSES)#final_cls_indsに、.max()してるのは、要素を1つだけvis()に渡したいので、.max()とすることで一つだけ渡している。他のタスクとかではできない。
            cv2.imshow("Frame",frame)
            self.video.write(frame)
            # qキーが押されたら途中終了
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

        

        self.cap.release()
        cv2.destroyAllWindows()


    def motor():
        while True:

            if reading(0) < 20:
            #止まる(物体の近くに接近した際)
                stop()
                GPIO.cleanup()
                break

            if  150 <= x_center and x_center < 540 :
            #まっすぐ進み動作(物体が中心近くにいる際)
                forward()
                time.sleep(3)
                time.sleep(2)
                CCW()
                time.sleep(0.3)
                stop()
                time.sleep(1)
                            
            elif x_center < 150: 
            #回転する動作(物体がカメラの中心から左にずれている際)
                CCW()
                time.sleep(0.2)
                stop()
                time.sleep(1)
    
            elif x_center >= 540:
            #回転する動作（物体がカメラの中心から右にずれている際）
                CW()
                time.sleep(0.2)
                stop()
                time.sleep(1)


            #止まる(物体の近くに接近した際)
                stop()
                GPIO.cleanup()
                break
    


cansat = Short_range()

thread_object_detection = threading.Thread(target=cansat.object_detection())
thread_motor = threading.Thread(target=cansat.motor())
thread_object_detection.start()
time.sleep(3)
time.sleep(1)
thread_motor.start()


    

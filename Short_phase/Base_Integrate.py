import numpy as np
import sys
sys.path.append("C:\\Users\\konan\\YOLOX") #要変更 or フォルダーの構成次第ではいらない
import onnxruntime
from yolox.data.data_augment import preproc as preprocess
from yolox.data.datasets import COCO_CLASSES
from yolox.utils import multiclass_nms, demo_postprocess, vis
import threading
from Base import BaseSR

class Short_Range(BaseSR):
    
    def __init__(self):

        self.x_before = None

        self.pwm_right_init = 0#pwmの初期値
        self.e1_right_init = 0#前回の偏差の初期値
        self.e2_right_init = 0#前々回の偏差の初期値

        self.pwm_left_init = 0#pwmの初期値
        self.e1_left_init = 0#前回の偏差の初期値
        self.e2_left_init = 0#前々回の偏差の初期値


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

                #座標がnumpy配列の一つの要素としてまとめられて渡されるので、区切る。
                x_left_upper = str(final_boxes)
                x_left_upper = x_left_upper.replace('[', '')
                x_left_upper = x_left_upper.replace(']', '')
                x_left_upper = x_left_upper.split()
                x_center = float(x_left_upper[0]) + float(x_left_upper[2]) / 2
                print(np.max(final_scores))
                arg = np.argmax(final_scores)#認識した物体の中で、Coneの確率が最も高いのを取り出す。
                print(final_cls_inds.max())

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


    def motor(self):
        #値の初期化
        pwm_right,e1_right,e2_right = self.pwm_right_init,self.e1_right_init,self.e2_right_init
        pwm_left,e1_left,e2_left = self.pwm_left_init,self.e1_left_init,self.e2_left_init

        while True:

            #止まる(物体の近くに接近した際)
            if self.reading(0) < 20:
            
                self.stop()
                self.GPIO.cleanup()
                break

            pwm_right,e1_right,e2_right = self.pid_right(pwm_right,e1_right,e2_right,x_center)
            pwm_left,e1_left,e2_left = self.pid_left(pwm_left,e1_left,e2_left,x_center)

            if(x_center != None):#中心座標が得れたら
                power_R = (30 + pwm_right - pwm_left) #30は初期値   要調節
                power_L = (30 + pwm_left - pwm_right) #30は初期値   要調節

            if(power_R > 100):#入力が100を超えたら
                power_R = 100
            if(power_R < 0):#入力が0を下回ったら
                power_R = 0  
            if(power_L > 100):#入力が100を超えたら
                power_L = 100
            if(power_L < 0):#入力が0を下回ったら
                power_L = 0 
            
            if (x_center == x_before):#見失ったら回転
                self.CCW()
                time.sleep(3)
                continue

            self.a1.ChangeDutyCycle(power_R)
            self.a2.ChangeDutyCycle(0)
            self.b1.ChangeDutyCycle(power_L)
            self.b2.ChangeDutyCycle(0)
            time.sleep(3)
       
            x_before = x_center 
    

if __name__ == 'main':
    cansat = Short_Range()

    thread_object_detection = threading.Thread(target=cansat.object_detection())
    #thread_motor = threading.Thread(target=cansat.motor())
    thread_object_detection.start()
    #time.sleep(3)
    #time.sleep(1)
    #thread_motor.start()


    
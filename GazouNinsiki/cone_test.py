import cv2
import numpy as np
import onnxruntime
from yolox.data.data_augment import preproc as preprocess
from yolox.data.datasets import COCO_CLASSES
from yolox.utils import multiclass_nms, demo_postprocess, vis
import os
import inspect


"""
2021年に発表されたYOLOX(nano)を使った。関数もほとんどがそのリポジトリにあるのを使わせてもらったので
詳しくは https://github.com/Megvii-BaseDetection/YOLOX
やgoogleでYOLOXと検索。
学習に使った画像はRoboFlowからダウンロードさせてもらった。
他人のリポジトリの中身をほとんど同じ状態でgithubにあげるのも忍びないので使う際は、上記のリポジトリからcloneして、下記のディレクトリー構成を参考にして再現してほしい。
他の学習した重みはgoogle driveのSC13の電装の中にzip形式で置いてある。
使うライブラリーはYOLOX\requirements.txtの中に書いてあるので pip install -r requirements.txtでインストールする。その際はYOLOXをカレントディレクトリーにする。

ディレクトリー構成
------YOLOX
   |
   |--yolox
   |
   |--tools
   |
   |--etc..
   |
   |--cone_test.py
   |
   |--xxxx.onnx

改善点
1.重みなどを量子化して処理を軽くする。
2.学習に使う画像を増やしたり、前処理をしてロバスト性を高める。
3.実際にcansatから撮った写真を使う。
4.何epochのが一番いいかまだよく見てないので、確認してそれを使うようにする。

#from yolox.data.datasets import COCO_CLASSES
でラベルをimportしてるが、初期の段階だと80個あるので、それを消して、Coneだけにするか、自分でラベル書いたのをimportするようにする。
"""
#print(inspect.getfile(multiclass_nms))

#カメラアクセス
cap = cv2.VideoCapture(0)
# 画像の読み込み、前処理
input_shape = (416, 416)

# ONNXセッション
session = onnxruntime.InferenceSession("yolox_nano_cone.onnx")
while(cap.isOpened()):
    # フレームを取得
    ret, frame = cap.read()
    origin_img = frame

    # 推論＋後処理
    img, ratio = preprocess(origin_img, input_shape)#モデルのinputが416*416で学習をしたため、カメラの映像のサイズを416*416にリサイズしている
    ort_inputs = {session.get_inputs()[0].name: img[None, :, :, :]}
    output = session.run(None, ort_inputs)
    predictions = demo_postprocess(output[0], input_shape, p6=False)[0]
    
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
        print(final_scores)
        arg = np.argmax(final_scores)#認識した物体の中で、Coneの確率が最も高いのを取り出す。


        #1つだけ認識できるようにした。もし複数物体認識したいのならvisualize.pyの#を外して、インデントを上げる。またmax()を外す。
        inference_img = vis(origin_img, final_boxes[arg], final_scores[arg], final_cls_inds.max(),
                        conf=0.5, class_names=COCO_CLASSES)
    cv2.imshow("Frame",frame)

    # qキーが押されたら途中終了
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
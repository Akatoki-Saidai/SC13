import cv2
import time
from Base_Motor import Motor

class BaseSR(Motor):
    
    def __init__(self):

        #カメラアクセス
        self.cap = cv2.VideoCapture(0)

        # 出力する動画ファイルの設定
        self.fps = 15
        self.size = (640,480)
        self.fourcc = cv2.VideoWriter_fourcc(*'H264')#H264は圧縮フォーマット形式なので、速度を上昇させることを目的で)
        self.video_name = 'VIDEO.avi'
        self.video = cv2.VideoWriter(self.video_name, self.fourcc, self.fps, self.size)

    def camera(self):
        while True:
            ret,frame = self.cap.read()
            cv2.imshow("Frame",frame)
            self.video.write(frame)

            # qキーが押されたら途中終了
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

    def motor_test(self):

        while True:

            button = input("前進なら1、右に曲がるなら2、左に曲がるなら3、後進なら4を押してください")

            if(button == "1"):
                self.forward()
                time.sleep(3)
                time.sleep(2)

            elif(button == "2"):
                self.CW()
                time.sleep(3)
                time.sleep(2)

            elif(button == "3"):
                self.CCW()
                time.sleep(3)
                time.sleep(2)

            elif(button == "4"):
                self.back()
                time.sleep(3)
                time.sleep(2)

            else:
                print("Error")
                break

if __name__ == '__main__':
    cansat = BaseSR()
    cansat.camera()

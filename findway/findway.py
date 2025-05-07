import time
from unicodedata import name
import cv2
import numpy as np
#import serial
import config
import utils
import globals



def main():
    cam = cv2.VideoCapture(config.CAM_INDEX)
    # cam = cv2.VideoCapture(config.CAM_INDEX,cv2.CAP_V4L2)    适用于linxu系统
    cam.set(4,config.FRAME_HEIGHT)
    cam.set(3,config.FRAME_WIDTH)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))  # 添加MJPG编码

    mode = 1
    ver_edge = np.zeros((1,4),np.uint8) 
    hor_edge = np.zeros((1,4),np.uint8)
    while True:
        # start=time.time()
        # 检测工作模式
        mode=utils.identify_mode(mode)

        if cam.isOpened():
            frame=cam.read()[1]
            # 得到canny边缘
            edges = utils.get_edges(frame)
            orgb = frame.copy()
            # 把roi区域绘制出来
            orgb,ver_edge=utils.slice_roi(orgb,config.VER_ROI_LEFT_RATIO,config.VER_ROI_RIGHT_RATIO,config.VER_ROI_TOP_RATIO,config.VER_ROI_BOTTOM_RATIO,slice=False,draw=True)
            orgb,hor_edge=utils.slice_roi(orgb,config.HOR_ROI_LEFT_RATIO,config.HOR_ROI_RIGHT_RATIO,config.HOR_ROI_TOP_RATIO,config.HOR_ROI_BOTTOM_RATIO,slice=False,draw=True)
            cv2.line(orgb, (0, int(config.HEIGHT_LINE*orgb.shape[0])), (orgb.shape[1], int(config.HEIGHT_LINE*orgb.shape[0])), (255, 0, 0), 5) # 绘制理想的高度线

            # 执行倒车任务
            utils.reversing_task(mode,edges,orgb,ver_edge,hor_edge)
            #bgblack=np.zeros_like(frame)
            #imageArray=[[frame,bin,med,opened],[closed,edges,orgb,bgblack]]
            '''
            imageArray=[[frame,edges,orgb]]
            imgstacked=utils.stack_images(imageArray,config.STACK_SCALE)
            cv2.imshow('stack',imgstacked)
            '''
            cv2.imshow('final',orgb)

        # end=time.time()
        # utils.myprint("fps: ",1/(end-start))

        if cv2.waitKey(1) & 0xFF == 27:
            break


    cam.release()
    cv2.destroyAllWindows()



if __name__=="__main__":
    main()
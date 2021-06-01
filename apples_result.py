import cv2 as cv
import numpy as np
import os
import cv2
import sys
import math
import time
import random
import string
#import pygame
import vrep
import numpy as np
#import test1





class UR5_RG2:
    # variates
    resolutionX = 640  # Camera resolution: 640*480
    resolutionY = 480
    joint_angle = [0, 0, 0, 0, 0, 0]  # each angle of joint
    RAD2DEG = 180 / math.pi  # transform radian to degrees

    # Handles information
    camera_rgb_Name = 'kinect_rgb'
    camera_depth_Name = 'kinect_depth'

    # communication and read the handles
    def __init__(self):
        camera_rgb_Name = self.camera_rgb_Name
        camera_depth_Name = self.camera_depth_Name

        print('Simulation started')
        vrep.simxFinish(-1)  # 关闭潜在的连接
        # 每隔0.2s检测一次, 直到连接上V-rep
        while True:
            # simxStart的参数分别为：服务端IP地址(连接本机用127.0.0.1);端口号;是否等待服务端开启;连接丢失时是否尝试再次连接;超时时间(ms);数据传输间隔(越小越快)
            clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
            if clientID > -1:
                print("Connection success!")
                break
            else:
                time.sleep(0.2)
                print("Failed connecting to remote API server!")
                print("Maybe you forget to run the simulation on vrep...")

        vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)  # 仿真初始化

        # 读取Base和Joint的句柄
        _, cameraRGBHandle = vrep.simxGetObjectHandle(clientID, camera_rgb_Name, vrep.simx_opmode_blocking)
        _, cameraDepthHandle = vrep.simxGetObjectHandle(clientID, camera_depth_Name, vrep.simx_opmode_blocking)

        self.clientID = clientID
        self.cameraRGBHandle = cameraRGBHandle
        self.cameraDepthHandle = cameraDepthHandle



    def __del__(self):
        clientID = self.clientID
        vrep.simxFinish(clientID)
        print('Simulation end')

    # show Handles information
    def showHandles(self):
        cameraRGBHandle = self.cameraRGBHandle
        cameraDepthHandle = self.cameraDepthHandle

        print('Handles available!')
        print("==============================================")
        print("Handles:  ")
        print("cameraRGBHandle:" + cameraRGBHandle)
        print("cameraDepthHandle:" + cameraDepthHandle)
        print("===============================================")


    # get RGB images
    def getImageRGB(self):
        clientID = self.clientID
        cameraRGBHandle = self.cameraRGBHandle
        resolutionX = self.resolutionX
        resolutionY = self.resolutionY

        res1, resolution1, image_rgb = vrep.simxGetVisionSensorImage(clientID, cameraRGBHandle, 0,
                                                                     vrep.simx_opmode_blocking)

        image_rgb_r = [image_rgb[i] for i in range(0, len(image_rgb), 3)]
        image_rgb_r = np.array(image_rgb_r)
        image_rgb_r = image_rgb_r.reshape(resolutionY, resolutionX)
        image_rgb_r = image_rgb_r.astype(np.uint8)

        image_rgb_g = [image_rgb[i] for i in range(1, len(image_rgb), 3)]
        image_rgb_g = np.array(image_rgb_g)
        image_rgb_g = image_rgb_g.reshape(resolutionY, resolutionX)
        image_rgb_g = image_rgb_g.astype(np.uint8)

        image_rgb_b = [image_rgb[i] for i in range(2, len(image_rgb), 3)]
        image_rgb_b = np.array(image_rgb_b)
        image_rgb_b = image_rgb_b.reshape(resolutionY, resolutionX)
        image_rgb_b = image_rgb_b.astype(np.uint8)

        result_rgb = cv2.merge([image_rgb_b, image_rgb_g, image_rgb_r])
        # 镜像翻转, opencv在这里返回的是一张翻转的图
        result_rgb = cv2.flip(result_rgb, 0)
        return result_rgb

    # get depth images
    def getImageDepth(self):
        clientID = self.clientID
        cameraDepthHandle = self.cameraDepthHandle
        resolutionX = self.resolutionX
        resolutionY = self.resolutionY

        # 获取 Depth Info
        sim_ret, resolution, depth_buffer = vrep.simxGetVisionSensorDepthBuffer(clientID, cameraDepthHandle,
                                                                               vrep.simx_opmode_blocking)
        depth_img = np.asarray(depth_buffer)
        depth_img.shape = (resolution[1], resolution[0])
        #depth_img = np.fliplr(depth_img)
        depth_img = cv2.flip(depth_img, 0)
        zNear = 0.01
        zFar = 2
        result_depth = depth_img * (zFar - zNear) + zNear


        # res2, resolution2, image_depth = vrep.simxGetVisionSensorImage(clientID, cameraDepthHandle, 0,
        #                                                                vrep.simx_opmode_blocking)
        #
        # image_depth_r = [image_depth[i] for i in range(0, len(image_depth), 3)]
        # image_depth_r = np.array(image_depth_r)
        # image_depth_r = image_depth_r.reshape(resolutionY, resolutionX)
        # image_depth_r = image_depth_r.astype(np.uint8)
        #
        # image_depth_g = [image_depth[i] for i in range(1, len(image_depth), 3)]
        # image_depth_g = np.array(image_depth_g)
        # image_depth_g = image_depth_g.reshape(resolutionY, resolutionX)
        # image_depth_g = image_depth_g.astype(np.uint8)
        #
        # image_depth_b = [image_depth[i] for i in range(2, len(image_depth), 3)]
        # image_depth_b = np.array(image_depth_b)
        # image_depth_b = image_depth_b.reshape(resolutionY, resolutionX)
        # image_depth_b = image_depth_b.astype(np.uint8)
        #
        # result_depth = cv2.merge([image_depth_b, image_depth_g, image_depth_r])
        # # 镜像翻转, opencv在这里返回的是一张翻转的图
        # result_depth = cv2.flip(result_depth, 0)
        #
        # # 黑白取反
        # height, width, channels = result_depth.shape
        # for row in range(height):
        #     for list in range(width):
        #         for c in range(channels):
        #             pv = result_depth[row, list, c]
        #             result_depth[row, list, c] = 255 - pv

        return result_depth




# control robot by keyboard

robot = UR5_RG2()
resolutionX = robot.resolutionX
resolutionY = robot.resolutionY
ImageDepth=robot.getImageDepth()
ImageRGB=robot.getImageRGB()
cv2.imshow("zhk",ImageDepth)
cv2.imshow("zz", ImageRGB)
print(ImageDepth.shape)
d=ImageDepth[10,380]
print(d)















def distance_seg(image):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)  # 获取灰度图像
    ret, binary = cv.threshold(gray, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)

    # 形态学操作，进一步消除图像中噪点
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (7, 7))
    sure_bg = cv.dilate(binary, kernel, iterations=1)  # 3次膨胀,可以获取到大部分都是背景的区域

    # 距离变换
    dist = cv.distanceTransform(sure_bg, cv.DIST_L2, 5)
    cv.imshow("dist", dist)
    dist_output = cv.normalize(dist, 0, 1.0, cv.NORM_MINMAX)

    cv.imshow("distinct-t", dist_output * 50)
    # ret, sure_fg = cv.threshold(dist, dist.max() * 0.5, 255, cv.THRESH_BINARY)
    dist = np.uint8(dist)
    ret, sure_fg = cv.threshold(dist, 0, 255, cv.THRESH_BINARY|cv.THRESH_OTSU)
    # dist=np.uint8(dist)
    # sure_fg = cv.adaptiveThreshold(dist, 255, cv.ADAPTIVE_THRESH_MEAN_C,cv.THRESH_BINARY, 19, 0)
    cv.imshow("sure_fg", sure_fg)
    return sure_fg



def watershed_demo(image):
    gray = cv.cvtColor(image,cv.COLOR_BGR2GRAY)  #获取灰度图像


    gray1 = cv.cvtColor(src, cv.COLOR_BGR2GRAY)  # 获取灰度图像
    ret1,binary1 = cv.threshold(gray1,0,255,cv.THRESH_BINARY|cv.THRESH_OTSU)
    cv.imshow("binary1",binary1)



    ret,binary = cv.threshold(gray,0,255,cv.THRESH_BINARY|cv.THRESH_OTSU)
    #形态学操作，进一步消除图像中噪点
    kernel = cv.getStructuringElement(cv.MORPH_RECT,(7,7))
    mb = cv.morphologyEx(binary,cv.MORPH_OPEN,kernel)
    sure_bg = cv.dilate(mb,kernel,iterations=1) #3次膨胀,可以获取到大部分都是背景的区域
    cv.imshow("sure_bg",sure_bg)
    #距离变换
    dist = cv.distanceTransform(sure_bg,cv.DIST_L2,5)
    cv.imshow("dist",dist)
    dist_output = cv.normalize(dist,0,1.0,cv.NORM_MINMAX)

    cv.imshow("distinct-t",dist_output*50)
    ret, sure_fg = cv.threshold(dist, dist.max() * 0.5, 255, cv.THRESH_BINARY)
    cv.imshow("sure_fg",sure_fg)
    return sure_fg






def watershed_va(image):
    gray = cv.cvtColor(image,cv.COLOR_BGR2GRAY)  #获取灰度图像
    gray1 = cv.cvtColor(src, cv.COLOR_BGR2GRAY)  # 获取灰度图像
    ret,binary = cv.threshold(gray,0,255,cv.THRESH_BINARY|cv.THRESH_OTSU)
    ret, binary1 = cv.threshold(gray1, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
    # #形态学操作，进一步消除图像中噪点
    # kernel = cv.getStructuringElement(cv.MORPH_RECT,(7,7))
    # mb = cv.morphologyEx(binary,cv.MORPH_OPEN,kernel)
    # sure_bg = cv.dilate(mb,kernel,iterations=0) #3次膨胀,可以获取到大部分都是背景的区域
    # cv.imshow("sure_bg",sure_bg)
    # #距离变换
    # dist = cv.distanceTransform(sure_bg,cv.DIST_L2,5)
    # cv.imshow("dist",dist)
    # dist_output = cv.normalize(dist,0,1.0,cv.NORM_MINMAX)
    #
    # cv.imshow("distinct-t",dist_output*50)
    # ret, sure_fg = cv.threshold(dist, dist.max() * 0.5, 255, cv.THRESH_BINARY)
    # cv.imshow("sure_fg",sure_fg)
    #
    # #获取未知区域
    # surface_fg = np.uint8(sure_fg)  #保持色彩空间一致才能进行运算，现在是背景空间为整型空间，前景为浮点型空间，所以进行转换


    unknown = cv.subtract(binary1,binary)
    cv.imshow("z1",binary)
    cv.imshow("z2", binary1)
    cv.imshow("unkown",unknown)
    #获取maskers,在markers中含有种子区域
    ret,markers = cv.connectedComponents(binary)
    #print(ret)

    #分水岭变换
    markers = markers + 1
    markers[unknown==255] = 0

    markers = cv.watershed(src,markers=markers)
    emptydian = np.zeros(image.shape, np.uint8)
    emptydian[markers==-1]=[255,255,255]

    kernel = cv.getStructuringElement(cv.MORPH_RECT,(3,3))
    emptydian = cv.dilate(emptydian,kernel,iterations=1) #3次膨胀,可以获取到大部分都是背景的区域
    cv.imshow("emptydian", emptydian)
    emptydian=cv.cvtColor(emptydian,cv.COLOR_BGR2GRAY)


    zh[markers == 1] = [0,0, 0]
    zh[markers ==2] = [0, 50, 0]
    zh[markers == 3] = [50, 0, 0]
    zh[markers == 4] = [0, 50, 50]
    zh[markers == 5] = [50, 0, 50]
    zh[markers == 6] = [50, 50, 0]
    zh[markers == 7] = [50, 50, 255]
    zh[markers == 8] = [50, 255, 50]
    zh[markers == 9] = [255, 50, 50]
    zh[markers == 10] = [100, 0, 30]
    zh[markers == 11] = [30, 0, 100]
    zh[markers == 12] = [0, 30, 100]
    zh[markers == 13] = [150, 0, 200]
    zh[markers == 14] = [150, 200, 0]
    zh[markers == 15] = [0, 150, 200]
    zh[markers == 16] = [0, 0, 255]
    zh[markers == 17] = [255, 0, 0]
    zh[markers == 18] = [0, 255, 0]
    zh[markers == 19] = [50, 50, 150]
    zh[markers == 20] = [50, 150, 50]
    zh[markers == 21] = [150, 50, 50]
    zh[markers == 22] = [50, 50, 255]
    zh[markers == 23] = [255, 50, 50]
    zh[markers == 24] = [50, 255, 50]
    zh[markers == 25] = [50, 250, 70]
    zh[markers == 26] = [70, 60, 250]
    zh[markers == 27] = [30, 250, 50]
    zh[markers == 28] = [50, 200, 30]
    zh[markers == 29] = [50, 0, 200]
    zh[markers == 30] = [50, 180, 50]
    zh[markers == 31] = [50, 90, 100]
    zh[markers == 32] = [90, 10, 200]
    zh[markers == 33] = [90, 0, 100]
    zh[emptydian == 255] = [0, 0, 0]
    cv.imshow("zh",zh)

    src[emptydian == 255] = [0, 0, 0]
    cv.imshow("result",src)


    graydemo = cv.cvtColor(src, cv.COLOR_BGR2GRAY)  # 获取灰度图像
    ret, binarydemo = cv.threshold(graydemo, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
    cv.imshow("erzhihua",binarydemo)
    cloneImage,contours, heriachy = cv.findContours(binarydemo, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    # for i, contour in enumerate(contours):
    #     cv.drawContours(zhk, contours, i, (0, 0, 255), 2)
    #     # print(i)
    # cv.imshow("contours", zhk)

    area = []
    arr=[]
    appleword=[]

    max_idx = 0
    x = 0
    y = 0
    w = 0
    h = 0
    vrepx=-1.4
    vrepy=-1.5758
    vrepz=1.9001
    for k in range(len(contours)):
        area.append(cv.contourArea(contours[k]))
        if (area[k]>300):
            xyz=[]
            world=[]
            x, y, w, h = cv.boundingRect(contours[k])

            fen=zhk[y,x]
            print(fen)

            cv.rectangle(zhk, (x, y), (x + w, y + h), (0, 255, 0), 1)
            cv.circle(zhk,(x+int(w/2),y+int(h/2)),2,(0,255,255),2)

            a = x + int(w / 2)
            b = y + int(h / 2)
            d = ImageDepth[b, a]
            xyz.append(a)
            xyz.append(b)
            xyz.append(d)
            arr.append(xyz)

            applex=(a-320)/640+vrepx
            applex=round(applex, 3)
            applez=(240-b)/640+vrepz
            applez = round(applez, 3)
            appley=vrepy+d
            appley = round(appley, 3)
            world.append(applex)
            world.append(appley)
            world.append(applez)
            appleword.append(world)






    cv.imshow("dingwei",zhk)
    print(arr)
    print(appleword)




















def reasure_object(image):
    img1 = image.copy()
    dst= cv.cvtColor(image,cv.COLOR_BGR2GRAY)
    ret,binary=cv.threshold(dst,0,255,cv.THRESH_BINARY|cv.THRESH_OTSU)
    cv.imshow("binary",binary)

    kernel = cv.getStructuringElement(cv.MORPH_RECT, (7, 7))
    sure_bg = cv.erode(binary, kernel, iterations=0)  # 3次膨胀,可以获取到大部分都是背景的区域

    cloneImage,contours,bireachy=cv.findContours(sure_bg,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    X=[]
    tu=[]
    for i,contour in enumerate(contours):
        area=cv.contourArea(contour)
        X.append(area)
        print("area:%s"%area)

    mean_value=np.mean(X)
    print(mean_value)
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (5,5))
    kernel1 = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
    kernel2 = cv.getStructuringElement(cv.MORPH_RECT, (7, 7))

    emptyImage = np.zeros(image.shape, np.uint8)
    for i,contour in enumerate(X):

        rect = cv.minAreaRect(contours[i])
        #box = cv.cv.BoxPoints(rect)
        #box = np.int0(box)
        #cv.drawContours(img, [box], 0, (0, 0, 255), 2)
        h=rect[1][0]
        w=rect[1][1]
        bi=h/w



        #x, y, w, h = cv.boundingRect(contours[i])
        if (X[i]>mean_value or (h/w)>=2):
            x,y,w,h= cv.boundingRect(contours[i])
            ROI = img1[y:y +h, x:x +w]
            cv.imshow("i", ROI)


            #iterations_value=int((max(round( X[i]/3600),bi)))
            # if (iterations_value>2):
            #     iterations_value=2
            #print(iterations_value)
            # if (X[i]>10000):
            #     ROI = cv.erode(ROI, kernel2, iterations=iterations_value)  # 3次膨胀,可以获取到大部分都是背景的区域
            #
            #     ROI = cv.erode(ROI, kernel, iterations=int(iterations_value*1.7))  # 3次膨胀,可以获取到大部分都是背景的区域
            #     ROI = cv.erode(ROI, kernel1, iterations=5)  # 3次膨胀,可以获取到大部分都是背景的区域
            # else:
            #     ROI = cv.erode(ROI, kernel, iterations=int(iterations_value * 1.7))  # 3次膨胀,可以获取到大部分都是背景的区域
            #     ROI = cv.erode(ROI, kernel1, iterations=3)  # 3次膨胀,可以获取到大部分都是背景的区域

            if (X[i]>20000):
                iterations_value = int((max(round(X[i] / 10000), bi)))
                if (iterations_value>=5):
                    iterations_value=5
                print(iterations_value)
                ROI = cv.erode(ROI, kernel2, iterations=1)  # 3次膨胀,可以获取到大部分都是背景的区域

                ROI = cv.erode(ROI, kernel, iterations=iterations_value)  # 3次膨胀,可以获取到大部分都是背景的区域
                ROI = cv.erode(ROI, kernel1, iterations=iterations_value)  # 3次膨胀,可以获取到大部分都是背景的区域
            elif  (X[i]>10000):
                iterations_value = int((max(round(X[i] / 4000), bi)))
                if (iterations_value>=5):
                    iterations_value=5
                print(iterations_value)
                ROI = cv.erode(ROI, kernel2, iterations=1)  # 3次膨胀,可以获取到大部分都是背景的区域

                ROI = cv.erode(ROI, kernel, iterations=int(iterations_value))  # 3次膨胀,可以获取到大部分都是背景的区域
                ROI = cv.erode(ROI, kernel1, iterations=iterations_value)  # 3次膨胀,可以获取到大部分都是背景的区域

            else:
                #ROI = cv.erode(ROI, kernel3, iterations=1)  # 3次膨胀,可以获取到大部分都是背景的区域

                iterations_value = int((max(round(X[i] / mean_value), bi)))
                if (iterations_value>=5):
                    iterations_value=5
                print(iterations_value)
                ROI = cv.erode(ROI, kernel, iterations=int(iterations_value ))  # 3次膨胀,可以获取到大部分都是背景的区域
                ROI = cv.erode(ROI, kernel1, iterations=iterations_value)  # 3次膨胀,可以获取到大部分都是背景的区域


            end_image=distance_seg(ROI)
            end_image = cv.cvtColor(end_image, cv.COLOR_GRAY2BGR)
            cv.imshow("end_image",end_image)


            emptyImage[y:y +h, x:x +w]=end_image
            cv.imshow("emptyImage",emptyImage)

            img1[y:y +h, x:x +w]=[0,0,0]


    cv.imshow("111",img1)
    cv.imshow("222", emptyImage)

    gra = cv.cvtColor(emptyImage, cv.COLOR_BGR2GRAY)
    cloneImage,contours,bireachy=cv.findContours(gra,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    Y=[]

    for i,contour in enumerate(contours):
        area=cv.contourArea(contour)
        Y.append(area)
        print("area:%s"%area)

    mean_zhi=np.mean(Y)

    for i,contour in enumerate(Y):

        rect = cv.minAreaRect(contours[i])
        # box = cv.cv.BoxPoints(rect)
        # box = np.int0(box)
        # cv.drawContours(emptyImage, [box], 0, (0, 0, 255), 2)
        h=rect[1][0]
        w=rect[1][1]
        w2=w
        h2=h
        if(w==0 or h==0):
            bi=0
        elif(w>=h):
            bi=w/h
        else:
            bi=h/w
        print(w)
        print(h)
        print(bi)
        print(rect[2])
        print("sdddddddddddddddddddddddd")

        box = cv.boxPoints(rect)
        # 标准化坐标到整数
        box = np.int0(box)
        # 画出边界
        #cv.drawContours(emptyImage, [box], 0, (0, 0, 255), 3)



        #x, y, w, h = cv.boundingRect(contours[i])
        if ((Y[i]>mean_zhi and bi>=1.8 )):
            x,y,w,h= cv.boundingRect(contours[i])
            ROI_zh = emptyImage[y:y +h, x:x +w]
            cv.imshow("zzzzzzzzzzzzzzzzzzzzz", ROI_zh)

            x,y=rect[0]
            #print(rect)
            x=int(x)
            y=int(y)
            print("zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz")
            print(rect[2])
            print(w2)
            print(h2)
            if(abs(rect[2])>45 and h2<w2 or abs(rect[2])<45 and h2>w2):
                emptyImage[y-65:y+65,x]=(0,0,0)
            else:
                emptyImage[y, x-65:x+65] = (0, 0, 0)


    cv.imshow("aaaaaaaaaaaa",emptyImage)

















    hui = watershed_demo(img1)
    hui = cv.cvtColor(hui, cv.COLOR_GRAY2BGR)
    hui = np.uint8(hui)
    emptyImage = np.uint8(emptyImage)

    jia=cv.add(hui,emptyImage)

    print(hui.shape)
    print(emptyImage.shape)
    cv.imshow("aaa", jia)
    watershed_va(jia)






def FillHole(img):
    im_in = img;
    cv.imwrite("im_in.png", im_in)
    # 复制 im_in 图像
    im_floodfill = im_in.copy()

    # Mask 用于 floodFill，官方要求长宽+2
    h, w = im_in.shape[:2]
    mask = np.zeros((h + 2, w + 2), np.uint8)

    # floodFill函数中的seedPoint对应像素必须是背景
    isbreak = False
    for i in range(im_floodfill.shape[0]):
        for j in range(im_floodfill.shape[1]):
            if (im_floodfill[i][j] == 0):
                seedPoint = (i, j)
                print(seedPoint)
                isbreak = True
                break
        if (isbreak):
            break

    # 得到im_floodfill 255填充非孔洞值
    cv.floodFill(im_floodfill, mask, seedPoint, 255)
    cv.imshow("12",im_floodfill)

    # 得到im_floodfill的逆im_floodfill_inv
    im_floodfill_inv = cv.bitwise_not(im_floodfill)
    cv.imshow("123", im_floodfill_inv)

    # 把im_in、im_floodfill_inv这两幅图像结合起来得到前景
    im_out = im_in | im_floodfill_inv

    # 保存结果
    cv.imshow("zzz", im_out)
    return im_out


def clahe_demo(image):
    #gray=cv.cvtColor(image,cv.COLOR_BGR2GRAY)
    clahe=cv.createCLAHE(clipLimit=2.0,tileGridSize=(8,8))
    dst=clahe.apply(image)
    #cv.imshow("clahe_demo",dst)
    return dst

def BGR_equalHist(image):
    (B, G, R) = cv.split(image)
    B=clahe_demo(B)
    G = clahe_demo(G)
    R = clahe_demo(R)
    img2 = cv.merge([B, G, R])
    return img2





#区域生长
def regionGrow(gray, seeds, thresh, p):
    seedMark = np.zeros(gray.shape)
    #八邻域
    if p == 8:
        connection = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]
    elif p == 4:
        connection = [(-1, 0), (0, 1), (1, 0), (0, -1)]

    #seeds内无元素时候生长停止
    while len(seeds) != 0:
        #栈顶元素出栈
        pt = seeds.pop(0)
        for i in range(p):
            tmpX = pt[0] + connection[i][0]
            tmpY = pt[1] + connection[i][1]

            #检测边界点
            if tmpX < 0 or tmpY < 0 or tmpX >= gray.shape[0] or tmpY >= gray.shape[1]:
                continue

            if abs(int(gray[tmpX, tmpY]) - int(gray[pt])) < thresh and seedMark[tmpX, tmpY] == 0:
                seedMark[tmpX, tmpY] = 255
                seeds.append((tmpX, tmpY))
    return seedMark








#src=cv.imread("apples/5.png")
src=ImageRGB
zhk=src.copy()
zh=src.copy()
#src=BGR_equalHist(src)#减少光照影响
hsv=cv.cvtColor(src,cv.COLOR_BGR2HSV)
gray=cv.cvtColor(src,cv.COLOR_BGR2GRAY)
(B,G,R) = cv.split(src)
(H,S,V) = cv.split(hsv)
cv.namedWindow("zhk",cv.WINDOW_AUTOSIZE)
hsv = cv.cvtColor(src, cv.COLOR_BGR2HSV)

#hsv = cv.medianBlur(hsv, 5)#减少光照影响


# lower_green=np.array([35,43,46])
# high_green=np.array([77,255,255])
# mask_green=cv.inRange(hsv,lower_green,high_green)
# green=cv.bitwise_and(src,src,mask=mask_green)
# cv.imshow("green",green)



lower_hsv=np.array([156,43,46])
high_hsv=np.array([180,255,255])
mask=cv.inRange(hsv,lower_hsv,high_hsv)
seeds = []  # 为了记录种子坐标
X_seed, Y_seed  = np.where(mask > 0)  # mask中值为255的像素的坐标
cv.imshow("zz",mask)
for i in range(X_seed.shape[0]):
    seeds.append((X_seed[i], Y_seed[i]))  # 将种子坐标写入seeds

seedMark = regionGrow(H, seeds, thresh=1,p=4)#对于局部苹果8邻域较好，对于全景苹果4邻域较好。选择H空间减少光照影响


kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
kernel2 = cv.getStructuringElement(cv.MORPH_RECT, (7, 7))
seedMark = cv.morphologyEx(seedMark, cv.MORPH_DILATE, kernel,iterations=1)
seedMark = cv.morphologyEx(seedMark, cv.MORPH_OPEN, kernel2,iterations=1)
cv.imshow("seedMark",seedMark)

cv.imshow("quyushengzhang",seedMark)
print(seedMark.dtype)
print(mask.dtype)
seedMark=np.uint8(seedMark)




heigh,wide= seedMark.shape
seedMark[0,0:wide]=0
seedMark[:,0]=0
seedMark[:,wide-1]=0
seedMark[heigh-1,:]=0
cv.imshow("zzza",seedMark)




tianchong=FillHole(seedMark)

dst=cv.bitwise_and(src,src,mask=tianchong)
cv.imshow("frame",dst)







tianchong=cv.cvtColor(tianchong,cv.COLOR_GRAY2BGR)
src=tianchong
reasure_object(src)











cv.waitKey(0)



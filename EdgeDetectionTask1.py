
# coding: utf-8

# In[ ]:
import copy
import cv2
import numpy as np
#Reading the image in grey scale
img = cv2.imread("task1.png", 0)
#Converting the image into list from numpy array
image=list(img)
#Kernels for x and y direction
xkernel=[[-1,0,1], [-2,0,2], [-1,0,1]]
ykernel=[[-1,-2,-1],[0,0,0],[1,2,1]]
#Convolution function
def convolutionfunc(image,kernel):
    rows=len(image)
    cols=len(image[0])
    krows=len(kernel)
    kcols=len(kernel[0])
    #creating a copy list of the same size as image
    #to capture the result after convolution
    output = copy.deepcopy(image)
    # find center position of kernel (half of kernel size)
    kCenterX = kcols // 2
    kCenterY = krows // 2
    for i in range(rows):
        for j in range(cols):
            sum=0
            for m in range(krows):
                mm = krows - 1 - m;
                for n in range(kcols):
                    nn = kcols - 1 - n
                    ii = i + (kCenterY - mm)
                    jj = j + (kCenterX - nn)
                    # ignoring input samples which are out of bound
                    if( ii >= 0 and ii < rows and jj >= 0 and jj < cols ):
                        sum += image[ii][jj] * kernel[mm][nn]
            if sum > 255:
                sum = 255
            if sum < 0:
                sum = 0
            output[i][j]=sum
    return output
#Calling the convolution function to get adges along the x-direction
gx=convolutionfunc(image,xkernel)
gx1=np.asarray(gx)
cv2.imshow('Edges along X direction', gx1)
cv2.imwrite('Edges_along_X_direction.png',gx1)
#Calling the convolution function to get adges along the y-direction
gy=convolutionfunc(image,ykernel)
gy1=np.asarray(gy)
cv2.imshow('Edges along Y direction', gy1)
cv2.imwrite('Edges_along_Y_direction.png',gy1)
cv2.waitKey(0)
cv2.destroyAllWindows()

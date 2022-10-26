from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
import rclpy
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import numpy as np

def canny(img):
    gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)
    canny = cv2.Canny(blur,50,150)
    return canny

def perspective_warp(img, 
                     dst_size=(1280,720),
                     src=np.float32([(0.43,0.65),(0.58,0.65),(0.1,1),(1,1)]),
                     dst=np.float32([(0,0), (1, 0), (0,1), (1,1)])):
    img_size = np.float32([(img.shape[1],img.shape[0])])
    src = src* img_size
    # For destination points, I'm arbitrarily choosing some points to be
    # a nice fit for displaying our warped result 
    # again, not exact, but close enough for our purposes
    dst = dst * np.float32(dst_size)
    # Given src and dst points, calculate the perspective transform matrix
    M = cv2.getPerspectiveTransform(src, dst)
    # Warp the image using OpenCV warpPerspective()
    warped = cv2.warpPerspective(img, M, dst_size)
    return warped

def inv_perspective_warp(img, 
                     dst_size=(1280,720),
                     src=np.float32([(0,0), (1, 0), (0,1), (1,1)]),
                     dst=np.float32([(0.43,0.65),(0.58,0.65),(0.1,1),(1,1)])):
    img_size = np.float32([(img.shape[1],img.shape[0])])
    src = src* img_size
    # For destination points, I'm arbitrarily choosing some points to be
    # a nice fit for displaying our warped result 
    # again, not exact, but close enough for our purposes
    dst = dst * np.float32(dst_size)
    # Given src and dst points, calculate the perspective transform matrix
    M = cv2.getPerspectiveTransform(src, dst)
    # Warp the image using OpenCV warpPerspective()
    warped = cv2.warpPerspective(img, M, dst_size)
    return warped

def pipeline(img, s_thresh=(120, 255), sx_thresh=(15, 255)):
    img = np.copy(img)
    # Convert to HLS color space and separate the V channel
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS).astype(np.float)
    l_channel = hls[:,:,1]
    s_channel = hls[:,:,2]
    h_channel = hls[:,:,0]
    # Sobel x
    sobelx = cv2.Sobel(l_channel, cv2.CV_64F, 1, 1) # Take the derivative in x
    abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    ret,thresh1 = cv2.threshold(scaled_sobel,100,255,cv2.THRESH_BINARY)
    return thresh1
    
'''    # Threshold x gradient
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1
    
    # Threshold color channel
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1
    
    color_binary = np.dstack((np.zeros_like(sxbinary), sxbinary, s_binary)) * 255
    
    combined_binary = np.zeros_like(sxbinary)
    combined_binary[(s_binary == 1) | (sxbinary == 1)] = 1'''


def roi(img):
    mask = np.zeros_like(img)
    region = np.array([[(25,500),(300,125),(400,125),(700,500),(0,500)]], dtype=np.int32)
    cv2.fillPoly(mask, region, 255)
    masked_image = cv2.bitwise_and(img,mask)
    return masked_image

def make_points(img,lineSI):
    slope,intercept = lineSI
    height = img.shape[0]
    y1 = int(height)
    y2 = int(y1*(2/5))
    x1 = int((y1-intercept)/slope)
    x2 = int((y2-intercept)/slope)
    return [x1,y1,x2,y2]

def average_slope_intercept(img,lines):
    left_fit = []
    right_fit = []
    for line in lines:
        for x1,y1,x2,y2 in line:
            fit = np.polyfit((x1,x2),(y1,y2),1)
            slope,intercept = fit[0],fit[1]
            if slope < 0:
                left_fit.append((slope,intercept))
            else :
                right_fit.append((slope,intercept))
    
    left_fit_avg = np.average(left_fit,axis=0)
    right_fit_avg = np.average(right_fit,axis=0)

    if len(left_fit) == 0:
        left_line = [img.shape[1],img.shape[0],img.shape[1],img.shape[0]]
        if len(right_fit) == 0:
            right_line = [500,500,500,500]    
            return (left_line,right_line)
        else:
            right_line = make_points(img,right_fit_avg)
            return (left_line,right_line)
    else:
        left_line = make_points(img,left_fit_avg)
        right_line = make_points(img,right_fit_avg)
        return (left_line,right_line)
    
def houghTransform(img):
    houghline = cv2.HoughLinesP(img, 2, np.pi/180,100, np.array([]),minLineLength = 40, maxLineGap = 5)
    line_img = np.zeros_like(img)
    left_line,right_line = average_slope_intercept(img,houghline)
    if (left_line is not None):
        cv2.line(line_img,(left_line[0],left_line[1]),(left_line[2],left_line[3]),(255,0,0),2)
        cv2.line(line_img,(right_line[0],right_line[1]),(right_line[2],right_line[3]),(255,0,0),2)
    return line_img
    '''if houghline is not None:
        for line in houghline:
            x1,y1,x2,y2 = line.reshape(4)
            cv2.line(line_img,(x1,y1),(x2,y2), (255,0,0),2)
    return line_img'''
   
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image,'/camera_follow/camera1/image_raw',self.listener_callback,10)
        self.br = CvBridge()

    def listener_callback(self,data):
        self.get_logger().info("Recieving video frames")
        current_frame = self.br.imgmsg_to_cv2(data)
        
        current_frame_RGB = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        dst = pipeline(current_frame_RGB)
        dst = perspective_warp(dst, dst_size=(current_frame_RGB.shape[1],current_frame_RGB.shape[0]))

        cv2.imshow("camera",houghTransform(roi(canny(current_frame))))
        #cv2.imshow("camera",roi(canny(current_frame)))
        #cv2.imshow("Warped_image", roi(dst))
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
        

from turtle import left
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
import rclpy
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import numpy as np
import math
from interface_pkg.msg import Lanedata

left_a, left_b, left_c = [],[],[]
right_a, right_b, right_c = [],[],[]

def sliding_window(img, nwindows=24, margin=150, minpix = 1, draw_windows=True):
    global left_a, left_b, left_c,right_a, right_b, right_c 
    left_fit_= np.empty(3)
    right_fit_ = np.empty(3)
    out_img = np.dstack((img, img, img))*255

    histogram = get_hist(img)
    # find peaks of left and right halves
    midpoint = int(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    
    
    # Set height of windows
    window_height = np.int(img.shape[0]/nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    
    
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = img.shape[0] - (window+1)*window_height
        win_y_high = img.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        # Draw the windows on the visualization image
        if draw_windows == True:
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),
            (100,255,255), 3) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),
            (100,255,255), 3) 
        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
        
        
#        if len(good_right_inds) > minpix:        
#            rightx_current = np.int(np.mean([leftx_current +900, np.mean(nonzerox[good_right_inds])]))
#        elif len(good_left_inds) > minpix:
#            rightx_current = np.int(np.mean([np.mean(nonzerox[good_left_inds]) +900, rightx_current]))
#        if len(good_left_inds) > minpix:
#            leftx_current = np.int(np.mean([rightx_current -900, np.mean(nonzerox[good_left_inds])]))
#        elif len(good_right_inds) > minpix:
#            leftx_current = np.int(np.mean([np.mean(nonzerox[good_right_inds]) -900, leftx_current]))


    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds] 

    # Fit a second order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    
    left_a.append(left_fit[0])
    left_b.append(left_fit[1])
    left_c.append(left_fit[2])
    
    right_a.append(right_fit[0])
    right_b.append(right_fit[1])
    right_c.append(right_fit[2])
    
    left_fit_[0] = np.mean(left_a[-10:])
    left_fit_[1] = np.mean(left_b[-10:])
    left_fit_[2] = np.mean(left_c[-10:])
    
    right_fit_[0] = np.mean(right_a[-10:])
    right_fit_[1] = np.mean(right_b[-10:])
    right_fit_[2] = np.mean(right_c[-10:])
    
    # Generate x and y values for plotting
    ploty = np.linspace(0, img.shape[0]-1, img.shape[0] )
    left_fitx = left_fit_[0]*ploty**2 + left_fit_[1]*ploty + left_fit_[2]
    right_fitx = right_fit_[0]*ploty**2 + right_fit_[1]*ploty + right_fit_[2]

    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 100]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 100, 255]
    
    return out_img, (left_fitx, right_fitx), (left_fit_, right_fit_), ploty

def get_curve(img, leftx, rightx):
    ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
    y_eval = np.max(ploty)
    ym_per_pix = 30.5/720 # meters per pixel in y dimension
    xm_per_pix = 3.7/720 # meters per pixel in x dimension

    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)
    # Calculate the new radii of curvature
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])

    car_pos = img.shape[1]/2
    l_fit_x_int = left_fit_cr[0]*img.shape[0]**2 + left_fit_cr[1]*img.shape[0] + left_fit_cr[2]
    r_fit_x_int = right_fit_cr[0]*img.shape[0]**2 + right_fit_cr[1]*img.shape[0] + right_fit_cr[2]
    lane_center_position = (r_fit_x_int + l_fit_x_int) /2
    center = (car_pos - lane_center_position) * xm_per_pix / 10
    # Now our radius of curvature is in meters
    return (left_curverad, right_curverad, center)

def draw_lanes(img, left_fit, right_fit):
    ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
    color_img = np.zeros_like(img)
    left = np.transpose(np.vstack([left_fit, ploty]))
    right = np.flipud(np.transpose(np.vstack([right_fit, ploty])))   
    left = np.array(left,np.int32)
    right = np.array(right,np.int32)
    
    if ((len(left)>100)):
        if (len(right)>100):
            left_endpoints = [left[100][:],left[0][:]]
            right_endpoints = [right[0][:],right[100][:]]
        else:
            left_endpoints = [left[100][:],left[0][:]]
            right_endpoints = None
    else:
        if (len(right)>100):
            left_endpoints = None
            right_endpoints = [right[0][:],right[100][:]]
        else:
            left_endpoints = None
            right_endpoints = None
                    
    
    #points = np.hstack((left, right))
    left_reshaped = left.reshape((-1,1,2))
    right_reshaped = right.reshape((-1,1,2))
    img = perspective_warp(img)
    #cv2.fillPoly(img, np.int_(points), (221,195,5))
    img = cv2.polylines(img,[left_reshaped],False,(221,195,5),3)
    img1 = cv2.polylines(img,[right_reshaped],False,(221,195,5),3)
    inv_perspective = inv_perspective_warp(img1)
    #inv_perspective = cv2.addWeighted(img, 1, inv_perspective, 0.7, 0)
    return inv_perspective,img1,left_endpoints,right_endpoints

def get_hist(img):
    hist = np.sum(img[img.shape[0]//2:,:], axis=0)
    return hist

def perspective_warp(img,
                     src=np.float32([[175,175],[0,400],[500,175],[650,400]]),
                     dst=np.float32([(0,0),(0,480),(640,0),(640,480)])):
    img_size = np.float32([(img.shape[1],img.shape[0])])
    dst_size=(img.shape[1],img.shape[0])
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, dst_size)
    return warped

def inv_perspective_warp(img,
                     src=np.float32([(0,0),(0,480),(640,0),(640,480)]),
                     dst=np.float32([[175,175],[0,400],[500,175],[650,400]])):
    img_size = np.float32([(img.shape[1],img.shape[0])])
    dst_size=(img.shape[1],img.shape[0])
    M = cv2.getPerspectiveTransform(src, dst)
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
    ret,thresh1 = cv2.threshold(scaled_sobel,40
                                ,255,cv2.THRESH_BINARY)
    #return thresh1
    # Threshold x gradient
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1
    
    # Threshold color channel
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1
    
    color_binary = np.dstack((np.zeros_like(sxbinary), sxbinary, s_binary)) * 255
    
    combined_binary = np.zeros_like(sxbinary)
    combined_binary[(s_binary == 1) | (sxbinary == 1)] = 100
    return combined_binary


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

def findlaneCurvature(x1,y1,x2,y2):
    offset_Vert=90# angle found by tan-1 (slop) is wrt horizontal --> This will shift to wrt Vetical

    if((x2-x1)!=0):
        slope = (y2-y1)/(x2-x1)
        y_intercept = y2 - (slope*x2) #y= mx+c
        anlgeOfinclination = math.atan(slope) * (180 / np.pi)#Conversion to degrees
    else:
        slope=1000#infinity
        y_intercept=0#None [Line never crosses the y axis]

        anlgeOfinclination = 90#vertical line

        #print("Vertical Line [Undefined slope]")
    if(anlgeOfinclination!=90):
        if(anlgeOfinclination<0):#right side
            angle_wrt_vertical = offset_Vert + anlgeOfinclination
        else:#left side
            angle_wrt_vertical = anlgeOfinclination - offset_Vert
    else:
        angle_wrt_vertical= 0#aligned
    return angle_wrt_vertical
   
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image,'/camera_follow/camera1/image_raw',self.listener_callback,10)
        self.br = CvBridge()
        self.publisher_ = self.create_publisher(Lanedata, 'lanedata_topic', 20)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.Done_sub = False

    def listener_callback(self,data):
        self.get_logger().info("Recieving video frames")
        current_frame = self.br.imgmsg_to_cv2(data)
        
        current_frame_RGB = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        thresh_img = pipeline(current_frame_RGB)
        dst1 = perspective_warp(thresh_img)
        out_img, curves, lanes, ploty = sliding_window(dst1)
        curverad=get_curve(current_frame_RGB, curves[0],curves[1])
        final_lane,unwarped,left_endpoints,right_endpoints = draw_lanes(current_frame_RGB, curves[0], curves[1])
        if ((left_endpoints != None) and (right_endpoints != None)):
            traj_highp = [int((left_endpoints[1][0] + right_endpoints[1][0])/2),int((left_endpoints[1][1] + right_endpoints[1][1])/2)]
            traj_lowp = [int((left_endpoints[0][0] + right_endpoints[0][0])/2),int((left_endpoints[0][1] + right_endpoints[0][1])/2)]
        
        self.perp_dist = current_frame.shape[1]/2 - traj_lowp[0]
        self.curvature = findlaneCurvature(traj_lowp[0],traj_lowp[1],traj_highp[0],traj_highp[1])
        final_lane = cv2.line(final_lane,traj_lowp,traj_highp,(0,0,255),3)
        final_lane = cv2.line(final_lane,(int(current_frame.shape[1]/2),current_frame.shape[0]),
                              (int(current_frame.shape[1]/2),current_frame.shape[0]-150),(0,0,255),3)
        text_in_img = 'distance: '+str(self.perp_dist)+' curvature: ' + str(self.curvature)
        final_lane = cv2.putText(final_lane,text_in_img,(20,20),
                                 cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
        self.Done_sub = True
        
        #plt.imshow(unwarped)
        #plt.show()
        cv2.imshow("thresh_image",thresh_img)
        cv2.imshow("Warped_image", dst1)
        cv2.imshow("final_unwarped",unwarped)
        cv2.imshow("final_lane",final_lane)
        cv2.waitKey(1)
    def timer_callback(self):
        if self.Done_sub:
            lane_msg = Lanedata()
            lane_msg.perpendicular_distance = self.perp_dist
            lane_msg.curvature = self.curvature
            self.publisher_.publish(lane_msg)
            self.get_logger().info('Publishing data')
            self.Done_sub = False
    
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
        

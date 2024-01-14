import rosbag
import cv2
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()

depth_range = 6.03

# groundtruth
gt_bag_file = "bag/indoor_flying3_depth_gt.bag"
gt_bag = rosbag.Bag(gt_bag_file,'r')
gt_data = gt_bag.read_messages('/davis/left/depth_image_rect')
gt_ts = []
gt_msgs = []

for _,msg,t in gt_data:
    gt_ts.append(t)
    gt_msgs.append(msg)
gt_bag.close()

print("Groundtruth done.")

# reference
est_bag_file_ref = "upenn3/upenn3_depthMap_ref.bag"
est_bag_ref = rosbag.Bag(est_bag_file_ref,'r')
est_data_ref = est_bag_ref.read_messages('/DepthMap_Ref')

gt_index = 0
errors_ref = []

for est_topic, est_msg, est_t in est_data_ref: # 遍历estimate
    est_img = bridge.imgmsg_to_cv2(est_msg) # estimate深度图
    while gt_index < len(gt_ts): # 查找最近的groundtruth
        gt_t = gt_ts[gt_index]
        if gt_t > est_t:
            break
        else:
            gt_index+=1
    if gt_index>0 and (est_t-gt_ts[gt_index-1]) < (gt_ts[gt_index]-est_t): # 选择最近的groundtruth
        gt_msg = gt_msgs[gt_index-1]
    else:
        gt_msg = gt_msgs[gt_index]
    gt_img = bridge.imgmsg_to_cv2(gt_msg) # groundtruth深度图
    
    height,width = est_img.shape
    for y in range(height):
        for x in range(width):
            est_value = est_img[y,x]
            gt_value = gt_img[y,x]
            if est_value>0 and est_value<=depth_range and gt_value!=0 and not np.isnan(gt_value):
                errors_ref.append(abs(est_value-gt_value))

est_bag_ref.close()
print("Reference done.")

# current
est_bag_file_cur = "upenn3/upenn3_depthMap_cur.bag"
est_bag_cur = rosbag.Bag(est_bag_file_cur,'r')
est_data_cur = est_bag_cur.read_messages('/DepthMap_Cur')

gt_index = 0
errors_cur = []

for est_topic, est_msg, est_t in est_data_cur: # 遍历estimate
    est_img = bridge.imgmsg_to_cv2(est_msg) # estimate深度图
    while gt_index < len(gt_ts): # 查找最近的groundtruth
        gt_t = gt_ts[gt_index]
        if gt_t > est_t:
            break
        else:
            gt_index+=1
    if gt_index>0 and (est_t-gt_ts[gt_index-1]) < (gt_ts[gt_index]-est_t): # 选择最近的groundtruth
        gt_msg = gt_msgs[gt_index-1]
    else:
        gt_msg = gt_msgs[gt_index]
    gt_img = bridge.imgmsg_to_cv2(gt_msg) # groundtruth深度图
    
    height,width = est_img.shape
    for y in range(height):
        for x in range(width):
            est_value = est_img[y,x]
            gt_value = gt_img[y,x]
            if est_value>0 and est_value<=depth_range and gt_value!=0 and not np.isnan(gt_value):
                errors_cur.append(abs(est_value-gt_value))

est_bag_cur.close()
print("Current done.")

sum_ref = sum(errors_ref)
sum_cur = sum(errors_cur)
# output
print("Reference depthMap error:")
print("\tMean error\t",sum_ref/len(errors_ref),"m")
print("\tMedian error\t",sorted(errors_ref)[len(errors_ref)//2],"m")
print("\tRelative error\t",sum_ref/len(errors_ref)/depth_range*100,"%")
print("Current depthMap error:")
print("\tMean error\t",sum_cur/len(errors_cur),"m")
print("\tMedian error\t",sorted(errors_cur)[len(errors_cur)//2],"m")
print("\tRelative error\t",sum_cur/len(errors_cur)/depth_range*100,"%")

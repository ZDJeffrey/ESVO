import rosbag
import numpy as np

_EPS = np.finfo(float).eps * 4.0

def transform44(position,orientation):
    qx, qy, qz ,qw = orientation
    x, y, z = position

    # Construct the rotation matrix from the unit quaternion
    rotation_matrix = np.array([[1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                                [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                                [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]])

    # Construct the homogeneous transformation matrix
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix
    homogeneous_matrix[:3, 3] = [x, y, z]

    return homogeneous_matrix

def transform_pose(homogeneous_matrix):
    position = homogeneous_matrix[:3, 3]
    rotation_matrix = homogeneous_matrix[:3, :3]

    # 将旋转矩阵转换为四元数
    qw = np.sqrt(1 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]) / 2
    qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4 * qw)
    qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4 * qw)
    qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4 * qw)

    quaternion = [qx, qy, qz, qw]

    return position, quaternion

names = ["rpg_bin","rpg_boxes","rpg_desk","rpg_monitor"]
T_rig_left = np.array((
    [5.36262328777285e-01, -1.748374625145743e-02, -8.438296573030597e-01, -7.009849865398374e-02],
    [8.433577587813513e-01, -2.821937531845164e-02, 5.366109927684415e-01, 1.881333563905305e-02],
    [-3.31943162375816e-02, -9.994488408486204e-01, -3.897382049768972e-04, -6.966829200678797e-02],
    [0.0, 0.0, 0.0, 1.0]
    ),dtype=np.float64)

R = T_rig_left[:3,:3]
U, S, Vt = np.linalg.svd(R)
R = np.dot(U,Vt)
T_rig_left[:3,:3] = R


for data_name in names:
    bag_file = "bag/"+data_name+"_edited.bag"
    output_file = "bag/"+data_name+"_groundtruth.tum"

    bag = rosbag.Bag(bag_file,'r')
    data = bag.read_messages('/optitrack/davis_stereo')
    output = open(output_file,'w')

    for _,msg,t in data:
        position = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z] # 坐标
        orientation = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w] # 四元数
        T_world_rig = transform44(position,orientation)
        T_world_left = np.dot(T_world_rig,T_rig_left)
        position,orientation = transform_pose(T_world_left)
        output.write(f"{t.to_sec():.18e} {position[0]:.18e} {position[1]:.18e} {position[2]:.18e} {orientation[0]:.18e} {orientation[1]:.18e} {orientation[2]:.18e} {orientation[3]:.18e}\n")

    bag.close()
    output.close()
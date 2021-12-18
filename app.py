import open3d as o3d
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from PIL import Image
  
def pointcloud(depth, fov):                                                             # -> 1 fov=40
    fy = fx = 0.5 / np.tan(fov * 0.5) # assume aspectRatio is one.
    height = depth.shape[0]
    print(height)
    width = depth.shape[1]
    print(width)
    mask = np.where(depth >=0)
    print("Length of mask:" +str(len(mask)))
    print(mask)
    x = mask[1]
    print(x)
    y = mask[0]
    
    normalized_x = (x.astype(np.float32) - width * 0.5) / width
    normalized_y = (y.astype(np.float32) - height * 0.5) / height
    
    world_x = normalized_x * depth[y, x] / fx
    world_y = normalized_y * depth[y, x] / fy
    world_z = depth[y, x]
    ones = np.ones(world_z.shape[0], dtype=np.float32)

    return np.vstack((world_x, world_y, world_z, ones)).T


with Image.open("unnamed.png") as im:
    rgbd_image=np.array(im)
    rgbd_image=rgbd_image[:,:,0]
    print(rgbd_image.shape)


point_cloud=pointcloud(rgbd_image,40)  ######
pcd = o3d.geometry.PointCloud()

pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])


o3d.visualization.draw_geometries([pcd])  # will not work with colab

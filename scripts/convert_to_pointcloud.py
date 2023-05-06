import copy
import json
from utils import *

ITODD = True
ITODD_VAL = False
TLESS = False

def iterative_segment_plane(pc):
    remain_pc = copy.deepcopy(pc)
    result_pc = o3d.geometry.PointCloud()
    while len(remain_pc.points)>100:
        print("剩余", len(remain_pc.points))
        _, ind = remain_pc.segment_plane(distance_threshold=0.5, ransac_n=3, num_iterations=100)
        next_remain_pc = remain_pc.select_by_index(ind, invert=True)
        plane_pc = remain_pc.select_by_index(ind, invert=False)
        plane_pc.paint_uniform_color(np.random.random(3))
        result_pc = result_pc + plane_pc
        remain_pc = next_remain_pc
    o3d.visualization.draw_geometries([result_pc])

if ITODD:
    # ITODD 原始
    with open('../data/itodd/itodd_test_bop19/test/000001/scene_camera.json', 'r', encoding='utf8') as fp:
        camera_info = json.load(fp)

    with open('../data/itodd/test_targets_bop19.json', 'r', encoding='utf8') as fp:
        test_targets = json.load(fp)

    color_foler = "../data/itodd/itodd_test_bop19/test/000001/gray/"
    depth_folder = "../data/itodd/itodd_test_bop19/test/000001/depth/"

if ITODD_VAL:
    # ITODD 真值
    with open('../data/itodd/itodd_val/val/000001/scene_camera.json', 'r', encoding='utf8') as fp:
        camera_info = json.load(fp)
    with open('../data/itodd/itodd_val/val/000001/scene_gt.json', 'r', encoding='utf8') as fp:
        val_gt = json.load(fp)
    # with open('itodd/itodd_val/val/000001/scene_gt_info.json','r',encoding='utf8') as fp:
    #     val_gt_info = json.load(fp)
    # with open('itodd/itodd_models/models_eval/models_info.json','r',encoding='utf8') as fp:
    #     models_info = json.load(fp)
    color_foler = "../data/itodd/itodd_val/val/000001/gray/"
    depth_folder = "../data/itodd/itodd_val/val/000001/depth/"
    model_folder = "../data/itodd/itodd_models/models_eval/"
    models = []
    for i in range(1,29):
        model = o3d.io.read_triangle_mesh(model_folder+'obj_%06d.ply'%i)
        models.append(model)


if TLESS:
# TLESS
    folder = 1
    with open('/home/zlt/Projects/Pose_estimation/data/tless/tless_test_primesense_bop19/test_primesense/%06d/scene_camera.json'%folder,'r',encoding='utf8') as fp:
        scene_camera_info = json.load(fp)
    color_foler = "../data/tless/tless_test_primesense_bop19/test_primesense/%06d/rgb/"%folder
    depth_folder = "../data/tless/tless_test_primesense_bop19/test_primesense/%06d/depth/"%folder


if __name__ == '__main__':
    if ITODD:
        # ITODD 原始
        for t in test_targets:
            scene_id = t['im_id']
            # if scene_id<100: continue
            camera_K = np.array(camera_info[str(scene_id)]['cam_K']).reshape(3, 3)
            depth_scale = camera_info[str(scene_id)]['depth_scale']
            color = cv2.imread(color_foler+'%06d.tif'%scene_id)
            depth = cv2.imread(depth_folder+'%06d.tif'%scene_id, cv2.IMREAD_UNCHANGED)

            # ITODD 原始数据显示
            pc = depth_to_pointcloud(depth, camera_K, color, mask=depth>500).scale(depth_scale, [0, 0, 0])
            o3d.visualization.draw_geometries([pc],  height=1200, width=1600,
                                              lookat=[ -0.04447, -4.90672, 723.05805],
                                              up=[ -0.00543, 0.99960, -0.02792],
                                              front=[ 0.10378, -0.02721, -0.99423], zoom=0.39)



        # ITODD 深度图边缘检测
        # pesudo_depth = depth.copy()
        # pesudo_depth[depth==0] = np.nan
        # d_max = np.nanmax(pesudo_depth)
        # d_min = np.nanmin(pesudo_depth)
        # pesudo_depth = (pesudo_depth - d_min) / (d_max - d_min) * 255
        # pesudo_depth = pesudo_depth.astype(np.uint8)
        #
        # pesudo_depth_blur = cv2.blur(pesudo_depth, (5,5))
        # edge = cv2.Canny(pesudo_depth_blur, 10, 20)
        # cv2.imshow('pesudo_depth', pesudo_depth)
        # cv2.imshow('edge', edge)
        # cv2.waitKey(0)

    if ITODD_VAL:
        # ITODD 真值显示
        for scene_id in val_gt:
            camera_K = np.array(camera_info[str(scene_id)]['cam_K']).reshape(3, 3)
            depth_scale = camera_info[str(scene_id)]['depth_scale']
            color = cv2.imread(color_foler+'%06d.tif'%int(scene_id))
            depth = cv2.imread(depth_folder+'%06d.tif'%int(scene_id), cv2.IMREAD_UNCHANGED)
            scene_pc = depth_to_pointcloud(depth, camera_K, color, mask=depth>500).scale(depth_scale, [0, 0, 0])

            show = []
            gts = val_gt[scene_id]
            for gt in gts:
                R = np.array(gt['cam_R_m2c']).reshape(3, 3)
                t = gt['cam_t_m2c']
                model_id = gt['obj_id']
                model_vis = copy.deepcopy(models[model_id-1]).paint_uniform_color([0,1,0])
                model_vis.rotate(R, center=[0,0,0])
                model_vis.translate(t)
                show.append(model_vis)

            o3d.visualization.draw_geometries(show+[scene_pc])

    if TLESS:
        # TLESS 原始
        for scene_id in scene_camera_info:
            camera_K = np.array(scene_camera_info[scene_id]['cam_K']).reshape(3, 3)
            depth_scale = scene_camera_info[scene_id]['depth_scale']
            color = cv2.imread(color_foler+'%06d.png'%int(scene_id))
            depth = cv2.imread(depth_folder+'%06d.png'%int(scene_id), cv2.IMREAD_UNCHANGED)

            # TLESS 原始数据显示
            pc = depth_to_pointcloud(depth, camera_K, color, mask=depth>500).scale(depth_scale, [0, 0, 0])
            o3d.visualization.draw_geometries([pc],  height=1200, width=1600,
                                              lookat=[ -0.04447, -4.90672, 723.05805],
                                              up=[ -0.00543, 0.99960, -0.02792],
                                              front=[ 0.10378, -0.02721, -0.99423], zoom=0.39)




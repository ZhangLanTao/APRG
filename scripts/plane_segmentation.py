import cv2

from utils import *

patch_sizes = [320, 160, 80, 40, 20, 10]
IMG_WID = 1280
IMG_HGT = 960

segment_result = []

# video_out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'XVID'), 15, (IMG_WID, IMG_HGT))

def segment(depth, img_show, upleft_xy, patch_size, camera_mtx=None, tree_level=None):
    points = depth_to_points(depth, camera_mtx, upleft_xy, patch_size)
    if len(points) < 50:
        return
    # pc = np_to_pcd(points)
    # _, ind = pc.segment_plane(distance_threshold=2,ransac_n=3, num_iterations=10)
    # 协方差矩阵
    cov = np.cov(points, rowvar=False)
    # 特征值和特征向量
    eig_val, eig_vec = np.linalg.eig(cov)

    # 特征值从小到大排序
    eig_val = np.sort(eig_val)

    # print("特征值", eig_val)
    # print('平面度', eig_val[1]/eig_val[0])
    # print('最小特征值', eig_val[0])
    # o3d.visualization.draw_geometries([pc], height=1200, width=1600,)
    # if len(ind)/len(points) > 0.99:
    h = w = patch_size
    if eig_val[0] < 0.1 and eig_val[1]/eig_val[0] > 50:
        cv2.rectangle(img_show, upleft_xy, (upleft_xy[0]+w, upleft_xy[1]+h), (0, 0, 255), 2)
        print(tree_level)
        print('最小特征值', eig_val[0])
        print('平面度', eig_val[1]/eig_val[0])
        segment_result.append(tree_level)
        cv2.imshow('img_show', img_show)
        cv2.waitKey(0)
        return
    # else:
    #     cv2.rectangle(img_show, upleft_xy, (upleft_xy[0]+w, upleft_xy[1]+h), (0, 255, 0), 2)
    # 递归分割
    segment(depth, img_show, upleft_xy, patch_size//2, camera_mtx, tree_level+[0])
    segment(depth, img_show, (upleft_xy[0]+w//2, upleft_xy[1]), patch_size//2, camera_mtx, tree_level+[1])
    segment(depth, img_show, (upleft_xy[0], upleft_xy[1]+h//2), patch_size//2, camera_mtx, tree_level+[2])
    segment(depth, img_show, (upleft_xy[0]+w//2, upleft_xy[1]+h//2), patch_size//2, camera_mtx, tree_level+[3])


def draw_patch_by_name(img_show, hirachical_index):
    (upleft_x, upleft_y), level = convert_hirachical_index_to_xy_level_index(hirachical_index)
    size = patch_sizes[level]
    cv2.rectangle(img_show, (upleft_x, upleft_y), (upleft_x + size, upleft_y+size), (0,233,0), -1)

def find_index_in_indexes(index, indexes):
    for length in range(1,len(index)+1):
        if index[0:length] in indexes:
            return index[0:length]
    return None

def get_possible_neighbor_level_i(start_index, i):
    neighbors = []
    (x, y), l = convert_hirachical_index_to_xy_level_index(start_index)
    y_nb_u = y - patch_sizes[i]  # 上方
    if y_nb_u >= 0:
        for x_nb_u in range(x, x+patch_sizes[l], patch_sizes[i]):
            neighbors.append(convert_xy_level_index_to_hirachical_index((x_nb_u, y_nb_u), i))

    y_nb_d = y + patch_sizes[l] # 下方
    if y_nb_d < IMG_HGT:
        for x_nb_d in range(x, x + patch_sizes[l], patch_sizes[i]):
            neighbors.append(convert_xy_level_index_to_hirachical_index((x_nb_d, y_nb_d), i))

    x_nb_l = x-patch_sizes[i]   # 左侧
    if x_nb_l >= 0:
        for y_nb_l in range(y, y + patch_sizes[l], patch_sizes[i]):
            neighbors.append(convert_xy_level_index_to_hirachical_index((x_nb_l, y_nb_l), i))

    x_nb_r = x+patch_sizes[l]   # 右侧
    if x_nb_r < IMG_WID:
        for y_nb_l in range(y, y + patch_sizes[l], patch_sizes[i]):
            neighbors.append(convert_xy_level_index_to_hirachical_index((x_nb_r, y_nb_l), i))

    return neighbors


def get_possible_neighbor(start_index):
    neighbors = []
    for i in range(len(start_index)-1, 6):
        neighbors_i = get_possible_neighbor_level_i(start_index, i)
        if len(neighbors_i)>0:
            neighbors += neighbors_i
    return neighbors

def convert_hirachical_index_to_xy_level_index(hirachical_index):
    upleft_x = 0
    upleft_y = 0
    for i, ind in enumerate(hirachical_index):
        if i == 0:
            upleft_x += patch_sizes[i] * (ind % 4)
            upleft_y += patch_sizes[i] * (ind // 4)
        else:
            upleft_x += patch_sizes[i] * (ind % 2)
            upleft_y += patch_sizes[i] * (ind // 2)
    level = len(hirachical_index) - 1
    return (upleft_x, upleft_y), level

def convert_xy_level_index_to_hirachical_index(xy, level):
    x, y = xy
    hirachical_index = []
    hirachical_index.append( 4*(y//patch_sizes[0])+(x//patch_sizes[0]) )
    x = x%patch_sizes[0]
    y = y%patch_sizes[0]
    for i in range(1, level+1):
        hirachical_index.append(2 * (y // patch_sizes[i]) + (x // patch_sizes[i]))
        x = x % patch_sizes[i]
        y = y % patch_sizes[i]
    return hirachical_index

visited = []
def reigon_growing(img_show, segment_result, start_index):
    visited.append(start_index)
    neighbors = get_possible_neighbor(start_index)
    for n in neighbors:
        # img_show_copy = img_show.copy()
        if find_index_in_indexes(n, visited) is not None:
            continue
        else:
            result = find_index_in_indexes(n, segment_result)
            if result is not None:
                draw_patch_by_name(img_show, result)
                cv2.imshow("neighbor", img_show)
                video_out.write(img_show)
                cv2.waitKey(1)
                reigon_growing(img_show, segment_result, result)


# ITODD 原始
with open('../data/itodd/itodd_test_bop19/test/000001/scene_camera.json', 'r', encoding='utf8') as fp:
    camera_info = json.load(fp)

with open('../data/itodd/test_targets_bop19.json', 'r', encoding='utf8') as fp:
    test_targets = json.load(fp)

color_foler = "../data/itodd/itodd_test_bop19/test/000001/gray/"
depth_folder = "../data/itodd/itodd_test_bop19/test/000001/depth/"

for t in test_targets:
    segment_result = []
    scene_id = t['im_id']
    camera_K = np.array(camera_info[str(scene_id)]['cam_K']).reshape(3, 3)
    depth_scale = camera_info[str(scene_id)]['depth_scale']
    color = cv2.imread(color_foler + '%06d.tif' % scene_id)
    depth = cv2.imread(depth_folder + '%06d.tif' % scene_id, cv2.IMREAD_UNCHANGED)

    tic = time.time()
    h, w = depth.shape
    patch_size = h // 3
    for i in range(12):
        segment(depth, color, (patch_size * (i % 4), patch_size * (i // 4)), patch_size, camera_K, [i])
    toc = time.time()
    print('time: ', toc-tic)

    # cv2.imwrite('../data/out/%06d.png'%scene_id, color)




    cv2.imshow('color', color)
    key = cv2.waitKey(0)
    if key == ord('n'):
        continue

    largest_patch = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    for r in segment_result:
        if len(r)<len(largest_patch):
            largest_patch = r
    draw_patch_by_name(color, largest_patch)
    cv2.imshow("startfrom", color)
    # video_out.write(color)
    cv2.waitKey()
    visited = []
    reigon_growing(color, segment_result, largest_patch)

    # video_out.release()


    # ITODD 原始数据显示
    # pc = depth_to_pointcloud(depth, camera_K, color, mask=depth > 500).scale(depth_scale, [0, 0, 0])
    # o3d.visualization.draw_geometries([pc], height=1200, width=1600,
    #                                   lookat=[-0.04447, -4.90672, 723.05805],
    #                                   up=[-0.00543, 0.99960, -0.02792],
    #                                   front=[0.10378, -0.02721, -0.99423], zoom=0.39)

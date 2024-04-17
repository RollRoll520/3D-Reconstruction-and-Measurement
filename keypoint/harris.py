import open3d as o3d
import numpy as np
import time

# This function is only used to make the keypoints look better on the rendering
def keypoints_to_spheres(keypoints):
    spheres = o3d.geometry.TriangleMesh()
    for keypoint in keypoints.points:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
        sphere.translate(keypoint)
        spheres += sphere
    spheres.paint_uniform_color([1, 0, 0])
    return spheres

def compute_harris3d_keypoints( pcd, radius=0.01, max_nn=10, threshold=0.001 ):
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    harris = np.zeros( len(np.asarray(pcd.points)) )
    is_active = np.zeros( len(np.asarray(pcd.points)), dtype=bool )

    # Compute Harris response
    for i in range( len(np.asarray(pcd.points)) ):
        [num_nn, inds, _] = pcd_tree.search_knn_vector_3d(pcd.points[i], max_nn)
        pcd_normals = pcd.select_by_index(inds)
        pcd_normals.points = pcd_normals.normals
        [_, covar] = pcd_normals.compute_mean_and_covariance()
        harris[ i ] = np.linalg.det( covar ) / np.trace( covar )
        if (harris[ i ] > threshold):
            is_active[ i ] = True

    # NMS
    for i in range( len(np.asarray(pcd.points)) ):
        if is_active[ i ]:
            [num_nn, inds, _] = pcd_tree.search_knn_vector_3d(pcd.points[i], max_nn)
            inds.pop( harris[inds].argmax() )
            is_active[ inds ] = False

    keypoints = pcd.select_by_index(np.where(is_active)[0])
    return keypoints

def harris(filename):
    print("Loading a point cloud from", filename)
    pcd = o3d.io.read_point_cloud(filename)
    print(pcd)
    tic = time.time()
    keypoints = compute_harris3d_keypoints( pcd,0.05,10,0.0005 )
    print(keypoints)
    toc = 1000 * (time.time() - tic)
    print("KeyPoints Computation took {:.0f} [ms]".format(toc))
    pcd.paint_uniform_color([1, 1, 1])
    o3d.visualization.draw_geometries([keypoints_to_spheres(keypoints), pcd])

#示例代码
# harris(reconstructed_file_env)
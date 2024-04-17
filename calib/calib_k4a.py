import cv2
from pykinect_azure import * 
import calib.calib_extrinsic as calib_extrinsic
from pykinect_azure.k4a import _k4a as k4a

def CopyIntrinsics(from_intrinsics, to_intrinsics):
    # print("-------Start CopyIntrinsics-------")
    to_intrinsics.Width = from_intrinsics.resolution_width
    to_intrinsics.Height = from_intrinsics.resolution_height

    params = from_intrinsics.intrinsics.parameters
    to_intrinsics.cx = params.param.cx
    to_intrinsics.cy = params.param.cy
    to_intrinsics.fx = params.param.fx
    to_intrinsics.fy = params.param.fy
    to_intrinsics.k[0] = params.param.k1
    to_intrinsics.k[1] = params.param.k2
    to_intrinsics.k[2] = params.param.k3
    to_intrinsics.k[3] = params.param.k4
    to_intrinsics.k[4] = params.param.k5
    to_intrinsics.k[5] = params.param.k6
    to_intrinsics.codx = params.param.codx
    to_intrinsics.cody = params.param.cody
    to_intrinsics.p1 = params.param.p1
    to_intrinsics.p2 = params.param.p2


def CalibrationFromK4a(from_calibration, to_calibration):
    CopyIntrinsics(from_calibration.depth_camera_calibration, to_calibration.Depth)
    CopyIntrinsics(from_calibration.color_camera_calibration, to_calibration.Color)

    # Extrinsics from depth to color camera
    extrinsics = from_calibration.extrinsics[k4a.K4A_CALIBRATION_TYPE_DEPTH][k4a.K4A_CALIBRATION_TYPE_COLOR]
    for i in range(3):
        for j in range(3):
            to_calibration.RotationFromDepth[i][j] = extrinsics.rotation[3*i+j]
    
    for i in range(3):
        to_calibration.TranslationFromDepth[i] = extrinsics.translation[i]
    # print("------Finish CalibrationFromK4a------")

class Recording:
    def __init__(self):
        self.filename = None
        self.handle = None
        self.record_config = None
        self.k4a_calibration = None
        self.k4a_transformation = None
        self.calibration = calib_extrinsic.CameraCalibration()
        self.capture = None

def first_capture_timestamp(capture):
    min_timestamp = 2**64 - 1
    images = [None] * 3
    images[0] = k4a.k4a_capture_get_color_image(capture)
    images[1] = k4a.k4a_capture_get_depth_image(capture)
    images[2] = k4a.k4a_capture_get_ir_image(capture)

    for i in range(3):
        if images[i] is not None:
            timestamp = k4a.k4a_image_get_device_timestamp_usec(images[i])
            if timestamp < min_timestamp:
                min_timestamp = timestamp
            k4a.k4a_image_release(images[i])
            images[i] = None

    return min_timestamp


def print_calibration(calibration):
    print("Depth camera:")
    calib = calibration.depth_camera_calibration
    print("resolution width:", calib.resolution_width)
    print("resolution height:", calib.resolution_height)
    print("principal point x:", calib.intrinsics.parameters.param.cx)
    print("principal point y:", calib.intrinsics.parameters.param.cy)
    print("focal length x:", calib.intrinsics.parameters.param.fx)
    print("focal length y:", calib.intrinsics.parameters.param.fy)
    print("radial distortion coefficients:")
    print("k1:", calib.intrinsics.parameters.param.k1)
    print("k2:", calib.intrinsics.parameters.param.k2)
    print("k3:", calib.intrinsics.parameters.param.k3)
    print("k4:", calib.intrinsics.parameters.param.k4)
    print("k5:", calib.intrinsics.parameters.param.k5)
    print("k6:", calib.intrinsics.parameters.param.k6)
    print("center of distortion in Z=1 plane, x:", calib.intrinsics.parameters.param.codx)
    print("center of distortion in Z=1 plane, y:", calib.intrinsics.parameters.param.cody)
    print("tangential distortion coefficient x:", calib.intrinsics.parameters.param.p1)
    print("tangential distortion coefficient y:", calib.intrinsics.parameters.param.p2)
    print("metric radius:", calib.intrinsics.parameters.param.metric_radius)

    print("Color camera:")
    calib = calibration.color_camera_calibration
    print("resolution width:", calib.resolution_width)
    print("resolution height:", calib.resolution_height)
    print("principal point x:", calib.intrinsics.parameters.param.cx)
    print("principal point y:", calib.intrinsics.parameters.param.cy)
    print("focal length x:", calib.intrinsics.parameters.param.fx)
    print("focal length y:", calib.intrinsics.parameters.param.fy)
    print("radial distortion coefficients:")
    print("k1:", calib.intrinsics.parameters.param.k1)
    print("k2:", calib.intrinsics.parameters.param.k2)
    print("k3:", calib.intrinsics.parameters.param.k3)
    print("k4:", calib.intrinsics.parameters.param.k4)
    print("k5:", calib.intrinsics.parameters.param.k5)
    print("k6:", calib.intrinsics.parameters.param.k6)
    print("center of distortion in Z=1 plane, x:", calib.intrinsics.parameters.param.codx)
    print("center of distortion in Z=1 plane, y:", calib.intrinsics.parameters.param.cody)
    print("tangential distortion coefficient x:", calib.intrinsics.parameters.param.p1)
    print("tangential distortion coefficient y:", calib.intrinsics.parameters.param.p2)
    print("metric radius:", calib.intrinsics.parameters.param.metric_radius)

    extrinsics = calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR]
    print("depth2color translation: ({},{},{})".format(extrinsics.translation[0], extrinsics.translation[1], extrinsics.translation[2]))
    print("depth2color rotation: |{},{},{}|".format(extrinsics.rotation[0], extrinsics.rotation[1], extrinsics.rotation[2]))
    print("                      |{},{},{}|".format(extrinsics.rotation[3], extrinsics.rotation[4], extrinsics.rotation[5]))
    print("                      |{},{},{}|".format(extrinsics.rotation[6], extrinsics.rotation[7], extrinsics.rotation[8]))



def process_capture(file,frame):
    images = [None, None]
    images[0] = k4a.k4a_capture_get_color_image(file.capture._handle)
    images[1] = k4a.k4a_capture_get_depth_image(file.capture._handle)

    # Copy color
    frame.ColorWidth = k4a.k4a_image_get_width_pixels(images[0])
    frame.ColorHeight = k4a.k4a_image_get_height_pixels(images[0])
    frame.ColorStride = k4a.k4a_image_get_stride_bytes(images[0])
    color_image = k4a.k4a_image_get_buffer(images[0])
    color_size = k4a.k4a_image_get_size(images[0])
    color_buffer = ctypes.cast(color_image, ctypes.POINTER(ctypes.c_uint8 * color_size))
    color_np = np.frombuffer(color_buffer.contents, dtype=np.uint8).reshape((frame.ColorHeight, frame.ColorWidth,4)).copy()
    frame.ColorImage = color_np

    # Copy depth
    frame.DepthWidth = k4a.k4a_image_get_width_pixels(images[1])
    frame.DepthHeight = k4a.k4a_image_get_height_pixels(images[1])
    frame.DepthStride = k4a.k4a_image_get_stride_bytes(images[1])
    depth_size = frame.DepthStride * frame.DepthHeight
    depth_image = k4a.k4a_image_get_buffer(images[1])
    # 创建一个 ctypes 指向 depth_image 的指针
    depth_ptr = ctypes.cast(depth_image, ctypes.POINTER(ctypes.c_uint16))
    # 将数据复制到 frame.DepthImage 中
    frame.DepthImage = bytearray(ctypes.string_at(depth_ptr, depth_size))

    transformed_depth_image = Image.create(k4a.K4A_IMAGE_FORMAT_DEPTH16,
                                                   frame.ColorWidth,
                                                   frame.ColorHeight,
                                                   frame.ColorWidth * ctypes.sizeof(ctypes.c_uint16),)
        
    k4a.k4a_transformation_depth_image_to_color_camera(file.k4a_transformation, images[1], transformed_depth_image._handle)

    
    point_cloud_image = Image.create(K4A_IMAGE_FORMAT_CUSTOM,
                        frame.ColorWidth,
                        frame.ColorHeight,
                        frame.ColorWidth * 3 * ctypes.sizeof(ctypes.c_int16))



    if k4a.k4a_transformation_depth_image_to_point_cloud(file.k4a_transformation, transformed_depth_image._handle, K4A_CALIBRATION_TYPE_COLOR, point_cloud_image._handle) != K4A_RESULT_SUCCEEDED:
        print("Failed to compute point cloud image")

    # Copy point cloud
    cloud_size = k4a.k4a_image_get_size(point_cloud_image._handle)
 
    point_cloud_image_data = k4a.k4a_image_get_buffer(point_cloud_image._handle)
    frame.PointCloudData= bytearray(cloud_size)
    frame.PointCloudData[:cloud_size] =point_cloud_image_data[:cloud_size] 

    # k4a.k4a_image_release(transformed_depth_image._handle)
    # k4a.k4a_image_release(point_cloud_image._handle)
    transformed_depth_image.reset()
    point_cloud_image.reset


    print("%-32s" % file.filename, end="")
    for i in range(2):
        if images[i] is not None:
            timestamp = k4a.k4a_image_get_device_timestamp_usec(images[i])
            print("  %7d usec" % timestamp, end="")
            k4a.k4a_image_release(images[i])
            images[i] = None
        else:
            print("  %12s" % "", end="")
    print()

    return frame
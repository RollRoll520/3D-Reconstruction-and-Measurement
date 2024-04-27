# Reference

---

## K4aRecorder

```cmd
k4arecorder.exe -l 5 D:\Knowledge\Graduation_Design\WorkSpace\Resources\output-2.mkv

Options:
  -h, --help              Prints this help
  --list                  List the currently connected K4A devices
  --device                Specify the device index to use (default: 0)
  -l, --record-length     Limit the recording to N seconds (default: infinite)
  -c, --color-mode        Set the color sensor mode (default: 1080p), Available options:
                            3072p, 2160p, 1536p, 1440p, 1080p, 720p, 720p_NV12, 720p_YUY2, OFF
  -d, --depth-mode        Set the depth sensor mode (default: NFOV_UNBINNED), Available options:
                            NFOV_2X2BINNED, NFOV_UNBINNED, WFOV_2X2BINNED, WFOV_UNBINNED, PASSIVE_IR, OFF
  --depth-delay           Set the time offset between color and depth frames in microseconds (default: 0)
                            A negative value means depth frames will arrive before color frames.
                            The delay must be less than 1 frame period.
  -r, --rate              Set the camera frame rate in Frames per Second
                            Default is the maximum rate supported by the camera modes.
                            Available options: 30, 15, 5
  --imu                   Set the IMU recording mode (ON, OFF, default: ON)
  --external-sync         Set the external sync mode (Master, Subordinate, Standalone default: Standalone)
  --sync-delay            Set the external sync delay off the master camera in microseconds (default: 0)
                            This setting is only valid if the camera is in Subordinate mode.
  -e, --exposure-control  Set manual exposure value (-11 to 1) for the RGB camera (default: auto exposure)

```

You can create aruco and apriltag board image at [aruco board](https://www.2weima.com/aruco.html)

## Calibration

This extrinsic calibration is a Python implementation based on the article [Extrinsic Calibration of Multiple Azure Kinect Cameras](https://tianyusong.com/2021/06/04/multiple-azure-kinect-extrinsic-calibration/).
To perform the calibration, run the following command:

```cmd
python calib/calib.py
```

## Kinect Fusion

The implementation of Kinect Fusion is referred [the repository by JinWenWang](https://github.com/JingwenWang95/KinectFusion/tree/master). Please refer to the repository for detailed prerequisites.

### Data Preparation

To obtain the open-source dataset, you can download it from [TUM dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset/download). After downloading the raw sequences, you will need to run the pre-processing script under `dataset/`. For example:

```cmd
python dataset/preprocess.py --config configs/fr1_desk.yaml
python dataset/preprocess.py --config configs/fr1_metallic_sphere.yaml
```

### Run Kinect Fusion

After obtaining the processed sequence,you can simply run kinfu.py,**which will perform the tracking and mapping headlessly and save the results.**

For example:

```cmd
python kinfu.py --config configs/fr1_desk.yaml --save_dir reconstruct/fr1_desk
python kinfu.py --config configs/fr1_metallic_sphere.yaml --save_dir reconstruct/fr1_metallic_sphere
```

 Or if you want to visualize the tracking and reconstruction process on-the-fly.
 run the following command:

```cmd
python kinfu_gui.py --config configs/fr1_desk.yaml
```

# Reference

---

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
```

### Run Kinect Fusion

After obtaining the processed sequence,you can simply run kinfu.py,**which will perform the tracking and mapping headlessly and save the results.**

For example:

```cmd
python dataset/preprocess.py --config configs/fr1_desk.yaml
```

 Or if you want to visualize the tracking and reconstruction process on-the-fly.
 run the following command:

```cmd
python kinfu_gui.py --config configs/fr1_desk.yaml
```

from typing import List
import cv2
from pykinect_azure import *
from calib.calib_extrinsic import *
import calib.calib_k4a as calib_k4a
from pykinect_azure.k4a import _k4a as k4a


def calib(mkv1_file_env,mkv2_file_env) -> int:
    filePaths:List[str] = [mkv1_file_env,
                           mkv2_file_env]


    file_count = len(filePaths) - 1
    master_found = False
    result = K4A_RESULT_SUCCEEDED
    initialize_libraries()

    # Allocate memory to store the state of N recordings.
    files = [calib_k4a.Recording() for _ in range(file_count)]
    playbacks: List[Playback] = []
    for i in range(file_count):
        files[i].filename = filePaths[i]
        playback = Playback(files[i].filename)
        if(playback.is_valid):
            playbacks.append(playback)
        else:
            print(f"ERROR: Failed To Open : {files[i].filename}")
            return -1
    
    try:
        print("----Open each recording file and validate they were recorded in master/subordinate mode.")
        for i in range(file_count):
            playbacks[i].open(files[i].filename)

            files[i].record_config = playbacks[i].get_record_configuration()._handle

            #Attentions!!:You need to add some code at "playback.py:50" to get full calibration
            # def get_full_calibration(self):
		    #     calibration_handle = _k4arecord.k4a_calibration_t()
		    #     if self.is_valid():		
			#         _k4arecord.VERIFY(_k4arecord.k4a_playback_get_calibration(self._handle, calibration_handle),"Failed to read device calibration from recording!")
		    #     return calibration_handle
            files[i].k4a_calibration = playbacks[i].get_full_calibration()

            calib_k4a.print_calibration(files[i].k4a_calibration)
            calib_k4a.CalibrationFromK4a(files[i].k4a_calibration, files[i].calibration)
            files[i].k4a_transformation = k4a.k4a_transformation_create(files[i].k4a_calibration)
            if not files[i].k4a_transformation:
                print("Transformation failed")


            if files[i].record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER:
                print(f"Opened master recording file: {files[i].filename}")
                if master_found:
                    print("ERROR: Multiple master recordings listed!")
                    result = K4A_RESULT_FAILED
                    break
                else:
                    master_found = True
            elif files[i].record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE:
                print(f"Opened subordinate recording file: {files[i].filename}")
            else:
                print(f"Warning:Recording file was not recorded in master/sub mode: {files[i].filename}")
                # result = K4A_RESULT_FAILED
                # break

            playbacks[i].set_color_conversion( k4a.K4A_IMAGE_FORMAT_COLOR_BGRA32)
            
            for j in range(30):
                # Skip the first 30 capture of each recording.
                stream_result,files[i].capture = playbacks[i].get_next_capture()

                if not stream_result:
                    print(f"ERROR: Recording file is empty: {files[i].filename}")
                    result = K4A_RESULT_FAILED
                    break
                # k4a.k4a_capture_release(files[i].capture._handle)
            

        if result == K4A_RESULT_SUCCEEDED:
            print("%-32s  %12s  %12s  %12s" % ("Source file", "COLOR", "DEPTH", "IR"))
            print("==========================================================================")

            terminated = False

            extrinsicsCalib = ExtrinsicsCalibration()

            while not terminated:
                frames: List[FrameInfo] = []
                for i in range(file_count):
                    frame = FrameInfo()
                    frames.append(frame)
                extrinsics = []

                # Find the lowest timestamp out of each of the current captures.
                for i in range(file_count):
                    stream_result,files[i].capture = playbacks[i].get_next_capture()
                    if stream_result:
                        min_file = files[i]
                        calib_k4a.process_capture(min_file,frames[i])
                                              
                        frames[i].Calibration = files[i].calibration
                        imu_sample = k4a_imu_sample_t
                        imu_sample = playbacks[i].get_next_imu_sample()
                        frames[i].Accelerometer = [imu_sample._struct.acc_sample.v[0],
                                               imu_sample._struct.acc_sample.v[1],
                                               imu_sample._struct.acc_sample.v[2]]
                        frames[i].CameraIndex = i + 1
                        frames[i].filename = files[i].filename  


                        transform = AlignmentTransform()
                        transform.Identity = True
                        extrinsics.append(transform)

                    else:
                        print(f"ERROR: Failed to read capture from file: {files[i].filename}")
                        result = K4A_RESULT_FAILED 
                        terminated = True
                        break
                    k4a.k4a_capture_release(files[i].capture._handle)                    

                if result != K4A_RESULT_SUCCEEDED:
                    break
                
                # cv2.imshow("Color Image", frames[0].ColorImage)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()

                extrinsics = extrinsicsCalib.calculate_extrinsics(frames)
                if(len(extrinsics)==0):
                    print(f"ERROR: Full registration failed")
                else :
                    break

                
            print("==========================================================================")

            # if result == K4A_RESULT_SUCCEEDED:
            #     # Save the extrinsics calibration to a file.
            #     extrinsicsCalib.save_to_file("extrinsics_calib.txt")

        for i in range(file_count):
            playbacks[i].close()

    except Exception as e:
        print(f"ERROR: Exception occurred: {str(e)}")
        result = K4A_RESULT_FAILED

    return result == K4A_RESULT_SUCCEEDED
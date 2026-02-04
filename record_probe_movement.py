from sksurgerynditracker.nditracker import NDITracker
import ndicapy
import os
import time
import numpy as np
from scipy.spatial.transform import Rotation


def conect_tracker():
    '''
    Configures the connection to the camera and initiation of the tracking

    Args : none

    Output : NDITracker - camera comunication object
    '''
    SETTINGS = {
        "tracker type": "vega",
        "ip address": "192.168.0.120",
        "romfiles" : ['/home/racunalo/KUKA_LBRMed_servoing/markeri/8700338.rom'],
        
        }
    tracker = NDITracker(SETTINGS)

    ndicapy.ndiCommand(tracker._device,'SET Param.Tracking.Track Frequency=2')
    

    tracker.start_tracking()
    return tracker





def main():
    tracker = conect_tracker()
    time.sleep(2)
    prev_frame = 0
    #os.makedirs('/home/racunalo/KUKA_LBRMed_servoing/nditracker_logs/', exist_ok=True)
    os.makedirs('/home/racunalo/KUKA_LBRMed_servoing/nditracker_logs/{}'.format(time.strftime("%Y-%m-%d_%H-%M-%S")))
    log_file = open('/home/racunalo/KUKA_LBRMed_servoing/nditracker_logs/{}/probe_movement.log'.format(time.strftime("%Y-%m-%d_%H-%M-%S")),'w')
    time_start = time.time()
    nan_counter = 0
    data_counter = 0
    while True:
        try:
            port_handles, timestamps, framenumbers, tracking, quality = tracker.get_frame()
            
            if framenumbers[0] != prev_frame:
                prev_frame = framenumbers[0]
                time_now = time.time() - time_start
                transform_mat  = np.array(tracking[0])
                rot_mat = transform_mat[0:3,0:3]
                rot = Rotation.from_matrix(rot_mat)
                rot_quat = rot.as_euler('xyz',degrees=True)
                transform = np.zeros(6)
                transform[0:3] = transform_mat[0:3,3]
                transform[3:6] = rot_quat

                if np.isnan(transform).any():
                    nan_counter += 1
                else:
                    data_counter += 1


                log_file.write('{:.6f}, {}, {}, {}, {}, {}, {}\n'.format(time_now,
                                                                               transform[0],transform[1],transform[2],
                                                                                transform[3],transform[4],transform[5]))
        except KeyboardInterrupt:
            time_now = time.time() - time_start
            log_file.write('#TOTAL TIME: {:.6f} seconds\n'.format(time_now))
            log_file.write('#TOTAL FRAMES: {}\n'.format(data_counter + nan_counter))
            log_file.write('#VALID FRAMES: {}\n'.format(data_counter))
            log_file.write('#NAN FRAMES: {}\n'.format(nan_counter))
            log_file.write('#NAN PERCENTAGE: {:.2f} %\n'.format((nan_counter/(data_counter + nan_counter))*100))
            log_file.close()
            tracker.stop_tracking()
            return



if __name__ == '__main__':
    main()
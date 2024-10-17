# Indoor Navigation with ArduPilot Drone

## Vision to MAVROS using Intel RealSense T265 Tracking Camera


ArduPilot parameters:



    For ArduPilot-4.1 (and later):

        AHRS_EKF_TYPE 2 = 3 (EKF3)
        EK2_ENABLE = 0 (disabled)
        EK3_ENABLE = 1 (enabled)
        EK3_SRC1_POSXY = 6 (ExternalNav)
        EK3_SRC1_VELXY = 6 (ExternalNav)
        EK3_SRC1_POSZ = 1 (Baro which is safer because of the camera’s weakness to high vibrations)
        EK3_SRC1_VELZ = 6 (ExternalNav)
        GPS_TYPE = 0 to disable the GPS
        VISO_TYPE 13 = 2 (IntelT265)

    If you wish to use the camera’s heading:

        COMPASS_USE = 0, COMPASS_USE2 = 0, COMPASS_USE3 = 0 to disable all compasses
        EK3_SRC1_YAW = 6 (ExternalNav)

    If you wish to use the autopilot’s compass for heading:

        COMPASS_USE = 1 (the default)
        EK3_SRC1_YAW = 1 (Compass)
        RC7_OPTION 1 = 80 (Viso Align) to allow the pilot to re-align the camera’s yaw with the AHRS/EKF yaw before flight with auxiliary switch 7. Re-aligning yaw before takeoff is a good idea or loss of position control (aka “toilet bowling”) may occur.

Source: https://discuss.ardupilot.org/t/fail-to-setup-t265-for-arducopter/89826/8


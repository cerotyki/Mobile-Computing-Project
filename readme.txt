In part1 directory, there are codes for part1. Kalman filter library (actually one cpp code) that we used for part1 is in the same directory.
To compile part1, you only need to execute the ino file and upload to a board.
Kalman filter code is included in Kalman.h file.

In part2 directory, there are codes for part2, and libraries are for part2 as well. It includes BLE, HTS, IMU and (though we have not implemented yet, ) PDM.
To compile part2, you only need to execute the ino file and upload to each board, then run main.py on Windows, maybe with powershell.
We have already modified the IMU libraries to support large range and high sampling frequency. Please use our library in the same directory.
FYI, do not forget to attach these boards to your drum sticks. Enjoy your play.
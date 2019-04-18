---------------------------------------------------------------------------
                  Part of the Malaga Urban Dataset

See more docs online: 
- http://www.mrpt.org/MalagaUrbanDataset
Paper: 
 Blanco-Claraco, J. L., Moreno-Dueñas, F. Á., & González-Jiménez, J. (2014). 
 The Málaga urban dataset: High-rate stereo and LiDAR in a realistic urban scenario. 
 The International Journal of Robotics Research, 33(2), 207-214.

---------------------------------------------------------------------------

Note: This directory contains an extract of the full dataset. These sections 
       are much easier to handle that the entire raw dataset, which is however
       also available online.

============================
1) Images
============================
- Images/*.jpg: Raw stereo images.
- {*}_rectified_a=0_1024x768_Images/*.jpg: Rectified 1024x768 images. 
- {*}_rectified_a=0_800x600_Images/*.jpg: Rectified 800x600 images. 

Read below about the binary files (.rawlog) and how to use the application 
"RawLogViewer" to quickly browse images and the rest of sensor data.

=================================
2) Plain text files
=================================
- {*}_IMAGES.txt: A simple list with all image file names.

- {*}_CAM+GPS.kml: A Google Earth file with a representation
   of the dataset path (as generated with the tool "rawlog-edit").

- {*}_GPS.txt: Each line contains an entry with the following fields (columns):
  - Time: UNIX time_t (with decimals)
  - Lat: WGS84 Latitude (radians)
  - Lon: WGS84Longitude (radians)
  - Alt: WGS84Altitude (meters)  
  - fix: 0=signal lost, 1=standalone, 4=fix. RTK, 5=float RTK, 6=dead reckoning
  - #sats: Number of satellites in use
  - speed: In knots
  - dir: Heading in degrees
  - Local X,Local Y,Local Z: Local Cartesian coordinates with respect to a 
     point nearby the starting position (+Z points up).
  - rawlog ID: Index of the entry in the original .rawlog log file.
  - Geocen X, Geocen Y, Geocen Z: Cartesian coordinates with respect to 
     the Earth center. 
  - GPS X, GPS Y, GPS Z: Not used
  - GPS VX, GPS VY, GPS VZ: Not used
  - Local VX, Local VY, Local VZ: Not used
  - Satellite time: Identical to the first field

- {*}_{LASER_NAME}_times.txt:
  - Each line contains the UNIX time_t (with decimals) of one laser scan.

- {*}_{LASER_NAME}.txt:
  - For each line in the file above there is one line here with the 
    corresponding scan range data (in meters). The number of columns is 
    the number of scan rays.
	
- {*}_IMU.txt: Inertial Measuring Unit (IMU) data file, with these columns:
  - Time: UNIX time_t (with decimals)
  - IMU_{X,Y,Z}_ACC: Acceleration (m/s^2)
  - IMU_{YAW,PITCH,ROLL}_VEL: Angular velocities (gyroscope) (rad/s)
  - IMU_{X,Y,Z}_VEL: Internal estimation of the velocities (m/s)
  - IMU_{YAW,PITCH,ROLL}: Internal estimation of the global attitude (rad) 
  - IMU_{X,Y,Z}: Internal estimation of global coordinates (m)
  

=================================
3) Binary files
=================================

- {*}_all-sensors.rawlog: The main binary log with all sensors.

- {*}_CAM+GPS.rawlog: A filtered version of the one above, 
   where the only sensor streams are that of the stereo camera and the 
   GPS receiver. 

All .rawlog files can be opened with the program "RawLogViewer", 
part of MRPT. In Ubuntu, install the package "mrpt-apps".

If you use MRPT apps or C++ classes to parse the .rawlog files, please create
a directory named "Images" and put all images there for the software to find
them.


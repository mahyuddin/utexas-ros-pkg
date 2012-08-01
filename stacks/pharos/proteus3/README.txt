To compile:

$ ../../rosjava_core/gradlew installApp

To run:

$ ./build/install/proteus3/bin/proteus3 [name of main class file]

where [name of main class file] may be:
 - edu.utexas.ece.pharos.proteus3.apps.navigation.MoveOutdoorCompassGPS 

To clean:

$ ../../rosjava_core/gradlew clean

Note 1: 
The class edu.utexas.ece.pharos.proteus3.sensors.GPSBuffer 
requires the system time to be accurate since it compares 
the system time to the GPS time to determine the age of the 
latest GPS measurement.  Calibrate the system time using
NTP by executing the following command:

$ sudo ntpdate ntp.ubuntu.com

Note 2:
The pharos node relies on various java messages.  Create them 
as follows:

$ roscd rosjava_messages
$ ../gradlew clean
$ ../gradlew install

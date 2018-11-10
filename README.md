# UltrasonicSensor module for Raspberry Pi with filtering techniques

## Apply different filtering techniques and compared them

###Comparisons:

#### It contains two different basic kalman filter(only position) and does comparisons between two different filtered position and the actual measurement:

```
main_position.cpp

rqt_plot  /UltrasonicSensor_filtr2/z_Position /UltrasonicSensor_filtr/z_Position /UltrasonicSensor/z_Position 
```


It returns the distance and the estimated velocity on the z axis.


It publishes the results on a lcm topic.

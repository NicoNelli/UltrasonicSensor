# UltrasonicSensor module for Raspberry Pi with filtering techniques

## Apply different filtering techniques and compared them

### Comparisons:

#### It contains two different basic kalman filter(only position) and does comparisons between two different filtered position and the actual measurement:

```
main_position.cpp:

rqt_plot  /UltrasonicSensor_filtr2/z_Position /UltrasonicSensor_filtr/z_Position /UltrasonicSensor/z_Position 
```

#### It containts one basic kalman filter (only position) and does comparisons between the velocity obtained by the filtered position(difference quotience) and the noised velocity(not filtered position):

```
main_velocity.cpp:

rqt_plot  /UltrasonicSensor_filtr/z_Velocity /UltrasonicSensor/z_Velocity 
```

#### It compares position and velocity of three different filters:

##### Kalman filter with only position as state variable (velocity estimated after with difference quotience)

##### Kalman filter with position and velocity as state variable

##### Actual measurement and velocity estimated after

##### Avarage median filter for position, velocity estimated after with difference quotience

###### For plotting position:

```
main_FullPosition.cpp:

rqt_plot  /UltrasonicSensor_Pos_filtr/z_Position /UltrasonicSensor_Full_filtr/z_Position /UltrasonicSensor/z_Position
```

###### For plotting velocity:

```
main_FullPosition.cpp:

rqt_plot  /UltrasonicSensor_Pos_filtr/z_Velocity /UltrasonicSensor_Full_filtr/z_Velocity /UltrasonicSensor/z_Velocity
```

#### It also takes into account the acceleration along z axis to affine the estimate for the velocity:

```
main_finalKalman.cpp
```











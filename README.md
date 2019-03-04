# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Build + Run Instructions.

To Build execute the following commands in the root directory...
> cmake .
> make
To Run execute the following commands in the root directory...
> ./path_planning

### Reflection

This attempt at completing the path planning project is built ontop of the method outlined in the project's Q&A section which in short works using the following steps
1. Set initial two cooridinates such that the start of the next path is tangental to the end of the previous path
2. Get the X and Y coordinates of the next 3 waypoints spaced 30 units apart
3. Shift coordinate system of the x/y coordinates of the waypoints such that the 'x' direction is aligned with the heading of the path/vehicle
4. Add what's left of the previous path into the next path
5. Append interpolated points until 50 points are in the path vector

An additional discrete state machine was employed to control lane changes:

```
IF The ego car is too close to the closes car in the lane:

   Slow Dow 
   
   IF A left lane exists AND There is not a vehicle in the left lane behind the ego car that is moving faster AND there is not a vehicle in the left lane ahead of the ego car that is moving slower than the ego car :
   
      Move over ot the elft lane
      
   ELSE IF a right lane exists AND There is not a vehicle in the right lane behind the ego car that is moving faster AND there is not a vehicle in the right lane ahead of the ego car that is moving slower than the ego car :
   
      Move Over to the right lane
```
      
While simple, this state machine does a good enough job to meet the project requrement
Additonal improvements that I would consider adding if time allowed:
1. Trajectory prediction - This might be difficult considering a vast amount of data about the behavior of the other vehicles will need to be collected
2. More sophisticated state machines, perhaps using cost functions
3. Addressing the current issue with this existing statemachine in the case where the vehicle in the left and the right lane are both going the same speed, which currently causes the ego car to split lanes

Reflection

# 1. Jerk, acceleration and speed control mechanisms. #
To control the speed, I use a speed control. First define the maximum speed is 49.5 miles/hour, and define a speed diff is 0.224 mile/hour, because 0.224 mile/hour = 0.5 m/s. The start-up velocity is 0 miles/hour.  

When the car first start up, the reference velocity = 0, then velocity accumulated in the loop to maximum speed.  

      ref_vel += speed_diff;  
    if ( ref_vel > MAX_SPEED ) {
       ref_vel = MAX_SPEED;  
    } else if ( ref_vel < MAX_ACC ) {
       ref_vel = MAX_ACC;
    }  
When detected in front it is a car and the distance is very close, the car will check if it is possible to change lane or not, if it is not possible, the speed will slow down.   
 
    else 
    {
      speed_diff -= MAX_ACC;
    }

    
If it is no car in front of ego car, it will speed up until 49.5 mile/hour.  

    if ( ref_vel < MAX_SPEED )
    {
      speed_diff += MAX_ACC;
    }

![Alt text](/IMG/safe5.11.png)
# 2. Collision avoidance mechanism. #
The sensor fusion vectors have contained useful information of other cars in the road. Loop the sensor fusion and calculate the lane of the cars and check if it is in the same lane as the ego car.   

    int car_lane = -1;
    // is it on the same lane we are
    if ( d 0 && d < 4 ) 
    {
      car_lane = 0;
    } 
    else if ( d 4 && d < 8 ) 
    {
      car_lane = 1;
    } 
    else if ( d 8 && d < 12 ) 
    {
      car_lane = 2;
    }
    if (car_lane < 0) 
    {
      continue;
    }

If other car is in the same lane of ego car front, check the distance and if the distance is too close, then prepare to change lane. Because the start state of ego car is in the center lane = 1, so it has two options either to change to left lane or change to right lane, before change lane there's also a distance check about cars in the left/right lane.  
If both lane is not able to change, then slow down the car speed.  

    if ( car_ahead ) 
			{ // Car ahead
              if ( !car_left && lane > 0 ) 
			  {
                // if there is no car left and there is a left lane.
                lane--; // Change lane left.
              } 
			  else if ( !car_righ && lane != 2 )
			  {
                // if there is no car right and there is a right lane.
                lane++; // Change lane right.
              } 
			  else 
			  {
                speed_diff -= MAX_ACC;
              }
            } 

If the ego car is not in center lane, check the center lane is any car in front, if no, then change to center lane.  
 
    if ( lane != 1 ) 
			  { // if we are not on the center lane.
                if ( ( lane == 0 && !car_righ ) || ( lane == 2 && !car_left ) ) 
				{
                  lane = 1; // Back to center.
                }
              }

# 3. Path generator #
This code does the calculation of the trajectory based on the speed and lane output from the behavior, car coordinates and past path points.

First, the last two points of the previous trajectory are used.   

    ref_x = previous_path_x[previous_path_x.size() - 1];
			  ref_y = previous_path_y[previous_path_y.size() - 1];

			  double ref_x_prev = previous_path_x[previous_path_x.size() - 2];
			  double ref_y_prev = previous_path_y[previous_path_y.size() - 2];
			  ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

			  ptsx.push_back(ref_x_prev);
			  ptsx.push_back(ref_x);

			  ptsy.push_back(ref_y_prev);
			  ptsy.push_back(ref_y);  

in conjunction three points at a far distance (30, 60, 90) to initialize the spline calculation.  

    vector<double> next_wp0 = getXY(car_s + 30, 2 + (4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> next_wp1 = getXY(car_s + 60, 2 + (4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> next_wp2 = getXY(car_s + 90, 2 + (4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

		  ptsx.push_back(next_wp0[0]);
		  ptsx.push_back(next_wp1[0]);
		  ptsx.push_back(next_wp2[0]);

		  ptsy.push_back(next_wp0[1]);
		  ptsy.push_back(next_wp1[1]);
		  ptsy.push_back(next_wp2[1]);  


To make the work less complicated to the spline calculation based on those points, the coordinates are transformed (shift and rotation) to local car coordinates.
To achieve the spline calculation, I use a free spline library that recommended in the course.  
 
		  for (int i = 0; i < ptsx.size(); i++)
		  {
			  double dx = ptsx[i] - ref_x;
			  double dy = ptsy[i] - ref_y;
			  ptsx[i] = (dx * cos(0 - ref_yaw) - dy * sin(0 - ref_yaw));
			  ptsy[i] = (dx * sin(0 - ref_yaw) + dy * cos(0 - ref_yaw));
		  }
		  tk::spline s;
		  s.set_points(ptsx, ptsy);  

In order to ensure more continuity on the trajectory, the pass trajectory points are copied to the new trajectory.  

    for (int i = 0; i < previous_path_x.size(); i++)
		  {
			  next_x_vals.push_back(previous_path_x[i]);
			  next_y_vals.push_back(previous_path_y[i]);
		  }

The rest of the points are calculated by evaluating the spline and transforming the output coordinates to not local coordinates.  

    for (int i = 1; i <= 50 - previous_path_x.size(); i++)
		  {
			  ref_vel += speed_diff;
              if ( ref_vel > MAX_SPEED ) {
                ref_vel = MAX_SPEED;
              } else if ( ref_vel < MAX_ACC ) {
                ref_vel = MAX_ACC;
              }
			  double N = target_dist / (0.02 * ref_vel / 2.24);
			  double x_point = x_addon + target_x / N;
			  double y_point = s(x_point);

			  x_addon = x_point;
			  double x_ref = x_point;
			  double y_ref = y_point;
			  x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
			  y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
			  x_point += ref_x;
			  y_point += ref_y;

			  next_x_vals.push_back(x_point);
			  next_y_vals.push_back(y_point);
		  }

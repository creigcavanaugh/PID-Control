
## PID Controller Project

**Creig Cavanaugh - July 2017**

[//]: # (Image References)
[image1]: ./output/screenshot_1.png
[image2]: ./output/screenshot_2.png
[video1]: ./output/output.mp4

### [Rubric](https://review.udacity.com/#!/rubrics/824/view) Points

---
### Writeup

In this project I implemented a PID controller to set the steering value based on the value of the cross track error (CTE).  My goal was to obtain the highest velocity while still keeping the car safely on the track.  

I primarily used manual tuning to optimize the PID gains.  For steering I used the proportional component (Kp) to determine how aggressive the car would steer back to the centerline.  The differential component (Kd) is used to reduce over-steer and help prevent oscillations around the centerline. The differential error was simply calculated by subtracting the previous CTE from the latest CTE.

![alt text][image1]

A negative cross track error indicates the car is left of the centerline, likewise a positive error indicates the car is right of the centerline. Because we need to steer in the opposite direction of the error, all the steering gains are negative.  The integral gain was always set to zero, since it would only be needed if there was a consistent drift component, such as if the car had an alignment error.  It was assumed the simulator did not add any alignment error to the car.

I also added a PID controller to reduce throttle or apply the brake based on the absolute value of the cross track error.  The controller gains were tuned manually, and similar to the steering controller, the integral gain was not used.  

![alt text][image2]

In order to speed up the process of manually tuning the gains and prevent the need to recompile after changing the PID values in the code, I implemented arguments to modify the values when calling the program.  After experimentation, I came up with the following gains for the steering and throttle PID controllers.

### Steering Values
Kp = -0.063
Ki = 0.0
Kd = -0.80

### Throttle Values
Kp = 0.65     
Ki = 0.0  
Kd = 0.5

The controller implemented with these gains, as well as some additional throttle logic, achieves nearly 80 mph.

Here's a [link to my video result](./output/output.mp4)


In addition to manual tuning, I created an implementation similar to Twiddle with the intent to further optimize the gains.  Although it was automatically cycling through gain combinations, its usefulness was limited, since as it was tuning the parameters, the car would run off the road when it selected sub-optimal combinations, and there was no automated way to reset the course.  Due to this limitation, it was not practical to leave in the final version, although I have linked it [here](./output/main_with_twiddle.cpp) for reference.  A future improvement of the simulator could be to implement a "reset" command and perhaps also a lap timing mechanism.

During testing, once the gains were tuned properly, I could leave the simulation going in autonomous mode for over an hour without any crashes.  I further noticed that if I was performing a computationally intensive process on the same computer, the car would sometimes crash.  This implies the PID controller was not able to compute and transmit the new steering and throttle values to the model in the required amount of time, given the car's current velocity and steering angle. 

A potential fix could be to use more conservative gains (at expense of car velocity), or in real world applications use a real-time operating system or dedicated hardware to guarantee calculations would be complete within a certain time frame. 



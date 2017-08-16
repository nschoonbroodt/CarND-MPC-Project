# Project: MPC controller

## Video of the results
Here is a youtube video of my drive. The target speed is 20 but it drives faster when I'm not recording videos.

[![Result in video](https://img.youtube.com/vi/Q5Y7xBDGNHM/0.jpg)](https://www.youtube.com/watch?v=Q5Y7xBDGNHM)

## Answer to Project Rubric Questions

### The model
The model used for this MPC contoller is a model with six states and two actuators.

The states are:

- x, the x position
- y, the y position
- psi, the heading
- v, the speed
- cte, the cross track error
- ePsi, the heading error

The actuators are:

- a, the acceleration
- delta, the steering angle

The cinematic model is the following (if ' denotes the next state value, dt the timestep and psiDes the desired psi):

    x' = x + v * cos(psi) * dt
    y' = y + v * sin(psi) * dt
    psi' = psi + V / Lf * delta * dt
    v ' = v + a * dt
    cte' = f(x) - y  + v * sin(ePsi) * dt
    ePsi' = psi - psiDes + v/Lf * delta * dt

### N and dt
~~I have used N = 10 and dt = 0.1. It works reasonably at the speed that I have set (about 20, but this is reduced when the car rotates). These were the values from the classroom. I have experimented around these values, but increasing N was not really giving me better results, while increasing the computation time. I had degradating performances when diminishing it (less predictions). When increasing the dt value, the performance was also worse. So I have kept these values, as the result was ok.~~

I did new tests with my new submission (dealing differently with latency) and now I use N=5 and dt = 0.3. The performance seems ok with these.

### Preprocessing of waypoints, polynomial fitting

I transform the measured waypoints into the car reference frame **of the predicted position, after latency**, using the usual rotation equations (see main.cpp lines 95 to 103)

I then fit a 3rd degree polynomial to these waypoints, and this polynom is the target trajectory that I use as a reference.

### Dealing with latency

~~To deal with latency, I had to tune the different coefficients of the cost. I have also added, at that point, a penalty linking the speed of the car and the curvature of the road (line 70), to "force" the car to slow down in curves, as this was the part of the driving were I had the most troubles, once I did introduce the latency. I had to make sure that this additional cost was not to high, as it must not overtakes the penalty linked to the car stopping.~~

Following the advice of the reviewer of my first submission, I now deal with latency by predicting the future state using the same model (and with a dt equal to the latency).

## Other changes in second submission

Also I have corrected my model to use m/s instead of mph where appropriate, and have done a few corrections to handle differently the sign difference between my model and the steering angle of the simulator.

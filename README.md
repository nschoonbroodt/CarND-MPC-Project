# Project: MPC controller

## Video of the results
Here is a youtube video of my drive. The target speed is 20 but it drives faster when I'm not recording videos.

[![Result in video](https://img.youtube.com/vi/ZbqYGJVhzjI/0.jpg)](https://www.youtube.com/watch?v=ZbqYGJVhzjI)

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
I have used N = 10 and dt = 0.1. It works reasonably at the speed that I have set (about 20, but this is reduced when the car rotates). These were the values from the classroom. I have experimented around these values, but increasing N was not really giving me better results, while increasing the computation time. I had degradating performances when diminishing it (less predictions). When increasing the dt value, the performance was also worse. So I have kept these values, as the result was ok.

### Preprocessing of waypoints, polynomial fitting

I transform the measured waypoints into the car reference frame, using the usual rotation equations (see main.cpp lines 95 to 103)

I then fit a 3rd degree polynomial to these waypoints, and this polynom is the target trajectory that I use as a reference.

### Dealing with latency

To deal with latency, I had to tune the different coefficients of the cost. I have also added, at that point, a penalty linking the speed of the car and the curvature of the road (line 70), to "force" the car to slow down in curves, as this was the part of the driving were I had the most troubles, once I did introduce the latency. I had to make sure that this additional cost was not to high, as it must not overtakes the penalty linked to the car stopping.

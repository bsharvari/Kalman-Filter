# Kalman-Filter
Multi-dimensional Kalman filter for localization

Basic matrix operations like generation of identity matrix, addition, sutraction, multiplication of two marices, and transpose of a matrix are implemented in a separate class. The Kalman filter is implemented using equations for multiple dimensions. The code is tested for sample input where the robot is in a 2-dimensional world, so it has both motion and velocity in x and y, making it a 4-dimensional problem.

In the code, dt is a small time step, x is the initial state, u is the external motion (if any), P is the initial uncertainty, F is the next state, H is the measurement function, R is the measurement uncertainty, and I is the N-dimensional identity matrix.

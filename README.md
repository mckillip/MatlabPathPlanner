# MatlabPathPlanner
Implementation of [Park 2011] 2D path planning feedback algorithm

This repo contains MATLAB code that runs a GUI, designed using App Designer, that provides a tabular entry of robot "pose" locations for the robot to reach as part of completing one of three autonomous navigation tasks for FRC at Home for 2021.  Radio buttons provide selection of one of the three courses, and an associated text file is read, and provided for editing (and saving) for feeding the algorithm desired locations and headings (and velocities) for each segment of the path to be followed.  A check box can be used to generate a GIF "movie" file of a path traversal to the file myPath.gif, which should be renamed if it is to be saved, as it will be overwritten on each subsequent path recording.  And, a rough estimate of the time to traverse the path is presented that uses the simulation time step and  the number of steps taken in following the designed path profile.

A 6-minute video movie is also located here to demo the code and provide additional guidance on its use.

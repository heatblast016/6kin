# 6kin: A kinematics engine for a 6 degree-of-freedom arm

This project is an inverse kinematics engine for a 6 DOF arm as described in the below diagram:
![6R](https://user-images.githubusercontent.com/15475418/165378867-3d46f5cb-33d8-41d0-8ba6-01d22df7eddc.svg)


[Source](https://motion.cs.illinois.edu/RoboticSystems/Kinematics.html)

## Installation and dependencies

The repository can be cloned directly from github.
```
git clone https://github.com/heatblast016/6kin.git
```

In addition, you will need to install [Matplotlib](https://matplotlib.org/) and [NumPy](https://numpy.org/) if they aren't already installed

## Configuring and running the program
To specify the joint lengths L1, L2, L3, and L4, you can modify config.txt, with each line representing the length of the corresponding joint.

To run the program, run the command:
```python3 main.py```

The program should prompt for a desired 3-d position in xyz coordinates, as well as a desired tool orientation specified by roll, pitch, and yaw (rpy). More information about specifying rotations using rpy can be found [here](http://planning.cs.uiuc.edu/node102.html)

It should then output a list of joint angles for each solution, and plot them in a separate window.

![image](https://user-images.githubusercontent.com/15475418/165385769-eb79e251-7dea-461d-a489-6a0d2cd4eb69.png)
![image](https://user-images.githubusercontent.com/15475418/165386115-2ad32303-4138-433f-841b-8d2d57382aed.png)




# Robotic hand control (ih2_azzurra)

## Overview of the repostitory
<div align="justify">
Human hands are the sought-after end effector, because of their high performance and remarkable flexibility. Emulating and embedding human hand capabilities into robotic grasping is crucial to further advance the object manipulation field. State-of-the-art robot hands allow the exertion of grasps which can be leveraged to accomplish complex tasks (e.g. in hand manipulation scenarios). Among these state-of-the-art robotic hands is the <a href="https://www.prensilia.com/ih2-azzurra-hand/">ih2 azzurra hand</a> developed by <a href="https://www.prensilia.com/">Prensilia</a>. This hand offers 5 degrees of actuation with 11 degrees of freedom. Additionally, the hand allows position, current, velocity, and force control on its fringers. Since interacting with this robotic hand is not intuitive at first, this repository provides some ready to use functionalities that can be used to build more complex actions.   
<br />
<br /> 
<p align="center">
   <img src="/Visualizations/gif_azzurra.gif" width="300" />
</p>
   
## Understanding repository

The reository contains two files, namely:
```
- toolkits: Python file containing all classes and controllers to connect, use, and control the robotic hand.
- main: A main file containing samples on how to use the functions on toolkits.
```
The classes are written in a way that facilitates the connection with the hand. The repository also reduces the complexity of handling bytes transmission from and to the hand. This allows the user to focus directly on high level controllers and experiment within different applications.
<br />
Whenever using position, current or force controllers, you will be able to see and extract the error and response on each one of the fingers you desired to control. The following images correspond to the position error and response signals of the motion on the gif above:
<br />


### Position error vs. Time
<p align="center">
   <img src="/Visualizations/Error_response.png" width="750" />
</p>
<br />


### Position error vs. Time
<p align="center">
   <img src="/Visualizations/Response_response.png" width="750" />
</p>

<br />
<strong>Go ahead and explore the functionalities of this repository!</strong>
<br />

</div>

## Contributions

The contributions of this repository can be summarized as follows:

```
- A class that facilitates the connection via serial port to a device.
- A class that handles the byte transmission to and from the robotic hand.
- Most of the functionalities explained in the manual of the ih2 azzurra hand (you won't need to develop things on your own)
- Ready to use controllers for position, current and force
- Visualization functions to check if the controllers responses were what you expected.
```

## License

Developed by Diego Hidalgo C. (2021). This repository is intended for research purposes only. If you wish to use any parts of the provided code for commercial purposes, please contact the author at hidalgocdiego@gmail.com.

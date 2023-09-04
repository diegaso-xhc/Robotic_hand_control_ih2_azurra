# Robotic hand control (ih2_azzurra)

## Overview of the repostitory
<div align="justify">
Human hands are the sought-after end effector, because of their high performance and remarkable flexibility. Emulating and embedding human hand capabilities into robotic grasping is crucial to further advance the object manipulation field. State-of-the-art robot hands allow the exertion of grasps which can be leveraged to accomplish complex tasks (e.g. in hand manipulation scenarios). Among these state-of-the-art robotic hands is the <a href="https://www.prensilia.com/ih2-azzurra-hand/">ih2 azzurra hand</a> developed by <a href="https://www.prensilia.com/">Prensilia</a>. This hand offers 5 degrees of actuation with 11 degrees of freedom. Additionally, the hand allows position, current, velocity, and force control on its fringers. Since interacting with this robotic hand is not intuitive at first, this repository provides some ready to use functionalities that can be used to build more complex actions.   
<br />
<br /> 
<p align="center">
   <img src="/Visualizations/ih2_azzurra.png" width="500" />
</p>

## Understanding repository

Although this repository is easily extendable, it is worth noting that during the development of this code a given fixed setup was used. The setup consisted of:

```
- VICON motion tracking capture with 16 IR-cameras
- 12 purposely selected 3D-printed (PLA) objects
- Artec Eva Lite 3D - Scanner
- 26 4-mm marker hand setup (see image below)
```
<p align="center">
   <img src="/Visualizations/Setup_markers.png" width="400" />
</p>
 
<!---
your comment goes here
and here
![This is an image](/Visualizations/vis_1.png)
![This is an image](/Visualizations/grasp_wine_glass.gif) ![This is an image](/Visualizations/grasp_cup.gif)
<img src = "/Visualizations/grasp_wine_glass.gif" width="400"> <img src = "/Visualizations/grasp_cup.gif" width="570">

-->
</div>

Nevertheless, the contributions of our work provide tools to extend our results to different setups, provided that changes are made on the pertinent locations. If you are unsure about this, please contact the author at (diego.hidalgo-carvajal@tum.de)

## Contributions

The contributions of this repository can be summarized as follows:

```
- A library for post processing and visualization of motion capture data --> HMCL_lib
- A set of purposely selected objects, which guarantee grasping postures variability --> Info_objects
- Algorithms to track known 3D objects with random marker placements --> Track_object_lib
- Algorithms for mesh manipulation --> Mesh_Manip_lib
- Pseudonymized data from trials (saved as Study objects, see HMCL_lib) --> Trials
- Algorithms to calculate contact surfaces information betwen a hand mesh and an object mesh --> see main
- Python algorithms to handle the MANO model, including marker position optimization --> fit_MANO (Developed in 
  cooperation with Omid Taheri (https://is.mpg.de/person/otaheri).
```

## Examples of hand-object contact level human manipulation

### Visualization of human hand-object interaction

Plots of the contact interaction of the subject in a specified frame. Contact surfaces are visible in read. 

<p align="center">
   <img src="/Visualizations/Frame_visualization.png" width="650" />
</p>

### Grasping of a cylinder

Plots of object along contact surfaces for grasping analysis. Contact surfaces are visible in red.

<p align="center">
  <img src="/Visualizations/grasp_cylinder.png" width="550" />  
</p>

### Grasping of a wine glass and a cup

Digital representation of the manipulation of a wine glass and a cup.

<p align="center">
  <img src="/Visualizations/grasp_wine_glass.gif" width="342" />
  <img src="/Visualizations/grasp_cup.gif" width="575" />   
</p>

### Contact surfaces with different hand parameters

A generic and a customized hand models were used to calculate the contact surfaces during the human manipulation recording. It can be seen that the hand model parameters play a role on the accuracy of the contact surfaces.

<p align="center">
  <img src="/Visualizations/Contact_surface_diff.PNG" width="600" />  
</p>

## License

Developed by Diego Hidalgo C. (2021). This repository is intended for research purposes only. If you wish to use any parts of the provided code for commercial purposes, please contact the author at diego.hidalgo-carvajal@tum.de.

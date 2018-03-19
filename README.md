# Self-Driving RC-Car project  
ROS Software for performing autonomous driving using monocular vision on an RC car.  
This package was designed to run on an (Nvidia Jetson TX2)[https://developer.nvidia.com/embedded/buy/jetson-tx2] using a PS4 controller. It provides two driving mode:  
- `manual`: The default mode. In this mode, you can drive the RC Car manually using the paired PS4 controller. You can also recording driving sequences to assemble new training data. Pressing L1, you can retrain a model from scratch using. 
- `autonomous`: The self-driving mode. This mode enables the trained residual network to perform steering and throttle inference from input images. 

<br />
<div align="center"><img width="60%" src ="https://raw.githubusercontent.com/germain-hug/Autonomous-RC-Car/master/images/controller.png" /><p style="text-align=center";> RC Car PS4 Controls</p></div>  
<br /> 

## Getting started:  
### Software   
To install the ros package run:  
```bash  
cd $CATKIN_WS/src/  
git clone https://github.com/germain-hug/Autonomous-RC-Car.git  
cd .. && catkin_make
```  
To install dependencies run:  
```bash  
pip install -r requirements.txt
```  
You will also need to have OpenCV installed.
### Hardware  
Here is an overview of the hardware used in this project:
- 
## Running the software


## Node description  


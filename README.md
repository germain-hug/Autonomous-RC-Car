# Self-Driving RC-Car project  
ROS Software for performing autonomous driving using monocular vision on an RC car.  
This package was designed to run on an [Nvidia Jetson TX2](https://developer.nvidia.com/embedded/buy/jetson-tx2) using a PS4 controller. It provides two driving mode:  
- `manual`: The default mode. In this mode, you can drive the RC Car manually using the paired PS4 controller. You can also recording driving sequences to assemble new training data. Pressing L1, you can retrain a model from scratch. 
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
Here is a non-exhaustive list of the hardware used in this project:
- [Nvidia Jetson TX2](https://developer.nvidia.com/embedded/buy/jetson-tx2)   
- [Orbitty Carrier Board](http://connecttech.com/product/orbitty-carrier-for-nvidia-jetson-tx2-tx1/)  
- [Traxxas 4WD RC-Car](http://a.co/hHWuW0X)
- [Pololu Micro Maestro Controller](https://www.pololu.com/product/1350)
- [Spinel 2Mp / 140° FOV USB Camera](https://www.amazon.com/gp/product/B075DJ7KMB/ref=oh_aui_detailpage_o08_s00?ie=UTF8&psc=1)
- [4000mA 11.1V LiPo Battery](http://a.co/hUDvt8R)
- [Playstation Dualshock 4 Controller](http://a.co/hkcDLfu)
- [LiPo Battery Charger](http://a.co/7dZDCbX)  

Hardware assembly should be fairly simple, and additional ressources can be found [here](https://diyrobocars.com/resources/)  

## Running the software
To launch the software, run:
```bash  
roslaunch rccar.launch
```  

## Node description  
- `mode_management.py`: Mode manager, activates manual and self-driving modes through `/mode` and allows for model retraining.
- `manual_driver.py`: Publishes PS4 manual commands on `/cmd`, enables data capture mode. 
- `self_driver.py`: Streams from camera, and publishes inferred throttle and steering commands on `/cmd`
- `actuator.py`: Receives throttle and steering commands through the `/cmd` topic, pre-processes them transmits them to the servo-controller.  

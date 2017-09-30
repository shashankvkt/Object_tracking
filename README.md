# Object_tracking
Vision based object tracking and following uses the technique of visual servoing using a camera mounted on a 3-axis Gimbal.  
Unlike using a fixed camera, using a 3-axis gimbal adds better solution for object tracking and following as the camera can always focus on the target keeping it within the frame.  
This project involves the technique where the target is always kept at the centre of the frame of reference and the camera mounted on the Gimbal tracks the target with an attempt to keep it at the centre and then follows the target keeping it within its proximity. 

1. Select the target on the web app - flytgimbal as shown in the picture below

  ![f46doyniscb1arv medium](https://user-images.githubusercontent.com/23419376/31027736-f7362c72-a568-11e7-8343-5c9e9d695533.jpg)  
    
2. Once the target is selected run the code..you should see that the camera mounted on the gimbal tracks the target continously as shown below  
![fb7i1cciscb0u46 medium](https://user-images.githubusercontent.com/23419376/31027855-719de3b0-a569-11e7-8b2d-a8f099a95c36.jpg) ![fdp3e8uiscb0y64 medium](https://user-images.githubusercontent.com/23419376/31027867-75ec13c4-a569-11e7-9ca0-089a7fe7e13e.jpg)  

The following are the list of commands to execute the code  

    cd catkin_ws/src  
    
    catkin_create_package object_tracking  
    
    cd object_tracking  
    
    mkdir data  
insert your camera calibration file (.yaml) here after you calibrate your camera (http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)  

    mkdir src  
    cd src  
    
    git clone www.github.com/shashankvkt/Object_tracking  
    
    <go to catkin_ws root directory>  
    
open a new tab on your terminal and start flytos. Next you can use the command to start the web app of FlytGimbal and then run the code using the command as shown below. You can find the commands to start flytos and flytgimbal from http://api.flytbase.com/

    rosrun Object_tracking Object_tracking  
    
This project was done as a part of my internship at Flytbase Inc. under the guidance of Mr. Pradeep Gaidhani, Mr. Sharvashish Das and Mr. Nitin Gupta.   
![flyt-base-logo-1](https://user-images.githubusercontent.com/23419376/31042783-7b17bbe6-a5cd-11e7-9004-1383fc45a23a.png)

  
    
    
    
    
    
    


  
  

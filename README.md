## Integration of Neurotechnology's Verilook SDK into ROS

### Verilook setup
1. Link include and library directory so GCC can see the SDK. One way is to add the following
to .bashrc file (replace correct directory):

export CPATH="$CPATH:/opt/Neurotec_Biometric_6_0_SDK/Include"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/opt/Neurotec_Biometric_6_0_SDK/Lib/Linux_x86_64"

### ROS topics
1. ```event_in```: subscriber which expects ```std_msgs/String``` messages:
  1. ```e_enroll```: enroll a new subject
  2. ```e_identify```: identify faces
  3. ```e_createTemplate```: create a new template from an image received from the ```/usb_cam/image_raw``` topic
  4. ```e_showTemplate```: publish image with detected faces and bounding boxes on ```processed_image``` topic
2. ```subject_id```: subscriber which accepts subject ID as ```std_msgs/String``` for creating templates
3. ```processed_image```: publisher which publish images as ```const sensor_msgs::Image::ConstPtr```

### ROS parameters

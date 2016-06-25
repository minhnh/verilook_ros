## Integration of Neurotechnology's Verilook SDK into ROS

### Verilook setup
1. Link include and library directory so GCC can see the SDK. One way is to add the following
to ```.bashrc``` file (replace correct directory):
  * ```export CPATH="$CPATH:/opt/Neurotec_Biometric_6_0_SDK/Include"```
  * ```export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/opt/Neurotec_Biometric_6_0_SDK/Lib/Linux_x86_64"```

### ROS topics
1. ```event_in```: subscriber which expects ```std_msgs/String``` messages:
  * ```e_enroll```: enroll a new subject
  * ```e_identify```: identify faces
  * ```e_createTemplate```: create a new template from an image received from the ```/usb_cam/image_raw``` topic
  * ```e_showTemplate```: publish image with detected faces and bounding boxes on ```processed_image``` topic
2. ```subject_id```: subscriber which accepts subject ID as ```std_msgs/String``` for creating templates
3. ```processed_image```: publisher which publish images as ```const sensor_msgs::Image::ConstPtr```
4. ```face_image```: publisher which publish cropped image of the first detected face, as ```const sensor_msgs::Image::ConstPtr```
5. ```face_list```: publisher which publish lists of detected faces, as ```mcr_perception_msgs::FaceList```

### ROS parameters
1. ```database_path``` (string type, default: ```faces.db```): path to database file. Default CWD for ROS is ```~/.ros```
2. ```enable_database``` (boolean type, default: ```false```): enable connection to database
3. ```image_batch_size``` (integer type, default: 1): number of images to add for a template for ```e_enroll``` or ```e_createTemplate```.
 
### Build with Eclipse
* Build command: ```catkin build --force-cmake -G"Eclipse CDT4 - Unix Makefiles" verilook_ros```
* ```.project``` file for import under ```build/verilook_ros``` in the ROS workspace
* See [ROS Wikis](http://wiki.ros.org/IDEs#Eclipse "IDEs - ROS Wikis") for dealing with linking problems.


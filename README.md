# franka_realsense_test

## Instructions
1. Start a rosmaster in a terminal.
    ```
    roscore
    ```
2. Go to the launch folder in this package and start the realsense.
    ```
    cd ~/franka_realsense_test/launch
    roslaunch realsense.launch
    ```
3. Check that the realsense is running using `rqt_image_view` or `rqt` or `rviz`.
    ```
    rqt_image_view
    ```
4. Rostopic echo the camera_info and make sure the config/realsense.intr file matches the output (The first number in K is fx, the third number in K is cx, the fifth number in K in fy, and the 6th number in K is cy).
    ```
    rostopic echo /camera/color/camera_info
    ```
5. Start Frankapy as usual.
    ```
    cd Prog/frankapy
    ./bash_scripts/start_control_pc.sh -u student -i iam-<robot_name>
    ```
6. Enter the virtual environment and enter this folder.
    ```
    senv
    cd ~/franka_realsense_test
    ```
7. Run the example script with a wooden block on the table.
    ```
    python scripts/pick_up_using_realsense.py
    ```
8. Click on the center of the block in the image that pops up and then press enter.
9. The robot should go pick up the top of the block.


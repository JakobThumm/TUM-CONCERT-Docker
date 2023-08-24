# Real Robot Tests

## Step 1: Connect to the Robot using 
    ssh_embedded

## Step 2: Install sara shield.
* Since the code is running locally, the docker container should not be used.
* Concert description is already installed, so one must install concert_msgs (and human-gazebo) and then sara-shield 

## Step 3: Enable CONCERT robot:
* Press the remote, the lights should turn white

## Step 4: 
Run the command:

    reset && ecat_master
 
 and then wait. The last line of the output should say something like `non_periodic_thread`

## Step 5: Start sara-shield and the other plugins:

For that, there are two options: 

* Option 1: Actuators in idle mode (should be used first, since this cannot harm the robot)

      reset && xbot2-core --hw ec_idle -C ~/tum_integration_ws/src/sara_shield/config/xbot2_config.yaml

* Option 2: Moving acuators:
       
      reset && xbot2-core --hw ec_imp -C ~/tum_integration_ws/src/sara_shield/config/xbot2_config.yaml   

## Step 6: Open xbot gui. 
    xbot2-gui
and then start the sara-shield plugin



# Notes

* You can also move joints of the robot without sara-shield. In the `xbot2-gui`, select `chain_E` and enable it. Then you can move the joints by scrolling your mouse wheel while hovering over a joint. 
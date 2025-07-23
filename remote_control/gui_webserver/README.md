# GUI Webserver

## Package Description
To control aiformula, 

### requirements
```
sudo apt install -y ros-$ROS_DISTRO-rosbridge-suite
```

## Usage
1.Launch the server
```sh
$ colcon build --symlink-install --packages-up-to gui_webserver
$ ros2 launch gui_webserver gui_webserver.launch.py 
```
2.Click this address

http://localhost:8000

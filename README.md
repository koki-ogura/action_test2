# action_test2
## prepare
```.sh
cd dev_ws
cd src
git clone https://github.com/koki-ogura/action_test2.git
cd ..
colcon build --packages-select action_test2_interfaces
colcon build --packages-select action_test2
. install/setup.zsh
```
## run server
```.sh
cd dev_ws
. install/setup.zsh
ros2 run action_test2 server
```
## run client
```.sh
cd dev_ws
. install/setup.zsh
ros2 run action_test2 client
```
## check via top
```.sh
top
```
check server's pid (server_pid) and client's pid (client_pid), and
```.sh
top -p server_pid, client_pid
```

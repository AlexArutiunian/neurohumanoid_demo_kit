python3 sliders_arm.py --ip 192.168.123.211 --port 6000 
python3 hand_remote_poses.py eth0 --hand-ip 192.168.123.211 --reset-on-start


Страница с полезными ссылками для руки:
https://robosavvy.co.uk/rh56dftphands.html

Управление руками реализуется через:

библиотеку на питон с modbus

Репозиторий:

https://github.com/plus-m-r/RH56DFTP_teach?ysclid=mlgbmxbtz1519138614

Управление происходит за счет присваивания нужным регистрам значений, в заданных диапазонах (например, для управления положением указательного пальца, в регистр POS_SET_3 записывается число от 0 (выпрямлен) до 2000 (полностью согнут)), снятие значения происходит соответственно прочтением определенных регистров (например, регистр HAND_ID содержит id данной руки).

Список регистров находится в репозитории:

https://github.com/plus-m-r/RH56DFTP_teach/blob/main/Register/config/configFTP/ftp_registers.py


Примеры работы см. в папке с этим документом

Программы запускаются на бортовом компьютере робота.

Левая рука: ip 192.168.123.210:6000
Правая рука: ip 192.168.123.211:6000

Ros2 пакет, использующий modbus
Кратко:

ZIP архив см. на диске.

Зависимости:
sudo apt install libmodbus-dev
sudo apt install ros-humble-std-msgs

После установки зависимостей, распаковать папки из архива в ваш ws/src, затем из корня воркспейса: colcon build --packages-select service_interfaces inspire_hand_modbus_ros2

Затем source install/setup.bash, и можно использовать. Сервисы и топики читай в конце документа (переведенный readme)

Для работы с рукой нужно быть подключенным к ней по проводу ethernet, выбрать ip4 адрес: 192.168.123.111, маска посети: 255.255.255.0 (если ip вашей руки 192.168.123.xxx)

В файлах пакета захардкожен ip адрес руки. Если он не соответствует реальному адресу вашей руки, нужно вручную поменять его во всех файлах в папке /inspire_hand_modbus_ros2/src, где он фигурирует.

Далее прикладываю README, переведенный на английский язык:

inspire_hand Package
1. The inspire_hand package supports the use of INSPIRE Robots' dexterous hands and mechanical grippers on the ROS platform. We have only tested it in the Ubuntu 22.04 ROS2 Humble environment; other ROS environments require further testing.

2. To ensure the program runs properly, the following environment configuration operations need to be performed: (Required for first-time use, no need after configuration)
    1) Install ROS2 Humble environment. Specific installation method as follows: (Если у вас установлен Ros2 humble, можно обойтись без этого шага)
       (1) Use FishROS one-click installation. Installation process reference: https://blog.csdn.net/weixin_55944949/article/details/140373710?fromshare=blogdetail&sharetype=blogdetail&sharerId=140373710&sharerefer=PC&sharesource=weixin_60461072&sharefrom=from_link
       Installation process: Download script: wget http://fishros.com/install -O fishros && . fishros

       (2) Install modbus library. Terminal command: sudo apt-get install libmodbus-dev
Note: If other dependencies are missing, download the missing compilation items according to the terminal error messages during CMake compilation.

    2) Create catkin workspace
       Terminal commands: mkdir -p ~/inspire_hand_ws/src
                cd ~/inspire_hand_ws
                colcon build
                source install/setup.bash (Use in each terminal when starting, helps you find the ROS installation directory)
    3) Place inspire_hand_ros2.zip in the /src folder under inspire_hand_ws and unzip
       Terminal commands: cd ~/inspire_hand_ws/src
             unzip inspire_hand_ros2
                  Place the extracted inspire_hand_modbus_ros2 and service_interfaces in inspire_hand_ws/src, delete the inspire_hand_ros2 folder.
                 
    4) Recompile the installation package
       Terminal commands: colcon build --packages-select service_interfaces
             colcon build --packages-select inspire_hand_modbus_ros2
Note: Avoid adding too many source commands in bash files using sudo gedit ~/.bashrc to prevent environment variable conflicts. Minimize duplicate package names such as "service_interfaces"; otherwise, it may cause message format reference errors or node startup errors.

3.1 Using the Dexterous Hand (Ваш ip может отличаться от ip в этом официальном readme. Для роботов unitree g1 у них обычно адреса 192.168.123.210  или 192.168.123.211. Для проверки: ping 192.168.123.210)
    1) Connect inspire_hand to the corresponding host using a network cable. Modify IPv4 address to: 192.168.11.222, subnet mask to 255.255.255.0; if ping 192.168.11.210 returns data, the connection is successful; otherwise, check if the line connection is normal.

    2) Start using the inspire_hand_modbus_ros2 function package
       Note: Need to open a new terminal and execute terminal commands: source install/setup.bash
                                                  ros2 run inspire_hand_modbus_ros2 hand_modbus_control_node 
 
    (1) Set ID------ID range 1-254
    ros2 service call /Setid service_interfaces/srv/Setid "{id: 2, status: 'set_id'}"

    (2) Set baud rate------Parameter redu_ratio range 0-4
    ros2 service call /Setreduratio service_interfaces/Setreduratio "{redu_ratio: 0,id: 1, status: 'set_reduratio'}" 

    (3) Set six driver positions------Parameter pos range 0-2000 
    ros2 service call /Setpos service_interfaces/srv/Setpos "{pos0: 1000,pos1: 1000,pos2: 1000,pos3: 1000,pos4: 1000,pos5: 1000,id: 1, status: 'set_pos'}"
Замечание: хоть тут и написано range 0-2000, на самом деле значения setpos, которые будут принимать пальцы, находятся в меньшем диапазоне. Например, на нашей руке для указательного пальца это где-то (130 - 1990). Узнать этот диапазон вы можете, скомандовав положение 0, затем вызвав сервис (19). Показанное значение и будет минимальным. Все значения от 0 до этого положения будут приводить к одному и тому же результату. Но в реальном диапазоне, определенным таким способом, сервисы (3) и (19) будут работать точно.

    (4) Set speed------Parameter speed range 0-1000
    ros2 service call /Setspeed service_interfaces/srv/Setspeed "{speed0: 50,speed1: 50,speed2: 50,speed3: 50,speed4: 50,speed5: 50,id: 1, status: 'set_speed'}"

    (5) Set dexterous hand angle------Parameter angle range 0-1000
    ros2 service call /Setangle service_interfaces/srv/Setangle "{angle0: 1000,angle1: 1000,angle2: 1000,angle3: 1000,angle4: 1000,angle5: 1000,id: 1, status: 'set_angle'}"

    (6) Set force control threshold------Parameter force range 0-1000
    ros2 service call /Setforce service_interfaces/srv/Setforce "{force0: 0,force1: 0,force2: 0,force3: 1000,force4: 0,force5: 0,id: 1, status: 'set_force'}"

    (7) Set current threshold------Parameter current range 0-1500
    ros2 service call /Setcurrentlimit service_interfaces/Setcurrentlimit "{current0: 1500, current1: 1500, current2: 1500, current3: 1500, current4: 1500, current5: 1500,id: 1, status: 'set_currentlimit'}"

    (8) Set power-on speed------Parameter speed range 0-1000 (executed after power-off and power-on again)
    ros2 service call /Setdefaultspeed service_interfaces/srv/Setdefaultspeed "{speed0: 1000,speed1: 1000,speed2: 1000,speed3: 1000,speed4: 1000,speed5: 100,id: 1, status: 'set_defaultspeed'}"

    (9) Set power-on force control threshold------Parameter force range 0-1000 (executed after power-off and power-on again)
    ros2 service call /Setdefaultforce service_interfaces/srv/Setdefaultforce "{force0: 1000, force1: 1000, force2: 1000, force3: 1000, force4: 1000, force5: 1000,id: 1, status: 'set_defaultforce'}"

    (10) Set power-on current threshold------Parameter current range 0-1500 (executed after power-off and power-on again)
    ros2 service call /Setdefaultcurrentlimit service_interfaces/srv/Setdefaultcurrentlimit "{current0: 1500, current1: 1500, current2: 1500, current3: 1500, current4: 1500, current5: 1500,id: 1, status: 'set_defaultcurrentlimit'}"

    (11) Force sensor calibration------This command needs to be executed twice (after execution, the palm will fully open, then force calibration is performed)
    ros2 service call /Setforceclb service_interfaces/srv/Setforceclb "{id: 1, status: 'set_forceclb'}" 

    (12) Clear dexterous hand error
    ros2 service call /Setclearerror service_interfaces/srv/Setclearerror "{id: 1, status: 'set_clearerror'}" 

    (13) Restore factory settings
    ros2 service call /Setresetpara service_interfaces/srv/Setresetpara "{id: 1, status: 'set_resetpara'}"  

    (14) Save parameters to FLASH
    ros2 service call /Setsaveflash service_interfaces/srv/Setsaveflash "{id: 1, status: 'set_saveflash'}" 

    (15) Read driver set position values
    ros2 service call /Getposset service_interfaces/srv/Getposset "{id: 1, status: 'get_posset'}" 

    (16) Read set dexterous hand angle values
    ros2 service call /Getangleset service_interfaces/srv/Getangleset "{id: 1, status: 'get_angleset'}" 

    (17) Read set force control threshold
    ros2 service call /Getforceset service_interfaces/srv/Getforceset "{id: 1, status: 'get_forceset'}"

    (18) Read current current values
    ros2 service call /Getcurrentact service_interfaces/srv/Getcurrentact "{id: 1, status: 'get_currentact'}"

    (19) Read driver actual position values
    ros2 service call /Getposact service_interfaces/srv/Getposact "{id: 1, status: 'get_posact'}"

    (20) Read actual dexterous hand angle values
    ros2 service call /Getangleact service_interfaces/srv/Getangleact "{id: 1, status: 'get_angleact'}"

    (21) Read actual force
    ros2 service call /Getforceact service_interfaces/srv/Getforceact "{id: 1, status: 'get_forceact'}"

    (22) Read temperature information
    ros2 service call /Gettemp service_interfaces/srv/Gettemp "{id: 1, status: 'get_temp'}"

    (23) Read fault information
    ros2 service call /Geterror service_interfaces/srv/Geterror "{id: 1, status: 'get_error'}"
    
    (24) Read set speed values
    ros2 service call /Getspeedset service_interfaces/srv/Getspeedset "{id: 1, status: 'get_speedset'}"

    (25) Read status information
    ros2 service call /Getstatus service_interfaces/srv/Getstatus "{id: 1, status: 'get_status'}"
    
    (26) Gesture sequence
     ros2 service call /Setgestureno service_interfaces/srv/Setgestureno "{gesture_no: 1,id: 1, status: 'setgesture'}"
    
    5) ROS topic usage example: Real-time reading of dexterous hand tactile sensor data. The current script outputs 01. To print all raw data, you can call handcontroltopicpublisher_modbus (raw data).
      Need to open a new terminal and execute terminal commands:    source install/setup.bash
                                            ros2 run inspire_hand_modbus_ros2 handcontroltopicpublisher_modbus_node
                                                                                                   Open a new terminal
                                            ros2 run inspire_hand_modbus_ros2 handcontroltopicsubscriber_modbus_node 
This example will print the sending frequency and real-time current tactile sensor data of the entire hand in the terminal.

    6) ROS service usage example: Script calls services contained in service_interfaces/srv: Setpos
      Need to open a new terminal and execute terminal commands:    source install/setup.bash
                                            ros2 run inspire_hand_modbus_ros2 hand_control_client_modbus_node





	


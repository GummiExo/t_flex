# T-FLEX Repository

This repository contains all versions of T-FLEX, **please BE CAREFUL** if you perform changes in the code.

Select your branch according to the country:

  -> Chile: t_flex_chile
  
  -> France: t_flex_france
  
Linux Command to clone the specific repository
  ```
  git clone -b branch_name --single-branch https://github.com/gummiexo/t_flex.git
  ```
  
  Example:
  ```
  git clone -b t_flex_france --single-branch https://github.com/gummiexo/t_flex.git
  ```

Linux Command to push
  ```
  git add files_modified
  git push
  ```

**PLEASE, YOU MUST NOT MODIFY OTHER BRANCHES**

## Dependencies

Clone and compile the following repositories in the catkin_ws:

Dynamixel SDK: https://github.com/ROBOTIS-GIT/DynamixelSDK
Dynamixel Motors: https://github.com/arebgun/dynamixel_motor

## Configuration of Dynamixel Motors

1. Change the motor's ids (Frontal ID: 1 - Posterior ID: 2) and the baud rate (1000000)

  - Windows: DynamixelWizard software (http://www.robotis.us/roboplus/). For the connection, you can follow this tutorial http://emanual.robotis.com/docs/en/software/rplus1/dynamixel_wizard/
  
  - Linux: the recommended software for Linux is mixcell https://github.com/clebercoutof/mixcell.git. (Please be careful with the parameters modified in the motor, it could generate permanent damage.
  
  Default values of the motor: ID = 1, baudrate = 57600
  
2. Adjust the range of motion of the motor with the hinge set in order to that this range **does not present** a change of 0 to 4095

3. Modify the file motors_parameters.yaml located in the yaml folder. The only parameters which you must modify are min, max and init.

  min: minimum value of the motor registered 
  
  max: maximum value of the motor registered
  
  init: min o max value in where the hinge set is in the lowest position 
  
  (**these values must guarantee that the hinge set does not have contact with the motor or the actuation support system**)
  
 ## Configuration of Raspberry Pi as a router
 
 Follow this tutorial to enable the raspberry pi as a router:
  
  ```
  sudo apt-get update
  sudo apt-get upgrade
  sudo apt-get install isc-dhcp-server hostapd
  sudo cp /etc/dhcp/dhcpd.conf /etc/dhcp/dhcpd.conf.orig
  sudo nano /etc/dhcp/dhcpd.conf
  ```
  
  Find the following section and comment it out by placing a hashtag at the beginning of the line.
  ```
    #option domain-name "example.org";
    #option domain-name-servers ns1.example.org, ns2.example.org;
  ```

  Next, find the section below and un-comment the word authoritative (remove hashtag):
  ```
    # If this DHCP server is the official DHCP server for the local
    # network, the authoritative directive should be uncommented.
    authoritative;
  ```
  
  Copy the following lines at the end of this file:
  ```
    subnet 192.168.4.0 netmask 255.255.255.0 {
       range 192.168.4.2 192.168.4.254;
       option broadcast-address 192.168.4.255;
       option routers 192.168.4.1;
       default-lease-time 600;
       max-lease-time 7200;
       option domain-name "local-network";
       option domain-name-servers 8.8.8.8, 8.8.4.4;
    }
  ```
  
  Close and save changes, then execute:
  
  ```
  sudo cp /etc/default/isc-dhcp-server /etc/default/isc-dhcp-server.orig
  sudo nano /etc/default/isc-dhcp-server
  ```
  
  Scroll down to the line saying interfaces and update the line to say:
  ```
  INTERFACESv4="wlan0"
  ```
  
  Close and save changes, now execute
  
  ```
  sudo ifdown wlan0 
  ```
  If it is not working use: *sudo ip link set wlan0 down*
  
  ```
  sudo cp /etc/network/interfaces /etc/network/interfaces.orig
  sudo nano /etc/network/interfaces
  ```
  
  This file should look like:
  
  ```
  auto lo
  iface lo inet loopback

  auto eth0
  allow-hotplug eth0
  iface eth0 inet dhcp

  allow-hotplug wlan0
  iface wlan0 inet static
  address 192.168.4.1
  netmask 255.255.255.0

  up iptables-restore < /etc/iptables.ipv4.nat
  ```
  
  Close, save changes and execute:
  
  ```
  sudo ifconfig wlan0 192.168.4.1
  sudo cp /etc/hostapd/hostapd.conf /etc/hostapd/hostapd.conf.orig
  sudo nano /etc/hostapd/hostapd.conf
  ```
  
  In this file copy the following:
  
  ```
  interface=wlan0
  ssid=T-FLEX
  hw_mode=g
  channel=6
  macaddr_acl=0
  auth_algs=1
  ignore_broadcast_ssid=0
  wpa=2
  wpa_passphrase=t-flex1234
  wpa_key_mgmt=WPA-PSK
  wpa_pairwise=TKIP
  rsn_pairwise=CCMP
  ```
  
  Close and save changes, then execute
  
  ```
  sudo cp /etc/default/hostapd /etc/default/hostapd.orig
  sudo nano /etc/default/hostapd
  ```
  Find the line *#DAEMON_CONF="/etc/hostapd/hostapd.conf"* and uncomment it. Now, execute:
  
  ```
  sudo cp /etc/sysctl.conf /etc/sysctl.conf.orig
  sudo nano /etc/sysctl.conf
  ```
  Find and uncomment the next line to enable packet forwarding for IPv4:
  ```
  net.ipv4.ip_forward=1
  ```
  Close, save changes and execute:
  ```
  sudo sysctl -p /etc/sysctl.conf
  sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"
  ```
  If the last command fails, create a file with *sudo nano /proc/sys/net/ipv4/ip_forward* in this file press the spacebar, close and save changes. Then, change the permissions with *sudo chmod 777 //proc/sys/net/ipv4/ip_forward*. Now, execute again the command *sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"*
  
  After that, execute:
  ```
  sudo ifup wlan0 
  ```
  If it is not working use: *sudo ip link set wlan0 up*. Now, execute
  ```
  sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
  sudo iptables -A FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
  sudo iptables -A FORWARD -i wlan0 -o eth0 -j ACCEPT
  sudo iptables-save > /etc/iptables.ipv4.nat
  up iptables-restore < /etc/iptables.ipv4.nat
  sudo service hostapd start
  ```
  If the last command fails, execute 
  ```
  sudo systemctl unmask hostapd.service
  sudo systemctl enable hostapd.service
  sudo service hostapd start
  sudo service isc-dhcp-server start
  sudo update-rc.d hostapd enable 
  sudo update-rc.d isc-dhcp-server enable
  sudo reboot
  ```
  In this point, the Wifi Network already has been created. You sould see it in your smart device
   
 ## Configuration of Web Application
 
 In the raspberry pi, after the compilation of t_flex package, execute: 
 
 ```
 source ~/.bashrc
 roscd t_flex
 cp -rf ./WebController ~/
 cp -rf ~/WebController/* ~/
 nano ~/.bashrc
 ```
 
 In the last line add the following command:
 
 ```
 python listener.py
 ```
 
 Save changes and test using this command:
 
 ```
 source .bashrc
 ```
 
 In the terminal, you will see the communication with the smart device and all commands received and executed.
 
 ## Access to the Web Application
 
 For the access, you must connect to the T-FLEX Network (password: t-flex1234) and in a browser page you must put:
 
 192.168.4.1:3012
 
 Enter and you will have the main menu of the device, with items such as calibration, therapy mode, and assistance mode.
 
 
 
 *If you have any question you can contact the developer of this software: Daniel Gómez Vargas - daniel.gomez-v@mail.escuelaing.edu.co*

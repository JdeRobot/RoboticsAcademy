# Follow Line Exercise using WebTemplates

## How to launch the exercise?

- Launch the Gazebo simulation

```bash
roslaunch ./launch/simple_line_follower_ros.launch
```

- Start the host application with DNS server IP address which is usually 127.0.0.53 for Linux machine.For Windows machine,it would be 192.168.0.1 

```bash
python host.py 127.0.0.53
```

- Open the browser template from `index.html`



**__NOTE:__**  If you get **gaierror: [Errno -2] Name or service not known** error,you have to check the correct IP address on your machine



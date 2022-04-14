<h1 align="center"> Pozyx_tutorial </h1>

<h2 align="left"> Run checking UWB information </h2>

```
# computer
$ cd ~/master_thesis
$ source docker/laptop/docker_run.sh

# docker container
$ cd ~/master_thesis/catkin_ws/src/pozyx_uwb/tutorials
$ python3 checking_pozyx.py
```

<h2 align="left"> Run UWB ranging </h2>

- Run ranging with LED ( Need to confirm your "serial_port, remote_id and destination_id" parameters )
```
# computer
$ cd ~/master_thesis
$ source docker/laptop/docker_run.sh

# docker container
$ cd ~/master_thesis/catkin_ws/src/pozyx_uwb/tutorials
$ python3 ranging_with_led.py
```

- Run ranging without LED ( Need to confirm your "serial_port, remote_id and destination_id" parameters )
```
# computer
$ cd ~/master_thesis
$ source docker/laptop/docker_run.sh

# docker container
$ cd ~/master_thesis/catkin_ws/src/pozyx_uwb/tutorials
$ python3 ranging_without_led.py
```

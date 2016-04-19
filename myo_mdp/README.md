## To launch the program for task demostration and prompting, run the command:
```
rosrun myo_raw prompt2.py
```
Of course you need a ros core.

## If it does not work, maybe the package is not properly configured, you can try (in an appropriate directory):
```
python myo_python/prompt2.py
```

Then the program will wait for a ROS message: 
```
/excercise/mode std_msgs/Int32 0
```
which should be triggered by clicking the "begin_trial" button of the GUI.

This program uses Matlab to compute the expected trajectory, via Gaussian Mixture Model. 
It checks the directory /usr/local/MATLAB. 
You may want to set a symbolic link, or change the code if your matlab is found somewhere else.
If you do not have Matlab, the program can still obtain the expected trajectory, by using the average of training data.

## The ROS bags (training data) should be in myo_raw/data/work/. You can change it if you want to. 

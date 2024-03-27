# Instruction to run the code

* Install the required dependencies
```cmd
  sudo pip install scipy math matplotlib tqdm
```

1. LOADING THE ENVIRONMENT
    -
    Run main.blend file and dorp down at bar in scripting tab and run env.py. 
    * Make sure map path is added in line 7.
    * For now 3 maps are already loaded in the blender env. Hide and unhide the map according to the requirement.

2. ADDING MAP PATH
    -
    In env3D.py add map path location at line 8.For eg. For eg.
    ```python
    file_path = '/home/ankitmittal/Documents/STUDY/RBE595/HW2a/amittal_p2a/src/sample_maps/map2.txt' 
    ```

3. SET STARTING AND GOAL POSITION
    -
    In env3D.py set starting and goal positon in line 52 and 53 repectively.
    ```python
    self.start = np.array([0, 20, 2])
    self.goal = np.array([10, 20, 3])
    ```

4. SET STARTING POSITION FOR BLENDER
    -
    In main.py in blender set the starting location of the drone for blender environment in line 93 - 95.
    ```python
    startx = 0
    starty = 20
    startz = 2
    ```
# Note

* Currently we using 0.25m bloated obstacle to avoid any collision. If Incase starting or goal position is closer than the 0.25m then append the bloating in line 35 of env3D.py. For eg.
  ```python
  bloated_bocks = np.reshape(np.array([np.add(i[0:3], -0.25), np.add(i[3:6],  0.25)]),6)
  ```



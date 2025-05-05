## Welcome to Prometheus!
This repository contains source materials for Prometheus, an open-source teleoperation system with force feedback.

Feel free to visit our [website](https://eterwait.github.io/Prometheus/) for more information!

### Repository content

- **teleoperation** - main code.
- **Hardware** - 3D models and electronic components.
- **STM32 code** - STM32 CubeIDE project files.
- **Prometheus.bin** - ready-to-use microcontroller firmware. Download [Cube Programmer](https://www.st.com/en/development-tools/stm32cubeprog.html) and use it to upload the firmware to the board.
- **Prometheus.hex** - ready-to-use microcontroller firmware.

### Installation

1. Clone this repo on your PC. 

2. Move to `teleoperation` folder and install dependensies:

````sh
cd teleoperation
pip install -r requirements.txt
````

3. Modify `config.json` according to your hardware. Here you can find instructions how to get necessary data: https://pypi.org/project/vive-tracker-apiserver/

4. To control a gripper with your hardware, overwrite `read_pose` function in file `teleop_lip.py`. This function should return number in range 0..255 that will be mapped to gripper position.

5. In file `teleop_ur.py` change variables `hand_tracker` and `robot_ip` according to your setup.

6. You would like also modify base pose of the robot according to your environment. Therefor you can use `rtde_r.getActualQ()` function to obtain joint angles in your base pose and paste them in `base()` function instead of default angles.

7. You can modify control settings just at the beginning of `teleop_lib.py` file. In function `isInJointLimits` you can modify joint limits of your robot. We disabled check of this limits for now to avoid possible problems at the first launch, but you can easily enable it in `teleop_ur.py file`.

### Usage

1. Run `teleop_ur.py` file, pay attention to possible error messages that will be printed in console.

2. If all starts successfully, robot will follow your tracker.

3. To stop teleoperation, press `s` button. You will see all other instructions in your console.

### Citation

```
Future Bibtex here
```


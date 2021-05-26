# Small robotic arm based on Poppy project


## Goal
My objective is to develop a robotic arm, for the sake of learing robotics and hopefully using it in everyday life, capable of pushing, pulling and 
turning buttons/knobs/etc, with computer vision support, voice-controlled, and text to speech for interfacing with human users.
As I can't spend much time on this project, I intend to limit the span of these robotic skills in order to save development time, and trade this off for more features.

![](https://github.com/ezececi/poppy/blob/master/poppy_github_metainfo/img/2021_05_Poppy_pictures/Poppy_Front.jpg)
![](https://github.com/ezececi/poppy/blob/master/poppy_github_metainfo/img/2021_05_Poppy_pictures/Poppy_Front2.jpg)
![](https://github.com/ezececi/poppy/blob/master/poppy_github_metainfo/img/2021_05_Poppy_pictures/Poppy_Side.jpg)

## Status
On-going on dev-senses : Use computer vision (object detection with Tensorflow) to detect a custom target in order to reposition the arm before taking action.
- Node to pre-position the arm close to the target with moves recorder : OK
- Object detection with custom network for custom target recogition : OK, to be improved
- Reposition the arm depending on target location : OK, to be improved
- Take actions following repositioning : to be done
- Integrate all the steps in a single service / node : to be done

## Roadmap
- Integrate poppy_senses' nodes on the rpi
- Improve Object Detection network (more screenshots for training, to begin with)
- Add custom stand for camera close to the gripper (replacement of metal wires)
- Add pre-established scenarios, eg : 
  - prepositioning depending on variables : x degrees pitch, y cm height relative to base, z degrees yaw
  - actions : push, pull, to the left / right / up / down
- Add voice control  + text to speech + LEDs thanks to [Adafruit voice bonnet](https://www.adafruit.com/product/4757)
- Add battery + charger for off-the-grid autonomy ; replace rpi by rpi zero 
- Add 2nd camera / RGBD camera for distance estimation ?
- Change gripper for a better environment-compliant robotic "hand" (such as [Yale OpenHand ?](https://www.eng.yale.edu/grablab/openhand/) )
- Add motors in order to add some degrees of freedom ?
- Update the whole architecture for a bigger stronger robotic arm ?

## References

Poppy project : https://www.poppy-project.org/fr/robots/poppy-ergo-jr/


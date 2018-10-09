###How to launch PBVS with mico simulation

1. roslaunch arm_vs vs_ar_tag_tracking.launch
2. rosservice call /mico_interaction/switch_controllers "controllers_to_be_turned_on: 'velocity'"
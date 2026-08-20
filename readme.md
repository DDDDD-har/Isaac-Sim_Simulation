### 2025.09.09
1. 修改车辆TF发布的ROS节点、wheelloader的base_link等模块，确保数据结构正确。
2. 重新将ground命名为map，将堆料命名为pile，将z轴为0作为参考系。
3. 增加sim_interface功能，将仿真器给出的基本数据转化为Sa_msgs中定义的消息内容，并且接收其中的数据。

### 2025.09.08
1. 为了统一建图规范，将参考系由``ground``改为了``map``
2. 重新配置并优化了从 map 坐标系到各个坐标系的TF（坐标变换）关系，如map到车辆根坐标系 base_link、map到卡车root坐标系、map到车辆前后激光雷达Airy_front和Airy_rear的关系 ，适配当前接口，确保了全局定位与导航的准确性。


### 2025.09.03：
1. 适当更新了场景文件，发现上一版本的map数据原点与原始点云不一致，因此直接采用原始拼接的obj文件作为仿真原点，与地图匹配。新的地图数据是CZG/simple.obj，对应放到了场景大的usda中
2. 更新了车辆的模型，采用液压方式驱动，由于新的模型修改了车辆的各个关节信息，需要修改joint_command来控制（控制指令参考Controller仓库中的keyboard_piston_joint_publisher.py文件）
3. 更新了激光雷达的位置和名称，变成前后2个雷达。采用了Airy_new雷达增多线数。对应雷达的名称也进行了修改。

[[[TODO事项]]]
- 修改Controller仓库中的控制器以适配车辆控制
- 修改Controller仓库中的雷达接受数据以适配环境感知
- 修改Controller仓库中关于frame_id为“ground”的部分，当前的/tf数据全部建立在“map"这个frame基础上，需要对ros消息header中的frameID进行修改。


### 2025.08.25：
1. 适当修改常州港场景，放宽区域范围
2. 移除了上一版本激光类通过on time进行触发的机制（不清楚原来的方法为何会卡顿），使用on playback tick进行触发。

当前版本：

Ubuntu 22.04

Isaac sim 4.5.0

ROS2 Humble


## foxglove-docker-main
基于web的foxglove studio可视化软件

## sensor_simulation
新增的传感器模拟内容，包括livox和robosense雷达的自定义模拟，看readme复制到对应的isaacsim文件中


## warp_modification
针对warp粒子模拟器的修改内容，让warp可以正确模拟，看readme修改对应的语句


## world
主要的isaac sim场景文件，以及对应的各种素材

# ACFLY_A9_Firmware_Unofficial
ACFLY A9飞控非官方固件代码。

本项目修改自官方ACFly Prophet V10 20221008飞控固件版本，完善了进入OFFBOARD模式后直接通过上位机控制飞控的相关代码。

上位机向飞控传递SLAM信息使用MavROS自带插件acfly_slam_sensor，不需要额外编写串口通信，力图减少硬件复杂程度。

### 实现功能：

- 通过mavros进入OFFBOARD模式
- 通过mavros进行坐标/速度控制（本地和全局坐标系）

### 使用方法：

1. 将代码编译并烧录进A9中
2. 在地面站中设置启动模式为M32
3. 通过飞控上的Mavlink链接口连接上位机
4. 启动[MavROS](https://github.com/geniusdo/mavros-acfly/blob/acfly-develop/acfly-mavros%E4%BD%BF%E7%94%A8%E6%8C%87%E5%8D%97.md)
5. 检查acfly_slam_sensor插件是否正常向飞控传递SLAM信息（飞控屏幕上是否显示了XY轴速度）
6. 编写代码控制飞控

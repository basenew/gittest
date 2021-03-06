更新说明:

1.0.0:
1.程序数据保存位置修改
2.定位程序优化
3.感知坐标轴修正

1.0.1:
1.雷达ip配置修改

1.0.2:
1.导航增加充电流程
2.建图功能优化

1.0.3:
1.精准停靠点存pcd文件增加强度
2.sdk更新，可通过id导航到点

1.1.0
1.增加低矮障碍判断
2.增加并优化1cm精度到点
3.导航状态上报
4.优化控制在app遥控后，始终有速度下发的情况

1.1.1
1.修复控制程序更新后导航角速度有误

1.1.2
1.更新区分1cm停靠点和其它停靠点

1.1.3
1.优化控制转向

1.1.4
1.pc矢量图工具更新之后车端程序同步适配
2.增加电信号接收

1.1.5
1.优化1cm停靠精度功能

1.2.0
1.修改发路没有找到id，返回失败和可能会出现的订阅失败的问题
2.在车体周围30-50公分，被结构挡住的位置的点云直接剔除

1.2.1
1.修复定位丢失问题
2.优化调的接口获取停靠点列表时，没有精准停靠点的问题

1.2.2
1.更新搜路程序
2.导航功能优化
3.控制程序增加日志

1.2.3
1.搜路程序回退，新版搜路程序异常

1.2.4
1.优化搜路异常问题

1.3.0
1.解决车辆转弯需先原地转向再行驶，转弯侧滑的现象
2.解决车辆行驶前后卡顿挪动现象
3.解决之前左右两侧20-30cm有障碍物，会出现虚警现象，优化窄通道通行能力
4.增加tsp，实现多点最优路线
5.解决车辆被阻塞，围绕一圈后车辆会自动旋转一圈问题
6.解决改优化原地转向速度

1.3.1
1.底层控制限制前后最大速度
2.反馈速度异常时，取消平滑处理

1.3.2
1.更新行驶过程中定位丢失问题

1.3.3
1.定位程序日志增加
2.导航软件license加密
3.tsp搜路程序更新
4.验证导航软件license是否加密可通过查看CommunicationInterface程序进程是否存在来判断ps -ef | grep CommunicationInterface

1.3.4
1.更新了车辆在相对比较窄的道路上能自主行驶和停靠
现存问题：车辆无法停靠在离虚拟墙太近的停靠点（车离虚拟墙距离<15cm）
备注：现阶段为窄道场地，对矢量图的制作要求比较高，虚拟墙可以往路外多画5cm左右，可以有效提高停靠质量，不会导致车辆掉入悬崖。
后续会在程序上做进一步优化

1.3.5
1.定位程序更新（手动定位不上，行驶过程定位丢失）
2.规划程序更新 （无法通过窄道）
3.低障检测停障程序更新（低矮障碍物无法停障）

1.3.6
1.定位程序更新（车辆行驶过程中定位丢失，定位丢失后车继续走）
2.低障检测程序更新（优化10cm宽15cm高 低矮障物停障）
3.CI程序更新（部署工具精准停靠点增加pcd文件）
4.搜路程序更新（tsp收到的路线与实际不符）
5.规划程序更新（窄道通过程序优化）

1.3.7 2021/03/05
1.规划程序更新（5cm精准停靠未到停靠点停止，已修改，偏差在5cm以内）
2.定位程序更新（5cm精度定位程序优化）

1.3.8 2021/03/12
1.雷达检测程序更新（下坡识别为障碍物）
2.tsp相关程序更新（未规划出最优路径）
3.定位程序，规划程序更新（除1cm停靠的车辆抖动）
4.建图程序，定位程序更新（建图点云加密，提高定位得分）

1.3.9 2021/03/13
1.雷达检测程序更新（下坡识别为障碍物）
2.tsp相关程序更新（未规划出最优路径）
3.定位程序，规划程序更新（除1cm停靠的车辆抖动）
4.定位配置文件更新（导航过程中定位丢失）

1.4.0 2021/03/13
1 更新上一版本新功能未生效的bug

1.4.1 2021/03/17
1.更新上版本下坡时别为障碍物的问题，雷达检测程序更新（下坡识别为障碍物）
2.车辆在宽敞面上回充点电

1.4.2 2021/03/23（该版本地图编辑器最新版本：3.3.4）
1.修改矢量图上传后，需重启车辆，才能规划路径（已修改为无需重启，可直接规划）
2.PC工具打点后可直接将停靠点连接至路线上，缩短导航上下路线的时间
3.到点时间可配置超时时间（5cm和1cm可以分别设置，当前设置都为60s）
4.地图编辑器工具修改（识别停靠点以及自动生成路ID）
5.控制程序更新（修改到点逻辑，解决原地摆动的问题）
注：在将路线连接停靠点时，停靠点不要打在三条以及三条以上路线交叉口的交点，会影响规划路径的算法

1.4.3 2021/03/25（该版本地图编辑器最新版本：3.3.5）

1.自主过程中定位丢失后自动恢复
2.采的点类型可以在1cm，5cm和充电点之间切换
3.到达充点电后，后退距离改为可配置，当前默认为不后退
4.到达充电点后，若充电桩未上电，上传错误码415（因其他原因导致导航失败）


1.4.4 2021/03/25
修复上一个版本建图参数配置错误的bug

1.4.5 2021/03/30（分别为最新地图编辑器版本3.3.5和最新app版本2.1.7）
1.定位程序，规划程序更新（精准停靠5cm精度调整为3cm精度）。
2.初始化定位程序更新（定位得分质量去掉缓存，实时更新车辆真实定位得分）。
3.规划程序更新（车辆将充电桩识别为低矮障碍，导致无法到达充电点）。
4.CI程序更新（车辆在到达充点电后，若未充上电，上报错误码415）。


# 这个仓库放一些相关开源资料和代码。
以后我们的工程文件和代码工作尽量都会在这里同步和交流。git上的organization有很多功能。   
最后有git的一些操作说明。
## 一、一些值得学习的仓库链接：
1.这是国际上最屌的飞控系统，群机协同。膜拜：https://github.com/ZJU-FAST-Lab/Fast-Drone-250.git        
2.这是乐鑫ESP32的开源无人机方案https://github.com/espressif/esp-drone.git     
3.港大的规划代码开源https://github.com/HKUST-Aerial-Robotics/Fast-Planner.git     
4.推荐去看一下正点原子的开源无人机项目：http://47.111.11.73/docs/fouraxis-fly/wukong.html     
5.PX4飞控模块，相当于无人机的大脑。https://github.com/PX4/PX4-Autopilot     
6.ArduPilot开源自动驾驶系统：https://github.com/ArduPilot/ardupilot.git            
这个大疆早年无人机也在用的自动驾驶系统，我还没看懂，加油。      
7.https://github.com/mavlink/mavros.git    
MAVROS是一个在 ROS（Robot Operating System）系统上运行的 MAVLink 协议接口，它允许 ROS 与 MAVLink 兼容的自动驾驶飞行器（如 PX4 和 ArduPilot）进行通信。    
8. https://github.com/gazebosim/gazebo-classic.git    
Gazebo Classic仿真器软件。咱可以用来进行无人机开发和测试。      
它的仿真环境，可以模拟物理交互、传感器反馈和多种复杂的环境场景。    
2024.11.11zpy



## 二、目前主要的进展黑板：
1. 我们之前有一个stm32f103控制的无人机，但它功能有限，所以我们想换板子实现更多功能。
2. 现在的选择有：  
  (1)树莓派    
  (2)ESP32    
  (3)STM32   
  (4)NVIDA Jsetonano     
  (5)...
4. 我们可以自己设计一个板子，目前正在和罗勇老师交流。仓库里倒是有开源的原理图和PCB，物联的可以先看看。     
5. 人员情况：    
  可以推荐有技术的一起。
6. 主要目的：   
  申报大创等一系列经费很足的比赛和项目。
大创寒假写申报书，其他比赛还需要大家一起提建议参不参加。    
  申请专利和会议论文：新型实用专利比较好申请，学院其他项目都很顺利就申请到了。会议论文也类似。并且，根据研究生学长的说法，未完全设计完毕之前也可以先进行申报。
7. 排名问题：目前没有排第一的，多劳多得，按贡献排名，以后也想这样。     
### 11月8号更新
9. 目前想法是一人负责一个模块，确定好后在讨论区说。
10. 准备25年5月的一个国际会议，这个含金量比较高，会被EI和北大核心收录。
    ![image](https://github.com/user-attachments/assets/7c8db92c-5dfe-41a5-9158-2b2dd30670da)
    加油。

## 三、Git_tips：     
  ##### 分支分为本地分支和远程分支两种。     
        
  1、当执行提交操作后，只是提交到了本地分支，需要推送操作，才能推送到远程（仓库）分支。   
  
  2、推送到远程分支后，就涉及到最重要的操作，代码审查（PULL REQUESTS)和分支合并。   
  
  3、严谨来说，每个人一个分支，推送到远程仓库后，**需要在仓库首页申请NEW PULL REQUESTS请求**，
  选择你想要合并的两个分支，同时，右侧可以指定审查人等一系列十分cooperative的操作。   
  
  **4、完成PULL REQUESTS后，分支就被合并到主分支了。**   
  
  5、记得PULL拉取和同步远程仓库代码到本地分支，不然无法同步别人的代码操作。   
  
  一般main分支是代码最稳定的版本。   
  ![failedtoopen](Images/git操作解释1.png)   
        
  ![failedtoopen](Images/git操作解释2.png)
  2024.11.5.zpy

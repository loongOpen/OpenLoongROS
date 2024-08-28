## 项目名称
> OpenLoongROS

## 运行条件
> 电脑配置 
* 安装与电脑适配的英伟显卡
* ubuntu 20.04 及以下
* ROS 1.0
* gazebo


## 运行说明
> 1 创建ros工作空间
* 参考链接：http://t.csdnimg.cn/2ZUav
  创建src文件，放置功能包源码：mkdir -p ~/catkin_ws/src
  进入src文件夹： cd ~/catkin_ws/src
  初始化文件夹：catkin_init_workspace
  编译工作空间： cd ~/catkin_ws/;
                catkin_make
  设置环境变量：echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
               source ~/.bashrc
> 2 下载编译源码
* git clone https://atomgit.com/openloong/OpenLoongROS.git
* 将 OpenLoongROS 里面的两个文件夹，复制到上面创建的ros工作空间(~/catkin_ws/src)中
* 编译： cd ~/catkin_ws/;
        catkin_make
        
> 3 运行程序
* 控制部分：roslaunch azureloong_control azureloong_control.launch
* gazebo仿真：roslaunch azureloong_description gazebo.launch

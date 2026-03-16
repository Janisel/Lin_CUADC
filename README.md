教程：
1、克隆仓库

方式一：克隆为独立的工作空间（推荐）：

在 home 目录下或任意想要存放的路径
git clone https://github.com/Janisel/Lin_CUADC.git cuadc_ws
cd cuadc_ws
这样会得到一个完整的 cuadc_ws 文件夹，里面包含 src/ 和 .git/

方式二：克隆到已有工作空间的 src 下
如果已经有一个工作空间（例如 ~/cuadc_ws），可以：
cd ~/cuadc_ws/src
git clone https://github.com/Janisel/Lin_CUADC.git cuadc

2、安装依赖
进入工作空间根目录
rosdep install --from-paths src --ignore-src -r -y

3、编译工作空间
进入工作空间根目录
catkin_make

4. 设置环境变量
每次使用前，需要 source 工作空间的 setup 文件
source devel/setup.bash
也可以将这一行添加到 ~/.bashrc 中，避免每次手动输入（问AI）

# 如果找不到cuadc的功能包，说明没有source，最好直接添加到~/.bashrc

5、进行仿真



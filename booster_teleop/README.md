# 1 环境安装配置

1.  从[T1说明书-V1.1](https://booster.feishu.cn/wiki/UvowwBes1iNvvUkoeeVc3p5wnUg)获取booster_robotics_sdk库，参考`README.md`把python binding api 安装到本地

2. 将avp_teleop的包安装到本地，用于avp的通讯，进入[booster_avp_teleop](https://github.com/LufanM/booster_avp_teleop)项目目录

   ```bash
   pip install -e .
   ```

3. 参考AVP的APP安装，将app安装至AVP中，具体安装请参考章节3

4. 该仓库使用了pinocchio库进行ik解算，所以需要安装pinocchio环境，目前booster_robotics_sdk的python binding api不支持安装在conda的环境中，所以请把pinocchio库安装到本地环境，具体安装请参考章节2

   

# 2 pinocchio环境问题

## 2.1 安装步骤：

* 在虚拟环境中，直接使用conda-forge进行安装，不会有啥依赖问题，直接：`conda install pinocchio=3.1.0 numpy=1.26.4 -c conda-forge`即可

* 在本地环境中，需要进行如下安装，会出现较多依赖问题：该仓库需要用该库进行优化求解，pinocchio对该库进行了嵌入所以

  1. `pip install pin==3.7.0`，把pinocchio库需要的依赖会一并安装。

  2. 安装一些工具库

     ```
     `pip install casadi==3.7.0  meshcat==0.3.2`
     ```

  3. 运行程序 `python booster_ik.py`会发现报错

     ```
     Traceback (most recent call last):
       File "/home/master/Downloads/mo/booster_avp_teleop/booster_teleop/robot_arm_ik/booster_ik.py", line 6, in <module>
         from pinocchio import casadi as cpin
     ImportError: cannot import name 'casadi' from 'pinocchio' (/home/master/.local/lib/python3.10/site-packages/cmeel.prefix/lib/python3.10/site-packages/pinocchio/__init__.py)
     ```

     按照如下1.2.3的解法进行解决。

  

## 2.2 问题及解决办法：

在进行本地环境安装时，踩了不少依赖的坑。具体问题及解决办法如下：

### 2.2.1 包冲突问题：

* 按照[pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html)官网安装pin，参考官网使用：`python3 -m pip install pin==3.7.0`

并且安装一些依赖库后版本为：`pin==3.7.0  numpy==1.26.4 cmeel-boost==1.87.0.1`但是会报错 ，

```bash
cmeel-boost 1.87.0.1 has requirement numpy<2.4,>=2.2; python_version >= "3.10.0", but you have numpy 1.26.4.
```

可以忽略。通常安装cmeel-boost的时候会自动安装numpy>2.2以上的版本，但是我们的程序很多都是在1.2*版本下编译的会出错，这种情况直接`pip3 install numpy==1.26.4`即可，忽略上述报错。

* 如果采用降级cmeel-boost来解决numpy的冲突，会带来新的问题。如果cmeel-boost低于1.87会出现很多依赖冲突，如下：故建议采用上述方法解决，当然如果你有好的解决numpy冲突问题，欢迎留言。作者暂时没有好的办法解决，好在不影响实机的程序执行。

```bash
ERROR: pip's dependency resolver does not currently take into account all the packages that are installed. This behaviour is the source of the following dependency conflicts.
coal 3.0.1 requires eigenpy<4,>=3.10, but you have eigenpy 3.5.1 which is incompatible.
coal-library 3.0.1 requires eigenpy<4,>=3.10, but you have eigenpy 3.5.1 which is incompatible.
eigenpy 3.5.1 requires cmeel-boost~=1.83.0, but you have cmeel-boost 1.87.0.1 which is incompatible.
hpp-fcl 2.4.4 requires cmeel-boost~=1.83.0, but you have cmeel-boost 1.87.0.1 which is incompatible.
```



### 2.2.2 系统动态库缺失，可能由于某些不安全的卸载导致如下问题：

```bash
ImportError: libboost_python310.so.1.86.0: cannot open shared object file: No such file or directory #错误
```

而这个包无法通过`sudo apt install libboost-python* `安装，因为只提供了最高到1.83版本的包。所以直接[官网](https://www.boost.org/releases/1.86.0/)下载后安装：

```bash
# 1. 解压
tar -xzf boost_1_86_0.tar.gz
cd boost_1_86_0

# 2. 安装依赖（Ubuntu/Debian）
sudo apt update
sudo apt install build-essential g++ python3-dev libpython3.10-dev

# 3. 配置构建参数
./bootstrap.sh --with-libraries=python --with-python=python3.10 --with-python-root=/usr

# 4. 编译与安装
./b2 -j$(nproc)
sudo ./b2 install

sudo ldconfig

# 验证
sudo find / -name 'libpinocchio_collision.so*' 2>/dev/null
```

**系统包查看方法：**

```bash
sudo find / -name 'libpinocchio_collision.so*' 2>/dev/null
```

### 2.2.3 报错无法从pinocchio中找到casadi：

```bash
Traceback (most recent call last):
  File "/home/master/Downloads/mo/booster_avp_teleop/booster_teleop/robot_arm_ik/booster_ik.py", line 6, in <module>
    from pinocchio import casadi as cpin
ImportError: cannot import name 'casadi' from 'pinocchio' (/home/master/.local/lib/python3.10/site-packages/cmeel.prefix/lib/python3.10/site-packages/pinocchio/__init__.py)
```

或者

```bash
ImportError: /lib/x86_64-linux-gnu/libstdc++.so.6: version `GLIBCXX_3.4.31' not found (required by /home/molufan/.local/lib/python3.10/site-packages/cmeel.prefix/lib/python3.10/site-packages/pinocchio/../../.././libboost_filesystem.so.1.86.0)
```

**解法：**

尝试安装包来解决这个问题我是失败了。

所以参考官网的[linux](https://stack-of-tasks.github.io/pinocchio/download.html)安装步骤，通过robotpkg把pin安装下来，然后把环境变量保存：就解决这个问题了。

```bash
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH # Adapt your desired python version here
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

这样操作之后，可能还会报错`cmeel-boost 1.87.0.1 has requirement numpy<2.4,>=2.2; python_version >= "3.10.0", but you have numpy 1.26.4.`。忽略掉即可

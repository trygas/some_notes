# VINS

### 总结

​	VINS代码主要包含在两个文件中，分别是feature_tracker和vins_estimate，feature_tracker就像文件的名字一样，总体的作用是接收图像，使用KLT光流算法跟踪；vins_estimate包含相机和IMU数据的前端预处理（也就是预积分过程）、单目惯性联合初始化（在线的标定过程）、基于滑动窗口的BA联合优化、全局的图优化和回环检测等。

​	整体分为5部分。

​	第一部分为Measurement Preprocessing：观测值数据预处理，包含图像数据跟踪IMU数据预积分。

​	第二部分为Initialization：初始化，包含单纯的视觉初始化和视觉惯性联合初始化。

​	第三部分为Local Visual-Inertia BA and Relocalization：局部BA联合优化和重定位，包含一个基于滑动窗口的BA优化模型。

​	第四部分为Global Pose Graph Optimization：全局图优化，只对全局的位姿进行优化。

​	第五部分为Loop Detection：回环检测。

### 一、Feature_tracker文件夹中

1. 论文内容：每当进入新的图像，都会使用KLT稀疏光流法进行跟踪，同时提取100-300个角点信息。（可能：角点是用来建立图像，光流跟踪是用来快速定位。）同时在这里还进行了关键帧的选取，主要是两个剔除关键帧的策略，分别是平均视差法和跟踪质量法。平均视差法：如果当前帧的和上一个关键帧跟踪点的平均视差超出了一个设定的阈值，就将当前帧设为关键帧。这里有一个问题，就是旋转和平移都会产生视差（不只是平移哦），当出现纯旋转的时候特征点无法被三角化，无法计算出旋转值，也就无法计算跟踪点间的平均视差，为了解决这一问题，采用短时的陀螺仪观测值来补偿旋转，从而计算出视差，这一过程只应用到平均视差的计算，不会影响真实的旋转结果。
2. 具体代码实现：主要负责图像角点提取和光流跟踪，只有一个主线程。主要是三个源程序，分别是feature_tracker、feature_tracker_node以及parameters。feature_tracker_node是特征跟踪线程的系统入口，feature_tracker是特征跟踪算法的具体实现，parameters是设备等参数的读取和存放。

##### feature_tracker_node.cpp系统入口

- 进入main函数，调用readParameters(n)函数，读取参数。

- 读取相机内参。

- 判断是否是鱼眼，去除边缘噪声。

- 订阅话题和发布话题，监听IMAGE_TOPIC(/cam0/image_raw0)，有图像发布到这个话题上时，执行回调函数，直接进入到回调函数中接收图像。

  - 频率控制，保证每秒钟处理的image不多于FREQ，控制在10hz以内。

  - 处理单目相机，读取图像数据。

    - 如果图像太亮或者太黑，则让图像进行自适应直方图均衡化。如果正常，则设置为当前图像。

    - 在读取图像的时候进行光流跟踪和特征点的提取。这里涉及到3个img(prev_img, cur_img, forw_img)和pts(prev_pts,cur_pts, forw_pts)，两者是相似的。刚开始看不是太好理解，cur和forw分别是LK光流跟踪的前后两帧，forw才是真正的“当前”帧，cur实际上是上一帧，而prev是上一次发布的帧。如果不是第一帧，调用calcOpticalFlowPyrLK()跟踪cur_pts到forw_pts,根据status,把跟踪失败的点剔除(注意:prev, cur,forw, ids, track_cnt都要剔除),这里还加了个inBorder判断,把跟踪到图像边缘的点也剔除掉。在光流追踪成功就记被追踪+1，数值代表被追踪的次数，数值越大，说明被追踪的就越久。

    - 如果不需要发布特征点,则到这步就完了,把当前帧forw赋给上一帧cur, 然后退出.如果需要发布特征点(PUB_THIS_FRAME=1), 则执行下面的步骤。

    - 先调用rejectWithF()对prev_pts和forw_pts做ransac剔除outlier.(实际就是调用了findFundamentalMat函数),。

    - 调用setMask(), 先对跟踪点forw_pts按跟踪次数降排序, 然后依次选点, 选一个点, 在mask中将该点周围一定半径的区域设为0, 后面不再选取该区域内的点. 有点类似与non-max suppression, 但区别是这里保留track_cnt最高的点。

    - 在mask中不为0的区域,调用goodFeaturesToTrack提取新的角点n_pts, 通过addPoints()函数push到forw_pts中, id初始化-1,track_cnt初始化为1。（这里提取的角点可以用在下一帧来的时候进行光流跟踪）

    - 将特征点矫正（相机模型camodocal）后归一化平面的3D点（此时没有尺度信息，3D点p.z=1），像素2D点，以及特征的id，u、v方向上的速度封装成ros的sensor_msgs::PointCloud消息类型的feature_points实例中；将图像封装到cv_bridge::CvImageConstPtr类型的ptr实例中。

    - 发布消息的数据

      pub_img.publish(feature_points);

      pub_match.publish(ptr->toImageMsg())

      将处理完的图像信息用PointCloud实例feature_points和Image的实例ptr消息类型，发布到"feature"和"feature_img"的topic（此步骤在main函数中完成）

至此，已经将图像数据包装成特征点数据和图像数据发布出来了，下面就是在开一个线程，发布一个话题，接收这两种消息，也就是下面的vins_esitimate文件中做的事。

### Vins_estimate文件夹中

##### estimator_node.cpp系统入口

- 首先初始化设置节点vins_estimator，同时读取参数和设置相应的参数，为节点发布相应的话题，为节点订阅三个话题，分别用来接收和保存IMU数据、图像特征数据和原始图像数据，分别是在三个回调函数中imu_callback、feature_callback和raw_image_callback，每当订阅的节点由数据送过来就会进入到相应的回调函数中。
- 在这imu_回调函数中,将IMU数据保存到imu_buf中,同时预测未考虑观测噪声的p,v,q值,同时发布最新的IMU测量值消息,这里计算得到的pvq是估计值(没有观测噪声和偏置的结果),作用是与下面预积分计算得到的pvq(考虑了观测噪声和偏置)做差得到残差.同时唤醒process线程中获取观测值的函数,
  - 在predict()函数中,主要是计算tmp_P(位置),tmp_V(速度),tmp_Q(旋转),见公式(2)(以下的见公式是在崔华坤的文档中的公式)
- 在feature_callback()回调函数中将最新的特征点数据放入缓冲区feature_buf中.
- 如果进行闭环检测,后续才会对原始图像处理,将原始图像放入缓冲区image_buf.
- 多线程处理process(),如果有闭环检测,则会多线程处理process_loop_detection(),process_pose_graph().

##### process线程

- getMeasurements()函数,对IMU和图像数据进行初步对齐,使得一幅图像对应多组IMU数据,并确保图像对应时间戳内所有IMU数据.

  - 如果最新的IMU的数据时间戳小于最旧特征点的时间戳,则等待IMU刷新,如果最旧的IMU数据时间戳大于最旧特征时间戳,则弹出旧图像.
- 调用send_imu()函数,发送IMU数据进行预积分.
  - 获取imu加速度和角速度放入processIMU()函数.(这里的IMU数据减了bias,不过都是0)
  - 调用processIMU()函数进行预积分.
    - 假如是第一帧,则只给acc_0,gyr_0赋值,初始化pre_integrations[0]
    - 每一帧都需要初始化一个预积分,如果不是第一帧,则要开始预积分.pre_integration调用push_back()函数存入时间,这一帧的加速度和角速度.
      - 调用propagate()函数,计算两个关键帧之间IMU测量的变化量.
        - 调用midPointIntegration()函数使用中值积分法.得到状态变化量,雅克比矩阵和协方差矩阵.这里面计算的delta_q是这段时间内的变化量.
    - 然后tmp_pre_integration里面也会开始预积分,和pre_integration不一样的是tmp_pre_integration里面存的是当前帧的预积分,而pre_integration里面存了许多帧的预积分.
    - 最后计算Rs,Ps,Vs,这些是整体的R,P,V.
- 接着,取出对应的Feature数据,调用processImage()函数.
  - 检测两帧之间的视差决定是否作为关键帧,同时添加之前检测到的特征点到Feature容器中,计算每一个点跟踪的次数,以及它的视差.
    - 每个Feature有可能出现在多个帧中,将特征放入Feature容器中(这个容器存的是每个Feature出现的Frame)
    - 如果不是第一帧或者当前帧中的特征点在Feature容器中有相同的特征点数目小于20个,则直接判断当前帧为关键帧.
    - 如果上一步没判断为关键帧,在这一步计算视差.遍历每个特征点判断当前特征点至少有两帧观测到.判断倒数第二帧和倒数第三帧的共视关系,实际上是计算两帧中的特征点在归一化平面上的距离.统计视差和以及个数,计算平均视差,如果大于阈值,则边缘化最老的帧,否则不是关键帧,边缘化上一帧.

  - 将图像数据和时间存入图像帧类中:首先将数据和时间保存到图像帧的对象imageframe中,(ImageFrame对象中包含特征点,时间,位姿R,t,预积分对象pre_integration,是否为关键帧),同时将临时的预积分值保存到此对象中,最后将图像帧的对象imageframe保存到all_image_frame对象中,再更新临时预积分初始值.

  - 标定相机和IMU的外参.
    - 获取两帧之间特征点关系.
    - 计算camera和IMU之间的旋转偏移常量.
      - 通过特征点匹配点计算旋转矩阵R放入Rc(这个变量存的是特征点法算出的旋转矩阵R).

      - 计算Rimu(这是IMU的旋转矩阵),ric是imu到camera的旋转矩阵,所以计算出了Rc_g(这个变量的含义是通过IMU估算相机的两帧之间的旋转矩阵).

      - 然后通过这之前的帧的相机IMU关系构建$(R(q^{b_{k+1}}_{b_k}) - L(q^{c_{k+1}}_{c_k}))q^c_b = 0$公式,然后通过最小二乘方法计算IMU到camera的旋转矩阵.这里也有一个技巧,如果特征点计算的Rc和IMU计算的Rimu相差较大,则这个矩阵快的权重较小.

      - 旋转初始化成功判定依据：如果IMU三轴充分旋转，那么4*4矩阵$Q_N$的null space零空间的秩为1，可以得到稳定解；

        如果旋转退化，则$Q_N$零空间的秩大于1，使用SVD分解，判断倒数第二小（第三大）的奇异值是否大于阈值。

        因此，旋转标定初始化的关键在有**充分的三轴旋转**。

        在VINS代码里,还要确定当前帧大于WINDOW_SIZE的数目.

  - 如果frame_count的数目达到了WINDOW_SIZE(保证有足够的帧进行初始化),并且当前帧的时间大于0.1,则开始进行视觉的结构初始化.初始化采用视觉和 IMU 的松耦合方案，首先用 SFM 求解滑窗内所有帧的位姿，和所有路标点的 3D 位置，然后跟 IMU 预积分的值对齐，求解重力方向、尺度因子、陀螺仪 bias 及
    每一帧对应的速度。

    - 首先保证IMU的充分运动.如果线加速度的标准差大于0.25,则代表imu充分激励,足够初始化.
    - 纯视觉初始化.求解关键帧的位姿和特征点坐标.
      - 首先构建SFMFeature对象sfm_f，SFMFeature数组中包含了特征点状态（是否被三角化），id，2d点，3d坐标以及深度，将特征管理器中的特征信息保存到SFMFeature对象sfm_f中sfm_f.push_back(tmp_feature)。
      - 调用relativePose()函数,从对极约束中的F矩阵回复R和t.
        - 首先获取第i帧和最后一帧的特征匹配corrs(这个特征匹配是归一化平面上的点),判断corrs有没有大于20个点,再判断视差是否大于一个阈值,则求解这一帧的R和T.(这个帧是L帧,并且是整个系统中的初始帧)
      - 全体SFM初始化全部初始帧中的相机位置和特征点空间3D位置.
        - 把relativePose()函数找到的第L帧作为初始位置,最后一帧的pose为relative_R,relative_L.
        - 首先进行第L帧和最后一帧的三角化,得到共视的特征点.
        - 然后对L帧之后的帧都和最后一帧进行PnP匹配,再三角化一些点.
        - 然后再以L帧为参考帧,对后面的帧恢复3D点.(这里面的pose存的是对最后一帧的R和T)
        - 再以L帧为参考帧,恢复0-L帧的姿态和3D点.
        - 再根据以上的点,恢复其他的特征点.
        - 建立全局BA,优化位姿和特征点.(第L帧的R和translation,最后一帧的translation都是固定值,不进行优化)
      - 对于非滑动窗口的所有帧,提供一个初始的R,T,然后solvePnP求解pose.
        - 首先要赋给一个初值.(如果是滑动窗口中的R和T,则直接用上面求出的R乘外参矩阵)
        - 然后根据这个初值调用solvePnP求解pose.
      - 将视觉和IMU的数据对齐.
        - 估计陀螺仪bias.
          - 目标函数为视觉给出的相邻帧间旋转应该等于IMU预积分的旋转值.解除这个函数就能求出陀螺仪的bias,然后我们需要调用repropagate()函数对所有帧的IMU预积分值重新计算.
          - 然后在repropagate()函数中,我们需要对这段时间内的陀螺仪的bias进行更新.在这里会调用propagate()函数(我们在上面sendIMU数据的时候就存了每一帧的dt,acc,gyr数据,在这里我们就用到了).
            - 我们调用midPointIntegration()函数进行积分.
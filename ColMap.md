# ColMap

### Match

​	在ColMap代码中,总共有好几种Match模式,分别为ImagePairsFeatureMatcher(在一些图中两张图之间的匹配),TransitiveFeatureMatcher(根据现有的匹配图像进行匹配),SpatialFeatureMatcher(使用先前的位置信息将图像和空间中最近邻居匹配),VocabTreeFeatureMatcher(用字典树对每张图像上最近邻居进行匹配),SequentialFeatureMatcher(连续图像进行匹配,和之前帧进行匹配),ExhaustiveFeatureMatcher(穷举匹配对每个块对每个块进行匹配).

##### 代码流程

- 首先创建一个方法的对象,以ImagePairsFeatureMatcher为例.然后调用Run()函数,在这个对象中会有一个成员变量为SiftFeatureMatcher,在Run()函数中会调用该成员变量的Match()函数.
- 我们可以看到在SiftFeatureMatcher对象中有matcher_queue_ 对象,在Match()函数中,首先会在数据库中搜寻这些图像是否已经匹配好了.如果没有,就会将这些图像对放入matcher_queue_ 对象中.
- 在SiftFeatureMatcher对象中,有一个matcher_ 对象,这个对象会在SiftFeatureMatcher构造函数中初始化.其中放入的是我们matcher的方法对象,例如SiftCPUFeatureMatcher对象.在这个SIftCPUFeatureMatcher中放入了matcher_queue_ 的应用,传入的是matcher_queue_ 的指针.
- 现在让我们回到一开始ImagePairsFeatureMatcher构造函数,在这个构造函数中,我们会调SiftFeatureMatcher构造函数,然后在ImagePairsFeatureMatcher中的Run()函数中,会调用SiftFeatureMatcher类的Setup()函数,这个时候就开启了SiftCPUFeatureMatcher中的Run()函数线程,只要matcher_queue_ 指针有传值进去,就会开始进行匹配.

  - 接着我们开始匹配,首先我们计算Sift描述子的距离矩阵,这里我们采用了计算余弦相似度的思想.我们首先找第一张图到第二张图的匹配(第二好的点是最好的点距离的max_ratio倍).然后我们用一样的方法找第二张图到第一张图的匹配.然后判断这两次匹配是否是一样的匹配.我们将找到的匹配输出SiftFeatureMatcher类中的verifier_queue_成员变量中.这个成员变量同样是指针传递,传入到TwoViewGeometryVerifier类中,接下来调用该类的Run()函数.
- 在TwoViewGeometryVerifier类Run()函数中,我们将一直循环搜寻inliers,直到失败.

  - 在这个Run()函数中,我们首先要用归一化平面上的匹配点估计两帧之间的Essential矩阵,用图像上的匹配点估计Fundamental矩阵和Homography矩阵.在计算这三种矩阵的时候都使用了LORANSAC的方法,都会筛选一部分outliers,接下来计算这三种方法inliers的比例.
  - 按照Essential,Fundamental,Homography的顺序依次检测比例,内点数目,是否成功找出数目最大的inliers和mask,并且分出几类进行判断(CALIBRATED为Essential矩阵,UNCALIBRATED为Fundamental矩阵,PLANAR_OR_PANORAMIC为Homography)
  - 如果找出了最多的inliers和mask,就根据mask筛选出outliers.
  - 检测inliers是否是由watermark造成的.
  - 将这次的模型放入two_view_geometries中.
  - 从matches中剔除outliers.
- 将two_view_geometries输出guided_matcher_queue_ 中.
- 运行GuidedSiftCPUFeatureMatcher中的Run()函数.

  - 在这个函数中,我们将利用到在two_view_geometry中得到的检测类型.如果是CALIBRATED或者UNCALIBRATED类型的,我们就用极限约束判断,如果是PLANAR_OR_PANORAMIC类型的,我们就用Homography去判断.
  - 用这个标准再去判断描述子矩阵的距离,满足标准的距离为0.
  - 接下来我们进行的匹配就是最开始的那种匹配(在SiftCPUFeatureMatcher中的匹配)

### EstimateRelativePose

- 如果上面返回的检测类型为CALIBRATED或者UNCALIBRATED,则调用PoseFromEssentialMatrix()函数,从Essential矩阵分解pose.
- 如果上面返回的检测类型为PLANAR,PANORAMIC或者PLANAR_OR_PANORAMIC,则调用PoseFromHomographyMatrix()函数,从Homography矩阵分解pose.

### LO-RANSAC

- 随机选择
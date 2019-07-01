[TOC]

# VINS公式推导

### 名字定义

$( \cdot)^{w}$:世界坐标系

$(\cdot)^{b}$:本体坐标系

$(\cdot)^{c}$:相机坐标系

$q_{b}^{w}, p_{b}^{w}$:代表从本体坐标系到世界坐标系的旋转和平移

$b_k$:第k个图像时的本体坐标系

$c_k$:第k个图像时的相机坐标系

$g_{w}=[0,0, g]^{T}$:世界坐标系上的重力向量

$\left(^{\wedge}\right)$:表示某一具体量的噪声测量值或估计值

### 视觉处理前端

对每一幅新图像，KLT稀疏光流算法对现有特征进行跟踪，同时检测新的角点特征以保证每个图像中特征的最小数目。该检测器通过设置两个相邻特征之间的像素的最小间隔来执行均匀的特征分布。

在这个步骤中还选择了关键帧。总共有有两个关键帧选择标准，

- 第一个是与上一个关键帧的平均视差。如果在当前帧和最新关键帧之间跟着的特征点的平均视差超出某个特定阈值，则视为新的关键帧。值得注意的是，旋转也会产生视差，但是特征点在纯旋转的时候无法三角化。为了避免这种情况，在计算视差的时候我们会使用陀螺仪测量值的短时积分来补偿旋转。
- 另一个标准是跟踪质量，如果跟踪的特征数量低于某一阈值，则将此帧视为新的关键帧。

### 预处理

#### 连续时间

IMU的原始陀螺仪和加速度计$\hat{w}$和$\hat{a}$测量结果:
$$
\begin{align} \hat{\mathbf{a}}_{t} &=\mathbf{a}_{t}+\mathbf{b}_{a_{t}}+\mathbf{R}_{w}^{t} \mathbf{g}^{w}+\mathbf{n}_{a} \\ \hat{\omega}_{t} &=\omega_{t}+\mathbf{b}_{w_{t}}+\mathbf{n}_{w} \end{align}
$$
IMU测量值是在本体坐标系中测量的,是平衡重力和平台动力的合力,并受到加速度偏置$b_a$,陀螺仪偏置$b_w$和附加噪声的影响.

假设加速度计和陀螺仪中的附加噪声为高斯噪声,$ n_{a} \sim N\left(0, \sigma_{a}^{2}\right), n_{w} \sim N\left(0, \sigma_{w}^{2}\right)$.

加速度计偏置和陀螺仪偏置被建模为随机游走,其导数为高斯性的,$n_{b_{a}} \sim N\left(0, \sigma_{b_{a}}^{2}\right), n_{b_{w}} \sim N\left(0, \sigma_{b_{w}}^{2}\right)$.
$$
\dot{\mathbf{b}}_{a_{t}}=\mathbf{n}_{b_{a}}, \quad \dot{\mathbf{b}}_{w_{t}}=\mathbf{n}_{b_{w}}
$$
对于本体坐标系$b_k$和$b_{k+1}$的两个时刻,位置速度和方向状态可以在时间间隔$[t_k,t_{k+1}]$间,世界坐标系的数据通过以下关系测量值传递:
$$
\begin{align} \mathbf{p}_{b_{k+1}}^{w}=\mathbf{p}_{b_{k}}^{w} +\mathbf{v}_{b_{k}}^{w} \Delta t_{k} +\iint_{t \in\left[t_{k}, t_{k+1}\right]}\left(\mathbf{R}_{t}^{w}\left(\hat{\mathbf{a}}_{t}-\mathbf{b}_{a_{t}}-\mathbf{n}_{a}\right)-\mathbf{g}^{w}\right) d t^{2} \end{align}
$$

$$
\mathbf{v}_{b_{k+1}}^{w}=\mathbf{v}_{b_{k}}^{w}+\int_{t \in\left[t_{k}, t_{k+1}\right]}\left(\mathbf{R}_{t}^{w}\left(\hat{\mathbf{a}}_{t}-\mathbf{b}_{a_{t}}-\mathbf{n}_{a}\right)-\mathbf{g}^{w}\right) d t
$$

$$
\mathbf{q}_{b_{k+1}}^{w}=\mathbf{q}_{b_{k}}^{w} \otimes \int_{t \in\left[t_{k}, t_{k+1}\right]} \frac{1}{2} \boldsymbol{\Omega}\left(\hat{\boldsymbol{\omega}}_{t}-\mathbf{b}_{w_{t}}-\mathbf{n}_{w}\right) \mathbf{q}_{t}^{b_{k}} d t
$$

$$
\Omega(\omega)=\left[ \begin{array}{cc}{-\lfloor\omega\rfloor_{ \times}} & {\omega} \\ {-\omega^{T}} & {0}\end{array}\right],\lfloor\omega\rfloor_{ \times}=\left[ \begin{array}{ccc}{0} & {-\omega_{z}} & {\omega_{y}} \\ {\omega_{z}} & {0} & {-\omega_{x}} \\ {-\omega_{y}} & {\omega_{x}} & {0}\end{array}\right]
$$

可见,IMU状态传递需要坐标系$b_k$的旋转、位置和速度。当这些起始状态改变时，我们需要重新传递IMU测量值。特别是在基于优化的算法中，每次调整位姿时，都需要在它们之间重新传递IMU测量值。这种传递策略在计算上要求很高。为了避免重新传递，我们采用了预积分算法。

将参考坐标系从世界坐标系转变为局部坐标系$b_k$后，我们只能对线性的加速度$\hat a$和角速度$\hat w$相关的部分进行预积分，如下所示：
$$
\begin{align} \mathbf{R}_{w}^{b_{k}} \mathbf{p}_{b_{k+1}}^{w} &=\mathbf{R}_{w}^{b_{k}}\left(\mathbf{p}_{b_{k}}^{w}+\mathbf{v}_{b_{k}}^{w} \Delta t_{k}-\frac{1}{2} \mathbf{g}^{w} \Delta t_{k}^{2}\right)+\boldsymbol{\alpha}_{b_{k+1}}^{b_{k}} \\ \mathbf{R}_{w}^{b_{k}} \mathbf{v}_{b_{k+1}}^{w} &=\mathbf{R}_{w}^{b_{k}}\left(\mathbf{v}_{b_{k}}^{w}-\mathbf{g}^{w} \Delta t_{k}\right)+\boldsymbol{\beta}_{b_{k+1}}^{b_{k}} \\ \mathbf{q}_{w}^{b_{k}} \otimes \mathbf{q}_{b_{k+1}}^{w} &=\gamma_{b_{k+1}}^{b_{k}} \end{align}
$$

$$
\boldsymbol{\alpha}_{b_{k+1}}^{b_{k}}=\iint_{t \in\left[t_{k}, t_{k+1}\right]} \mathbf{R}_{t}^{b_{k}}\left(\hat{\mathbf{a}}_{t}-\mathbf{b}_{a_{t}}-\mathbf{n}_{a}\right) d t^{2}
$$

$$
\boldsymbol{\beta}_{b_{k+1}}^{b_{k}}=\int_{t \in\left[t_{k}, t_{k+1}\right]} \mathbf{R}_{t}^{b_{k}}\left(\hat{\mathbf{a}}_{t}-\mathbf{b}_{a_{t}}-\mathbf{n}_{a}\right) d t
$$

$$
\gamma_{b_{k+1}}^{b_{k}}=\int_{t \in\left[t_{k}, t_{k+1}\right]} \frac{1}{2} \Omega\left(\hat{\omega}_{t}-\mathbf{b}_{w_{t}}-\mathbf{n}_{w}\right) \gamma_{t}^{b_{k}} d t
$$

可以看到预积分项（11,12,13）可以通过将$b_{k}$视为参考帧的IMU测量值单独得到.公式只与$b_{k},b_{k+1}$中的IMU偏置相关,与其他状态无关.

**然后当我们需要进行优化的时候,因为已经转化到IMU坐标系下,每次都只需传入IMU偏置测量值,而不需要乘以$b_{k}$的旋转.**并且当偏置估计发生变化时,若偏置变化很小,则我们将$\alpha_{b_{k+1}}^{b k}, \beta_{b_{k+1}}^{b k}, \gamma_{b_{k+1}}^{b k}$按其对偏置的一阶近似来调整,否则就重新传递.

#### 离散时间

**在开始的时候,$\alpha_{b_k}^{b_{k}} , \beta_{b_{k}}^{b_{k}}$是0,$\gamma_{b_{k}}^{b_{k}}$是单位四元数.**$\alpha, \beta, \gamma$在公式(11,12,13)中的平均值是如下传递的.注意增加的噪声项$n_a,n_w$是未知的,在实现中被视为0.这样就得到了预积分的估计值,标记为$(\hat{\cdot})$
$$
\begin{align}&{\hat{\boldsymbol{\alpha}}_{i+1}^{b_{k}}=\hat{\boldsymbol{\alpha}}_{i}^{b_{k}}+\hat{\boldsymbol{\beta}}_{i}^{b_{k}} \delta t+\frac{1}{2} \mathbf{R}\left(\hat{\gamma}_{i}^{b_{k}}\right)\left(\hat{\mathbf{a}}_{i}-\mathbf{b}_{a_{i}}\right) \delta t^{2}} \\ &{\hat{\boldsymbol{\beta}}_{i+1}^{b_{k}}=\hat{\boldsymbol{\beta}}_{i}^{b_{k}}+\mathbf{R}\left(\hat{\gamma}_{i}^{b_{k}}\right)\left(\hat{\mathbf{a}}_{i}-\mathbf{b}_{a_{i}}\right) \delta t} \\ &{\hat{\gamma}_{i+1}^{b_{k}}=\hat{\gamma}_{i}^{b_{k}} \otimes\left[\begin{array}{cc}{1} \\ {\frac{1}{2}\left(\hat{\boldsymbol{\omega}}_{i}-\mathbf{b}_{w_{i}}\right) \delta t}\end{array}\right]}\end{align}
$$
i是在$[t_k,t_{k+1}]$中IMU测量值对应的离散时刻,$\delta t$是IMU测量值i和i+1之间的时间间隔.

然后讨论协方差传递问题,我们将四维旋转四元数误差项定义为围绕其平均值的扰动:
$$
\gamma_{t}^{b_{k}} \approx \hat{\gamma}_{t}^{b_{k}} \otimes\left[\begin{array}{c}{1} \\ {\frac{1}{2} \delta \boldsymbol{\theta}_{t}^{b_{k}}}\end{array}\right]
$$
其中$\delta \theta^{b_k}_t$是三维小扰动.



### 附录

##### 高斯白噪声

高斯白噪声为服从高斯分布的一种白噪声,一阶矩为常数,二阶矩随时间会发生变化.**连续的高斯白噪声**服从如下条件:
$$
\begin{array}{c}{E(n(t)) \equiv 0} \\ {E\left(n\left(t_{1}\right) n\left(t_{2}\right)\right)=\sigma_{g}^{2} \delta\left(t_{1}-t_{2}\right)}\end{array}
$$
其中$\delta(t)$表示狄拉克函数.$\sigma_g$表示高斯白噪声的方差.离散化之后,得到,
$$
n_{d}[k]=\sigma_{g d} \omega[k]
$$
其中,
$$
\omega[k] \sim N(0,1), \sigma_{g d}=\sigma_{g} \frac{1}{\sqrt{\Delta} t}
$$
证明如下,
$$
n_{d}[k]=n\left(t_{0}+\Delta t\right) \simeq \frac{1}{\Delta t} \int_{t_{0}}^{t_{0}+\Delta t} n(\tau) d \tau
$$

$$
\begin{align} E\left(n_{d}[k]^{2}\right) &=E\left(\frac{1}{\Delta t^{2}} \int_{t_{0}}^{t_{0}+\Delta t} \int_{t_{0}}^{t_{0}+\Delta t} n(\tau) n(t) d \tau d t\right) \\ &=\frac{\sigma_{g}^{2}}{\Delta t^{2}} \int_{t_{0}}^{t 0+\Delta t} \int_{t_{0}}^{t_{0}+\Delta t} \delta(t-\tau) d \tau d t \\ &=\frac{\sigma_{g}^{2}}{\Delta t} \end{align}
$$

则有$\sigma_{g d}^{2}=\frac{\sigma_{g}^{2}}{\Delta t}$,即$\sigma_{g d}=\frac{\sigma_{g}}{\sqrt{\Delta t}}$.

##### 随机游走

随机游走是一个离散模型，可以把它看做是一种布朗运动，或者将其称之为维纳过程。该模型可以看做是高斯白噪声的积分。该噪声参数一般是由传感器的内部构造、温度等变化量综合影响下的结果。
$$
\dot b_{g}(t)=\sigma_{b g} \omega(t)
$$
其中$ ω(t) $是单位的高斯白噪声，在上面提到了。如果把随机游走噪声离散化，可以写作：
$$
b_{d}[k]=b_{d}[k-1]+\sigma_{b g d} \omega[k]
$$
其中,
$$
\omega[k] \sim(0,1), \quad \sigma_{b g d}=\sigma _{b g} \sqrt{\triangle t}
$$
证明如下,
$$
b_{d}[k] \simeq b\left(t_{0}\right)+\int_{t_{0}}^{t_{0}+\Delta t} n(t) d t
$$

$$
\begin{align} E\left(\left(b_{d}[k]-b_{d}[k-1]\right)^{2}\right) &=E\left(\int_{t_{0}}^{t 0+\Delta t} \int_{t_{0}}^{t_{0}+\Delta t} n(t) n(\tau) d t d\tau\right) \\ &=\sigma_{b g}^{2} \int_{t_{0}}^{t 0+\Delta t} \int_{t_{0}}^{t_{0}+\Delta t} \delta(t-\tau) d t d \tau \\ &=\sigma_{b g}^{2} \Delta t \end{align}
$$

则有$\sigma_{b g d}^{2}=\sigma_{b g}^{2} \Delta t$,即$\sigma _{gb d}=\sigma_{b g} \sqrt{\Delta t}$

由随机游走的分布可以看出，随机游走都是在上一次的噪声的基础之上叠加了一个高斯噪声，所以下一步永远都是随机的。
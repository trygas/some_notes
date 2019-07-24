[TOC]

## 一致性和可观性及其改进方法

### 概述

在VIO中,有**四个自由度是不可观**的,**三个为世界坐标系下的平移**还有**一个为世界坐标系下的对重力向量的旋转**.但是当我们用估计的状态去估计雅克比矩阵的时候,就像EKF那样,不可观测的方向就少了一个.减少的这个自由度就是世界坐标系下对重力向量的旋转,这就会导致估计的状态会获得虚假状态,并且会导致不一致.

### VIO模型概述

我们使用EKF来融合相机和IMU的数据去估计系统状态,其中包括位姿,速度和IMU的偏置还有就是相机观测到的路标点的3D位置.在VIO框架中,我们使用两种类型的特征点.

- 一种是oppotunistic features(OFs),可以很准确并且很有效率的在很短的图像序列中被跟踪(比如KLT),但是在loop closure的时候因为和其他特征点没有那么大的区别所以不能识别出来.OFs能被用来在很短的时间内估计相机位姿(比如用在了MSCKF中),但是这中特征点不会包含在状态向量中.
- 还有一种就是distinguishable features(DFs),这种特征点数目少一点,但是在loop closure的时候能够有效被重新检测出来(比如SIFT点).DFs的3D坐标通常用来建立整个地图.

#### 系统状态和更新模型

EKF用线性时变的IMU偏置和地图上的视觉特征点来估计的IMU位姿和线性速度.滤波器的状态是一个(16+3N)×1的向量
$$
\begin{align} \mathbf{x}&=\left[\begin{array}{cccccccc}  ^{I}{\overline{q}_{G}^{T}} & {\mathbf{b}_{g}^{T}} & ^{G} {\mathbf{v}_{I}^{T}} & {\mathbf{b}_{a}^{T}} & {G_{I}^{T}} & { |} & ^{G}{ \mathbf{f}_{1}^{T}} & {\cdots} & ^{G}{ \mathbf{f}_{N}^{T}}\end{array}\right]^{T}
\\ &=\left[\begin{array}{ll}{\mathbf{x}_{s}^{T}} & { | \mathbf{x}_{f}^{T}}\end{array}\right]^{T}\end{align}
$$
其中$\mathbf{x}_{s}(t)$传感器平台的状态向量,$\mathbf{x}_{f}(t)$是特征地图的3N×1状态向量.其余状态就是VIO中都有的,不做赘述.

在这个状态向量中的$\mathbf{x}_{f}$包含的3D点都是DFs,$^G \mathbf{f}_{i}, i=1, \ldots, N$.并且我们不会把OFs放入地图中.但是,所有的OFs点都会通过MSCKF的方法就是处理和边缘化.我们现在来关注这个系统的连续时间模型.

##### 连续时间模型

$$
\begin{align}^{I} \dot{\overline{q}}_{G}(t) &=\frac{1}{2} \boldsymbol{\Omega}(\boldsymbol{\omega}(t))^{I} \overline{q}_{G}(t) \\ \dot{\mathbf{b}}_{g}(t) &=\mathbf{n}_{w g}(t) \\ ^G\dot{\mathbf{v}}_{I}(t) &=^{G} \mathbf{a}(t) \\ \dot{\mathbf{b}}_{a}(t) &=\mathbf{n}_{w a}(t) \\ ^G\dot{\mathbf{p}}_{I}(t) &= ^G\mathbf{v}_{I}(t) \\ ^G\dot{\mathbf{f}}_{i}(t)&=\mathbf{0}_{3 \times 1}, \quad i=1, \ldots, N \end{align}
$$

在上面的表达式中,$\boldsymbol{\omega}(t)=\left[\omega_{1}(t) \omega_{2}(t) \omega_{3}(t)\right]^{T}$是IMU坐标系下的IMU旋转速度,$^G{\mathbf{a}}$是世界坐标系下的IMU测量的加速度,并且
$$
\boldsymbol{\Omega}(\boldsymbol{\omega})=\left[\begin{array}{cc}{-\lfloor\boldsymbol{\omega} \times\rfloor} & {\boldsymbol{\omega}} \\ {-\boldsymbol{\omega}^{T}} & {0}\end{array}\right], \quad\lfloor\boldsymbol{\omega} \times\rfloor \triangleq\left[\begin{array}{ccc}{0} & {-\omega_{3}} & {\omega_{2}} \\ {\omega_{3}} & {0} & {-\omega_{1}} \\ {-\omega_{2}} & {\omega_{1}} & {0}\end{array}\right]
$$
角速度和加速度的测量值为
$$
\begin{array}{l}{\boldsymbol{\omega}_{m}(t)=\boldsymbol{\omega}(t)+\mathbf{b}_{g}(t)+\mathbf{n}_{g}(t)} \\ {\mathbf{a}_{m}(t)=\mathbf{C}\left(^{I}\overline{q}_{G}(t)\right)\left(^{G} \mathbf{a}(t)-^{G} \mathbf{g}\right)+\mathbf{b}_{a}(t)+\mathbf{n}_{a}(t)}\end{array}
$$
$\mathbf{n}_{g}$ 和 $\mathbf{n}_{a}$都是零均值高斯白噪声.$^{G} \mathrm{g}$是重力加速度.$\mathbf{C}(\overline{q})$是$\overline {q}$表示的旋转矩阵.DFs处在一个静态场景,所以倒数为0.

线性化当前估计放入上面公式,我们可以获得状态估计传播模型.
$$
\begin{align}^{I} \dot{\hat{\overline{q}}}_{G}(t) &=\frac{1}{2} \boldsymbol{\Omega}(\boldsymbol{\omega}(t))^{I} \hat{\overline{q}}_{G}(t) \\ \hat{\dot{\mathbf{b}}}_{g}(t) &=0 \\ \hat{^G\dot{\mathbf{v}}_{I}}(t) &=\mathbf{C}^{T}\left(^{I} \hat{\overline{q}}_{G}(t)\right) \hat{\mathbf{a}}(t)+^{G} \mathbf{g} \\ \hat{\dot{\mathbf{b}}}_{a}(t) &=0\\ ^G\hat{\dot{\mathbf{p}}}_{I}(t) &= ^G\hat{\mathbf{v}}_{I}(t) \\ ^G\hat{\dot{\mathbf{f}}}_{i}(t)&=\mathbf{0}_{3 \times 1}, \quad i=1, \ldots, N \end{align}
$$
其中$\hat{\mathbf{a}}(t)=\mathbf{a}_{m}(t)-\hat{\mathbf{b}}_{a}(t)$,$\hat{\omega}(t)=\omega_{m}(t)-\hat{\mathbf{b}}_{g}(t)$.

然后定义误差状态向量
$$
\tilde{\mathbf{x}}=\left[\begin{array}{llllll}^{I}{\delta \boldsymbol{\theta}_{G}^{T}} & {\widetilde{\mathbf{b}}_{g}^{T}} & ^{G} {\widetilde{\mathbf{v}}_{I}^{T}} & {\widetilde{\mathbf{b}}_{a}^{T}} & ^{G} {\widetilde{\mathbf{p}}_{I}^{T}} | ^G{\widetilde{\mathbf{f}}_{1}^{T}} & {\cdots} & ^G{ \widetilde{\mathbf{f}}_{N}^{T}}\end{array}\right]
=\left[\begin{array}{cc}{\tilde{\mathbf{x}}_{s}^{T}} & { | \widetilde{\mathbf{x}}_{f}^{T} ]^{T}}\end{array}\right.
$$
在这里面$\widetilde{\mathbf{x}}_{s}(t)$是一个15×1的状态向量,$\tilde{\mathbf{x}}_{f}(t)$是3N×1的地图误差状态.对于IMU位置,速度,偏置和地图,都是用的加法模型($\widetilde{x}=x-\hat{x}$,其中是$\hat x$是估计状态,$x$是真值).然而我们对于四元数来说使用的下面的模型,
$$
\delta \overline{q}=\overline{q} \otimes \hat{\overline{q}}^{-1} \simeq\left[\frac{1}{2} \boldsymbol{\delta} \boldsymbol{\theta}^{T} \quad 1\right]^{T}
$$
其中$\delta \overline{q}$是一个小的旋转误差,但是这个是四元数,会有奇异性.所以我们使用3×1的角度误差向量$\delta \boldsymbol{\theta}$作为误差状态向量.

线性化后的连续时间误差状态公式为
$$
\begin{aligned} \dot{\tilde{\mathbf{x}}} &=\left[\begin{array}{cc}{\mathbf{F}_{s}} & {\mathbf{0}_{15 \times 3 N}} \\ {\mathbf{0}_{3 N \times 15}} & {\mathbf{0}_{3 N}}\end{array}\right] \widetilde{\mathbf{x}}+\left[\begin{array}{c}{\mathbf{G}_{s}} \\ {\mathbf{0}_{3 N \times 12}}\end{array}\right] \mathbf{n} \\ &=\mathbf{F} \widetilde{\mathbf{x}}+\mathbf{G} \mathbf{n} \end{aligned}
$$
其中,$\mathbf{n}$是包含IMU测量噪声的状态向量.
$$
\mathbf{n}=\left[\begin{array}{cccc}{\mathbf{n}_{g}^{T}} & {\mathbf{n}_{w g}^{T}} & {\mathbf{n}_{a}^{T}} & {\mathbf{n}_{w a}^{T}}\end{array}\right]^{T}
$$
系统噪声被定义为一个零均值的高斯白噪声,那么$\mathbb{E}\left[\mathbf{n}(t) \mathbf{n}^{T}(\tau)\right]=\mathbf{Q}_{c} \delta(t-\tau)$.

##### 离散时间模型

如果是离散时间的话,那么假设IMU信号$\omega_m$和$\mathbf{a}_m$以**恒定速度**1$/ \delta t$被采样,其中$\delta t=t_{k+1}-t_{k}$.每当有一个新的IMU测量值到来的时候,状态估计值就会通过上面的公式(11-16)进行传播.为了**导出协方差传播矩阵**,我们计算出离散时间状态转移矩阵$\mathbf{\Phi}_{k+1, k}$,对于时间从$t_k$到$t_{k+1}$,作为以下矩阵微分方程的解
$$
\dot{\mathbf{\Phi}}_{k+1, k}=\mathbf{F} \mathbf{\Phi}_{k+1, k}
$$
初值为$\mathbf{\Phi}_{k, k}=\mathbf{I}_{18}$.

同样的,我们可以算出离散时间系统噪声协方差矩阵$\mathbf{Q}_{k}$为
$$
\mathbf{Q}_{k}=\int_{t_{k}}^{t_{k+1}} \mathbf{\Phi}_{k+1, \tau} \mathbf{G} \mathbf{Q}_{c} \mathbf{G}^{T} \mathbf{\Phi}_{k+1, \tau}^{T} \mathrm{d} \tau
$$
最后,我们可以获得协方差传播矩阵为(TODO:没理解)
$$
\mathbf{P}_{k+1 | k}=\mathbf{\Phi}_{k+1, k} \mathbf{P}_{k | k} \mathbf{\Phi}_{k+1, k}^{T}+\mathbf{Q}_{k}
$$

#### 测量更新模型

当相机-IMU平台移动的时候,相机能观测到OFs和DFs.我们会有三种滤波器更新

- 更新那些已经在地图中的DF
- 初始化还没在地图中的DF
- OF更新

我们首先来描述以下特征点测量模型,详细说明如何在每种情况使用它.

为了简化讨论,我们考虑一个观测到DF特征点$\mathbf{f}_i$,相机(归一化坐标)测量值为$\mathbf{z}_i$是3D点$^I{\mathbf{f}_{i}}$的透视投影,在IMU坐标系上.
$$
\mathbf{z}_{i}=\frac{1}{z}\left[\begin{array}{l}{x} \\ {y}\end{array}\right]+\boldsymbol{\eta}_{i}
$$
其中$\left[\begin{array}{l}{x} \\ {y} \\ {z}\end{array}\right]=^{I} \mathbf{f}_{i}=\mathbf{C}\left(^{I} \overline{q}_{G}\right)\left(^{G} \mathbf{f}_{i}-^{G} \mathbf{p}_{I}\right)$.

测量噪声$\boldsymbol{\eta}_{i}$为零均值高斯白噪声,协方差为$\mathbf{R}_i$,线性化误差模型为
$$
\tilde{\mathbf{z}}_{i}=\mathbf{z}_{i}-\hat{\mathbf{z}}_{i} \simeq \mathbf{H}_{i} \tilde{\mathbf{x}}+\boldsymbol{\eta}_{i}
$$
其中$\hat{\mathbf{z}}$是通过公式(23)计算出来的当前状态估计下的期待测量值,测量雅克比$\mathbf{H}_{i}$为
$$
\mathbf{H}_{i}=\mathbf{H}_{c}\left[\mathbf{H}_{\overline{q}} \quad \mathbf{0}_{3 \times 9} \quad \mathbf{H}_{\mathrm{p}} | \mathbf{0}_{3} \cdots \mathbf{H}_{\mathbf{f}_{i}} \cdots \mathbf{0}_{3}\right]
$$
其中
$$
\begin{align} \mathbf{H}_{c} &=\frac{1}{z^{2}}\left[\begin{array}{ccc}{z} & {0} & {-x} \\ {0} & {z} & {-y}\end{array}\right] \\ \mathbf{H}_{\overline{q}} &=\left\lfloor\mathbf{C}\left(^{I} \overline{q}_{G}\right)\left(^{G} \mathbf{f}_{i}-^{G} \mathbf{p}_{I}\right) \times\right\rfloor \\ \mathbf{H}_{\mathbf{p}} &=-\mathbf{C}\left(^{I} \overline{q}_{G}\right) \\ \mathbf{H}_{\mathbf{f}_{i}} &=\mathbf{C}\left(^{I} \overline{q}_{G}\right) \end{align}
$$
其中,$\mathbf{H}_c$是相机透视投影对3D点$^I \mathbf{f}_i$的雅克比矩阵,而$\mathbf{H}_{\overline{q}}, \mathbf{H}_{\mathrm{p}},$ 和 $\mathbf{H}_{\mathrm{f}_{\mathrm{i}}}$是3D点$^I\mathbf{f}_i$对$^{I} \overline{q}_{G},^{G} \mathbf{p}_{I},$ 和 $^{G} \mathbf{f}_{i}$的雅克比矩阵.

这个测量模型会用在上面三种更新方法中.对于那些已经在地图中的DFs,我们直接将公式(26-29)代入来更新滤波器.并且,我们计算出测量残差$\mathbf{r}_i$,协方差$\mathbf{S}_i$和卡尔曼增益$\mathbf{K}_i$
$$
\begin{align} \mathbf{r}_{i} &=\mathbf{z}_{i}-\hat{\mathbf{z}}_{i} \\ \mathbf{S}_{i} &=\mathbf{H}_{i} \mathbf{P}_{k+1 | k} \mathbf{H}_{i}^{T}+\mathbf{R}_{i} \\ \mathbf{K}_{i} &=\mathbf{P}_{k+1 | k} \mathbf{H}_{i}^{T} \mathbf{S}_{i}^{-1} \end{align}
$$
然后更新EKF状态和协方差为
$$
\begin{array}{l}{\hat{\mathbf{x}}_{k+1 | k+1}=\hat{\mathbf{x}}_{k+1 | k}+\mathbf{K}_{i} \mathbf{r}_{i}} \\ {\mathbf{P}_{k+1 | k+1}=\mathbf{P}_{k+1 | k}-\mathbf{K}_{i} \mathbf{S}_{i} \mathbf{K}_{i}^{T}}\end{array}
$$
每当一个新的DF被测量的时候,我们都需要初始化它进状态向量.因为一个单一的观测不能够提供足够的信息计算出一个DF特征点的3D坐标,我们需要利用多个观测去初始化.为了计算初始位置估计,不确定性和当前状态的互相关性,我们需要在一个短时间窗口中解决一个BA问题,详细间附录.

相比之下,OFs不会放入状态估计部分.相反,我们使用MSCKF-进行有效(线性复杂度)更新,该更新限制了所有观测到该特征的所有相机位姿.

### VIO可观性分析

在本节中,当执行任意运动的传感器平台检测到单点特征时,我们将会检查线性化VIO模型的可观测性.具体来说,我们首先研究并分析确定理想线性化VIO模型的四个不可观测方向.随后我们证明EKF使用的线性化VIO模型的雅克比使用当前状态估计值进行评估只有三个不可观测的方向,而对于全局坐标系下的重力向量的旋转是可观测到的.

观测矩阵$\mathbf{M}$定义如下
$$
\mathbf{M}\left(\mathbf{x}^{\star}\right)=\left[\begin{array}{c}{\mathbf{H}_{1}} \\ {\mathbf{H}_{2} \mathbf{\Phi}_{2,1}} \\ {\vdots} \\ {\mathbf{H}_{k} \mathbf{\Phi}_{k, 1}}\end{array}\right]
$$
其中,$\mathbf{\Phi}_{k, 1}=\mathbf{\Phi}_{k, k-1} \ldots \mathbf{\Phi}_{2,1}$是时间步长1到k的状态转移矩阵.而$\mathbf{H}_k$是测量值的雅克比,对应的特征点观测是时间步长k.我们注意到,因为所有的雅克比行列式都是在一个特殊的状态上估计的,
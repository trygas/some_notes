# VIO

### 为什么要VIO

- 因为单目相机做SLAM的时候是缺乏实际的尺度信息的,而且尺度还会产生漂移.
- 纯视觉SLAM系统如果不做特殊处理,无法知道惯性坐标系.
- 视觉SLAM无法得到一个准确的速度.

### VIO的好处

- IMU可以提供物理世界的尺度,短时间内能提供比较准确的位姿估计,还能估计惯性坐标系.

### 初始化的任务

- 将视觉SLAM所缺的东西都先估计出来,包括尺度,重力(方向和大小),每个pose的速度.
- 获得陀螺仪和加速度计的bias.
- (可选)获得相机和IMU之间的相对位姿关系.

### 初始化不好的缺点

- 基于滤波的SLAM系统可能很快就发散得很远了.
- 基于非线性优化的可能经常会陷入局部最优解.

### 初始化的难点

- 单目视觉SLAM中,绝对尺度和速度都是无法计算出来的,而要计算尺度和速度,系统就要做非匀速的运动.
- 而一旦动起来,重力方向相对于当前相机坐标系的方向就成了一个未知量,而我们知道IMU加速度计的读数是受重力影响的,如果重力方向未知,IMU就无法估计位姿了.
- 并且IMU的尺度信息受噪声和Bias的影响.

### 预积分

所有的SLAM问题都可以建模成状态估计问题，通过相机或者激光的约束方程，来对需要估计的状态进行约束，从而得到状态估计的最优估计。在VIO中，可以建立如下的状态估计问题：
$$
x_{i} \doteq\left[R_{i}, p_{i}, v_{i}, b_{i}\right]
$$
其中下标ii表示某个时刻，$\left(R_{i}, p_{i}\right)$表示机器人的位姿，即旋转矩阵和平移向量。$v_{i} \in \mathbb{R}^{3}$表示速度。$b_{i}=\left[b_{i}^{g}, b_{i}^{a}\right] \in \mathbb{R}^{6}$表示IMU中加速度计和陀螺仪的偏差。$X_{k} \doteq\left\{x_{i}\right\}_{i \in K_{k}}$**都是在系统运行过程中未知的，需要我们通过观测数据来进行估计的。**$K_k$代表$k$时间内所有的关键帧.

OK，那我们有哪些**观测数据**呢？当然就是相机数据和IMU数据了。用如下变量表示：
$$
Z_{k} \doteq\left\{C_{i}, I_{i j}\right\}_{(i, j) \in K_{k}}
$$
$C_i$表示图像关键帧,$I_{ij}$表示两个连续关键帧之间的IMU数据.

因此整个状态估计问题就可以建模成基于观测数据$Z_k$,求$X_k$的最大后验.
$$
\begin{array}{c}{p\left(\mathcal{X}_{k} | \mathcal{Z}_{k}\right) \propto p\left(\mathcal{X}_{0}\right) p\left(\mathcal{Z}_{k} | \mathcal{X}_{k}\right) \stackrel{(a)}{=} p\left(\mathcal{X}_{0}\right) \prod_{(i, j) \in \mathcal{K}_{k}} p\left(\mathcal{C}_{i}, \mathcal{I}_{i j} | \mathcal{X}_{k}\right)} \\ {\quad \stackrel{(b)}{=} p\left(\mathcal{X}_{0}\right) \prod_{(i, j) \in \mathcal{K}_{k}} p\left(\mathcal{I}_{i j} | \mathbf{x}_{i}, \mathbf{x}_{j}\right) \prod_{i \in \mathcal{K}_{k}} \prod_{l \in \mathcal{C}_{i}} p\left(\mathbf{z}_{i l} | \mathbf{x}_{i}\right)}\end{array}
$$

$$
\begin{array}{l}{\mathcal{X}_{k}^{\star} \doteq \arg \min _{\mathcal{X}_{k}}-\log _{e} p\left(\mathcal{X}_{k} | \mathcal{Z}_{k}\right)} \\ {=\arg \min _{\mathcal{X}_{k}}\left\|\mathbf{r}_{0}\right\|_{\mathbf{\Sigma}_{0}}^{2}+\sum_{(i, j) \in \mathcal{K}_{k}}\left\|\mathbf{r}_{\mathcal{I}_{i j}}\right\|_{\mathbf{\Sigma}_{i j}}^{2}+\sum_{i \in \mathcal{K}_{k}} \sum_{l \in \mathcal{C}_{i}}\left\|\mathbf{r}_{\mathcal{C}_{i l}}\right\|_{\mathbf{\Sigma}_{\mathcal{C}}}^{2}}\end{array}
$$

IMU测出来的角速度和加速度是收到bias和noise影响的。所以测量值等于真实值加上相应的噪声和偏差。 
其中的bias是state中的一个变量。加速度涉及到的旋转，偏差也是state中的变量。 
$$
_{\mathrm{B}} \tilde{\omega}_{\mathrm{WB}}(t)={}_{\mathrm{B}} \omega_{\mathrm{wB}}(t)+\mathbf{b}^{g}(t)+\eta^{g}(t)
$$

$$
_{\mathrm{B}} \tilde{\mathbf{a}}(t)=\mathrm{R}_{\mathrm{WB}}^{\mathrm{T}}(t)\left(_\mathrm{w} \mathbf{a}(t)-{}_{\mathrm{w}} \mathbf{g}\right)+\mathbf{b}^{a}(t)+\boldsymbol{\eta}^{a}(t)
$$

B代表本地坐标系,W代表世界坐标系,$WB$代表从本地坐标系到世界坐标系.

积分形式的动力学如下:
$$
            \dot{\mathrm{R}}_{\mathrm{WB}}=\mathrm{R}_{\mathrm{WB}}\  {_{\mathrm{B}}}\boldsymbol{\omega}_{\mathrm{WB}}^{\wedge}
$$

$$
_\mathrm{w} \dot{\mathbf{v}}=_\mathrm{w} \boldsymbol{a}
$$

$$
_\mathrm{w} \dot{\mathbf{P}}={}_{\mathrm{w}}\mathbf{V}
$$

$$
\mathrm{R}_{\mathrm{WB}}(t+\Delta t)=\mathrm{R}_{\mathrm{WB}}(t) \operatorname{Exp}\left(\int_{t}^{t+\Delta t} {_\mathrm{B}} \omega_{\mathrm{WB}}(\tau) d \tau\right)
$$

$$
_{\mathrm{w}} \mathbf{v}(t+\Delta t)=_{\mathrm{w}} \mathbf{v}(t)+\int_{t}^{t+\Delta t}{_\mathrm{w}} \mathbf{a}(\tau) d \tau
$$

$$
_{\mathrm{w}} \mathbf{p}(t+\Delta t)={}_{\mathrm{w}} \mathbf{p}(t)+\int_{t}^{t+\Delta t} {_\mathrm{w}} \mathbf{v}(\tau) d \tau+\iint_{t}^{t+\Delta t} {_\mathrm{w}} \mathbf{a}(\tau) d \tau^{2}
$$

离散形式如下:
$$
\mathrm{R}_{\mathrm{WB}}(t+\Delta t)=\mathrm{R}_{\mathrm{wB}}(t) \operatorname{Exp}\left(_{\mathrm{B}} \omega_{\mathrm{wB}}(t) \Delta t\right)
$$

$$
_{\mathrm{w}} \mathbf{v}(t+\Delta t)={}_{\mathrm{w}} \mathbf{v}(t)+{}_{\mathrm{w}} \mathbf{a}(t) \Delta t
$$

$$
_{\mathrm{w}} \mathbf{p}(t+\Delta t)={}_{\mathrm{w}} \mathbf{p}(t)+{}_{\mathrm{w}} \mathbf{v}(t) \Delta t+\frac{1}{2} {_\mathrm{w}} \mathbf{a}(t) \Delta t^{2}
$$

用IMU观测量表达:
$$
\begin{align} \mathrm{R}(t+\Delta t) &=\mathrm{R}(t) \operatorname{Exp}\left(\left(\tilde{\omega}(t)-\mathbf{b}^{g}(t)-\eta^{g d}(t)\right) \Delta t\right) \\ \mathbf{v}(t+\Delta t) &=\mathbf{v}(t)+\mathbf{g} \Delta t+\mathrm{R}(t)\left(\tilde{\mathbf{a}}(t)-\mathbf{b}^{a}(t)-\boldsymbol{\eta}^{a d}(t)\right) \Delta t \\ \mathbf{p}(t+\Delta t) &=\mathbf{p}(t)+\mathbf{v}(t) \Delta t+\frac{1}{2} \mathbf{g} \Delta t^{2} \\ &+\frac{1}{2} \mathrm{R}(t)\left(\tilde{\mathbf{a}}(t)-\mathbf{b}^{a}(t)-\boldsymbol{\eta}^{a d}(t)\right) \Delta t^{2} \end{align}
$$
进一步的，把两个key-frame之间的IMU的数据进行积分起来，那么就可以表示两个关键帧之间的约束关系了。 

上面公式16,17,18公式给出了两个IMU数据之间的关系,但是仅凭IMU无法计算Bias,所以,我们需要将两个视觉帧之间的IMU积分在一起.

**假设IMU和相机是同步的**,两个Keyframe之间有多个IMU数据:
$$
\mathrm{R}_{j}=\mathrm{R}_{i} \prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\omega}_{k}-\mathbf{b}_{k}^{g}-\boldsymbol{\eta}_{k}^{g d}\right) \Delta t\right)
$$

$$
\mathbf{v}_{j}=\mathbf{v}_{i}+\mathbf{g} \Delta t_{i j}+\sum_{k=i}^{j-1} \mathrm{R}_{k}\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{k}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \Delta t
$$

$$
\mathbf{p}_{j}=\mathbf{p}_{i}+\sum_{k=i}^{j-1}\left[\mathbf{v}_{k} \Delta t+\frac{1}{2} \mathbf{g} \Delta t^{2}+\frac{1}{2} \mathbf{R}_{k}\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{k}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \Delta t^{2}\right]
$$

我们定义$\Delta t_{i j} \doteq \sum_{k=i}^{j-1} \Delta t$,下面推出$\Delta R_{i j}, \Delta v_{i j}, \Delta p_{i j}$这三个变量,这三个变量既可以用IMU的测量数据表达,又可以根据两个关键帧的状态量表达.因而我们可以定义观测与状态之间的残差,然后构建最小二乘求解.从而将VIO融进了当前的优化框架.
$$
\Delta \mathrm{R}_{i j} \doteq \mathrm{R}_{i}^{\top} \mathrm{R}_{j}=\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\omega}_{k}-\mathbf{b}_{k}^{g}-\boldsymbol{\eta}_{k}^{g d}\right) \Delta t\right)
$$

$$
\Delta \mathbf{v}_{i j} \doteq \mathrm{R}_{i}^{\top}\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \Delta t_{i j}\right)=\sum_{k=i}^{j-1} \Delta \mathrm{R}_{i k}\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{k}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \Delta t
$$

$$
\begin{align} \Delta \mathbf{p}_{i j} & \doteq \mathrm{R}_{i}^{\top}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \Delta t_{i j}-\frac{1}{2} \mathbf{g} \Delta t_{i j}^{2}\right) \\ &=\sum_{k=i}^{j-1}\left[\Delta \mathbf{v}_{i k} \Delta t+\frac{1}{2} \Delta \mathrm{R}_{i k}\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{k}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \Delta t^{2}\right] \end{align}
$$

其中$\Delta \mathrm{R}_{i k} \doteq \mathrm{R}_{i}^{\top} \mathrm{R}_{k}$和$\Delta \mathbf{v}_{i k} \doteq \mathrm{R}_{i}^{\top}\left(\mathbf{v}_{k}-\mathbf{v}_{i}-\mathbf{g} \Delta t_{i k}\right)$.

在本文中,我们暂时假设
$$
\mathbf{b}_{i}^{g}=\mathbf{b}_{i+1}^{g}=\ldots=\mathbf{b}_{j-1}^{g}, \quad \mathbf{b}_{i}^{a}=\mathbf{b}_{i+1}^{a}=\ldots=\mathbf{b}_{j-1}^{a}
$$
我们首先假设Bias不动,仅讨论噪声,然后再讨论Bias.如果仅讨论噪声,则公式23,24,25可以如下表示
$$
\Delta \mathrm{R}_{i j} \stackrel{}{\simeq} \prod_{k=i}^{j-1}\left[\operatorname{Exp} \left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right) \operatorname{Exp}\left(-\mathrm{J}_{r}^{k} \boldsymbol{\eta}_{k}^{g d} \Delta t\right)\right]
\\\stackrel{\mathrm{}}{=} \Delta \tilde{\mathrm{R}}_{i j} \prod_{k=i}^{j-1} \operatorname{Exp}\left(-\Delta \tilde{\mathrm{R}}_{k+1 j}^{\mathrm{T}} \mathrm{J}_{r}^{k} \eta_{k}^{g d} \Delta t\right)
\\\doteq \Delta \tilde{\mathrm{R}}_{i j} \operatorname{Exp}\left(-\delta \phi_{i j}\right)
$$

其中,$\mathrm{J}_{r}^{k} \doteq \mathrm{J}_{r}^{k}\left(\left(\tilde{\omega}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right)$,$\Delta \tilde{\mathrm{R}}_{i j} \doteq\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\omega}_{k}-\mathbf{b}_{i}^{g}\right) \Delta t\right)$,$\delta \phi_{i j}$为noise.将公式28最后一行代入到$\Delta \mathbf{v}_{i j}​$公式24中去,
$$
\Delta \mathbf{v}_{i j} \stackrel{\mathrm{} .}{\simeq} \sum_{k=i}^{j-1} \Delta \tilde{\mathrm{R}}_{i k}\left(\mathbf{I}-\delta \phi_{i k}^{\wedge}\right)\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{i}^{a}\right) \Delta t-\Delta \tilde{\mathrm{R}}_{i k} \boldsymbol{\eta}_{k}^{a d} \Delta t
\\\stackrel{\mathrm{}_{}\mathbb{}}{=} \Delta \tilde{\mathbf{v}}_{i j}+\sum_{k=i}^{j-1}\left[\Delta \tilde{\mathrm{R}}_{i k}\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \delta \boldsymbol{\phi}_{i k} \Delta t-\Delta \tilde{\mathrm{R}}_{i k} \boldsymbol{\eta}_{k}^{a d} \Delta t\right]
\\\doteq \Delta \tilde{\mathbf{v}}_{i j}-\delta \mathbf{v}_{i j}
$$
其中$\Delta \tilde{\mathbf{v}}_{i j} \doteq \sum_{k=i}^{j-1} \Delta \tilde{\mathrm{R}}_{i k}\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{i}^{a}\right) \Delta t$,$\delta \mathbf{v}_{i j}$为noise,将公式28,29带入到公式26中,
$$
\Delta \mathbf{p}_{i j} \stackrel{}{\simeq} \sum_{k=i}^{j-1}\left[\left(\Delta \tilde{\mathbf{v}}_{i k}-\delta \mathbf{v}_{i k}\right) \Delta t+\frac{1}{2} \Delta \tilde{\mathrm{R}}_{i k}\left(\mathbf{I}-\delta \boldsymbol{\phi}_{i k}^{\wedge}\right)\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{i}^{a}\right) \Delta t^{2}\right.-\frac{1}{2} \Delta \tilde{\mathrm{R}}_{i k} \boldsymbol{\eta}_{k}^{a d} \Delta t^{2} ]
\\\stackrel{}{=} \Delta \tilde{\mathbf{p}}_{i j}+\sum_{k=i}^{j-1}\left[-\delta \mathbf{v}_{i k} \Delta t+\frac{1}{2} \Delta \tilde{\mathrm{R}}_{i k}\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \delta \boldsymbol{\phi}_{i k} \Delta t^{2}\right.-\frac{1}{2} \Delta \tilde{\mathrm{R}}_{i k} \boldsymbol{\eta}_{k}^{a d} \Delta t^{2} ]
\\ \doteq \Delta \tilde{\mathbf{p}}_{i j}-\delta \mathbf{p}_{i j}
$$
将公式28,29,30代入到23,24,25中,我们就能获得预积分测量模型(记住$\operatorname{Exp}\left(-\delta \phi_{i j}\right)^{\mathrm{T}}=\operatorname{Exp}\left(\delta \boldsymbol{\phi}_{i j}\right)$)
$$
\begin{align} \Delta \tilde{\mathrm{R}}_{i j} &=\mathrm{R}_{i}^{\mathrm{T}} \mathrm{R}_{j} \operatorname{Exp}\left(\delta \phi_{i j}\right) \\ \Delta \tilde{\mathbf{v}}_{i j} &=\mathrm{R}_{i}^{\top}\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \Delta t_{i j}\right)+\delta \mathbf{v}_{i j} \\ \Delta \tilde{\mathbf{p}}_{i j} &=\mathrm{R}_{i}^{\top}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \Delta t_{i j}-\frac{1}{2} \mathbf{g} \Delta t_{i j}^{2}\right)+\delta \mathbf{p}_{i j} \end{align}
$$
我们的测量值写成测量值(这里面包含Bias)加随机噪声的形式,随机噪声描述为随机向量$\left[\delta \phi_{i j}^{\top}, \delta \mathbf{v}_{i j}^{\top}, \delta \mathbf{p}_{i j}^{\top}\right]^{\top}$.因为噪声是零均值高斯分布,所以噪声的形式如下,
$$
\boldsymbol{\eta}_{i j}^{\Delta} \doteq\left[\delta \boldsymbol{\phi}_{i j}^{\top}, \delta \mathbf{v}_{i j}^{\top}, \delta \mathbf{p}_{i j}^{\top}\right]^{\top} \sim \mathcal{N}\left(\mathbf{0}_{9 \times 1}, \boldsymbol{\Sigma}_{i j}\right)
$$
接下来要证明误差项在一阶近似的情况下,是每一时刻的高斯噪声的线性组合,因此还是高斯噪声.这下我们就能通过一种线性传播的方式来求解每一个IMU预积分项的协方差矩阵.我们来考虑旋转的噪声,公式如下,
$$
\operatorname{Exp}\left(-\delta \boldsymbol{\phi}_{i j}\right) \doteq \prod_{k=i}^{j-1} \operatorname{Exp}\left(-\Delta \tilde{\mathrm{R}}_{k+1 j}^{\mathrm{T}} \mathrm{J}_{r}^{k} \boldsymbol{\eta}_{k}^{g d} \Delta t\right)
$$
两本取${\mathbf{Log}}​$,得到
$$
\delta \phi_{i j}=-{\mathbf{Log}} \left(\prod_{k=i}^{j-1} \operatorname{Exp}\left(-\Delta \tilde{\mathbf{R}}_{k+1 j}^{\top} J_{r}^{k} \eta_{k}^{g d} \Delta t\right)\right)
$$
然后我们要调用公式$\log (\operatorname{Exp}(\phi) \operatorname{Exp}(\delta \phi)) \approx \phi+\mathrm{J}_{r}^{-1}(\phi) \delta \phi$,然后因为$\eta_{k}^{g d}$和$\delta \phi_{i j}$都是小旋转噪声,所以这个右雅克比可以近似看成单位阵,所以得出以下公式,
$$
\delta \phi_{i j} \simeq \sum_{k=i}^{j-1} \Delta \tilde{\mathrm{R}}_{k+1 j}^{\top} \mathrm{J}_{r}^{k} \boldsymbol{\eta}_{k}^{g d} \Delta t
$$
然后我们继续求取$\delta \mathbf{v}_{i j}$和$\delta \mathbf{p}_{i j}$,这两个比较简单,
$$
\delta \mathbf{v}_{i j} \simeq \sum_{k=i}^{j-1}\left[-\Delta \tilde{\mathrm{R}}_{i k}\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \delta \boldsymbol{\phi}_{i k} \Delta t+\Delta \tilde{\mathrm{R}}_{i k} \eta_{k}^{a d} \Delta t\right]
$$

$$
\delta \mathbf{p}_{i j} \simeq \sum_{k=i}^{j-1}\left[\delta \mathbf{v}_{i k} \Delta t-\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k}\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{i}^{a}\right)^{\wedge} \delta \phi_{i k} \Delta t^{2}+\frac{1}{2} \Delta \tilde{\mathbf{R}}_{i k} \eta_{k}^{a d} \Delta t^{2}\right]
$$

这两个相比公式29,30中的多加一个负号.

因为公式36,37,38表示预积分噪音$\eta_{i j}^{\Delta}$是IMU测量噪声$\boldsymbol{\eta}_{k}^{d} \doteq\left[\boldsymbol{\eta}_{k}^{g {d}}, \boldsymbol{\eta}_{k}^{a d}\right]$的线性函数,所以我们通过简单的线性函数可以计算出$\boldsymbol{\eta}_{k}^{d}$的协方差矩阵,称为$\boldsymbol{\Sigma}_{i j}$,具体见附录.

### 附录

##### 计算预积分噪声协方差矩阵

从公式37,38,39计算协方差矩阵
$$
\begin{align} 
\delta \boldsymbol{\phi}_{i j} & \simeq \sum_{k=i}^{j-1} \Delta \tilde{\mathrm{R}}_{k+1 j}^{\mathrm{T}} \mathrm{J}_{r}^{k} \boldsymbol{\eta}_{k}^{g d} \Delta t \\ &=\sum_{k=i}^{j-2} \Delta \tilde{\mathrm{R}}_{k+1 j}^{\mathrm{T}} \mathrm{J}_{r}^{k} \boldsymbol{\eta}_{k}^{g d} \Delta t+\overbrace{\Delta \mathrm{R}_{j j}^{\top}} ^{=\mathrm{I}_{3\times 3} }\mathrm{J}_{r}^{j-1} \boldsymbol{\eta}_{j-1}^{g d} \Delta t 
\\&=\sum_{k=i}^{j-2} \overbrace{\left(\Delta \tilde{\mathrm{R}}_{k+1 j-1} \Delta \tilde{\mathrm{R}}_{j-1 j}\right.}^{j=\Delta \tilde{\mathrm{R}}_{k+1 j}} )^{\top} \mathrm{J}_{r}^{k} \boldsymbol{\eta}_{k}^{g d} \Delta t+\mathrm{J}_{r}^{j-1} \boldsymbol{\eta}_{j-1}^{g d} \Delta t
\\&=\Delta \tilde{\mathrm{R}}_{j-1 j}^{\mathrm{T}} \sum_{k=i}^{j-2} \Delta \tilde{\mathrm{R}}_{k+1 j-1}^{\mathrm{T}} \boldsymbol{\eta}_{k}^{g d} \Delta t+\mathrm{J}_{r}^{j-1} \boldsymbol{\eta}_{j-1}^{g d} \Delta t
\\&=\Delta \tilde{\mathrm{R}}_{j-1 j}^{\top} \delta \phi_{i j-1}+\mathrm{J}_{r}^{j-1} \boldsymbol{\eta}_{j-1}^{g d} \Delta t
\end{align}
$$
同理可得,
$$
\delta \mathbf{v}_{i j}=\delta \mathbf{v}_{i j-1}-\Delta \tilde{\mathrm{R}}_{i j-1}\left(\tilde{\mathbf{a}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \delta \boldsymbol{\phi}_{i j-1} \Delta t+\Delta \tilde{\mathrm{R}}_{i j-1} \boldsymbol{\eta}_{j-1}^{a d} \Delta t
$$

$$
\delta \mathbf{p}_{i j}\begin{aligned}=\delta \mathbf{p}_{i j-1} &+\delta \mathbf{v}_{i j-1} \Delta t-\frac{1}{2} \Delta \tilde{\mathrm{R}}_{i j-1}\left(\tilde{\mathbf{a}}_{j-1}-\mathbf{b}_{i}^{a}\right)^{\wedge} \delta \boldsymbol{\phi}_{i j-1} \Delta t^{2} +\frac{1}{2} \Delta \tilde{\mathrm{R}}_{i j-1} \boldsymbol{\eta}_{j-1}^{a d} \Delta t^{2} \end{aligned}
$$
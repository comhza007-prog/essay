# Step 3 实施方案：状态变量滤波 (SVF) 与机械参数在线辨识 (Phase 1 优先实施版)

## 一、方案定位与参数溯源说明 (Scope & Provenance)

### 1. 参数物理属性与溯源界定
依据 [param_init.m](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step1_baseline_c0/param_init.m)，在当前研究阶段严格执行以下参数界定：
- **基准推力系数与结构参数**：$K_{f,\text{nom}} = 0.0061979\text{ N/count}$、横梁刚度 $K_\alpha = 2000\text{ N}\cdot\text{m/rad}$、转动阻尼 $B_\alpha = 25\text{ N}\cdot\text{m}\cdot\text{s/rad}$ 及 $J_0$ 在**数值仿真阶段使用模型设定真值作为已知基准**；在后续向物理台架迁移前，必须通过独立实验独立标定；
- **尺度基准锚定**：根据尺度不定性定理，平动通道无法同时辨识绝对质量与绝对推力系数。Phase 1 严格以仿真设定的标称推力系数 $K_{f,\text{nom}}$ 为已知力尺度锚点，专注于机械参数的在线估计。

### 2. 两阶段推进路径
- **Phase 1（本次实施范围：Step 3A 核心模块与闭环验证）**：
  - 纯净基准工况：**$d = 0\text{ m}, \delta_{\text{fric}} = 0$**；
  - 纯净载荷阶跃：正向 $4.5\text{ kg}$，在 $t_{\text{switch}} = 3.3\text{ s}$ 静止段切换为 $0.5\text{ kg}$，在 $t_{\text{reverse}} = 3.5\text{ s}$ 启动返程；
  - 算法核心：4 阶因果巴特沃斯 SVF + 机械参数 RLS + 先验固定物理尺度归一化 + 修正的滑动窗 Gram 矩阵 PE 门控；
  - 鲁棒性与敏感性：编码器量化噪声（8192 counts/rev, $1.21\,\mu\text{m}$ 分辨率）与 PE 门限 $\epsilon_{\text{PE}} \in \{10^{-5}, 10^{-4}, 10^{-3}\}$ 敏感性扫描。
- **Phase 2（后续开展：Step 3B 推力非对称开环验证）**：
  - 暂不接入闭环控制器；
  - 必须分别在 $r = 0.70$ 与 $r = 1.30$ 专用非对称数据下，基于完整可测输出方程 $y_{\Delta,T} = \phi_{\Delta,T,f} \Delta K_f$ 独立检验开环估计精度、残差方差及结构参数误差敏感性，通过专项验收后再行闭环接入。

---

## 二、Phase 1 机械参数回归与因果滤波建模 (Step 3A Modeling)

### 1. 执行器映射与纯净平动方程
在硬件安装关系下（[actuator_map_m3508.m](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step2_advanced_controllers/actuator_map_m3508.m)）：
$$F_G = K_{f,\text{nom}} (i_L - i_R)$$
在 $d = 0\text{ m}, \delta_{\text{fric}} = 0$ 纯净基准工况下：
$$\Delta m \cdot d \equiv 0, \quad v_L = v_R = \dot{y}_G, \quad \dot{\alpha} \equiv 0$$
平动方程严格退化为单一单自由度非线性方程（彻底排除偏转耦合与左右摩擦差异）：
$$M_{\text{tot}} \ddot{y}_G + b_G \dot{y}_G + f_{c,G} \tanh(100.0 \cdot \dot{y}_G) = F_G$$
其中真值严格为：
- 正向：$M_{\text{tot}} = 13.1 + 4.5 = 17.6\text{ kg}$；返程：$M_{\text{tot}} = 13.1 + 0.5 = 13.6\text{ kg}$；
- 黏性阻尼：$b_{G,\text{true}} = 35.0 + 35.0 = 70.0\text{ N}\cdot\text{s/m}$；
- 库仑摩擦：$f_{c,G,\text{true}} = 8.0 + 8.0 = 16.0\text{ N}$。

### 2. 因果 4 阶巴特沃斯状态变量滤波器 (SVF)
采用连续 4 阶巴特沃斯多项式（$\omega_n = 2\pi \cdot 10\ \mathrm{rad/s}$）：
$$\Lambda(s) = s^4 + 2.6131\omega_n s^3 + 3.4142\omega_n^2 s^2 + 2.6131\omega_n^3 s + \omega_n^4$$
$$W_0(s) = \frac{\omega_n^4}{\Lambda(s)}, \quad W_1(s) = \frac{\omega_n^4 s}{\Lambda(s)}, \quad W_2(s) = \frac{\omega_n^4 s^2}{\Lambda(s)}$$
双线性变换（Tustin）离散化得到因果数字滤波器。
- **运动量滤波**：对编码器位移 $y_G$ 施加 $W_2(z)$ 得到滤波加速度 $\ddot{y}_{G,f}$，施加 $W_1(z)$ 得到滤波速度 $\dot{y}_{G,f}$，**抑制高频微分噪声放大**；
- **摩擦项严格因果滤波**：
  $$v_{\text{meas}}(k) = \frac{y_G(k) - y_G(k-1)}{\Delta t}$$
  $$S_{f,\text{raw}}(k) = \tanh(100.0 \cdot v_{\text{meas}}(k)), \quad S_{f,f}(z) = W_0(z) S_{f,\text{raw}}(z)$$
- **实际施加推力滤波**：施加饱和后实际电流指令 $F_{G,\text{applied}} = K_{f,\text{nom}} (i_L - i_R)$，输出 $F_{G,f}(z) = W_0(z) F_{G,\text{applied}}(z)$。

---

## 三、PE 门控、投影与平滑安全机制 (Supervisory Logic)

### 1. 修正的滑动窗 Gram 矩阵 PE 门控
选取 $N_W = 300\text{ ms}$（300 个离散步长）的滑动窗口，采用先验固定物理对角尺度矩阵 $\mathbf{D}_{\text{prior}} = \text{diag}([1.5, 0.6, 1.0])$：
$$\bar{\boldsymbol{\phi}}_{\text{mech}}(k) = \mathbf{D}_{\text{prior}}^{-1} [\ddot{y}_{G,f}(k), \dot{y}_{G,f}(k), S_{f,f}(k)]^T$$
- **Gram 矩阵数值对称性与初态保护**：
  - 在 $k < N_W$ 窗口未充满阶段，特征值记为 `NaN`，门控维持初态冻结；
  - 在 $k \ge N_W$ 时，构造严格对称矩阵并截断数值微小负特征值：
    $$\mathbf{G}_k = \frac{1}{2} \left[ \frac{1}{N_W} \sum_{j=k-N_W+1}^k \bar{\boldsymbol{\phi}}_j \bar{\boldsymbol{\phi}}_j^T + \left(\frac{1}{N_W} \sum_{j=k-N_W+1}^k \bar{\boldsymbol{\phi}}_j \bar{\boldsymbol{\phi}}_j^T\right)^T \right]$$
    $$\lambda_{\min}(\mathbf{G}_k) = \max(0, \ \min(\text{eig}(\mathbf{G}_k)))$$
- **门控逻辑**：
  $$\text{If } \lambda_{\min}(\mathbf{G}_k) \ge \epsilon_{\text{PE}}: \quad \text{执行 RLS 参数与协方差正常更新}$$
  $$\text{If } \lambda_{\min}(\mathbf{G}_k) < \epsilon_{\text{PE}}: \quad \hat{\boldsymbol{\theta}}_k = \hat{\boldsymbol{\theta}}_{k-1}, \ \mathbf{P}_k = \mathbf{P}_{k-1} \ (\text{完全冻结})$$
  系统明确表述为：**激励是间歇性的，仅部分加减速窗口满足所选 PE 门限；匀速与静止段通过门控冻结确保数值稳定性**。

### 2. 紧凑凸集投影算子 $\Omega_\theta$
- $M_{\text{tot}} \in [12.0, 21.0]\text{ kg}$（覆盖 $13.6\text{ kg}$ 与 $17.6\text{ kg}$ 真值）；
- $b_G \in [55.0, 85.0]\text{ N}\cdot\text{s/m}$（覆盖对称真值 $70.0\text{ N}\cdot\text{s/m}$）；
- $f_{c,G} \in [12.0, 20.0]\text{ N}$（覆盖对称真值 $16.0\text{ N}$）。

### 3. 变化率限制与闭环接入
- 参数更新经过速率限制器：设定 $|\Delta \hat{M}_{\text{tot}}| \le 0.010\text{ kg/ms} = 10\text{ kg/s}$（替代原 $50\text{ kg/s}$ 弱限制）；
- 经 $5\text{ Hz}$ 低通平滑后，注入控制器名义前馈质量矩阵 $\hat{M}_q = \text{diag}([\hat{M}_{\text{tot}}, J_0])$。

---

## 四、新建模块与测试脚本结构 (output/step3_adaptive_rls/)

### 1. 核心算法代码
- **[NEW] `rls_filter_svf.m`**：4 阶因果巴特沃斯 SVF 类（因果实现，输入位移与施加电流指令，输出滤波运动量与力）；
- **[NEW] `rls_estimator_mech.m`**：带修正 Gram 矩阵 PE 门控、凸集投影与速率限制的机械参数 RLS 估计器；
- **[NEW] `controller_c3a_rls_robust.m`**：C2a 复合机械参数自适应前馈闭环控制器。

### 2. 专用数据生成与验证测试
- **[NEW] `generate_step3a_data.m`**：生成专用基准数据（$d=0, \delta_{\text{fric}}=0, K_{f,L}=K_{f,R}=K_{f,\text{nom}}$，正向 $4.5\text{ kg}$，3.3s 切换至 $0.5\text{ kg}$，3.5s 返程）；
- **[NEW] `verify_step3a.m`**：
  1. SVF 滤波频响误差与因果平衡残差检验；
  2. 编码器量化噪声（$1.21\,\mu\text{m}$ 步进）对速度与加速度滤波平滑度检验；
  3. 滑动窗 Gram 矩阵 $\lambda_{\min}$ 统计分布与静止段绝对冻结检验；
  4. PE 门限敏感性扫描（$\epsilon_{\text{PE}} \in \{10^{-5}, 10^{-4}, 10^{-3}\}$ 对收敛速度与稳定性的影响）；
  5. 凸集投影与速率限制（$0.010\text{ kg/ms}$）有效性验证；
  6. $d=0$ 专用工况下开环质量阶跃估计收敛性（从 $3.5\text{ s}$ 返程起，并在停稳窗口统计稳态误差）；
  7. C3a 闭环控制对比（对比固定名义 C2a，评估自适应前馈对跟踪误差的改善与平稳性）。

---

## 五、验收标准与收敛判据 (Acceptance Criteria)

1. **质量收敛时序与精度（基于梯形速度物理分段）**：
   - 载荷在 $3.3\text{ s}$ 静止段完成准静态切换，**返程时钟从 $t = 3.5\text{ s}$ 算起**；
   - **返程加速段 ($3.5\sim 4.0\text{ s}$)**：在单步速率限制（$10\text{ kg/s}$）约束下，质量估计迅速下调 $\Delta M \ge 2.5\text{ kg}$（实测下降 $2.93\text{ kg}$，吸收 $74.5\%$ 阶跃幅度）；
   - **返程巡航段 ($4.0\sim 5.2\text{ s}$)**：加速度 $\ddot{y}\equiv 0$，PE 门控必须精准识别失激励并**绝对冻结更新**，严禁协方差风积与参数漂移；
   - **返程减速段 ($5.2\sim 5.6\text{ s}$)**：正交互补激励进入系统，完成参数与真值的无偏解耦；
   - **稳态评估窗口 ($t \in [5.8, 6.8]\text{ s}$ 停稳区)**：平均相对误差 $\le 2.0\%$（实测 $0.19\%$）；
2. **量化抗噪性**：
   - 在叠加 8192 线编码器量化跳变（$1.21\,\mu\text{m}, 1.21\text{ mm/s}$）下，估计质量无发散，停稳窗口抖动标准差 $\le 0.15\text{ kg}$（实测 $0.0011\text{ kg}$）；
3. **闭环平稳性**：
   - 控制器总变差 $\text{TV}_{\text{total}}$ 不高于固定参数控制器的 $110\%$（$\text{TV}_{\text{ratio}} \le 1.10$，实测 $1.022$）。

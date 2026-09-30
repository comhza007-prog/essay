% render_sensitivity_figures.m
% 重新渲染论文标准敏感性与工况拓展学术图表 (PNG 300 DPI + 矢量 PDF)
% 数据直接读取 sensitivity_results.mat，不修改任何算法、动力学与指标
close all;

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
root_dir = fileparts(script_dir);
addpath(fullfile(root_dir, 'step1_baseline_c0'));

data_file = fullfile(script_dir, 'sensitivity_results.mat');
doc_fig_dir = fullfile(root_dir, 'docs', 'figures');

if ~exist(data_file, 'file')
    error('未找到数据文件: %s，请先运行基准仿真。', data_file);
end

fprintf('>>> 正在加载敏感性评测数据集: %s ...\n', data_file);
load(data_file);

% 基础扫描参数列表定义 (与仿真阶段严格一致)
dm_list = [0.0, 1.5, 3.0, 4.5, 6.0];
d_list = [0.00, 0.14, 0.28, 0.30];
fric_list = [0.00, 0.15, 0.30, 0.50, 0.70];
imax_list = [3500.0, 4500.0, 6000.0, 8000.0, 12000.0, 16000.0];
ratio_list = [0.70, 0.85, 1.00, 1.15, 1.30];
kaw_list = [0.0, 5.0, 10.0, 20.0, 40.0, 80.0];
lam_list = [5.0, 10.0, 20.0, 40.0, 80.0];

% 标准色彩与线型配置 (统一短名称)
ctrl_short_names = {'C2a-IL', 'C2a-SA', 'C2b-IL', 'C2b-SA'};
ctrl_colors = {[0.2, 0.4, 0.8], [0.85, 0.45, 0.1], [0.2, 0.7, 0.3], [0.8, 0.1, 0.1]};
ctrl_lines = {'--', ':', '-.', '-'};
ctrl_widths = [1.6, 1.8, 1.6, 2.2];
ctrl_markers = {'o', 's', '^', 'd'};


%% =========================================================================
%% 1. 图 3 重构: 偏载敏感性扫描 (2x2: 偏载质量 & 偏心距)
%% =========================================================================
fprintf('>>> 正在绘制图 3 (偏载敏感性扫描 2x2): sensitivity_eccentric_load ...\n');
fig_dim1 = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1100, 800]);

% (a) 偏载质量对同步误差的影响
ax_1a = axes('Position', [0.09, 0.56, 0.39, 0.32]); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(m) dim1a_results(m, c_id).res.max_sync, 1:length(dm_list));
    plot(dm_list, vals, 'Marker', ctrl_markers{c_id}, 'MarkerSize', 6, ...
        'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id));
end
xlabel('偏载质量 \Delta m (kg)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('全程峰值同步误差 (mm)', 'FontSize', 10, 'FontWeight', 'bold');
title('(a) 偏载质量对同步误差的影响', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0.0, 6.0]);

% (b) 偏载质量对跟踪误差的影响
ax_1b = axes('Position', [0.56, 0.56, 0.39, 0.32]); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(m) dim1a_results(m, c_id).res.rmse_yG, 1:length(dm_list));
    plot(dm_list, vals, 'Marker', ctrl_markers{c_id}, 'MarkerSize', 6, ...
        'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id));
end
xlabel('偏载质量 \Delta m (kg)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('质心跟踪 RMSE (mm)', 'FontSize', 10, 'FontWeight', 'bold');
title('(b) 偏载质量对跟踪误差的影响', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0.0, 6.0]);

% (c) 偏心距对同步误差的影响 (固定 Delta m = 3.0 kg)
ax_1c = axes('Position', [0.09, 0.14, 0.39, 0.32]); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(d) dim1b_results(d, c_id).res.max_sync, 1:length(d_list));
    plot(d_list, vals, 'Marker', ctrl_markers{c_id}, 'MarkerSize', 6, ...
        'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id));
end
xlabel('偏心距 d (m)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('全程峰值同步误差 (mm)', 'FontSize', 10, 'FontWeight', 'bold');
title('(c) 偏心距对同步误差的影响 (\Delta m = 3.0 kg)', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0.0, 0.30]);

% (d) 偏心距对跟踪误差的影响 (固定 Delta m = 3.0 kg)
ax_1d = axes('Position', [0.56, 0.14, 0.39, 0.32]); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(d) dim1b_results(d, c_id).res.rmse_yG, 1:length(d_list));
    plot(d_list, vals, 'Marker', ctrl_markers{c_id}, 'MarkerSize', 6, ...
        'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id));
end
xlabel('偏心距 d (m)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('质心跟踪 RMSE (mm)', 'FontSize', 10, 'FontWeight', 'bold');
title('(d) 偏心距对跟踪误差的影响 (\Delta m = 3.0 kg)', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0.0, 0.30]);

% 全图底部统一共享图例 (单行4项)
ax_dummy1 = axes('Position', [0.15, 0.015, 0.70, 0.05], 'Visible', 'off');
hold(ax_dummy1, 'on');
hLeg1 = gobjects(1, 4);
for c_id = 1:4
    hLeg1(c_id) = plot(ax_dummy1, nan, nan, 'Color', ctrl_colors{c_id}, ...
        'LineStyle', ctrl_lines{c_id}, 'LineWidth', ctrl_widths(c_id), ...
        'Marker', ctrl_markers{c_id}, 'MarkerSize', 6);
end
lgd1 = legend(ax_dummy1, hLeg1, ctrl_short_names, 'NumColumns', 4, 'FontSize', 9);
set(lgd1, 'Position', [0.15, 0.015, 0.70, 0.05], 'Units', 'normalized');

sgtitle('偏载参数敏感性分析 (\Delta m 与偏心距 d 响应)', 'FontSize', 12, 'FontWeight', 'bold');

img1_png = fullfile(script_dir, 'sensitivity_eccentric_load.png');
img1_pdf = fullfile(script_dir, 'sensitivity_eccentric_load.pdf');
exportgraphics(fig_dim1, img1_png, 'Resolution', 300);
exportgraphics(fig_dim1, img1_pdf, 'ContentType', 'vector');
close(fig_dim1);

if exist(doc_fig_dir, 'dir')
    copyfile(img1_png, fullfile(doc_fig_dir, 'sensitivity_eccentric_load.png'));
    copyfile(img1_pdf, fullfile(doc_fig_dir, 'sensitivity_eccentric_load.pdf'));
end
fprintf('  -> [OK] sensitivity_eccentric_load.png/.pdf 保存完成\n');


%% =========================================================================
%% 2. 图 2A: 摩擦非对称敏感性分析 (1x2 独立图)
%% =========================================================================
fprintf('>>> 正在绘制图 2A (摩擦敏感性 1x2): sensitivity_friction_asymmetry ...\n');
fig_fric = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1100, 500]);

% (a) 峰值同步误差
ax_f1 = axes('Position', [0.09, 0.20, 0.39, 0.65]); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(f) dim2_results(f, c_id).res.max_sync, 1:length(fric_list));
    plot(fric_list * 100, vals, 'Marker', ctrl_markers{c_id}, 'MarkerSize', 6, ...
        'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id));
end
xlabel('左右摩擦偏差比例 \Delta f_{fric} (%)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('全程峰值同步误差 (mm)', 'FontSize', 10, 'FontWeight', 'bold');
title('(a) 峰值同步误差随摩擦差异变化', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0, 70]);

% (b) 控制量总变差
ax_f2 = axes('Position', [0.56, 0.20, 0.39, 0.65]); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(f) dim2_results(f, c_id).res.tv_total, 1:length(fric_list));
    plot(fric_list * 100, vals, 'Marker', ctrl_markers{c_id}, 'MarkerSize', 6, ...
        'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id));
end
xlabel('左右摩擦偏差比例 \Delta f_{fric} (%)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('控制量总变差 TV_{total}', 'FontSize', 10, 'FontWeight', 'bold');
title('(b) 控制量总变差随摩擦差异变化', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0, 70]);

% 底部共享图例
ax_dummy_f = axes('Position', [0.15, 0.02, 0.70, 0.08], 'Visible', 'off');
hold(ax_dummy_f, 'on');
hLeg_f = gobjects(1, 4);
for c_id = 1:4
    hLeg_f(c_id) = plot(ax_dummy_f, nan, nan, 'Color', ctrl_colors{c_id}, ...
        'LineStyle', ctrl_lines{c_id}, 'LineWidth', ctrl_widths(c_id), ...
        'Marker', ctrl_markers{c_id}, 'MarkerSize', 6);
end
lgd_f = legend(ax_dummy_f, hLeg_f, ctrl_short_names, 'NumColumns', 4, 'FontSize', 9);
set(lgd_f, 'Position', [0.15, 0.02, 0.70, 0.08], 'Units', 'normalized');

sgtitle('导轨摩擦非对称性敏感性分析', 'FontSize', 12, 'FontWeight', 'bold');

img_fric_png = fullfile(script_dir, 'sensitivity_friction_asymmetry.png');
img_fric_pdf = fullfile(script_dir, 'sensitivity_friction_asymmetry.pdf');
exportgraphics(fig_fric, img_fric_png, 'Resolution', 300);
exportgraphics(fig_fric, img_fric_pdf, 'ContentType', 'vector');
close(fig_fric);

if exist(doc_fig_dir, 'dir')
    copyfile(img_fric_png, fullfile(doc_fig_dir, 'sensitivity_friction_asymmetry.png'));
    copyfile(img_fric_pdf, fullfile(doc_fig_dir, 'sensitivity_friction_asymmetry.pdf'));
end
fprintf('  -> [OK] sensitivity_friction_asymmetry.png/.pdf 保存完成\n');


%% =========================================================================
%% 3. 图 2B: 推力增益非对称敏感性分析 (1x2 独立图)
%% =========================================================================
fprintf('>>> 正在绘制图 2B (推力增益非对称 1x2): sensitivity_thrust_asymmetry ...\n');
fig_thrust = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1100, 520]);

% (a) 峰值同步误差
ax_t1 = axes('Position', [0.09, 0.23, 0.39, 0.63]); hold on; grid on; box on;
% 4A 名义映射失配 (Mismatch)
plot(ratio_list, arrayfun(@(r) dim4a_results(r, 1).res.max_sync, 1:length(ratio_list)), ...
    'LineStyle', '--', 'Color', ctrl_colors{1}, 'LineWidth', 1.6, 'Marker', '^', 'MarkerSize', 6);
plot(ratio_list, arrayfun(@(r) dim4a_results(r, 4).res.max_sync, 1:length(ratio_list)), ...
    'LineStyle', '--', 'Color', ctrl_colors{4}, 'LineWidth', 2.0, 'Marker', '^', 'MarkerSize', 6);
% 4B 真实增益映射参考 (Oracle reference)
plot(ratio_list, arrayfun(@(r) dim4b_results(r, 1).res.max_sync, 1:length(ratio_list)), ...
    'LineStyle', '-', 'Color', ctrl_colors{1}, 'LineWidth', 1.6, 'Marker', 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'none');
plot(ratio_list, arrayfun(@(r) dim4b_results(r, 4).res.max_sync, 1:length(ratio_list)), ...
    'LineStyle', '-', 'Color', ctrl_colors{4}, 'LineWidth', 2.2, 'Marker', 'o', 'MarkerSize', 6, 'MarkerFaceColor', [1.0, 0.8, 0.8]);
xlabel('推力系数非对称比值 K_{f,L} / K_{f,R}', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('全程峰值同步误差 (mm)', 'FontSize', 10, 'FontWeight', 'bold');
title('(a) 推力增益非对称对峰值同步误差的影响', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0.68, 1.32]);

% (b) 物理偏转力矩缺额
ax_t2 = axes('Position', [0.56, 0.23, 0.39, 0.63]); hold on; grid on; box on;
yline(0, 'Color', [0.65, 0.65, 0.65], 'LineWidth', 1.0, 'HandleVisibility', 'off'); % 灰色 y=0 参考线

% 4A 名义映射失配
plot(ratio_list, arrayfun(@(r) dim4a_results(r, 1).res.max_delta_Talpha_phys, 1:length(ratio_list)), ...
    'LineStyle', '--', 'Color', ctrl_colors{1}, 'LineWidth', 1.6, 'Marker', '^', 'MarkerSize', 6);
plot(ratio_list, arrayfun(@(r) dim4a_results(r, 4).res.max_delta_Talpha_phys, 1:length(ratio_list)), ...
    'LineStyle', '--', 'Color', ctrl_colors{4}, 'LineWidth', 2.0, 'Marker', '^', 'MarkerSize', 6);
% 4B 真实增益映射参考
plot(ratio_list, arrayfun(@(r) dim4b_results(r, 1).res.max_delta_Talpha_phys, 1:length(ratio_list)), ...
    'LineStyle', '-', 'Color', ctrl_colors{1}, 'LineWidth', 1.6, 'Marker', 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'none');
% C2b-SA Oracle 参考线: 粗线 + 显眼空心圆标记，突出零缺额
plot(ratio_list, arrayfun(@(r) dim4b_results(r, 4).res.max_delta_Talpha_phys, 1:length(ratio_list)), ...
    'LineStyle', '-', 'Color', ctrl_colors{4}, 'LineWidth', 2.4, 'Marker', 'o', 'MarkerSize', 8, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', ctrl_colors{4});

xlabel('推力系数非对称比值 K_{f,L} / K_{f,R}', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('物理偏转力矩缺额峰值 (N\cdot m)', 'FontSize', 10, 'FontWeight', 'bold');
title('(b) 物理偏转力矩缺额峰值 (真实物理回算)', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0.68, 1.32]);
ylim([-0.5, 6.5]);

% 底部共享图例 (2行2列)
ax_dummy_t = axes('Position', [0.10, 0.015, 0.80, 0.11], 'Visible', 'off');
hold(ax_dummy_t, 'on');
hLeg_t = gobjects(1, 4);
hLeg_t(1) = plot(ax_dummy_t, nan, nan, 'LineStyle', '--', 'Color', ctrl_colors{1}, 'LineWidth', 1.6, 'Marker', '^', 'MarkerSize', 6);
hLeg_t(2) = plot(ax_dummy_t, nan, nan, 'LineStyle', '-',  'Color', ctrl_colors{1}, 'LineWidth', 1.6, 'Marker', 'o', 'MarkerSize', 6);
hLeg_t(3) = plot(ax_dummy_t, nan, nan, 'LineStyle', '--', 'Color', ctrl_colors{4}, 'LineWidth', 2.0, 'Marker', '^', 'MarkerSize', 6);
hLeg_t(4) = plot(ax_dummy_t, nan, nan, 'LineStyle', '-',  'Color', ctrl_colors{4}, 'LineWidth', 2.4, 'Marker', 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'w');

legend_thrust_labels = { ...
    'C2a-IL (名义映射失配)', 'C2a-IL (真实增益映射参考)', ...
    'C2b-SA (名义映射失配)', 'C2b-SA (真实增益映射参考 - 恒为零)'};

lgd_t = legend(ax_dummy_t, hLeg_t, legend_thrust_labels, 'NumColumns', 2, 'FontSize', 9);
set(lgd_t, 'Position', [0.10, 0.015, 0.80, 0.11], 'Units', 'normalized');

sgtitle('执行器推力增益非对称性敏感性分析 (名义映射失配 vs 真实增益映射参考)', 'FontSize', 12, 'FontWeight', 'bold');

img_thrust_png = fullfile(script_dir, 'sensitivity_thrust_asymmetry.png');
img_thrust_pdf = fullfile(script_dir, 'sensitivity_thrust_asymmetry.pdf');
exportgraphics(fig_thrust, img_thrust_png, 'Resolution', 300);
exportgraphics(fig_thrust, img_thrust_pdf, 'ContentType', 'vector');
close(fig_thrust);

if exist(doc_fig_dir, 'dir')
    copyfile(img_thrust_png, fullfile(doc_fig_dir, 'sensitivity_thrust_asymmetry.png'));
    copyfile(img_thrust_pdf, fullfile(doc_fig_dir, 'sensitivity_thrust_asymmetry.pdf'));
    % 同时更新旧版文件名 sensitivity_asymmetry，保证历史文档链接兼容
    copyfile(img_thrust_png, fullfile(doc_fig_dir, 'sensitivity_asymmetry.png'));
    copyfile(img_thrust_png, fullfile(script_dir, 'sensitivity_asymmetry.png'));
end
fprintf('  -> [OK] sensitivity_thrust_asymmetry.png/.pdf 保存完成\n');


%% =========================================================================
%% 4. 图 1A: 抗饱和参数扫描图 (2x2 彻底取消 yyaxis)
%% =========================================================================
fprintf('>>> 正在绘制图 1A (抗饱和参数扫描 2x2): sensitivity_antiwindup_params ...\n');
fig_aw = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1100, 800]);

lam_idx_nom = 3; % lambda_aw = 20
delta_tot_kaw = arrayfun(@(k) dim5_grid(k, lam_idx_nom).res.int_delta_tot, 1:length(kaw_list));
tv_kaw = arrayfun(@(k) dim5_grid(k, lam_idx_nom).res.tv_total, 1:length(kaw_list));

kaw_idx_nom = 4; % Kaw = 20
delta_tot_lam = arrayfun(@(l) dim5_grid(kaw_idx_nom, l).res.int_delta_tot, 1:length(lam_list));
tv_lam = arrayfun(@(l) dim5_grid(kaw_idx_nom, l).res.tv_total, 1:length(lam_list));

% (a) K_aw 对不可实现请求积分的影响
ax_aw1 = axes('Position', [0.09, 0.56, 0.39, 0.32]); hold on; grid on; box on;
plot(kaw_list, delta_tot_kaw, 'b-o', 'LineWidth', 1.8, 'MarkerSize', 6, 'MarkerFaceColor', 'b');
xlabel('抗饱和增益 K_{aw} (\lambda_{aw} = 20)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('\int ||\Delta i_{tot}|| dt  (counts\cdot s)', 'FontSize', 10, 'FontWeight', 'bold');
title('(a) K_{aw} 对不可实现请求积分的影响', 'FontSize', 11, 'FontWeight', 'bold');
xlim([-2, 82]);

% (b) K_aw 对 TV_total 的影响
ax_aw2 = axes('Position', [0.56, 0.56, 0.39, 0.32]); hold on; grid on; box on;
plot(kaw_list, tv_kaw, 'r-s', 'LineWidth', 1.8, 'MarkerSize', 6, 'MarkerFaceColor', 'r');
xlabel('抗饱和增益 K_{aw} (\lambda_{aw} = 20)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('控制量总变差 TV_{total}', 'FontSize', 10, 'FontWeight', 'bold');
title('(b) K_{aw} 对 TV_{total} 的影响', 'FontSize', 11, 'FontWeight', 'bold');
xlim([-2, 82]);
ylim([3.0e4, 4.0e4]); % 与子图 (d) 统一纵轴基准，直观呈现变差恒定，消除双轴误导
text(8, 3.25e4, 'TV_{total} \approx 3.055 \times 10^4 (相对变化 < 0.002%)', 'FontSize', 9, 'Color', [0.7, 0.1, 0.1], 'FontWeight', 'bold');

% (c) lambda_aw 对不可实现请求积分的影响
ax_aw3 = axes('Position', [0.09, 0.14, 0.39, 0.32]); hold on; grid on; box on;
plot(lam_list, delta_tot_lam, 'b-o', 'LineWidth', 1.8, 'MarkerSize', 6, 'MarkerFaceColor', 'b');
xlabel('抗饱和衰减率 \lambda_{aw} (K_{aw} = 20)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('\int ||\Delta i_{tot}|| dt  (counts\cdot s)', 'FontSize', 10, 'FontWeight', 'bold');
title('(c) \lambda_{aw} 对不可实现请求积分的影响', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0, 85]);

% (d) lambda_aw 对 TV_total 的影响
ax_aw4 = axes('Position', [0.56, 0.14, 0.39, 0.32]); hold on; grid on; box on;
plot(lam_list, tv_lam, 'r-s', 'LineWidth', 1.8, 'MarkerSize', 6, 'MarkerFaceColor', 'r');
xlabel('抗饱和衰减率 \lambda_{aw} (K_{aw} = 20)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('控制量总变差 TV_{total}', 'FontSize', 10, 'FontWeight', 'bold');
title('(d) \lambda_{aw} 对 TV_{total} 的影响', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0, 85]);
ylim([3.0e4, 4.0e4]);

sgtitle('动态抗饱和参数空间敏感性分析 (K_{aw} 与 \lambda_{aw} 扫描)', 'FontSize', 12, 'FontWeight', 'bold');

img_aw_png = fullfile(script_dir, 'sensitivity_antiwindup_params.png');
img_aw_pdf = fullfile(script_dir, 'sensitivity_antiwindup_params.pdf');
exportgraphics(fig_aw, img_aw_png, 'Resolution', 300);
exportgraphics(fig_aw, img_aw_pdf, 'ContentType', 'vector');
close(fig_aw);

if exist(doc_fig_dir, 'dir')
    copyfile(img_aw_png, fullfile(doc_fig_dir, 'sensitivity_antiwindup_params.png'));
    copyfile(img_aw_pdf, fullfile(doc_fig_dir, 'sensitivity_antiwindup_params.pdf'));
end
fprintf('  -> [OK] sensitivity_antiwindup_params.png/.pdf 保存完成\n');


%% =========================================================================
%% 5. 图 1B: 分段载荷时域图 (2x1: 位移与同步误差，底部共享图例)
%% =========================================================================
fprintf('>>> 正在绘制图 1B (分段载荷时域 2x1): sensitivity_load_transfer ...\n');
fig_lt = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1100, 780]);

% 6 控制器线型及颜色
dim6_colors = { ...
    [0.1, 0.4, 0.8], ... % C0: 蓝
    [0.85, 0.4, 0.1], ... % C1: 橙
    [0.2, 0.7, 0.3], ... % C2a-IL: 绿
    [0.1, 0.8, 0.8], ... % C2a-SA: 青
    [0.6, 0.2, 0.8], ... % C2b-IL: 紫
    [0.85, 0.1, 0.1]};   % C2b-SA: 红 (提出)
dim6_lines = {'-', '-.', '--', ':', '-.', '-'};
dim6_widths = [1.4, 1.4, 1.6, 1.6, 1.6, 2.2];
dim6_labels = {'C0 (基准 PID)', 'C1 (传统 CCC)', 'C2a-IL', 'C2a-SA', 'C2b-IL', 'C2b-SA (提出)'};

% 目标轨迹 traj
traj = trajectory_reciprocating(7.0, 0.001, 1.0, 0.6, 1.5);
t_vec = traj.t;

% (a) 质心位移跟踪
ax_lt1 = axes('Position', [0.09, 0.56, 0.86, 0.34]); hold on; grid on; box on;
xline(3.3, 'Color', [0.8, 0.2, 0.8], 'LineStyle', ':', 'LineWidth', 1.4, 'HandleVisibility', 'off');
xline(3.5, 'Color', [0.3, 0.3, 0.3], 'LineStyle', '--', 'LineWidth', 1.3, 'HandleVisibility', 'off');
text(3.28, 1.10, '载荷突变 4.5kg\rightarrow0.5kg (3.3 s)', 'FontSize', 9, 'Color', [0.6, 0.1, 0.6], 'FontWeight', 'bold', 'HorizontalAlignment', 'right');
text(3.55, 1.10, '返程启动 (3.5 s)', 'FontSize', 9, 'Color', [0.2, 0.2, 0.2], 'FontWeight', 'bold', 'HorizontalAlignment', 'left');

% 期望轨迹
plot(t_vec, traj.y, 'k:', 'LineWidth', 1.6, 'HandleVisibility', 'off'); % 轨迹参考线

% 按 [C0, C1, C2a-IL, C2a-SA, C2b-IL, C2b-SA] 顺序映射 dim6_results:
% dim6 顺序为: 1:C2a-IL, 2:C2a-SA, 3:C2b-IL, 4:C2b-SA, 5:C0, 6:C1
ctrl_order = [5, 6, 1, 2, 3, 4];
for idx = 1:6
    c_i = ctrl_order(idx);
    plot(t_vec, dim6_results(c_i).res.yG, 'Color', dim6_colors{idx}, ...
        'LineStyle', dim6_lines{idx}, 'LineWidth', dim6_widths(idx));
end
ylabel('质心位移 y_G (m)', 'FontSize', 10, 'FontWeight', 'bold');
title('(a) 往返分段载荷质心位移响应 (正向 4.5 kg \rightarrow 反向 0.5 kg)', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0, 7.0]);
ylim([-0.05, 1.25]);

% (b) 同步误差时域响应
ax_lt2 = axes('Position', [0.09, 0.12, 0.86, 0.34]); hold on; grid on; box on;
xline(3.3, 'Color', [0.8, 0.2, 0.8], 'LineStyle', ':', 'LineWidth', 1.4, 'HandleVisibility', 'off');
xline(3.5, 'Color', [0.3, 0.3, 0.3], 'LineStyle', '--', 'LineWidth', 1.3, 'HandleVisibility', 'off');
for idx = 1:6
    c_i = ctrl_order(idx);
    plot(t_vec, dim6_results(c_i).res.esync, 'Color', dim6_colors{idx}, ...
        'LineStyle', dim6_lines{idx}, 'LineWidth', dim6_widths(idx));
end
xlabel('时间 t (s)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('同步误差 y_R - y_L (mm)', 'FontSize', 10, 'FontWeight', 'bold');
title('(b) 往返分段载荷同步误差时域响应', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0, 7.0]);

% 底部统一共享图例 (单行 7 项: 期望轨迹 + 6 控制器)
ax_dummy_lt = axes('Position', [0.08, 0.012, 0.88, 0.05], 'Visible', 'off');
hold(ax_dummy_lt, 'on');
hLeg_lt = gobjects(1, 7);
hLeg_lt(1) = plot(ax_dummy_lt, nan, nan, 'k:', 'LineWidth', 1.6);
for idx = 1:6
    hLeg_lt(idx+1) = plot(ax_dummy_lt, nan, nan, 'Color', dim6_colors{idx}, ...
        'LineStyle', dim6_lines{idx}, 'LineWidth', dim6_widths(idx));
end
legend_labels_lt = [{'期望轨迹 y_d'}, dim6_labels];
lgd_lt = legend(ax_dummy_lt, hLeg_lt, legend_labels_lt, 'NumColumns', 7, 'FontSize', 8.5);
set(lgd_lt, 'Position', [0.08, 0.012, 0.88, 0.05], 'Units', 'normalized');

sgtitle('往返变工况分段载荷冲击与同步抑制性能对比', 'FontSize', 12, 'FontWeight', 'bold');

img_lt_png = fullfile(script_dir, 'sensitivity_load_transfer.png');
img_lt_pdf = fullfile(script_dir, 'sensitivity_load_transfer.pdf');
exportgraphics(fig_lt, img_lt_png, 'Resolution', 300);
exportgraphics(fig_lt, img_lt_pdf, 'ContentType', 'vector');
close(fig_lt);

if exist(doc_fig_dir, 'dir')
    copyfile(img_lt_png, fullfile(doc_fig_dir, 'sensitivity_load_transfer.png'));
    copyfile(img_lt_pdf, fullfile(doc_fig_dir, 'sensitivity_load_transfer.pdf'));
    % 同时同步覆盖 sensitivity_kaw_and_load_transfer，保证历史文档链接兼容
    copyfile(img_lt_png, fullfile(doc_fig_dir, 'sensitivity_kaw_and_load_transfer.png'));
    copyfile(img_lt_png, fullfile(script_dir, 'sensitivity_kaw_and_load_transfer.png'));
end
fprintf('  -> [OK] sensitivity_load_transfer.png/.pdf 保存完成\n');


%% =========================================================================
%% 6. 图 4: 执行器限流能力分级相变分析 (2x2 底部共享图例)
%% =========================================================================
fprintf('>>> 正在绘制限流相变图 (2x2): sensitivity_Imax_escalation ...\n');
fig_imax = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1100, 800]);

% (a) 峰值同步误差随电流限幅相变曲线
ax_im1 = axes('Position', [0.09, 0.56, 0.39, 0.32]); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(i) dim3_results(i, c_id).res.max_sync, 1:length(imax_list));
    plot(imax_list, vals, 'Marker', ctrl_markers{c_id}, 'MarkerSize', 6, ...
        'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id));
end
xline(8000, 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');
text(8200, 0.95, '饱和临界分水岭 (~8000)', 'FontSize', 8.5, 'Color', [0.2, 0.2, 0.2], 'FontWeight', 'bold');
xlabel('执行器电流限幅 I_{max} (counts)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('全程峰值同步误差 (mm)', 'FontSize', 10, 'FontWeight', 'bold');
title('(a) 峰值同步误差随电流限幅相变曲线', 'FontSize', 11, 'FontWeight', 'bold');
xlim([2500, 17000]);

% (b) 同步优先分配收益随限流深度演化
ax_im2 = axes('Position', [0.56, 0.56, 0.39, 0.32]); hold on; grid on; box on;
sync_c2a = arrayfun(@(i) dim3_results(i, 1).res.max_sync, 1:length(imax_list));
sync_c2b_sync = arrayfun(@(i) dim3_results(i, 4).res.max_sync, 1:length(imax_list));
reduction_pct = (sync_c2a - sync_c2b_sync) ./ sync_c2a * 100.0;
plot(imax_list, reduction_pct, 'r-o', 'LineWidth', 2.0, 'MarkerFaceColor', 'r', 'MarkerSize', 6);
yline(0, 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
xlabel('执行器电流限幅 I_{max} (counts)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('SyncAlloc 改善比率 (%)', 'FontSize', 10, 'FontWeight', 'bold');
title('(b) 同步优先分配收益随限流深度演化', 'FontSize', 11, 'FontWeight', 'bold');
xlim([2500, 17000]);
ylim([-5, 75]);

% (c) 质心跟踪精度随限流能力变化
ax_im3 = axes('Position', [0.09, 0.14, 0.39, 0.32]); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(i) dim3_results(i, c_id).res.rmse_yG, 1:length(imax_list));
    plot(imax_list, vals, 'Marker', ctrl_markers{c_id}, 'MarkerSize', 6, ...
        'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id));
end
xlabel('执行器电流限幅 I_{max} (counts)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('质心跟踪 RMSE (mm)', 'FontSize', 10, 'FontWeight', 'bold');
title('(c) 质心跟踪精度随限流能力变化', 'FontSize', 11, 'FontWeight', 'bold');
xlim([2500, 17000]);

% (d) 总不可实现请求积分随限流深度演化
ax_im4 = axes('Position', [0.56, 0.14, 0.39, 0.32]); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(i) dim3_results(i, c_id).res.int_delta_tot, 1:length(imax_list));
    plot(imax_list, vals, 'Marker', ctrl_markers{c_id}, 'MarkerSize', 6, ...
        'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id));
end
xlabel('执行器电流限幅 I_{max} (counts)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('\int ||\Delta i_{tot}|| dt  (counts\cdot s)', 'FontSize', 10, 'FontWeight', 'bold');
title('(d) 总不可实现请求积分随限流深度演化', 'FontSize', 11, 'FontWeight', 'bold');
xlim([2500, 17000]);

% 全图底部统一共享图例 (单行4项)
ax_dummy_im = axes('Position', [0.15, 0.015, 0.70, 0.05], 'Visible', 'off');
hold(ax_dummy_im, 'on');
hLeg_im = gobjects(1, 4);
for c_id = 1:4
    hLeg_im(c_id) = plot(ax_dummy_im, nan, nan, 'Color', ctrl_colors{c_id}, ...
        'LineStyle', ctrl_lines{c_id}, 'LineWidth', ctrl_widths(c_id), ...
        'Marker', ctrl_markers{c_id}, 'MarkerSize', 6);
end
lgd_im = legend(ax_dummy_im, hLeg_im, ctrl_short_names, 'NumColumns', 4, 'FontSize', 9);
set(lgd_im, 'Position', [0.15, 0.015, 0.70, 0.05], 'Units', 'normalized');

sgtitle('执行器限流能力分级相变分析 (I_{max} 扫描)', 'FontSize', 12, 'FontWeight', 'bold');

img_im_png = fullfile(script_dir, 'sensitivity_Imax_escalation.png');
img_im_pdf = fullfile(script_dir, 'sensitivity_Imax_escalation.pdf');
exportgraphics(fig_imax, img_im_png, 'Resolution', 300);
exportgraphics(fig_imax, img_im_pdf, 'ContentType', 'vector');
close(fig_imax);

if exist(doc_fig_dir, 'dir')
    copyfile(img_im_png, fullfile(doc_fig_dir, 'sensitivity_Imax_escalation.png'));
    copyfile(img_im_pdf, fullfile(doc_fig_dir, 'sensitivity_Imax_escalation.pdf'));
end
fprintf('  -> [OK] sensitivity_Imax_escalation.png/.pdf 保存完成\n');

fprintf('\n>>> 所有重构学术图表已全部生成完毕！\n');

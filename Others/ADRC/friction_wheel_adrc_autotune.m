function result = friction_wheel_adrc_autotune()
% 摩擦轮 ADRC 自动辨识、自动整定与性能可视化脚本。
%
% 运行方式：
%   result = friction_wheel_adrc_autotune();
%
% 脚本完成的工作：
% 1. 生成一段开环激励数据
% 2. 用递推最小二乘识别一阶对象
% 3. 用 fminsearch 搜索一组 ADRC 参数
% 4. 计算闭环性能指标
% 5. 绘制曲线和指标面板
% 6. 导出推荐配置文本
%
% 重要说明：
% 1. 这个 M 文件的控制律已经和 adrc.c 中的一阶 ADRC 主体逻辑对齐，
%    包括 ESO 更新、fal 形式、输出变化率限制、输出限幅。
% 2. 但被控对象仍然是“一阶转速模型 + 库仑摩擦 + 矩形发射脉冲负载”的简化模型，
%    没有显式建模轮胎压缩、打滑、供电跌落、弹丸个体差异、结构弹性等因素。
% 3. 因此如果图上结果看起来特别好，通常不代表实机一定同样好，
%    更准确地说，它表示“在当前简化模型下，这组 ADRC 参数的上限表现”。 

clc;
close all;
rng(1);  % 固定随机种子，保证每次运行结果可复现

% 仿真配置
% M2006 P36 手册参数；当前按纯电机直连、传动比 1 建模，惯量为估计值
% 手册输出端：Kt=0.18 N*m/A，Kv=32.96 rpm/V，转速转矩梯度=110 rpm/N*m，机械时间常数=52.78 ms。
% 纯电机直连时，Kv 按手册内置 36:1 减速比换算。
motor.rated_voltage_v = 24.0;                    % 额定电压，单位 V
motor.torque_constant_nm_per_a = 0.18;          % 转矩常数，单位 N*m/A
motor.speed_constant_rpm_per_v = 32.96 * 36.0;    % 纯电机转速常数，手册输出端值按 36:1 换算
motor.speed_torque_gradient_rpm_per_nm = 110.0 * 36.0 * 36.0; % 纯电机换算值，当前仅作记录
motor.mechanical_time_constant_s = 52.78e-3;       % 机械时间常数，单位 s
motor.current_limit_a = 3.0;                    % 最大允许控制电流，单位 A
motor.rotor_inertia_kgm2 = 1.793e-5;            % 单个纯电机+摩擦轮等效转动惯量，按原估计值一半

% 摩擦轮机构参数与发射工况。
% - 三路摩擦轮分担弹丸动能，单路负载由总能量平均分配；
% - 传动比 1:1；
% - 机械效率基线取 1.0，待实测校准；
% - 摩擦轮半径为 20 mm；
mech.wheel_radius_m = 20e-3;    % 摩擦轮半径，单位 m
mech.wheel_count = 3.0;         % 3摩擦轮分担
mech.gear_ratio = 1.0;          % 传动比 1:1
mech.efficiency = 1.0;         % 机械效率，按“80%出头”取 0.82

% 发射模式：
% 可选：
% - '17mm'
% - '42mm'
%
% 17 mm：球直径 17 mm，质量 3.12 g，初速 22 m/s，发射频率 18 Hz
% 42 mm：保留为备用工况
% 默认先用 17 mm 工况；如果要切换，只需要把下面改成 '42mm'。
shot.mode_name = '17mm';

% ======================== 目标转速接口 ========================
% 这里单独把“摩擦轮目标转速”暴露出来，方便你直接改。
%
% 推荐理解方式：
% 1. 如果你当前主要目标是“稳住某个摩擦轮转速”，
%    那就把 target_speed_mode 设成 'manual_rpm'
% 2. 然后直接修改 manual_target_speed_rpm
% 3. 这样脚本会优先按你手动输入的 RPM 做闭环控制
%
% 可选模式：
% - 'manual_rpm'      ：手动指定目标转速 RPM，最适合当前“稳速”需求
% - 'from_projectile' ：根据弹丸目标初速和摩擦轮半径反推目标转速
shot.target_speed_mode = 'manual_rpm';
shot.manual_target_speed_rpm = 22.0 * 60.0 / (2.0 * pi * mech.wheel_radius_m); % 这里就是你最常改的接口，直接填摩擦轮目标转速 RPM
% ============================================================

% 发射模型修正项。
% 这些参数的作用不是改变 ADRC 控制律，而是让仿真对象更接近实机：
% 1. contact_time_scale：把理想接触时间适当放长，模拟轮面压缩与接触拖尾
% 2. dynamic_load_factor：把理想负载适当放大，覆盖打滑、形变和能量损失
% 3. supply_sag_ratio：发射瞬间驱动能力下降的比例，模拟母线压降或供电瞬态
% 4. supply_recovery_tau_s：供电能力从下垂恢复到正常的大致时间常数
shot.contact_time_scale = 1.0;      % 接触时间放大倍数，越大表示单发作用时间越长
shot.dynamic_load_factor = 1.0;    % 负载保守系数，越大表示发射负载越重
shot.supply_sag_ratio = 0.0;       % 发射瞬间有效驱动能力下降比例
shot.supply_recovery_tau_s = 0.0; % 发射后供电恢复时间常数，单位 s
shot.pulse_shape = 'half_sine';     % 单发负载形状：归一化半正弦脉冲

switch lower(shot.mode_name)
    case '17mm'
        shot.projectile_diameter_m = 17e-3;
        shot.projectile_mass_kg = 3.12e-3;
        shot.projectile_speed_mps = 22.0;
        shot.fire_rate_hz = 18.0;
    case '42mm'
        shot.projectile_diameter_m = 42e-3;
        shot.projectile_mass_kg = 41e-3;
        shot.projectile_speed_mps = 11.0;
        shot.fire_rate_hz = 2.0;
    otherwise
        error('未知发射模式：%s。请使用 17mm 或 42mm。', shot.mode_name);
end

% 由数据手册可直接得到的辅助量。
motor.no_load_speed_rpm = motor.rated_voltage_v * motor.speed_constant_rpm_per_v; % 纯电机理论空载转速，约 28477 RPM
motor.max_torque_nm = motor.torque_constant_nm_per_a * motor.current_limit_a;      % 3 A 限流下的电机转矩估计

% 由发射工况或手动输入得到目标转速。
%
% 如果你现在主要想研究“某个固定摩擦轮转速能不能稳住”，
% 就直接使用 manual_rpm 模式。
switch lower(shot.target_speed_mode)
    case 'manual_rpm'
        shot.target_speed_rpm = shot.manual_target_speed_rpm;
        % 反算一个等效圆周线速度，方便后面做参考说明。
        shot.equivalent_surface_speed_mps = shot.target_speed_rpm * (2.0 * pi * mech.wheel_radius_m) / 60.0;
    case 'from_projectile'
        % 把“球速 = 摩擦轮圆周线速度”作为第一近似，因此目标转速可由半径反推：
        % target_rpm = v / (2*pi*r) * 60
        shot.target_speed_rpm = shot.projectile_speed_mps / (2.0 * pi * mech.wheel_radius_m) * 60.0;
        shot.equivalent_surface_speed_mps = shot.projectile_speed_mps;
    otherwise
        error('未知目标转速模式：%s。请使用 manual_rpm 或 from_projectile。', shot.target_speed_mode);
end

% 理想接触时间近似：
% contact_time_ideal = 2 * diameter / speed
% 实机中轮面压缩、弹丸变形、打滑都会把有效作用时间拖长，
% 因此这里再乘一个 contact_time_scale，使仿真更保守。
shot.contact_time_ideal_s = 2.0 * shot.projectile_diameter_m / shot.projectile_speed_mps;
shot.contact_time_s = shot.contact_time_ideal_s * shot.contact_time_scale;

% 单发弹丸动能，用于帮助理解负载大小。
shot.projectile_energy_j = 0.5 * shot.projectile_mass_kg * shot.projectile_speed_mps ^ 2;

% 理想平均接触力近似：
% F_avg * s = 1/2 * m * v^2，取接触距离 s ≈ 球径
% => F_avg = (1/2 * m * v^2) / diameter
shot.average_contact_force_ideal_n = 0.5 * shot.projectile_mass_kg * shot.projectile_speed_mps ^ 2 / ...
                                     shot.projectile_diameter_m;

% 实机里轮胎压缩、局部打滑、压弹阻力和材料损失都会让负载比理想值更重。
% 因此这里再乘一个保守系数 dynamic_load_factor。
shot.average_contact_force_n = shot.average_contact_force_ideal_n * shot.dynamic_load_factor;

% 单发弹丸动能按三路摩擦轮分配，换算为接触期间的平均负载转矩。
% 该负载作用在接触窗口内，避免把瞬时冲击误写成持续稳态转速跌落。
shot.load_energy_per_motor_j = shot.projectile_energy_j * shot.dynamic_load_factor / ...
                               (mech.wheel_count * max(mech.efficiency, 1.0e-6));
shaft_speed_rad_s = shot.target_speed_rpm * 2.0 * pi / 60.0;
shot.load_torque_per_motor_nm = shot.load_energy_per_motor_j / ...
                                (max(shot.contact_time_s, 1.0e-6) * max(shaft_speed_rad_s, 1.0e-6));
shot.load_accel_rpm_per_s = (shot.load_torque_per_motor_nm / motor.rotor_inertia_kgm2) * 60.0 / (2.0 * pi);
shot.speed_drop_per_shot_rpm = shot.load_accel_rpm_per_s * shot.contact_time_s;

cfg.ts = 1e-3;                 % 控制周期，单位 s，这里为 1 ms
cfg.total_time = 2.0;          % 总仿真时长，单位 s；用于观察一段稳定运行期间的连续发射效果
cfg.feedback_range = max(14000, shot.target_speed_rpm * 1.3); % 反馈量程，单位 RPM，需覆盖目标转速与一定超调余量
cfg.output_limit = motor.current_limit_a; % 控制输出绝对限幅，按电流命令 A 理解，即输出范围约为 [-4, 4]
cfg.output_rate_limit = 1000.0; % 输出变化率限制，单位 A/s；对应当前工程 SHOOT_FRIC*_OUTPUT_RATE_LIMIT
cfg.noise_std = 10.0;          % 测速噪声标准差，单位 RPM，用于模拟传感器测量噪声
cfg.start_from_steady_speed = true; % 真实工况下摩擦轮已稳速运行
cfg.recovery_target_s = 50e-3; % 目标恢复时间，单位 s

% 发射窗口放在“已经稳定运转”的时间段内。
% 这样算出来的掉速和恢复时间，更接近实战中连续供弹时的表现。
shot.fire_start_s = 0.20;      % 开始发射时间，单位 s
shot.fire_end_s = shot.fire_start_s + 17.0 / shot.fire_rate_hz + shot.contact_time_s;        % 结束发射时间，单位 s

% 摩擦轮等效对象参数
% 这里采用的一阶等效对象为：
% speed_dot = -speed/tau + b*u - coulomb*tanh(speed/100) - load
% 其中 speed 的单位按 RPM 理解，u 的单位按电流 A 理解。
plant.tau = motor.mechanical_time_constant_s; % 对象时间常数，取 M2006 手册机械时间常数 52.78 ms
plant.shot = shot;                           % 把发射工况打包进对象，供被控对象仿真使用

% 对于本脚本的一阶模型，plant.b 表示“单位电流产生的转速变化率增益”。
% 这里先用工程近似：
% 1. 由纯电机转速常数得到理论空载转速；
% 2. 以 3 A 电流上限估算对象输入增益；
% 3. 一阶模型稳态下 speed_ss = tau * b * u，因此可反推：
%    b ≈ no_load_speed_rpm / (tau * current_limit)。
% 注意：
% - 这是为了让脚本初值与数据手册量级一致的工程估算，不是严格物理辨识值；
% - speed_torque_gradient 在这个“输入为电流的一阶等效模型”里不能直接唯一确定 b；
% - 若后续做台架辨识，应以实测数据修正 plant.b。
plant.b = motor.no_load_speed_rpm / (plant.tau * motor.current_limit_a);

% 库仑摩擦和外部负载扰动通常无法直接由数据手册给出，仍需靠经验或实验辨识。
plant.coulomb = 0.0;        % 库仑摩擦等效项，数值越大表示低速区和换向时的摩擦阻力越明显
% 将单发转速跌落换算成对象中的 speed_dot 负载项：
% 一阶模型稳态下，持续负载 load 会造成 speed_ss = tau*(b*u - load - ...)
% 因而若希望单发作用期间体现为“等效稳态跌落 delta_speed_ss”，可取：
% load ≈ delta_speed_ss / tau
plant.shot_load_rpm_per_s = shot.load_accel_rpm_per_s; % 接触窗口内的实际转速减速度，单位 rpm/s

% 第一步：生成辨识数据
id_data = generate_identification_data(cfg, plant);

% 第二步：识别对象
estimate = identify_first_order_model(cfg.ts, id_data.u, id_data.speed);

% 第三步：构造整定初值
initial.wc = min(max(4.0 / estimate.tau, 80.0), 500.0);
initial.wo_ratio = 5.0;
initial.delta_ratio = 0.01;
initial.b0_scale = 1.0;

x0 = log([initial.wc, initial.wo_ratio, initial.delta_ratio, initial.b0_scale]);
objective = @(x) tuning_cost(x, cfg, plant, estimate);
% 这里把搜索迭代数适当放大，并关闭逐步打印，避免命令行输出过于杂乱。
options = optimset('Display', 'off', 'MaxIter', 200, 'MaxFunEvals', 400, 'TolX', 1e-3, 'TolFun', 1e-3);

% 第四步：自动搜索参数
best_x = fminsearch(objective, x0, options);
[best_cost, tuned] = tuning_cost(best_x, cfg, plant, estimate);

% 第五步：使用整定结果做最终仿真
sim = run_closed_loop_sim(cfg, plant, tuned);

% 第六步：计算性能指标
metrics = compute_performance_metrics(sim, best_cost);

% 第七步：输出、保存和显示结果
print_summary(cfg, plant, estimate, tuned, metrics);
export_recommended_config(cfg, plant, tuned, estimate, metrics);
plot_result(id_data, sim, plant, estimate, tuned, metrics);

result.estimate = estimate;
result.tuned = tuned;
result.metrics = metrics;
result.identification = id_data;
result.simulation = sim;
end

function data = generate_identification_data(cfg, plant)
% 生成开环辨识数据。
%
% 这里模拟 MCU 自动标定阶段的激励序列：
% 先静止，再给几段不同幅值和正负方向的输出，
% 让对象暴露出足够的动态特性供辨识使用。

n = floor(cfg.total_time / cfg.ts);
t = (0:n-1)' * cfg.ts;
u = zeros(n, 1);
speed = zeros(n, 1);
id_plant = plant;

% 开环辨识阶段默认按空载对象进行，不叠加发射脉冲负载，
% 避免把“发射事件引起的瞬态掉速”混入基础对象辨识结果。
id_plant.shot.fire_rate_hz = 0.0;

for k = 2:n
    if t(k) < 0.08
        u(k) = 0.0;
    elseif t(k) < 0.24
        u(k) = 4.0;
    elseif t(k) < 0.40
        u(k) = -1.4;
    elseif t(k) < 0.56
        u(k) = 2.8;
    else
        u(k) = -2.2;
    end

    speed(k) = plant_step(cfg.ts, speed(k-1), u(k-1), id_plant, t(k));
    speed(k) = speed(k) + cfg.noise_std * randn();
end

data.t = t;
data.u = u;
data.speed = speed;
end

function estimate = identify_first_order_model(ts, u, y)
% 使用递推最小二乘识别一阶离散对象：
%
% y(k) = A * y(k-1) + B * u(k-1) + C

theta = [0.98; 0.01; 0.0];
P = eye(3) * 1000;
lambda = 0.995;

for k = 2:numel(y)
    phi = [y(k-1); u(k-1); 1.0];
    gain = (P * phi) / (lambda + phi' * P * phi);
    err = y(k) - phi' * theta;
    theta = theta + gain * err;
    P = (P - gain * phi' * P) / lambda;
end

A = min(max(theta(1), 1e-4), 0.9999);
B = max(theta(2), 1e-6);
C = theta(3);
a = -log(A) / ts;
b0 = B * a / max(1e-6, (1 - A));

estimate.A = A;
estimate.B = B;
estimate.C = C;
estimate.a = a;
estimate.b0 = b0;
estimate.tau = 1 / a;
estimate.static_gain = B / max(1e-6, (1 - A));
estimate.bias = C / max(1e-6, (1 - A));
end

function [cost, tuned] = tuning_cost(x, cfg, plant, estimate)
% 代价函数：
% 综合考虑误差积分、超调、控制能量、控制变化量、扰动估计误差，
% 以及发射瞬间掉速与恢复时间，用于自动搜索一组更贴近实战的 ADRC 参数。

tuned.wc = min(max(exp(x(1)), 40.0), 800.0);
tuned.wo_ratio = min(max(exp(x(2)), 2.5), 10.0);
tuned.delta = cfg.feedback_range * min(max(exp(x(3)), 0.002), 0.05);
tuned.b0 = estimate.b0 * min(max(exp(x(4)), 0.4), 1.8);
tuned.alpha1 = 0.5;
tuned.alpha2 = 0.25;
tuned.tracking_gain = 0.0;

sim = run_closed_loop_sim(cfg, plant, tuned);

speed_err = sim.target - sim.speed;
iae = trapz(sim.t, abs(speed_err));
overshoot = max(max(sim.speed - sim.target), 0);
effort = trapz(sim.t, sim.u .^ 2);
du = sum(abs(diff(sim.u)));
disturbance_error = trapz(sim.t, abs(sim.real_disturbance - sim.z2));
shot_droop_cost = sim.max_shot_droop_rpm;

if isnan(sim.mean_recovery_time_s)
    recovery_cost = cfg.total_time;
else
    recovery_cost = max(sim.mean_recovery_time_s - cfg.recovery_target_s, 0.0);
end

cost = 0.004 * iae ...
     + 0.003 * overshoot ...
     + 0.01 * effort ...
     + 0.002 * du ...
     + 0.0005 * disturbance_error ...
     + 0.003 * shot_droop_cost ...
     + 10.0 * recovery_cost;
end

function sim = run_closed_loop_sim(cfg, plant, tuned)
% 按 MCU 侧 ADRC 结构进行闭环仿真。
% 当前版本更贴近真实摩擦轮工况：
% 1. 摩擦轮默认已经处于开启状态；
% 2. 仿真开始时直接从目标转速附近起步；
% 3. 在设定时间窗内按发射频率注入短时脉冲负载。

n = floor(cfg.total_time / cfg.ts);
t = (0:n-1)' * cfg.ts;

% 固定测量噪声序列，避免优化过程中目标函数随机波动。
noise_state = rng;
rng(2);
measurement_noise = cfg.noise_std * randn(n, 1);
rng(noise_state);

target = zeros(n, 1);
speed = zeros(n, 1);
speed_meas = zeros(n, 1);
u = zeros(n, 1);
z1 = zeros(n, 1);
z2 = zeros(n, 1);
real_disturbance = zeros(n, 1);

kp = tuned.wc;
wo = tuned.wc * tuned.wo_ratio;
beta1 = 2 * wo;
beta2 = wo * wo;

target(1) = speed_target_profile(t(1), plant);
if cfg.start_from_steady_speed
    % 直接从“已稳速”的工作点启动仿真，更接近摩擦轮常开工况。
    speed(1) = target(1);
    z1(1) = target(1);
    z2(1) = steady_state_disturbance(target(1), plant);
    u(1) = steady_state_control(target(1), plant);
    u(1) = min(max(u(1), -cfg.output_limit), cfg.output_limit);
    real_disturbance(1) = z2(1);
else
    speed(1) = 0.0;
    z1(1) = 0.0;
    z2(1) = 0.0;
    u(1) = 0.0;
end

for k = 2:n
    target(k) = speed_target_profile(t(k), plant);
    speed_meas(k-1) = speed(k-1) + measurement_noise(k-1);

    % ESO 更新
    observer_error = z1(k-1) - speed_meas(k-1);
    z1(k) = z1(k-1) + cfg.ts * (z2(k-1) - beta1 * observer_error + tuned.b0 * u(k-1));
    z2(k) = z2(k-1) + cfg.ts * (-beta2 * fal(observer_error, tuned.alpha2, tuned.delta));

    % 控制律
    track_error = target(k) - z1(k);
    raw_u = (kp * fal(track_error, tuned.alpha1, tuned.delta) - z2(k)) / tuned.b0;

    % 变化率限制 + 限幅
    delta_u = raw_u - u(k-1);
    delta_u = min(max(delta_u, -cfg.output_rate_limit * cfg.ts), cfg.output_rate_limit * cfg.ts);
    u(k) = min(max(u(k-1) + delta_u, -cfg.output_limit), cfg.output_limit);

    % 被控对象更新
    [speed(k), real_disturbance(k)] = plant_step(cfg.ts, speed(k-1), u(k), plant, t(k));
end

speed_meas(end) = speed(end) + measurement_noise(end);

sim.t = t;
sim.target = target;
sim.speed = speed;
sim.speed_meas = speed_meas;
sim.u = u;
sim.z1 = z1;
sim.z2 = z2;
sim.real_disturbance = real_disturbance;
sim.fire_start_s = plant.shot.fire_start_s;
sim.fire_end_s = plant.shot.fire_end_s;
[sim.max_shot_droop_rpm, sim.mean_shot_droop_rpm, sim.mean_recovery_time_s] = analyze_shot_response(sim, plant);
end

function [next_speed, total_disturbance] = plant_step(ts, speed, control, plant, current_time)
% 摩擦轮等效对象：
%
% speed_dot = -speed/tau + b*u - coulomb*tanh(speed/100) - load

load_term = shot_load_profile(current_time, plant);
drive_gain_scale = drive_gain_profile(current_time, plant);
coulomb_term = plant.coulomb * tanh(speed / 100.0);
input_gain_mismatch = plant.b * (drive_gain_scale - 1.0) * control;
total_disturbance = -(speed / plant.tau) - coulomb_term - load_term + input_gain_mismatch;
speed_dot = total_disturbance + plant.b * control;
next_speed = speed + ts * speed_dot;
end

function load_term = shot_load_profile(current_time, plant)
% 发射脉冲负载模型。
% 在 fire_start_s ~ fire_end_s 时间窗内，按 fire_rate_hz 周期性注入短脉冲负载。
%
% 当前版本不再使用理想矩形脉冲，而是使用“归一化半正弦脉冲”：
% 1. 平均负载仍然由 shot_load_rpm_per_s 决定
% 2. 峰值更高，更接近实机中压弹瞬间的冲击负载
% 3. 配合 contact_time_scale / dynamic_load_factor / supply_sag_ratio，
%    可以把模型从“乐观上限”往“更偏保守”方向拉
shot = plant.shot;
load_term = 0.0;

if (current_time < shot.fire_start_s) || (current_time > shot.fire_end_s) || (shot.fire_rate_hz <= 0.0)
    return;
end

shot_period_s = 1.0 / shot.fire_rate_hz;
phase_time = mod(current_time - shot.fire_start_s, shot_period_s);
if phase_time <= shot.contact_time_s
    load_term = plant.shot_load_rpm_per_s * shot_pulse_shape(phase_time, shot);
end
end

function gain_scale = drive_gain_profile(current_time, plant)
% 发射期间驱动能力修正。
% 用于模拟：
% 1. 母线电压短时下垂
% 2. 电调限流或供电瞬态
% 3. 发射后需要几毫秒恢复到正常输出能力

shot = plant.shot;
gain_scale = 1.0;

if (current_time < shot.fire_start_s) || (current_time > shot.fire_end_s) || (shot.fire_rate_hz <= 0.0)
    return;
end

shot_period_s = 1.0 / shot.fire_rate_hz;
phase_time = mod(current_time - shot.fire_start_s, shot_period_s);

if phase_time <= shot.contact_time_s
    gain_scale = 1.0 - shot.supply_sag_ratio;
elseif phase_time <= (shot.contact_time_s + 4.0 * shot.supply_recovery_tau_s)
    sag = shot.supply_sag_ratio * exp(-(phase_time - shot.contact_time_s) / ...
                                      max(shot.supply_recovery_tau_s, 1.0e-6));
    gain_scale = 1.0 - sag;
end

gain_scale = min(max(gain_scale, 0.5), 1.0);
end

function shape_value = shot_pulse_shape(phase_time, shot)
% 单发负载脉冲形状。
%
% 这里使用“平均值为 1 的半正弦”：
% shape = (pi/2) * sin(pi * x), x in [0, 1]
% 这样做的好处是：
% 1. 平均负载不变
% 2. 峰值比矩形脉冲更高
% 3. 更接近实际接触过程中的“先压入、后释放”特征

if phase_time < 0.0 || phase_time > shot.contact_time_s
    shape_value = 0.0;
    return;
end

normalized_phase = phase_time / max(shot.contact_time_s, 1.0e-9);
switch lower(shot.pulse_shape)
    case 'half_sine'
        shape_value = 0.5 * pi * sin(pi * normalized_phase);
    otherwise
        shape_value = 1.0;
end

shape_value = max(shape_value, 0.0);
end

function target = speed_target_profile(~, plant)
% 目标速度曲线。
% 摩擦轮在真实使用中通常一直开启，因此这里直接保持恒定目标转速。

target = plant.shot.target_speed_rpm;
end

function u_eq = steady_state_control(target_speed, plant)
% 估算“空载稳速时”为维持当前转速所需的控制输出。
% 这样仿真一开始就不会因为从 0 输出起步而人为出现一次掉速。

u_eq = -steady_state_disturbance(target_speed, plant) / max(plant.b, 1.0e-6);
end

function dist_eq = steady_state_disturbance(target_speed, plant)
% 估算空载稳速点对应的总扰动。
% 把它作为 ESO 初值后，控制器一开始就知道“维持稳速需要抵消多少扰动”，
% 从而避免常开工况下出现一次不真实的初始掉速。

coulomb_term = plant.coulomb * tanh(target_speed / 100.0);
dist_eq = -(target_speed / plant.tau) - coulomb_term;
end

function y = fal(e, alpha, delta)
% 与 MCU 端保持一致的 fal 非线性函数。

if abs(e) <= delta
    y = e / (delta ^ (1 - alpha));
else
    y = (abs(e) ^ alpha) * sign_nonzero(e);
end
end

function s = sign_nonzero(x)
% 符号函数，0 按正处理，避免返回 0。

if x >= 0
    s = 1;
else
    s = -1;
end
end

function metrics = compute_performance_metrics(sim, best_cost)
% 计算一组方便人工判断性能的指标。

speed_err = sim.target - sim.speed;
tail_count = min(50, numel(sim.t));
tail_err = speed_err(end-tail_count+1:end);
tail_speed = sim.speed(end-tail_count+1:end);

metrics.objective_cost = best_cost;
metrics.target_speed_rpm = sim.target(end);
metrics.iae = trapz(sim.t, abs(speed_err));
metrics.rmse = sqrt(mean(speed_err .^ 2));
metrics.max_abs_error = max(abs(speed_err));
metrics.final_mean_error = mean(tail_err);
metrics.final_mean_speed = mean(tail_speed);
metrics.control_peak = max(abs(sim.u));
metrics.control_rms = sqrt(mean(sim.u .^ 2));
metrics.disturbance_rmse = sqrt(mean((sim.real_disturbance - sim.z2) .^ 2));
metrics.max_shot_droop_rpm = sim.max_shot_droop_rpm;
metrics.mean_shot_droop_rpm = sim.mean_shot_droop_rpm;
metrics.mean_recovery_time_s = sim.mean_recovery_time_s;

[metrics.first_step_target, ...
 metrics.first_step_amplitude, ...
 metrics.first_step_start_s, ...
 metrics.first_step_end_s, ...
 metrics.rise_time_s, ...
 metrics.settling_time_s, ...
 metrics.overshoot_pct] = analyze_first_step(sim.t, sim.target, sim.speed);
end

function [max_droop_rpm, mean_droop_rpm, mean_recovery_time_s] = analyze_shot_response(sim, plant)
% 统计每次发射脉冲造成的掉速和恢复时间。
shot = plant.shot;
shot_period_s = 1.0 / max(shot.fire_rate_hz, 1.0e-6);
shot_times = shot.fire_start_s:shot_period_s:shot.fire_end_s;

if isempty(shot_times)
    max_droop_rpm = 0.0;
    mean_droop_rpm = 0.0;
    mean_recovery_time_s = NaN;
    return;
end

droop_list = zeros(numel(shot_times), 1);
recovery_list = NaN(numel(shot_times), 1);
recovery_band = max(0.005 * shot.target_speed_rpm, 10.0);

for i = 1:numel(shot_times)
    shot_time = shot_times(i);
    shot_end = min(shot_time + shot_period_s, sim.t(end));
    idx = find((sim.t >= shot_time) & (sim.t < shot_end));
    if isempty(idx)
        continue;
    end

    [min_speed, min_rel_idx] = min(sim.speed(idx));
    min_abs_idx = idx(min_rel_idx);
    droop_list(i) = max(shot.target_speed_rpm - min_speed, 0.0);

    recovery_idx = find((1:numel(sim.t))' >= min_abs_idx & ...
                        sim.speed >= (shot.target_speed_rpm - recovery_band), 1, 'first');
    if ~isempty(recovery_idx)
        recovery_list(i) = sim.t(recovery_idx) - sim.t(min_abs_idx);
    end
end

max_droop_rpm = max(droop_list);
mean_droop_rpm = mean(droop_list);
if all(isnan(recovery_list))
    mean_recovery_time_s = NaN;
else
    mean_recovery_time_s = mean(recovery_list(~isnan(recovery_list)));
end
end

function [step_target, amplitude, step_start_s, step_end_s, rise_time_s, settling_time_s, overshoot_pct] = ...
    analyze_first_step(t, target, speed)
% 分析第一次目标阶跃的动态指标：
% 1. 10%~90% 上升时间
% 2. 2% 调节时间
% 3. 超调百分比

change_idx = find(abs(diff(target)) > 1e-9) + 1;

step_target = NaN;
amplitude = NaN;
step_start_s = NaN;
step_end_s = NaN;
rise_time_s = NaN;
settling_time_s = NaN;
overshoot_pct = NaN;

if isempty(change_idx)
    return;
end

step_start = change_idx(1);
step_end = numel(target);
if numel(change_idx) >= 2
    step_end = change_idx(2) - 1;
end

baseline = target(step_start - 1);
step_target = target(step_start);
amplitude = step_target - baseline;
step_start_s = t(step_start);
step_end_s = t(step_end);

if abs(amplitude) < 1e-9
    return;
end

segment_t = t(step_start:step_end) - t(step_start);
segment_y = speed(step_start:step_end);

if amplitude > 0
    y10 = baseline + 0.1 * amplitude;
    y90 = baseline + 0.9 * amplitude;
    idx10 = find(segment_y >= y10, 1, 'first');
    idx90 = find(segment_y >= y90, 1, 'first');
    overshoot = max(segment_y) - step_target;
else
    y10 = baseline + 0.9 * amplitude;
    y90 = baseline + 0.1 * amplitude;
    idx10 = find(segment_y <= y10, 1, 'first');
    idx90 = find(segment_y <= y90, 1, 'first');
    overshoot = step_target - min(segment_y);
end

if ~isempty(idx10) && ~isempty(idx90) && (idx90 >= idx10)
    rise_time_s = segment_t(idx90) - segment_t(idx10);
end

band = 0.02 * abs(amplitude);
for k = 1:numel(segment_y)
    if all(abs(segment_y(k:end) - step_target) <= band)
        settling_time_s = segment_t(k);
        break;
    end
end

overshoot_pct = max(overshoot, 0.0) / abs(amplitude) * 100.0;
end

function print_summary(cfg, plant, estimate, tuned, metrics)
% 在命令行打印最终结果，便于快速查看。

fprintf('\n==== 识别结果 ====\n');
fprintf('离散 A                = %.6f\n', estimate.A);
fprintf('离散 B                = %.6f\n', estimate.B);
fprintf('连续时间常数 tau      = %.6f s\n', estimate.tau);
fprintf('连续域 b0             = %.6f\n', estimate.b0);

fprintf('\n==== 推荐 ADRC 参数 ====\n');
fprintf('sample_time_s         = %.6f\n', cfg.ts);
fprintf('b0                    = %.6f\n', tuned.b0);
fprintf('controller_bandwidth  = %.6f\n', tuned.wc);
fprintf('observer_ratio        = %.6f\n', tuned.wo_ratio);
fprintf('error_linear_zone     = %.6f\n', tuned.delta);
fprintf('output_limit          = %.6f\n', cfg.output_limit);
fprintf('output_rate_limit     = %.6f\n', cfg.output_rate_limit);

fprintf('\n==== 发射模型修正 ====\n');
fprintf('contact_time_scale    = %.6f\n', plant.shot.contact_time_scale);
fprintf('dynamic_load_factor   = %.6f\n', plant.shot.dynamic_load_factor);
fprintf('supply_sag_ratio      = %.6f\n', plant.shot.supply_sag_ratio);
fprintf('supply_recovery_tau_s = %.6f s\n', plant.shot.supply_recovery_tau_s);

fprintf('\n==== 目标转速设置 ====\n');
fprintf('target_speed_mode     = %s\n', plant.shot.target_speed_mode);
fprintf('target_speed_rpm      = %.6f rpm\n', plant.shot.target_speed_rpm);
fprintf('surface_speed_mps     = %.6f m/s\n', plant.shot.equivalent_surface_speed_mps);

fprintf('\n==== 稳速指标 ====\n');
fprintf('objective_cost        = %.6f\n', metrics.objective_cost);
fprintf('target_speed_rpm      = %.6f rpm\n', metrics.target_speed_rpm);
fprintf('IAE                   = %.6f\n', metrics.iae);
fprintf('speed_rmse            = %.6f rpm\n', metrics.rmse);
fprintf('max_abs_error         = %.6f rpm\n', metrics.max_abs_error);
fprintf('final_mean_speed      = %.6f rpm\n', metrics.final_mean_speed);
fprintf('final_mean_error      = %.6f rpm\n', metrics.final_mean_error);
fprintf('control_peak          = %.6f\n', metrics.control_peak);
fprintf('control_rms           = %.6f\n', metrics.control_rms);
fprintf('disturbance_rmse      = %.6f\n', metrics.disturbance_rmse);

fprintf('\n==== 发射指标 ====\n');
fprintf('max_shot_droop_rpm    = %.6f\n', metrics.max_shot_droop_rpm);
fprintf('mean_shot_droop_rpm   = %.6f\n', metrics.mean_shot_droop_rpm);
fprintf('mean_recovery_time_s  = %s\n', fmt_value(metrics.mean_recovery_time_s, '%.6f s'));
if ~isnan(metrics.rise_time_s) || ~isnan(metrics.settling_time_s) || ~isnan(metrics.overshoot_pct)
    fprintf('\n==== 起转指标（如果本次仿真包含起转过程） ====\n');
    fprintf('rise_time_10_90       = %s\n', fmt_value(metrics.rise_time_s, '%.6f s'));
    fprintf('settling_time_2pct    = %s\n', fmt_value(metrics.settling_time_s, '%.6f s'));
    fprintf('overshoot_pct         = %s\n', fmt_value(metrics.overshoot_pct, '%.3f %%'));
else
fprintf('\n==== 起转指标 ====\n');
fprintf('当前工况从稳速开始仿真，因此不统计起转时间和超调。\n');
end

print_copy_paste_snippet(cfg, tuned);
end

function print_copy_paste_snippet(cfg, tuned)
% 在命令行直接打印可复制到工程里的代码块。

fprintf('\n==== 可直接复制到工程的代码 ====\n');
fprintf('下面这段可以直接复制到 .c 文件里使用：\n\n');

fprintf('adrc_param_t param = {\n');
fprintf('    .sample_time_s = %.8ff,\n', cfg.ts);
fprintf('    .b0 = %.8ff,\n', tuned.b0);
fprintf('    .controller_bandwidth = %.8ff,\n', tuned.wc);
fprintf('    .observer_bandwidth_ratio = %.8ff,\n', tuned.wo_ratio);
fprintf('    .tracking_gain = 0.0f,\n');
fprintf('    .max_out = %.8ff,\n', cfg.output_limit);
fprintf('    .output_rate_limit = %.8ff,\n', cfg.output_rate_limit);
fprintf('    .error_linear_zone = %.8ff,\n', tuned.delta);
fprintf('    .alpha1 = 0.5f,\n');
fprintf('    .alpha2 = 0.25f,\n');
fprintf('};\n\n');

fprintf('adrc_type_def adrc;\n');
fprintf('ADRC_init(&adrc, &param);\n');
fprintf('/* 如果摩擦轮已经在稳速运行，可选：ADRC_hot_reset(&adrc, measured_speed_rpm, target_speed_rpm, hold_current_a); */\n');
fprintf('out = ADRC_Calc(&adrc, measured_speed_rpm, target_speed_rpm);\n');
fprintf('/* 停机或清零时：ADRC_clear(&adrc); */\n');
end

function export_recommended_config(cfg, plant, tuned, estimate, metrics)
% 导出推荐参数和性能指标，方便直接拷到 MCU 工程里。

file_id = fopen('friction_wheel_adrc_recommended.txt', 'w');
if file_id < 0
    warning('无法写出 friction_wheel_adrc_recommended.txt');
    return;
end

fprintf(file_id, '%% 摩擦轮 ADRC 推荐配置\n');
fprintf(file_id, 'sample_time_s = %.8f;\n', cfg.ts);
fprintf(file_id, 'identified_tau_s = %.8f;\n', estimate.tau);
fprintf(file_id, 'identified_b0 = %.8f;\n', estimate.b0);
fprintf(file_id, 'controller_bandwidth = %.8f;\n', tuned.wc);
fprintf(file_id, 'observer_ratio = %.8f;\n', tuned.wo_ratio);
fprintf(file_id, 'error_linear_zone = %.8f;\n', tuned.delta);
fprintf(file_id, 'recommended_b0 = %.8f;\n', tuned.b0);
fprintf(file_id, 'model_contact_time_scale = %.8f;\n', plant.shot.contact_time_scale);
fprintf(file_id, 'model_dynamic_load_factor = %.8f;\n', plant.shot.dynamic_load_factor);
fprintf(file_id, 'model_supply_sag_ratio = %.8f;\n', plant.shot.supply_sag_ratio);
fprintf(file_id, 'model_supply_recovery_tau_s = %.8f;\n', plant.shot.supply_recovery_tau_s);
fprintf(file_id, 'target_speed_mode = %s;\n', plant.shot.target_speed_mode);
fprintf(file_id, 'target_speed_rpm = %.8f;\n', plant.shot.target_speed_rpm);
fprintf(file_id, 'surface_speed_mps = %.8f;\n', plant.shot.equivalent_surface_speed_mps);
fprintf(file_id, 'output_limit = %.8f;\n', cfg.output_limit);
fprintf(file_id, 'output_rate_limit = %.8f;\n', cfg.output_rate_limit);
fprintf(file_id, 'objective_cost = %.8f;\n', metrics.objective_cost);
fprintf(file_id, 'target_speed_rpm = %.8f;\n', metrics.target_speed_rpm);
fprintf(file_id, 'iae = %.8f;\n', metrics.iae);
fprintf(file_id, 'speed_rmse = %.8f;\n', metrics.rmse);
fprintf(file_id, 'max_abs_error = %.8f;\n', metrics.max_abs_error);
fprintf(file_id, 'final_mean_speed = %.8f;\n', metrics.final_mean_speed);
fprintf(file_id, 'final_mean_error = %.8f;\n', metrics.final_mean_error);
fprintf(file_id, 'control_peak = %.8f;\n', metrics.control_peak);
fprintf(file_id, 'control_rms = %.8f;\n', metrics.control_rms);
fprintf(file_id, 'disturbance_rmse = %.8f;\n', metrics.disturbance_rmse);
fprintf(file_id, 'max_shot_droop_rpm = %.8f;\n', metrics.max_shot_droop_rpm);
fprintf(file_id, 'mean_shot_droop_rpm = %.8f;\n', metrics.mean_shot_droop_rpm);
fprintf(file_id, 'mean_recovery_time_s = %s;\n', fmt_value(metrics.mean_recovery_time_s, '%.8f'));
fprintf(file_id, 'rise_time_s = %s;\n', fmt_value(metrics.rise_time_s, '%.8f'));
fprintf(file_id, 'settling_time_s = %s;\n', fmt_value(metrics.settling_time_s, '%.8f'));
fprintf(file_id, 'overshoot_pct = %s;\n', fmt_value(metrics.overshoot_pct, '%.8f'));
fprintf(file_id, '\n');
fprintf(file_id, '%% ================= 可直接复制到工程的代码 =================\n');
fprintf(file_id, 'adrc_param_t param = {\n');
fprintf(file_id, '    .sample_time_s = %.8ff,\n', cfg.ts);
fprintf(file_id, '    .b0 = %.8ff,\n', tuned.b0);
fprintf(file_id, '    .controller_bandwidth = %.8ff,\n', tuned.wc);
fprintf(file_id, '    .observer_bandwidth_ratio = %.8ff,\n', tuned.wo_ratio);
fprintf(file_id, '    .tracking_gain = 0.0f,\n');
fprintf(file_id, '    .max_out = %.8ff,\n', cfg.output_limit);
fprintf(file_id, '    .output_rate_limit = %.8ff,\n', cfg.output_rate_limit);
fprintf(file_id, '    .error_linear_zone = %.8ff,\n', tuned.delta);
fprintf(file_id, '    .alpha1 = 0.5f,\n');
fprintf(file_id, '    .alpha2 = 0.25f,\n');
fprintf(file_id, '};\n');
fprintf(file_id, '\n');
fprintf(file_id, 'adrc_type_def adrc;\n');
fprintf(file_id, 'ADRC_init(&adrc, &param);\n');
fprintf(file_id, '/* 如果摩擦轮已经在稳速运行，可选：ADRC_hot_reset(&adrc, measured_speed_rpm, target_speed_rpm, hold_current_a); */\n');
fprintf(file_id, 'out = ADRC_Calc(&adrc, measured_speed_rpm, target_speed_rpm);\n');
fprintf(file_id, '/* 停机或清零时：ADRC_clear(&adrc); */\n');
fprintf(file_id, '\n');
fprintf(file_id, '%% ================= 底层通用配置结构 =================\n');
fprintf(file_id, 'ADRC_Config cfg = {\n');
fprintf(file_id, '    .sample_time_s = %.8ff,\n', cfg.ts);
fprintf(file_id, '    .b0 = %.8ff,\n', tuned.b0);
fprintf(file_id, '    .controller_bandwidth = %.8ff,\n', tuned.wc);
fprintf(file_id, '    .observer_bandwidth_ratio = %.8ff,\n', tuned.wo_ratio);
fprintf(file_id, '    .tracking_gain = 0.0f,\n');
fprintf(file_id, '    .output_min = %.8ff,\n', -cfg.output_limit);
fprintf(file_id, '    .output_max = %.8ff,\n', cfg.output_limit);
fprintf(file_id, '    .output_rate_limit = %.8ff,\n', cfg.output_rate_limit);
fprintf(file_id, '    .error_linear_zone = %.8ff,\n', tuned.delta);
fprintf(file_id, '    .alpha1 = 0.5f,\n');
fprintf(file_id, '    .alpha2 = 0.25f,\n');
fprintf(file_id, '    .motor = 0\n');
fprintf(file_id, '};\n');
fclose(file_id);
end

function plot_result(id_data, sim, plant, estimate, tuned, metrics)
% 绘制辨识曲线、闭环曲线、控制量/扰动曲线和指标面板。

font_name = choose_chinese_font();

figure('Name', '摩擦轮 ADRC 自动整定总览', ...
       'Color', 'w', ...
       'Position', [100, 100, 1200, 760]);

tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

% 子图 1：开环辨识曲线
ax1 = nexttile(1);
plot(ax1, id_data.t, id_data.speed, 'b', 'LineWidth', 1.1);
hold(ax1, 'on');
plot(ax1, id_data.t, id_data.u * 600, '--r', 'LineWidth', 1.0);
grid(ax1, 'on');
xlabel(ax1, '时间 (s)');
ylabel(ax1, '辨识阶段量');
legend(ax1, '速度 (rpm)', '缩放后的输入', 'Location', 'best');
title(ax1, '开环辨识阶段');

% 子图 2：闭环速度响应
ax2 = nexttile(2);
plot(ax2, sim.t, sim.target, '--k', 'LineWidth', 1.2);
hold(ax2, 'on');
plot(ax2, sim.t, sim.speed, 'b', 'LineWidth', 1.2);
if all(abs(sim.target - sim.target(1)) < 1.0e-9)
    band = 0.02 * abs(sim.target(1));
    yline(ax2, sim.target(1) + band, ':', '+2%带宽', 'Color', [0.4 0.4 0.4]);
    yline(ax2, sim.target(1) - band, ':', '-2%带宽', 'Color', [0.4 0.4 0.4]);
elseif ~isnan(metrics.first_step_target) && ~isnan(metrics.first_step_amplitude)
    band = 0.02 * abs(metrics.first_step_amplitude);
    yline(ax2, metrics.first_step_target + band, ':', '2%带宽', 'Color', [0.4 0.4 0.4]);
    yline(ax2, metrics.first_step_target - band, ':', 'Color', [0.4 0.4 0.4]);
end
xline(ax2, sim.fire_start_s, ':', '开始发射', 'Color', [0.8 0.3 0.1]);
xline(ax2, sim.fire_end_s, ':', '结束发射', 'Color', [0.8 0.3 0.1]);
grid(ax2, 'on');
xlabel(ax2, '时间 (s)');
ylabel(ax2, '速度 (rpm)');
legend(ax2, '目标速度', '实际速度', 'Location', 'best');
title(ax2, '稳速保持与发射扰动响应');

% 子图 3：控制输出和扰动估计
ax3 = nexttile(3);
yyaxis(ax3, 'left');
plot(ax3, sim.t, sim.u, 'm', 'LineWidth', 1.2);
ylabel(ax3, '控制输出');
yyaxis(ax3, 'right');
plot(ax3, sim.t, sim.real_disturbance, '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.4);
hold(ax3, 'on');
plot(ax3, sim.t, sim.z2, '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.2);
ylabel(ax3, '扰动量');
grid(ax3, 'on');
xlabel(ax3, '时间 (s)');
title(ax3, '控制输出与扰动估计');
legend(ax3, '控制输出 u', '真实扰动 d', 'ESO估计扰动 z2', 'Location', 'best');

% 子图 4：性能指标面板
ax4 = nexttile(4);
axis(ax4, [0 1 0 1]);
axis(ax4, 'off');

text_lines = {
    '结果怎么看'
    '---------------------------'
    ['目标转速 (rpm)    : ' num2str(metrics.target_speed_rpm, '%.1f')]
    ['识别 tau (s)      : ' num2str(estimate.tau, '%.5f')]
    ['推荐 b0           : ' num2str(tuned.b0, '%.2f')]
    ['控制带宽 wc       : ' num2str(tuned.wc, '%.2f')]
    ['观测器比例        : ' num2str(tuned.wo_ratio, '%.2f')]
    ' '
    '一、稳速表现'
    ['全程 RMSE (rpm)   : ' num2str(metrics.rmse, '%.2f')]
    ['最大误差 (rpm)    : ' num2str(metrics.max_abs_error, '%.2f')]
    ['末段平均速度 (rpm): ' num2str(metrics.final_mean_speed, '%.2f')]
    ['末段平均误差 (rpm): ' num2str(metrics.final_mean_error, '%.2f')]
    ['输出峰值 (A)      : ' num2str(metrics.control_peak, '%.3f')]
    ['输出 RMS (A)      : ' num2str(metrics.control_rms, '%.3f')]
    ['扰动估计 RMSE     : ' num2str(metrics.disturbance_rmse, '%.2f')]
    ' '
    '二、发射表现'
    ['最大发电掉速 (rpm): ' num2str(metrics.max_shot_droop_rpm, '%.2f')]
    ['平均单发掉速 (rpm): ' num2str(metrics.mean_shot_droop_rpm, '%.2f')]
    ['平均恢复时间      : ' fmt_value(metrics.mean_recovery_time_s, '%.4f s')]
    ' '
    '三、当前模型修正'
    ['接触时间放大      : ' num2str(plant.shot.contact_time_scale, '%.2f')]
    ['负载保守系数      : ' num2str(plant.shot.dynamic_load_factor, '%.2f')]
    ['供电下垂比例      : ' num2str(100.0 * plant.shot.supply_sag_ratio, '%.1f') '%']
    ['恢复时间常数 (ms) : ' num2str(1000.0 * plant.shot.supply_recovery_tau_s, '%.1f')]
    ' '
    '说明：本工况按“摩擦轮常开 + 保守发射模型”建模。'
    };

text(ax4, 0.02, 0.98, text_lines, ...
     'VerticalAlignment', 'top', ...
     'FontName', font_name, ...
     'FontSize', 11);

sgtitle('摩擦轮 ADRC 自动整定可视化结果', 'FontWeight', 'bold');

% 给整张图统一设置支持中文的字体，避免标题、图例、文本面板出现乱码或方框。
set(findall(gcf, '-property', 'FontName'), 'FontName', font_name);
end

function text_value = fmt_value(value, format_str)
% 把指标数值转换成字符串。
% 如果结果不可用，则显示 N/A。

if isnan(value) || isinf(value)
    text_value = 'N/A';
else
    text_value = num2str(value, format_str);
end
end

function font_name = choose_chinese_font()
% 选择一个本机存在、且通常支持中文显示的字体。
% 按常见 Windows / macOS / Linux 中文字体依次尝试。

persistent cached_font_name
if ~isempty(cached_font_name)
    font_name = cached_font_name;
    return;
end

candidate_fonts = { ...
    'Microsoft YaHei', ...
    '微软雅黑', ...
    'SimHei', ...
    '黑体', ...
    'SimSun', ...
    '宋体', ...
    'DengXian', ...
    '等线', ...
    'PingFang SC', ...
    'Noto Sans CJK SC', ...
    'Arial Unicode MS'};

available_fonts = listfonts;
font_name = get(groot, 'DefaultAxesFontName');

for i = 1:numel(candidate_fonts)
    if any(strcmpi(candidate_fonts{i}, available_fonts))
        font_name = candidate_fonts{i};
        break;
    end
end

cached_font_name = font_name;
end







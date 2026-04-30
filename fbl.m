function fbl()
    % fbl  机器人运动学仿真 GUI 界面
    
    close all;
    clear;
    clc;
    % 1. 机器人模型初始化 (DH 参数)
    function bot = init_robot()
        % 定义连杆参数: L = Link([Theta d a alpha])
        L(1) = Link([0     0      0      pi/2], 'standard');
        L(2) = Link([0     0      0.4318 0],    'standard');
        L(3) = Link([0     0.15005 0.0203 -pi/2], 'standard');
        L(4) = Link([0     0.4318 0      pi/2], 'standard');
        L(5) = Link([0     0      0      -pi/2],'standard');
        L(6) = Link([0     0      0      0],    'standard');
        
        bot = SerialLink(L, 'name', 'fbl-Robot');
    end

    robot = init_robot();
    current_q = [0 0 0 0 0 0]; % 当前关节角度
    target_q = [0 0 0 0 0 0];  % 目标关节角度
    % 2. 构建 GUI 界面
    hFig = figure('Name', 'fbl 机器人运动控制系统', ...
        'NumberTitle', 'off', 'Position', [100, 100, 1200, 750], ...
        'Color', [0.95 0.95 0.95]);

    % --- 机器人 3D 显示区 ---
    ax_robot = axes('Parent', hFig, 'Position', [0.05, 0.42, 0.45, 0.55]);
    % 预绘图提高响应速度
    axes(ax_robot);
    robot.plot(current_q, 'workspace', [-1 1 -1 1 -1 1]*1.5); 
    title('机器人实时姿态 (3D Workspace)');

    % --- 轨迹跟踪显示区 ---
    ax_pos = axes('Parent', hFig, 'Position', [0.55, 0.55, 0.4, 0.3]);
    grid on; hold on; title('关节位置跟踪 (Pos)');
    xlabel('Time (s)'); ylabel('Rad');

    ax_vel = axes('Parent', hFig, 'Position', [0.55, 0.06, 0.4, 0.3]);
    grid on; hold on; title('关节速度跟踪 (Vel)');
    xlabel('Time (s)'); ylabel('Rad/s');

    % --- 控制面板背景 ---
    uicontrol('Style', 'text', 'Position', [50, 30, 500, 280], 'BackgroundColor', [1 1 1]);

    % 模式选择
    uicontrol('Style', 'text', 'String', '运动模式选择:', 'Position', [60, 275, 100, 20], 'BackgroundColor', [1 1 1]);
    mode_selector = uicontrol('Style', 'popupmenu', 'String', {'Jog (点动)', 'Trap (梯形规划)', 'PT (五次多项式)'}, ...
        'Position', [160, 278, 120, 20], 'Callback', @change_mode);

    % 执行按钮
    btn_run = uicontrol('Style', 'pushbutton', 'String', '执行运动', 'Position', [300, 270, 100, 35], ...
        'BackgroundColor', [0.8 1 0.8], 'Callback', @execute_motion, 'Enable', 'off');

    % 回零按钮
    uicontrol('Style', 'pushbutton', 'String', '六轴回归零位', 'Position', [410, 270, 100, 35], ...
        'BackgroundColor', [1 0.9 0.8], 'Callback', @home_robot);

    % 速度/时间设置
    uicontrol('Style', 'text', 'String', '运行时间(s):', 'Position', [60, 240, 80, 20], 'BackgroundColor', [1 1 1]);
    edit_time = uicontrol('Style', 'edit', 'String', '2.0', 'Position', [145, 240, 50, 25]);

    % --- 6个关节控制行 ---
    sliders = gobjects(6,1);
    edit_boxes = gobjects(6,1);
    for i = 1:6
        y_p = 200 - (i-1)*32;
        uicontrol('Style', 'text', 'String', sprintf('Joint %d:', i), 'Position', [60, y_p, 50, 20], 'BackgroundColor', [1 1 1]);
        sliders(i) = uicontrol('Style', 'slider', 'Min', -pi, 'Max', pi, 'Value', 0, ...
            'Position', [120, y_p+5, 220, 15], 'Callback', {@slider_callback, i});
        edit_boxes(i) = uicontrol('Style', 'edit', 'String', '0.00', 'Position', [350, y_p, 60, 22], ...
            'Callback', {@edit_callback, i});
    end

    status_text = uicontrol('Style', 'text', 'String', '系统就绪 - I am fbl Robot Assistant', ...
        'Position', [40, 5, 500, 20], 'HorizontalAlignment', 'left', 'ForegroundColor', [0 0.4 0]);

    % 3. 核心功能函数

    function change_mode(src, ~)
        if src.Value == 1
            set(btn_run, 'Enable', 'off');
            set(status_text, 'String', '模式: Jog - 拖动滑块实时控制');
        else
            set(btn_run, 'Enable', 'on');
            set(status_text, 'String', '模式: 规划运动 - 设置目标后点击执行');
        end
    end

    function slider_callback(src, ~, idx)
        val = src.Value;
        set(edit_boxes(idx), 'String', sprintf('%.2f', val));
        target_q(idx) = val;
        if mode_selector.Value == 1
            current_q(idx) = val;
            update_robot_plot();
        end
    end

    function edit_callback(src, ~, idx)
        val = str2double(src.String);
        if isnan(val), val = 0; end
        val = max(min(val, pi), -pi);
        set(sliders(idx), 'Value', val);
        slider_callback(sliders(idx), [], idx);
    end

    function home_robot(~, ~)
        target_q = [0 0 0 0 0 0];
        for k=1:6
            set(sliders(k), 'Value', 0);
            set(edit_boxes(k), 'String', '0.00');
        end
        if mode_selector.Value == 1
            current_q = target_q;
            update_robot_plot();
        else
            set(status_text, 'String', '回零路径已就绪，点击执行');
        end
    end

    function execute_motion(~, ~)
        T = str2double(get(edit_time, 'String'));
        if isnan(T) || T <= 0, T = 2; end
        steps = 40; % 增加步数使动画更平滑
        t = linspace(0, T, steps);
        
        % 轨迹规划逻辑
        if mode_selector.Value == 2 % Trap (LSPB)
            q_traj = zeros(steps, 6); qd_traj = zeros(steps, 6);
            for k=1:6
                [q_traj(:,k), qd_traj(:,k)] = lspb(current_q(k), target_q(k), t);
            end
        else % PT (jtraj)
            [q_traj, qd_traj, ~] = jtraj(current_q, target_q, t);
        end
        
        % 清空绘图
        cla(ax_pos); cla(ax_vel);
        set(status_text, 'String', '正在执行多轴联动...');
        
        % 动画循环
        for k = 1:steps
            current_q = q_traj(k,:);
            update_robot_plot();
            
            % 绘图优化：一次绘制所有列
            plot(ax_pos, t(1:k), q_traj(1:k, :), 'LineWidth', 1.2);
            plot(ax_vel, t(1:k), qd_traj(1:k, :), 'LineWidth', 1.2);
            
            drawnow limitrate;
        end
        set(status_text, 'String', '运动任务已完成');
    end

    % 通用机器人绘图刷新函数，处理报错问题
    function update_robot_plot()
        axes(ax_robot);
        robot.plot(current_q); 
    end

end
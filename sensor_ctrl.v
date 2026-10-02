
module sensor_ctrl(
    input         clk,
    input         rst_n,
    // 传感器输入（0 = 白色/未检测；1 = 黑色/检测到）
    input  [3:0]  line_sensor,
    input  [1:0]  obs_sensor,
    // 控制输出
    output reg [1:0] dir_l,
    output reg [1:0] dir_r,
    output reg [6:0] duty_l,        // 0~100
    output reg [6:0] duty_r,        // 0~100
    output reg       buzzer
);

    // 方向编码
    localparam STOP = 2'b00;
    localparam FWD  = 2'b01;
    localparam REV  = 2'b10;

    // 速度档
    localparam [6:0] DUTY_HIGH   = 7'd50;   // 直行
    localparam [6:0] DUTY_MID    = 7'd50;   // 轻度转向
    localparam [6:0] DUTY_LOW    = 7'd50;   // 中度转向
    localparam [6:0] DUTY_SLOW   = 7'd50;   // 急转/避障
    localparam [6:0] DUTY_ZERO   = 7'd0;    // 停止

    // 刹车时间：约 50ms（50MHz 下 2,500,000 周期）
    localparam [24:0] BRAKE_TIME = 25'd2_500_000;

    // 避障序列计时
    localparam [31:0] OBS_STOP_TIME    = 32'd12_500_000;    // 约 250ms
    localparam [31:0] OBS_REV_TIME     = 32'd12_500_000;    // 约 250ms
    localparam [31:0] OBS_TURN_TIME    = 32'd40_000_000;    // 约 0.8s 掉头时间
    localparam [31:0] OBS_SEARCH_TIMEOUT = 32'd200_000_000; // 约 4s 搜索超时

    // 防抖参数
    localparam [3:0] DEBOUNCE_TOP = 4'd6;

    // 状态机状态
    localparam [3:0] ST_RUN       = 4'b0000;  // 正常运行
    localparam [3:0] ST_BRAKE     = 4'b0001;  // 循迹刹车
    localparam [3:0] ST_OBS_STOP  = 4'b0010;  // 避障-停止
    localparam [3:0] ST_OBS_REV   = 4'b0011;  // 避障-后退
    localparam [3:0] ST_OBS_TURN  = 4'b0100;  // 避障-掉头180度
    localparam [3:0] ST_OBS_SEARCH= 4'b0101;  // 避障-搜索赛道

    reg [3:0]   state;
    reg [24:0]  brake_timer;
    reg [31:0]  obs_timer;
    reg [3:0]   debounce_cnt;
    reg [3:0]   line_d1, line_d2, line_d3;
    reg [1:0]   obs_d1,  obs_d2,  obs_d3;

    wire [3:0] line_stable = (line_sensor == line_d1)
                           && (line_d1 == line_d2)
                           && (line_d2 == line_d3);
    wire [1:0] obs_stable  = (obs_sensor  == obs_d1)
                           && (obs_d1  == obs_d2)
                           && (obs_d2  == obs_d3);

    // 避障检测（0=检测到障碍）
    wire obs_detected = (obs_stable[1] && !obs_sensor[1]) ||
                        (obs_stable[0] && !obs_sensor[0]);

    // 赛道检测（任意传感器检测到黑线且稳定）
    wire line_found = (line_sensor != 4'b0000) && line_stable;

    // 冲出赛道时的方向记忆
    reg [1:0] last_offset_dir;
    reg [1:0] pending_dir_l, pending_dir_r;
    reg [6:0] pending_duty_l, pending_duty_r;

    // 判断是否需要转向（从直行变为转向）
    wire need_turn;
    wire is_straight = (dir_l == FWD) && (dir_r == FWD);
    wire will_turn   = !((pending_dir_l == FWD) && (pending_dir_r == FWD));
    assign need_turn = is_straight && will_turn;

    // 组合逻辑：计算下一步动作（仅循迹，避障由状态机处理）
    reg [1:0]  next_dir_l, next_dir_r;
    reg [6:0]  next_duty_l, next_duty_r;
    reg [1:0]  next_last_offset_dir;

    always @(*) begin
        // 默认：直行
        next_dir_l  = FWD;
        next_dir_r  = FWD;
        next_duty_l = DUTY_HIGH;
        next_duty_r = DUTY_HIGH;
        next_last_offset_dir = last_offset_dir;

        if (line_stable) begin
            case (line_sensor)
                // ---- 居中 → 直行 ----
                4'b0110, 4'b0010, 4'b0100: begin
                    next_dir_l  = FWD;
                    next_dir_r  = FWD;
                    next_duty_l = DUTY_HIGH;
                    next_duty_r = DUTY_HIGH;
                end

                // ---- 偏左 → 右转 ----
                4'b1000: begin
                    next_dir_l  = FWD;  next_duty_l = DUTY_HIGH;
                    next_dir_r  = REV;  next_duty_r = DUTY_SLOW;
                    next_last_offset_dir = 2'b01;
                end
                4'b1100: begin
                    next_dir_l  = FWD;  next_duty_l = DUTY_HIGH;
                    next_dir_r  = REV;  next_duty_r = DUTY_MID;
                    next_last_offset_dir = 2'b01;
                end
                4'b1110: begin
                    next_dir_l  = FWD;  next_duty_l = DUTY_HIGH;
                    next_dir_r  = REV;  next_duty_r = DUTY_HIGH;
                    next_last_offset_dir = 2'b01;
                end

                // ---- 偏右 → 左转 ----
                4'b0001: begin
                    next_dir_l  = REV;  next_duty_l = DUTY_SLOW;
                    next_dir_r  = FWD;  next_duty_r = DUTY_HIGH;
                    next_last_offset_dir = 2'b10;
                end
                4'b0011: begin
                    next_dir_l  = REV;  next_duty_l = DUTY_MID;
                    next_dir_r  = FWD;  next_duty_r = DUTY_HIGH;
                    next_last_offset_dir = 2'b10;
                end
                4'b0111: begin
                    next_dir_l  = REV;  next_duty_l = DUTY_HIGH;
                    next_dir_r  = FWD;  next_duty_r = DUTY_HIGH;
                    next_last_offset_dir = 2'b10;
                end

                // ---- 偏离（全白）→ 按最后偏移方向搜索 ----
                4'b0000: begin
                    if (last_offset_dir == 2'b01) begin
                        next_dir_l  = FWD;  next_duty_l = DUTY_HIGH;
                        next_dir_r  = REV;  next_duty_r = DUTY_HIGH;
                    end else if (last_offset_dir == 2'b10) begin
                        next_dir_l  = REV;  next_duty_l = DUTY_HIGH;
                        next_dir_r  = FWD;  next_duty_r = DUTY_HIGH;
                    end else begin
                        next_dir_l  = FWD;  next_duty_l = DUTY_HIGH;
                        next_dir_r  = FWD;  next_duty_r = DUTY_HIGH;
                    end
                    next_last_offset_dir = last_offset_dir;
                end

                default: ;
            endcase
        end
    end

    // 时序逻辑：状态机 + 防抖
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {line_d3, line_d2, line_d1} <= 12'h000;
            {obs_d3,  obs_d2,  obs_d1 } <= 6'h3;
            debounce_cnt <= 4'd0;
            state        <= ST_RUN;
            brake_timer  <= 25'd0;
            obs_timer    <= 32'd0;
            dir_l        <= FWD;
            dir_r        <= FWD;
            duty_l       <= DUTY_HIGH;
            duty_r       <= DUTY_HIGH;
            buzzer       <= 1'b0;
            last_offset_dir <= 2'b00;
            pending_dir_l  <= FWD;
            pending_dir_r  <= FWD;
            pending_duty_l <= DUTY_HIGH;
            pending_duty_r <= DUTY_HIGH;
        end else begin
            // 三级打拍
            {line_d3, line_d2, line_d1} <= {line_d2, line_d1, line_sensor};
            {obs_d3,  obs_d2,  obs_d1 } <= {obs_d2,  obs_d1,  obs_sensor };

            case (state)
                //--------------------------------------------------
                // ST_RUN: 正常运行
                //--------------------------------------------------
                ST_RUN: begin
                    if (obs_detected) begin
                        // 检测到障碍物 → 进入避障停止
                        state     <= ST_OBS_STOP;
                        obs_timer <= 32'd0;
                        dir_l     <= STOP;
                        dir_r     <= STOP;
                        duty_l    <= DUTY_ZERO;
                        duty_r    <= DUTY_ZERO;
                        buzzer    <= 1'b1;
                    end else if (debounce_cnt == DEBOUNCE_TOP) begin
                        debounce_cnt <= 4'd0;

                        pending_dir_l  <= next_dir_l;
                        pending_dir_r  <= next_dir_r;
                        pending_duty_l <= next_duty_l;
                        pending_duty_r <= next_duty_r;

                        if (need_turn) begin
                            state       <= ST_BRAKE;
                            brake_timer <= 25'd0;
                            dir_l       <= STOP;
                            dir_r       <= STOP;
                            duty_l      <= DUTY_ZERO;
                            duty_r      <= DUTY_ZERO;
                        end else begin
                            dir_l  <= next_dir_l;
                            dir_r  <= next_dir_r;
                            duty_l <= next_duty_l;
                            duty_r <= next_duty_r;
                        end

                        buzzer <= 1'b0;
                        last_offset_dir <= next_last_offset_dir;
                    end else begin
                        debounce_cnt <= debounce_cnt + 1'b1;
                    end
                end

                //--------------------------------------------------
                // ST_BRAKE: 循迹刹车
                //--------------------------------------------------
                ST_BRAKE: begin
                    if (brake_timer >= BRAKE_TIME - 1) begin
                        state       <= ST_RUN;
                        brake_timer <= 25'd0;
                        dir_l       <= pending_dir_l;
                        dir_r       <= pending_dir_r;
                        duty_l      <= pending_duty_l;
                        duty_r      <= pending_duty_r;
                    end else begin
                        brake_timer <= brake_timer + 1'b1;
                    end
                end

                //--------------------------------------------------
                // ST_OBS_STOP: 避障停止（蜂鸣器响）
                //--------------------------------------------------
                ST_OBS_STOP: begin
                    if (obs_timer >= OBS_STOP_TIME - 1) begin
                        // 停止结束 → 开始后退
                        state     <= ST_OBS_REV;
                        obs_timer <= 32'd0;
                        dir_l     <= REV;
                        dir_r     <= REV;
                        duty_l    <= DUTY_MID;
                        duty_r    <= DUTY_MID;
                    end else begin
                        obs_timer <= obs_timer + 1'b1;
                    end
                end

                //--------------------------------------------------
                // ST_OBS_REV: 避障后退（蜂鸣器响）
                //--------------------------------------------------
                ST_OBS_REV: begin
                    if (obs_timer >= OBS_REV_TIME - 1) begin
                        // 后退结束 → 只左转掉头（左正右反）
                        state     <= ST_OBS_TURN;
                        obs_timer <= 32'd0;
                        dir_l     <= FWD;  duty_l <= DUTY_HIGH;
                        dir_r     <= REV;  duty_r <= DUTY_HIGH;
                    end else begin
                        obs_timer <= obs_timer + 1'b1;
                    end
                end

                //--------------------------------------------------
                // ST_OBS_TURN: 避障掉头180度（蜂鸣器响）
                // 不检测赛道，持续转弯固定时间确保转够180度
                //--------------------------------------------------
                ST_OBS_TURN: begin
                    if (obs_timer >= OBS_TURN_TIME - 1) begin
                        // 掉头完成 → 搜索赛道
                        state     <= ST_OBS_SEARCH;
                        obs_timer <= 32'd0;
                    end else begin
                        obs_timer <= obs_timer + 1'b1;
                    end
                end

                //--------------------------------------------------
                // ST_OBS_SEARCH: 搜索赛道（蜂鸣器响）
                // 继续转弯直到检测到赛道
                //--------------------------------------------------
                ST_OBS_SEARCH: begin
                    if (line_found) begin
                        // 找到赛道 → 恢复正常运行
                        state     <= ST_RUN;
                        obs_timer <= 32'd0;
                        buzzer    <= 1'b0;
                    end else if (obs_timer >= OBS_SEARCH_TIMEOUT - 1) begin
                        // 超时 → 恢复正常运行（兜底）
                        state     <= ST_RUN;
                        obs_timer <= 32'd0;
                        buzzer    <= 1'b0;
                    end else begin
                        obs_timer <= obs_timer + 1'b1;
                    end
                end

                default: state <= ST_RUN;
            endcase
        end
    end

endmodule


module motor_ctrl(
    input         clk,
    input         rst_n,
    // 控制输入
    input  [1:0]  dir_l,
    input  [1:0]  dir_r,
    input  [6:0]  duty_l,         // 0~100
    input  [6:0]  duty_r,         // 0~100
    // L298N 输出
    output        IN1A, IN1B,
    output        IN2A, IN2B,
    output        IN3A, IN3B,
    output        IN4A, IN4B,
    output        PWMA, PWMB,
    output        PWMC, PWMD
);

    // 方向编码
    localparam STOP = 2'b00;
    localparam FWD  = 2'b01;
    localparam REV  = 2'b10;

    // PWM 参数: 50MHz / 50_000 = 1kHz, 分辨率 1us
    localparam PWM_TOP = 16'd49_999;

    // 内部：PWM 计数器（共用，简化时序）
    reg  [15:0] pwm_cnt;
    wire [15:0] duty_cmp_l = duty_l * (PWM_TOP / 8'd100); // 0~PWM_TOP
    wire [15:0] duty_cmp_r = duty_r * (PWM_TOP / 8'd100);
    wire        pwm_l_run  = (dir_l != STOP);
    wire        pwm_r_run  = (dir_r != STOP);
    wire        pwm_l_raw  = (pwm_cnt < duty_cmp_l) & pwm_l_run;
    wire        pwm_r_raw  = (pwm_cnt < duty_cmp_r) & pwm_r_run;

    // PWM 计数器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pwm_cnt <= 16'd0;
        else if (pwm_cnt == PWM_TOP)
            pwm_cnt <= 16'd0;
        else
            pwm_cnt <= pwm_cnt + 1'b1;
    end

    // 方向解码
    assign IN1A = (dir_l == FWD);
    assign IN1B = (dir_l == REV);
    assign IN2A = (dir_l == FWD);
    assign IN2B = (dir_l == REV);

    assign IN3A = (dir_r == FWD);
    assign IN3B = (dir_r == REV);
    assign IN4A = (dir_r == FWD);
    assign IN4B = (dir_r == REV);

    // PWM 输出
    assign PWMA = pwm_l_raw;
    assign PWMB = pwm_l_raw;
    assign PWMC = pwm_r_raw;
    assign PWMD = pwm_r_raw;

endmodule

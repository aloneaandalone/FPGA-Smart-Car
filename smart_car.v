
module smart_car(
    input         clk,           // 50MHz
    input         rst_n,         // 异步复位

    // ---- DHT11 温湿度传感器 ----
    inout         dht11,

    // ---- LCD1602 液晶显示 ----
    output        lcd_rs,
    output        lcd_rw,
    output        lcd_en,
    output [7:0]  lcd_data,

    // ---- TCRT5000 红外传感器（6 路） ----
    input  [3:0]  line_sensor,   // 循迹 [3:0] = 最左→最右
    input  [1:0]  obs_sensor,    // 避障 [1:0] = 左前,右前

    // ---- L298N 电机驱动（4 路） ----
    output        IN1A, IN1B,    // 左前
    output        IN2A, IN2B,    // 左后
    output        IN3A, IN3B,    // 右前
    output        IN4A, IN4B,    // 右后
    output        PWMA, PWMB,    // 左侧 PWM
    output        PWMC, PWMD,    // 右侧 PWM

    // ---- 蜂鸣器 ----
    output        buzzer
);

    // ----- 内部连线 -----
    wire [7:0]  hum_int, temp_int;
    wire [7:0]  hum_dec, temp_dec;
    wire        dht11_valid;
    reg         sample_en;
    wire [1:0]  dir_l, dir_r;
    wire [6:0]  duty_l, duty_r;
    wire        buzzer_trig;

    // ----- 1Hz 采样使能脉冲生成（50MHz / 50,000,000 = 1Hz） -----
    reg [25:0] cnt_1hz;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_1hz   <= 26'd0;
            sample_en <= 1'b0;
        end else if (cnt_1hz == 26'd49_999_999) begin
            cnt_1hz   <= 26'd0;
            sample_en <= 1'b1;
        end else begin
            cnt_1hz   <= cnt_1hz + 26'd1;
            sample_en <= 1'b0;
        end
    end

    // ----- 子模块例化 -----

    // DHT11 温湿度采集
    dht11_ctrl u_dht11 (
        .clk         (clk),
        .rst_n       (rst_n),
        .sample_en   (sample_en),
        .dht11_data  (dht11),
        .hum_int     (hum_int),
        .hum_dec     (hum_dec),
        .temp_int    (temp_int),
        .temp_dec    (temp_dec),
        .data_valid  (dht11_valid)
    );

    // LCD1602 液晶显示
    lcd1602_ctrl u_lcd (
        .clk         (clk),
        .rst_n       (rst_n),
        .temp_int    (temp_int),
        .hum_int     (hum_int),
        .data_valid  (dht11_valid),
        .lcd_rs      (lcd_rs),
        .lcd_rw      (lcd_rw),
        .lcd_en      (lcd_en),
        .lcd_data    (lcd_data)
    );

    // 传感器控制（循迹 + 避障决策 + 差速调速）
    sensor_ctrl u_sensor (
        .clk         (clk),
        .rst_n       (rst_n),
        .line_sensor (line_sensor),
        .obs_sensor  (obs_sensor),
        .dir_l       (dir_l),
        .dir_r       (dir_r),
        .duty_l      (duty_l),
        .duty_r      (duty_r),
        .buzzer      (buzzer_trig)
    );

    // 电机驱动（双路独立 PWM）
    motor_ctrl u_motor (
        .clk   (clk),
        .rst_n (rst_n),
        .dir_l (dir_l),
        .dir_r (dir_r),
        .duty_l(duty_l),
        .duty_r(duty_r),
        .IN1A  (IN1A),
        .IN1B  (IN1B),
        .IN2A  (IN2A),
        .IN2B  (IN2B),
        .IN3A  (IN3A),
        .IN3B  (IN3B),
        .IN4A  (IN4A),
        .IN4B  (IN4B),
        .PWMA  (PWMA),
        .PWMB  (PWMB),
        .PWMC  (PWMC),
        .PWMD  (PWMD)
    );

    // 蜂鸣器
    buzzer_ctrl u_buzzer (
        .clk    (clk),
        .rst_n  (rst_n),
        .trig   (buzzer_trig),
        .buzzer (buzzer)
    );

endmodule

// top.v
module top (
    input         clk,          // 50 MHz 时钟
    input         rst_n,        // 低有效复位
    input  [3:0]  line_sensor,  // 4 路循迹传感器
    input  [1:0]  obs_sensor,   // 2 路避障传感器

    // 电机控制输出（可直接接 L298N）
    output        IN1A, IN1B, IN2A, IN2B,
    output        IN3A, IN3B, IN4A, IN4B,
    output        PWMA, PWMB, PWMC, PWMD,
    output        buzzer
);

    wire [1:0] dir_l, dir_r;

    sensor_ctrl u_sensor (
        .clk        (clk),
        .rst_n      (rst_n),
        .line_sensor(line_sensor),
        .obs_sensor (obs_sensor),
        .dir_l      (dir_l),
        .dir_r      (dir_r),
        .buzzer     (buzzer)
    );

    motor_ctrl u_motor (
        .clk  (clk),
        .rst_n(rst_n),
        .dir_l(dir_l),
        .dir_r(dir_r),
        .IN1A (IN1A), .IN1B(IN1B),
        .IN2A (IN2A), .IN2B(IN2B),
        .IN3A (IN3A), .IN3B(IN3B),
        .IN4A (IN4A), .IN4B(IN4B),
        .PWMA (PWMA), .PWMB(PWMB),
        .PWMC (PWMC), .PWMD(PWMD)
    );

endmodule
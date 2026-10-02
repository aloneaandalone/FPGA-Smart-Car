
module dht11_ctrl(
    input  wire       clk,           // 50MHz系统时钟
    input  wire       rst_n,         // 异步复位，低有效
    input  wire       sample_en,     // 1Hz采样使能脉冲
    inout  wire       dht11_data,    // DHT11双向数据线
    output reg  [7:0] hum_int,       // 湿度整数
    output reg  [7:0] hum_dec,       // 湿度小数
    output reg  [7:0] temp_int,      // 温度整数
    output reg  [7:0] temp_dec,      // 温度小数
    output reg        data_valid     // 数据有效标志
);

    //==================================================
    // 状态机定义
    //==================================================
    localparam S_IDLE      = 4'd0;  // 空闲，等待采样使能
    localparam S_START     = 4'd1;  // 主机拉低>18ms（使用20ms）
    localparam S_RELEASE   = 4'd2;  // 主机拉高20-40us（使用30us）
    localparam S_RESP_WAIT = 4'd3;  // 等待DHT11拉低（响应开始）
    localparam S_RESP_LOW  = 4'd4;  // 等待DHT11响应低结束（80us低）
    localparam S_RESP_HIGH = 4'd5;  // 等待DHT11响应高结束（80us高）
    localparam S_BIT_LOW   = 4'd6;  // 等待位数据低电平结束（50us低）
    localparam S_BIT_HIGH  = 4'd7;  // 测量位数据高电平宽度，判断0/1
    localparam S_CHECK     = 4'd8;  // 校验和验证
    localparam S_DONE      = 4'd9;  // 完成，置data_valid

    //==================================================
    // 时序常量（50MHz时钟，20ns/周期）
    //==================================================
    localparam TIME_20MS   = 32'd1_000_000;  // 20ms = 1,000,000周期
    localparam TIME_30US   = 32'd1_500;      // 30us = 1,500周期
    localparam TIME_100US  = 32'd5_000;      // 100us = 5,000周期（超时用）
    localparam TIME_40US   = 32'd2_000;      // 40us = 2,000周期（0/1判别阈值）

    //==================================================
    // 内部寄存器
    //==================================================
    reg [3:0]  state;
    reg [31:0] timer;        // 通用定时器
    reg [5:0]  bit_cnt;      // 位计数器（0~39）
    reg [39:0] data_reg;     // 40位数据寄存器
    reg [31:0] high_timer;   // 高电平持续时间计数器

    //==================================================
    // 数据线方向控制
    // data_dir=0: 主机驱动（输出模式）
    // data_dir=1: 主机释放（输入模式，高阻态）
    //==================================================
    reg data_dir;            // 0=输出, 1=输入(高阻)
    reg data_out;            // 输出数据值

    assign dht11_data = data_dir ? 1'bz : data_out;

    //==================================================
    // 数据线同步（防亚稳态）
    //==================================================
    reg data_sync1;
    reg data_sync2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_sync1 <= 1'b1;
            data_sync2 <= 1'b1;
        end else begin
            data_sync1 <= dht11_data;
            data_sync2 <= data_sync1;
        end
    end

    // 使用同步后的信号进行判断
    wire data_in = data_sync2;

    //==================================================
    // 校验和计算辅助信号
    //==================================================
    wire [7:0] checksum_calc;
    assign checksum_calc = data_reg[39:32] + data_reg[31:24] +
                           data_reg[23:16] + data_reg[15:8];

    //==================================================
    // 主状态机
    //==================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            timer       <= 32'd0;
            bit_cnt     <= 6'd0;
            data_reg    <= 40'd0;
            high_timer  <= 32'd0;
            data_dir    <= 1'b1;     // 默认输入模式（释放总线）
            data_out    <= 1'b1;     // 默认高电平
            data_valid  <= 1'b0;
            hum_int     <= 8'd0;
            hum_dec     <= 8'd0;
            temp_int    <= 8'd0;
            temp_dec    <= 8'd0;
        end else begin
            case (state)
                //--------------------------------------------------
                // S_IDLE: 空闲等待
                //--------------------------------------------------
                S_IDLE: begin
                    data_dir   <= 1'b1;    // 释放总线
                    data_out   <= 1'b1;
                    data_valid <= 1'b0;    // 清除有效标志
                    timer      <= 32'd0;
                    bit_cnt    <= 6'd0;
                    if (sample_en) begin
                        state <= S_START;
                    end
                end

                //--------------------------------------------------
                // S_START: 主机拉低>18ms（20ms）
                //--------------------------------------------------
                S_START: begin
                    data_dir <= 1'b0;      // 输出模式
                    data_out <= 1'b0;      // 拉低
                    if (timer >= TIME_20MS - 1) begin
                        timer <= 32'd0;
                        state <= S_RELEASE;
                    end else begin
                        timer <= timer + 32'd1;
                    end
                end

                //--------------------------------------------------
                // S_RELEASE: 主机拉高20-40us（30us）
                //--------------------------------------------------
                S_RELEASE: begin
                    data_dir <= 1'b0;      // 输出模式
                    data_out <= 1'b1;      // 拉高
                    if (timer >= TIME_30US - 1) begin
                        timer    <= 32'd0;
                        data_dir <= 1'b1;  // 释放总线，切换为输入
                        data_out <= 1'b1;
                        state    <= S_RESP_WAIT;
                    end else begin
                        timer <= timer + 32'd1;
                    end
                end

                //--------------------------------------------------
                // S_RESP_WAIT: 等待DHT11拉低（响应开始）
                // 超时保护：100us内未响应则返回IDLE
                //--------------------------------------------------
                S_RESP_WAIT: begin
                    if (data_in == 1'b0) begin
                        timer <= 32'd0;
                        state <= S_RESP_LOW;
                    end else begin
                        if (timer >= TIME_100US) begin
                            // 超时，传感器未响应，返回空闲
                            state <= S_IDLE;
                        end else begin
                            timer <= timer + 32'd1;
                        end
                    end
                end

                //--------------------------------------------------
                // S_RESP_LOW: 等待DHT11响应低电平结束（80us低）
                // DHT11拉低约80us后释放为高
                //--------------------------------------------------
                S_RESP_LOW: begin
                    if (data_in == 1'b1) begin
                        timer <= 32'd0;
                        state <= S_RESP_HIGH;
                    end else begin
                        if (timer >= TIME_100US) begin
                            state <= S_IDLE;  // 超时返回
                        end else begin
                            timer <= timer + 32'd1;
                        end
                    end
                end

                //--------------------------------------------------
                // S_RESP_HIGH: 等待DHT11响应高电平结束（80us高）
                // DHT11拉高约80us后开始发送数据
                //--------------------------------------------------
                S_RESP_HIGH: begin
                    if (data_in == 1'b0) begin
                        timer    <= 32'd0;
                        bit_cnt  <= 6'd0;
                        state    <= S_BIT_LOW;
                    end else begin
                        if (timer >= TIME_100US) begin
                            state <= S_IDLE;  // 超时返回
                        end else begin
                            timer <= timer + 32'd1;
                        end
                    end
                end

                //--------------------------------------------------
                // S_BIT_LOW: 等待位数据低电平结束（50us低）
                // 每位数据以50us低电平开始
                //--------------------------------------------------
                S_BIT_LOW: begin
                    if (data_in == 1'b1) begin
                        // 低电平结束，高电平开始，开始计时
                        high_timer <= 32'd0;
                        timer      <= 32'd0;
                        state      <= S_BIT_HIGH;
                    end else begin
                        if (timer >= TIME_100US) begin
                            state <= S_IDLE;  // 超时返回
                        end else begin
                            timer <= timer + 32'd1;
                        end
                    end
                end

                //--------------------------------------------------
                // S_BIT_HIGH: 测量高电平持续时间，判断0或1
                // 高电平26-28us = 0，高电平70us = 1
                // 判别阈值：40us（2000周期）
                //--------------------------------------------------
                S_BIT_HIGH: begin
                    if (data_in == 1'b0) begin
                        // 高电平结束，判断位值
                        // high_timer < TIME_40US → 0, >= TIME_40US → 1
                        if (high_timer < TIME_40US) begin
                            // 当前位为0
                            data_reg <= {data_reg[38:0], 1'b0};
                        end else begin
                            // 当前位为1
                            data_reg <= {data_reg[38:0], 1'b1};
                        end
                        // 判断是否读完40位
                        if (bit_cnt >= 6'd39) begin
                            bit_cnt <= 6'd0;
                            state   <= S_CHECK;
                        end else begin
                            bit_cnt <= bit_cnt + 6'd1;
                            state   <= S_BIT_LOW;
                        end
                    end else begin
                        // 高电平仍在持续，继续计时
                        high_timer <= high_timer + 32'd1;
                        if (high_timer >= TIME_100US) begin
                            state <= S_IDLE;  // 超时返回
                        end
                    end
                end

                //--------------------------------------------------
                // S_CHECK: 校验和验证
                // 校验和 = 前4字节之和的低8位
                //--------------------------------------------------
                S_CHECK: begin
                    if (checksum_calc == data_reg[7:0]) begin
                        // 校验通过，更新输出数据
                        hum_int  <= data_reg[39:32];
                        hum_dec  <= data_reg[31:24];
                        temp_int <= data_reg[23:16];
                        temp_dec <= data_reg[15:8];
                        data_valid <= 1'b1;
                    end else begin
                        // 校验失败，不更新数据
                        data_valid <= 1'b0;
                    end
                    state <= S_DONE;
                end

                //--------------------------------------------------
                // S_DONE: 完成状态，返回空闲
                //--------------------------------------------------
                S_DONE: begin
                    data_dir <= 1'b1;    // 释放总线
                    data_out <= 1'b1;
                    state    <= S_IDLE;
                end

                //--------------------------------------------------
                // 默认：返回空闲
                //--------------------------------------------------
                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule

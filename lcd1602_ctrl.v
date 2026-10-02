
module lcd1602_ctrl(
    input  wire       clk,           // 50MHz系统时钟
    input  wire       rst_n,         // 异步复位，低有效
    input  wire [7:0] temp_int,      // 温度整数（来自DHT11）
    input  wire [7:0] hum_int,       // 湿度整数（来自DHT11）
    input  wire       data_valid,    // 数据有效标志
    output reg        lcd_rs,        // 寄存器选择
    output reg        lcd_rw,        // 读/写
    output reg        lcd_en,        // 使能
    output reg  [7:0] lcd_data       // 数据总线
);

    //==================================================
    // 主状态定义
    //==================================================
    localparam ST_INIT_WAIT   = 6'd0;   // 上电等待15ms
    localparam ST_INIT_FUNC   = 6'd1;   // 功能设置0x38
    localparam ST_INIT_DISP   = 6'd2;   // 显示开关0x0C
    localparam ST_INIT_CLEAR  = 6'd3;   // 清屏0x01
    localparam ST_INIT_ENTRY  = 6'd4;   // 输入方式0x06
    localparam ST_SET_L1_ADDR = 6'd5;   // 设置第1行地址0x80
    localparam ST_WRITE_L1    = 6'd6;   // 写第1行数据
    localparam ST_SET_L2_ADDR = 6'd7;   // 设置第2行地址0xC0
    localparam ST_WRITE_L2    = 6'd8;   // 写第2行数据
    localparam ST_REFRESH     = 6'd9;   // 刷新等待

    // 写操作子状态
    localparam ST_WR_SETUP    = 6'd10;  // 写准备：设置RS/RW/数据
    localparam ST_WR_EN_HI    = 6'd11;  // EN拉高
    localparam ST_WR_EN_LO    = 6'd12;  // EN拉低（锁存数据）
    localparam ST_WR_EXEC     = 6'd13;  // 等待命令执行完成

    //==================================================
    // 时序常量（50MHz时钟，20ns/周期）
    //==================================================
    localparam TIME_15MS  = 32'd750_000;    // 15ms上电等待
    localparam TIME_2MS   = 32'd100_000;    // 2ms清屏等待
    localparam TIME_50US  = 32'd2_500;      // 50us普通命令执行
    localparam TIME_1US   = 32'd50;         // 1us EN脉冲/建立时间
    localparam TIME_500MS = 32'd25_000_000; // 500ms刷新间隔

    //==================================================
    // 内部寄存器
    //==================================================
    reg [5:0]  state;
    reg [5:0]  ret_state;       // 写操作完成后返回的状态
    reg [31:0] timer;
    reg [7:0]  wr_data;         // 当前写入的数据/命令
    reg        wr_rs;           // 当前写入的RS值
    reg [31:0] wr_exec_time;    // 命令执行等待时间
    reg [4:0]  char_idx;        // 字符索引（0~15）

    //==================================================
    // BCD转换：限制最大显示值为99
    //==================================================
    wire [7:0] temp_disp = (temp_int > 8'd99) ? 8'd99 : temp_int;
    wire [7:0] hum_disp  = (hum_int  > 8'd99) ? 8'd99 : hum_int;

    //==================================================
    // 第1行固定显示内容："Smart Car V1.0  "
    //==================================================
    function [7:0] get_line1_char;
        input [4:0] idx;
        begin
            case (idx)
5'd0:  get_line1_char = "S";
5'd1:  get_line1_char = "m";
5'd2:  get_line1_char = "a";
5'd3:  get_line1_char = "r";
5'd4:  get_line1_char = "t";
5'd5:  get_line1_char = " ";
5'd6:  get_line1_char = "C";
5'd7:  get_line1_char = "a";
5'd8:  get_line1_char = "r";
5'd9:  get_line1_char = " ";
5'd10: get_line1_char = "V";
5'd11: get_line1_char = "1";
5'd12: get_line1_char = ".";
5'd13: get_line1_char = "0";
5'd14: get_line1_char = " ";
5'd15: get_line1_char = " ";
default: get_line1_char = " ";
            endcase
        end
    endfunction

    //==================================================
    // 第2行动态显示内容："T:XXC H:XX%     "
    // T=54 :=3A 数字 C=43 空格=20 H=48 :=3A 数字 %=25
    //==================================================
    function [7:0] get_line2_char;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  get_line2_char = 8'h54;  // 'T'
                5'd1:  get_line2_char = 8'h3A;  // ':'
                5'd2:  get_line2_char = 8'h30 + (temp_disp / 8'd10);  // 温度十位
                5'd3:  get_line2_char = 8'h30 + (temp_disp % 8'd10);  // 温度个位
                5'd4:  get_line2_char = 8'h43;  // 'C'
                5'd5:  get_line2_char = 8'h20;  // ' '
                5'd6:  get_line2_char = 8'h48;  // 'H'
                5'd7:  get_line2_char = 8'h3A;  // ':'
                5'd8:  get_line2_char = 8'h30 + (hum_disp  / 8'd10);  // 湿度十位
                5'd9:  get_line2_char = 8'h30 + (hum_disp  % 8'd10);  // 湿度个位
                5'd10: get_line2_char = 8'h25;  // '%'
                5'd11: get_line2_char = 8'h20;  // ' '
                5'd12: get_line2_char = 8'h20;  // ' '
                5'd13: get_line2_char = 8'h20;  // ' '
                5'd14: get_line2_char = 8'h20;  // ' '
                5'd15: get_line2_char = 8'h20;  // ' '
                default: get_line2_char = 8'h20; // ' '
            endcase
        end
    endfunction

    //==================================================
    // 主状态机
    //==================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_INIT_WAIT;
            ret_state    <= ST_INIT_WAIT;
            timer        <= 32'd0;
            wr_data      <= 8'd0;
            wr_rs        <= 1'b0;
            wr_exec_time <= TIME_50US;
            char_idx     <= 5'd0;
            lcd_rs       <= 1'b0;
            lcd_rw       <= 1'b0;
            lcd_en       <= 1'b0;
            lcd_data     <= 8'h00;
        end else begin
            case (state)

                //--------------------------------------------------
                // ST_INIT_WAIT: 上电等待15ms
                //--------------------------------------------------
                ST_INIT_WAIT: begin
                    lcd_en <= 1'b0;
                    if (timer >= TIME_15MS - 1) begin
                        timer <= 32'd0;
                        // 准备写功能设置命令0x38
                        wr_data      <= 8'h38;
                        wr_rs        <= 1'b0;   // 命令
                        wr_exec_time <= TIME_50US;
                        ret_state    <= ST_INIT_DISP;
                        state        <= ST_WR_SETUP;
                    end else begin
                        timer <= timer + 32'd1;
                    end
                end

                //--------------------------------------------------
                // ST_INIT_FUNC: 功能设置0x38（已合并到INIT_WAIT中）
                //--------------------------------------------------
                ST_INIT_FUNC: begin
                    wr_data      <= 8'h38;
                    wr_rs        <= 1'b0;
                    wr_exec_time <= TIME_50US;
                    ret_state    <= ST_INIT_DISP;
                    state        <= ST_WR_SETUP;
                end

                //--------------------------------------------------
                // ST_INIT_DISP: 显示开关0x0C（开显示，关光标）
                //--------------------------------------------------
                ST_INIT_DISP: begin
                    wr_data      <= 8'h0C;
                    wr_rs        <= 1'b0;
                    wr_exec_time <= TIME_50US;
                    ret_state    <= ST_INIT_CLEAR;
                    state        <= ST_WR_SETUP;
                end

                //--------------------------------------------------
                // ST_INIT_CLEAR: 清屏0x01（需要较长执行时间2ms）
                //--------------------------------------------------
                ST_INIT_CLEAR: begin
                    wr_data      <= 8'h01;
                    wr_rs        <= 1'b0;
                    wr_exec_time <= TIME_2MS;
                    ret_state    <= ST_INIT_ENTRY;
                    state        <= ST_WR_SETUP;
                end

                //--------------------------------------------------
                // ST_INIT_ENTRY: 输入方式0x06（光标右移）
                //--------------------------------------------------
                ST_INIT_ENTRY: begin
                    wr_data      <= 8'h06;
                    wr_rs        <= 1'b0;
                    wr_exec_time <= TIME_50US;
                    ret_state    <= ST_SET_L1_ADDR;
                    state        <= ST_WR_SETUP;
                end

                //--------------------------------------------------
                // ST_SET_L1_ADDR: 设置第1行DDRAM地址0x80
                //--------------------------------------------------
                ST_SET_L1_ADDR: begin
                    wr_data      <= 8'h80;
                    wr_rs        <= 1'b0;
                    wr_exec_time <= TIME_50US;
                    ret_state    <= ST_WRITE_L1;
                    char_idx     <= 5'd0;
                    state        <= ST_WR_SETUP;
                end

                //--------------------------------------------------
                // ST_WRITE_L1: 写第1行数据（16个字符）
                //--------------------------------------------------
                ST_WRITE_L1: begin
                    wr_data      <= get_line1_char(char_idx);
                    wr_rs        <= 1'b1;   // 数据
                    wr_exec_time <= TIME_50US;
                    if (char_idx >= 5'd15) begin
                        ret_state <= ST_SET_L2_ADDR;
                    end else begin
                        ret_state <= ST_WRITE_L1;
                    end
                    char_idx <= char_idx + 5'd1;  // 在此递增索引
                    state <= ST_WR_SETUP;
                end

                //--------------------------------------------------
                // ST_SET_L2_ADDR: 设置第2行DDRAM地址0xC0
                //--------------------------------------------------
                ST_SET_L2_ADDR: begin
                    wr_data      <= 8'hC0;
                    wr_rs        <= 1'b0;
                    wr_exec_time <= TIME_50US;
                    ret_state    <= ST_WRITE_L2;
                    char_idx     <= 5'd0;
                    state        <= ST_WR_SETUP;
                end

                //--------------------------------------------------
                // ST_WRITE_L2: 写第2行数据（16个字符）
                //--------------------------------------------------
                ST_WRITE_L2: begin
                    wr_data      <= get_line2_char(char_idx);
                    wr_rs        <= 1'b1;   // 数据
                    wr_exec_time <= TIME_50US;
                    if (char_idx >= 5'd15) begin
                        ret_state <= ST_REFRESH;
                    end else begin
                        ret_state <= ST_WRITE_L2;
                    end
                    char_idx <= char_idx + 5'd1;  // 在此递增索引
                    state <= ST_WR_SETUP;
                end

                //--------------------------------------------------
                // ST_REFRESH: 刷新等待，500ms后重新更新显示
                //--------------------------------------------------
                ST_REFRESH: begin
                    lcd_en <= 1'b0;
                    if (timer >= TIME_500MS - 1) begin
                        timer     <= 32'd0;
                        state     <= ST_SET_L1_ADDR;
                        char_idx  <= 5'd0;
                    end else begin
                        timer <= timer + 32'd1;
                    end
                end

                //--------------------------------------------------
                // ST_WR_SETUP: 写操作准备阶段
                // 设置RS、RW=0、数据总线，等待1us建立时间
                //--------------------------------------------------
                ST_WR_SETUP: begin
                    lcd_rs   <= wr_rs;
                    lcd_rw   <= 1'b0;       // 写模式
                    lcd_data <= wr_data;
                    lcd_en   <= 1'b0;
                    if (timer >= TIME_1US - 1) begin
                        timer <= 32'd0;
                        state <= ST_WR_EN_HI;
                    end else begin
                        timer <= timer + 32'd1;
                    end
                end

                //--------------------------------------------------
                // ST_WR_EN_HI: EN拉高，保持1us
                //--------------------------------------------------
                ST_WR_EN_HI: begin
                    lcd_en <= 1'b1;
                    if (timer >= TIME_1US - 1) begin
                        timer <= 32'd0;
                        state <= ST_WR_EN_LO;
                    end else begin
                        timer <= timer + 32'd1;
                    end
                end

                //--------------------------------------------------
                // ST_WR_EN_LO: EN拉低（下降沿锁存数据），保持1us
                //--------------------------------------------------
                ST_WR_EN_LO: begin
                    lcd_en <= 1'b0;
                    if (timer >= TIME_1US - 1) begin
                        timer <= 32'd0;
                        state <= ST_WR_EXEC;
                    end else begin
                        timer <= timer + 32'd1;
                    end
                end

                //--------------------------------------------------
                // ST_WR_EXEC: 等待命令执行完成
                //--------------------------------------------------
                ST_WR_EXEC: begin
                    if (timer >= wr_exec_time - 1) begin
                        timer <= 32'd0;
                        state <= ret_state;
                    end else begin
                        timer <= timer + 32'd1;
                    end
                end

                //--------------------------------------------------
                // 默认：回到初始化等待
                //--------------------------------------------------
                default: begin
                    state <= ST_INIT_WAIT;
                end
            endcase
        end
    end

endmodule

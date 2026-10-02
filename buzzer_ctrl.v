// 有源蜂鸣器驱动模块
// 触发后至少响 200ms，避免短脉冲导致听不到
module buzzer_ctrl(
    input  clk,
    input  rst_n,
    input  trig,           // 触发信号（高有效）
    output buzzer
);

    // 200ms @ 50MHz = 10_000_000
    localparam HOLD_TIME = 24'd9_999_999;

    reg [23:0] cnt;
    reg        sounding;

    assign buzzer = sounding;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sounding <= 0;
            cnt      <= 0;
        end else begin
            if (trig) begin
                sounding <= 1;
                cnt      <= 0;
            end else if (sounding) begin
                if (cnt == HOLD_TIME) begin
                    sounding <= 0;
                end else begin
                    cnt <= cnt + 1'b1;
                end
            end
        end
    end

endmodule

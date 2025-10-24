module bitrev (
    input  sck,
    input  ss,
    input  mosi,
    output miso
);

wire sel;

// 内部寄存器的定义.
reg [3:0] count;        // SCK 时钟周期计数器 (0~15).
reg [7:0] shift_reg_rx; // 数据接收移位寄存器.
reg [7:0] rev_data;     // 反转后的数据, 用于输出.

/*
指定寄存器的初值.
(因为 bitrev 模块最后不会引入硬件电路, 所以可以用不可综合语法)
 */
initial begin
    count = 0;
    shift_reg_rx = 0;
    rev_data = 0;
end

// 使用时钟下降沿触发.
always @(negedge sck) begin
    if (ss) begin
        // SS 处于高电平. 表明该模块没有被选中.
        // 那就重置寄存器吧. (｡・ω・｡)
        count <= 0;
        shift_reg_rx <= 0;
        rev_data <= 0;
    end
    else begin
        // SS 处于低电平. 该模块被选中.

        // 计数器逻辑 (0~15之间循环).
        if (count < 4'd15) begin
            count <= count + 4'd1;
        end
        else begin
            count <= 4'd0;
        end

        // 前 8 个周期接收来自 master 的数据.
        if (count < 4'd8) begin
            shift_reg_rx <= {shift_reg_rx[6:0], mosi};
        end
        
        // 第 7 个周期结束时完成数据的接收. 生成反转数据.
        if (count == 4'd7) begin
            rev_data <= {shift_reg_rx[6:0], mosi};
        end
    end
end

assign sel = ~ss;
// 指定输出信号 MISO.
assign miso =
    // 第 8 个周期之前: 尚未未完成数据接收, 输出默认的高电平.
    (sel && count < 4'd8) ? 1'b1 :
    // 第 8 个周期及之后: 输出反转数据.
    (sel && count >= 4'd8) ? rev_data[count - 8] :
    // 其他情况: 输出默认的高电平.
    1'b1;

endmodule

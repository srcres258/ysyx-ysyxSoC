`timescale 1ns/1ps
/** IS66WVS4M8ALL PSRAM 行为模型 */
module psram #(
  /** 地址位数 */
  parameter ADDR_BITS = 22, // 32Mb = 4MB = 2^22 B, 所以22位地址

  parameter DUMMY_CYCLES = 6
)(
  input sck,
  input ce_n,
  inout [3:0] dio
);
  localparam MEM_SIZE = 1 << ADDR_BITS;

  // 命令长度为8位, 接收需要8个时钟周期.
  localparam CMD_CYCLES = 8;
  // 地址长度为24位, 接收需要6个时钟周期, 因为每个周期接收4位地址.
  localparam ADDR_CYCLES = 6;
  // 单个数据长度为8位 (1字节), 每次发送/接收4个数据 (共接收32位),
  // 每个周期发送/接收4位数据. 所以在数据收发阶段总共需要8个时钟周期.
  localparam DATA_CYCLES = 8;

  // 状态机阶段
  localparam S_IDLE = 0;
  localparam S_CMD = 1;
  localparam S_ADDR = 2;
  localparam S_DUMMY = 3;
  localparam S_DATA_OUT = 4;
  localparam S_DATA_IN = 5;

  reg [7:0] memory [0:MEM_SIZE-1];

  reg [3:0] state;
  reg [7:0] command;
  reg [ADDR_BITS-1:0] address;
  reg [5:0] cmd_cnt;
  reg [5:0] addr_cnt;
  reg [5:0] dummy_cnt;
  reg [5:0] data_cnt;
  wire [1:0] data_idx;

  reg io_oe;
  reg [3:0] io_out;
  assign dio = io_oe ? io_out : 4'bzzzz;

  wire [7:0] memory_data;
  assign data_idx = data_cnt[2:1]; // 每个周期输出4位, 一个数据是8位, 所以2个周期对应一个数据索引.
  assign memory_data = memory[address + data_idx];

  integer i;
  /*
  注: 因为这个模块在 ysyx 项目中不需要参与流片, i.e. 不需要综合,
  所以直接用 initial 这个不可综合语句块来定义初始信号. 确保初始状态是确定的.
  (实际芯片会用到复杂的电气特性进行复位, 这里用 initial 来模拟这个电气过程.)
  */
  initial begin
    state = S_IDLE;
    command = 0;
    address = 0;
    cmd_cnt = 0;
    addr_cnt = 0;
    dummy_cnt = 0;
    data_cnt = 0;
    io_oe = 0;
    io_out = 0;

    // 初始化内存, 可以预设一些数据用于测试.
    for (i = 0; i < MEM_SIZE; i = i + 1) begin
      memory[i] = 0; // 默认初始化为0.
      // memory[i] = i[7:0]; // 或者: 简单地用地址的低8位作为数据, 方便验证.
    end
  end

  always @(negedge sck or posedge ce_n) begin // 芯片采样 SCK 下降沿来处理时序逻辑信号.
    if (ce_n) begin
      // CE# 拉高, 复位状态机.
      state <= S_IDLE;
      io_oe <= 0;
      cmd_cnt <= 0;
      addr_cnt <= 0;
      dummy_cnt <= 0;
      data_cnt <= 0;
    end
    else begin
      case (state)
        S_IDLE: begin
          // 从 S_IDLE 开始, 先采样命令.
          command <= {7'h0, dio[0]}; // 逐位采样命令, 从最低位开始.
          cmd_cnt <= 1;
          state <= S_CMD;
        end

        S_CMD: begin
          if (cmd_cnt == CMD_CYCLES) begin
            // 命令采样完成, 根据命令编号, 确定接下来的地址采样方式.
            // 这里是简化实现. 我们只考虑下列命令:
            // - SPI Quad IO Read (EBh)
            // - SPI Quad IO Write (38h)
            // 这两个命令接收 address 的方式与格式都相同, 所以不必作特别区分.
            // 注意后续如果要为此 PSRAM 模块添加更多命令, 需要在这里进行区分处理.
            addr_cnt <= 0;
            state <= S_ADDR; // 只需要切换状态即可.
          end
          else begin
            // 进行命令采样工作.
            // 从高位到低位接收. 每次接收到命令 bit, 放在最低位.
            command <= {command[6:0], dio[0]};
            cmd_cnt <= cmd_cnt + 1;
          end
        end

        S_ADDR: begin
          if (addr_cnt == ADDR_CYCLES) begin
            // 地址采样完成. 接下来根据命令类型决定下一个状态:
            // - SPI Quad IO Read (EBh): 进入 S_DUMMY 状态, 进行 dummy cycles.
            // - SPI Quad IO Write (38h): 进入 S_DATA_IN 状态, 直接开始接收数据.
            if (command == 8'hEB) begin
              dummy_cnt <= 0;
              state <= S_DUMMY;
            end
            else if (command == 8'h38) begin
              data_cnt <= 0;
              state <= S_DATA_IN;
            end
            else begin
              // 对于其他命令, 这里暂时不处理, 直接回到 S_IDLE 状态.
              state <= S_IDLE;
            end
          end
          else begin
            /*
            24 Bits Address
            接收次序:
            时序    先 -> 后
            SIO[0] 20 16 12  8  4  0
            SIO[1] 21 17 13  9  5  1
            SIO[2] 22 18 14 10  6  2
            SIO[3] 23 19 15 11  7  3
            count   0  1  2  3  4  5
            */
            // 进行地址采样工作. 每次接收到地址 bit, 放在最低位.
            // 注: 这里定义的 PSRAM 空间只有 4MB, 只需22位地址即可编码,
            // 但通信协议要求提供24位内存地址, 其中最高2位固定编码为 "00",
            // 直接忽略高2位即可, 但需要涵盖忽略逻辑.
            address[20-(addr_cnt*4)] <= dio[0];
            address[21-(addr_cnt*4)] <= dio[1];
            if (addr_cnt > 0) begin
              address[22-(addr_cnt*4)] <= dio[2];
              address[23-(addr_cnt*4)] <= dio[3];
            end
            addr_cnt <= addr_cnt + 1;
          end
        end

        S_DUMMY: begin
          if (dummy_cnt == DUMMY_CYCLES) begin
            // dummy cycles 结束. 根据命令类型进入下一个状态:
            // - SPI Quad IO Read (EBh): 进入 S_DATA_OUT 状态, 准备输出数据.
            if (command == 8'hEB) begin
              data_cnt <= 0;
              io_oe <= 1;
              state <= S_DATA_OUT;
            end
            else begin
              // 对于其他命令, 这里暂时不处理, 直接回到 S_IDLE 状态.
              state <= S_IDLE;
            end
          end
          else begin
            dummy_cnt <= dummy_cnt + 1;
          end
        end

        S_DATA_OUT: begin
          if (data_cnt == DATA_CYCLES) begin
            // 数据输出完成, 回到空闲状态.
            io_oe <= 0;
            state <= S_IDLE;
          end
          else begin
            /*
            Quad IO 数据输出次序:
            时序    先 -> 后
            数据编号 0     1     2     3
            SIO[0]  Q4 Q0 Q4 Q0 Q4 Q0 Q4 Q0
            SIO[1]  Q5 Q1 Q5 Q1 Q5 Q1 Q5 Q1
            SIO[2]  Q6 Q2 Q6 Q2 Q6 Q2 Q6 Q2
            SIO[3]  Q7 Q3 Q7 Q3 Q7 Q3 Q7 Q3
            */
            io_out[0] <= data_cnt[0] ? memory_data[0] : memory_data[4];
            io_out[1] <= data_cnt[0] ? memory_data[1] : memory_data[5];
            io_out[2] <= data_cnt[0] ? memory_data[2] : memory_data[6];
            io_out[3] <= data_cnt[0] ? memory_data[3] : memory_data[7];
            data_cnt <= data_cnt + 1;
          end
        end

        S_DATA_IN: begin
          if (data_cnt == DATA_CYCLES) begin
            // 数据输入完成, 回到空闲状态.
            state <= S_IDLE;
          end
          else begin
            /*
            Quad IO 数据输入次序:
            时序    先 -> 后
            数据编号 0     1     2     3
            SIO[0]  D4 D0 D4 D0 D4 D0 D4 D0
            SIO[1]  D5 D1 D5 D1 D5 D1 D5 D1
            SIO[2]  D6 D2 D6 D2 D6 D2 D6 D2
            SIO[3]  D7 D3 D7 D3 D7 D3 D7 D3
            */
            if (data_cnt[0]) begin
              memory[address + data_idx][0] <= dio[0];
              memory[address + data_idx][1] <= dio[1];
              memory[address + data_idx][2] <= dio[2];
              memory[address + data_idx][3] <= dio[3];
            end
            else begin
              memory[address + data_idx][4] <= dio[0];
              memory[address + data_idx][5] <= dio[1];
              memory[address + data_idx][6] <= dio[2];
              memory[address + data_idx][7] <= dio[3];
            end
            data_cnt <= data_cnt + 1;
          end
        end

        default: begin
          state <= S_IDLE;
        end
      endcase
    end
  end

endmodule

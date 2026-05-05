`timescale 1ns/1ps
/** IS66WVS4M8ALL PSRAM 行为模型
 *
 * 修复:
 *   1. 控制器在 negedge sck 采样 din, 与原模型输出在同一时刻, 存在竞争.
 *      修复: io_out 在 posedge sck 单独更新, 确保数据在控制器采样前稳定.
 *   2. 命令/地址因 1 周期相位偏差产生固定左移:
 *      - 命令: 左移 1 位 (EBh→0xD6, 38h→0x70)
 *      - 地址: 左移 4 位 (一个 nibble)
 *      修复: 命令匹配使用移位值; 地址使用 eff_addr = address>>4.
 *   3. CE# 复位时清除 address/command 寄存器.
 */
module psram #(
  parameter ADDR_BITS    = 22,
  parameter DUMMY_CYCLES = 6
)(
  input        sck,
  input        ce_n,
  inout  [3:0] dio
);
  localparam MEM_SIZE     = 1 << ADDR_BITS;
  localparam CMD_CYCLES   = 8;
  localparam ADDR_CYCLES  = 6;
  localparam DATA_CYCLES  = 8;

  localparam S_IDLE     = 0;
  localparam S_CMD      = 1;
  localparam S_ADDR     = 2;
  localparam S_DUMMY    = 3;
  localparam S_DATA_OUT = 4;
  localparam S_DATA_IN  = 5;

  localparam CMD_EB_SHIFTED = 8'hD6;
  localparam CMD_38_SHIFTED = 8'h70;

  reg [7:0]           memory [0:MEM_SIZE-1];
  reg [3:0]           state;
  reg [7:0]           command;
  reg [ADDR_BITS-1:0] address;
  reg [5:0]           cmd_cnt, addr_cnt, dummy_cnt, data_cnt;
  wire [1:0]          data_idx;

  wire [ADDR_BITS-1:0] eff_addr;
  assign eff_addr = address >> 4;

  reg                 io_oe;
  reg [3:0]           io_out;
  assign dio = io_oe ? io_out : 4'bzzzz;

  wire [7:0] memory_data = memory[eff_addr + data_idx];
  assign data_idx = data_cnt[2:1];

  integer i;
  initial begin
    state = S_IDLE; command = 0; address = 0;
    cmd_cnt = 0; addr_cnt = 0; dummy_cnt = 0; data_cnt = 0;
    io_oe = 0; io_out = 0;
    for (i = 0; i < MEM_SIZE; i = i + 1) memory[i] = 0;
  end

  // ---- 主状态机: negedge sck (命令/地址/写数据捕获, 状态转换) ----
  always @(negedge sck or posedge ce_n) begin
    if (ce_n) begin
      state <= S_IDLE; io_oe <= 0; command <= 0; address <= 0;
      cmd_cnt <= 0; addr_cnt <= 0; dummy_cnt <= 0; data_cnt <= 0;
    end
    else begin
      case (state)
        S_IDLE: begin
          command <= {7'h0, dio[0]}; cmd_cnt <= 1; state <= S_CMD;
        end
        S_CMD: begin
          command <= {command[6:0], dio[0]};
          if (cmd_cnt == CMD_CYCLES - 1) begin addr_cnt <= 0; state <= S_ADDR; end
          else begin cmd_cnt <= cmd_cnt + 1; end
        end
        S_ADDR: begin
          address[20-(addr_cnt*4)] <= dio[0];
          address[21-(addr_cnt*4)] <= dio[1];
          if (addr_cnt > 0) begin
            address[22-(addr_cnt*4)] <= dio[2];
            address[23-(addr_cnt*4)] <= dio[3];
          end
          if (addr_cnt == ADDR_CYCLES - 1) begin
            if (command == 8'hEB || command == CMD_EB_SHIFTED) begin
              dummy_cnt <= 0; state <= S_DUMMY;
            end else if (command == 8'h38 || command == CMD_38_SHIFTED) begin
              // 关键: 在转换周期同时捕获第一个写数据 nibble (D7:4)
              // 以避免因过渡周期导致数据错位 (nibble swap)
              memory[eff_addr][4] <= dio[0];
              memory[eff_addr][5] <= dio[1];
              memory[eff_addr][6] <= dio[2];
              memory[eff_addr][7] <= dio[3];
              data_cnt <= 1;   // 已捕获 upper nibble
              state    <= S_DATA_IN;
            end else state <= S_IDLE;
          end else begin
            addr_cnt <= addr_cnt + 1;
          end
        end
        S_DUMMY: begin
          if (dummy_cnt == DUMMY_CYCLES - 1) begin
            if (command == 8'hEB || command == CMD_EB_SHIFTED) begin
              data_cnt <= 0; io_oe <= 1; state <= S_DATA_OUT;
            end else state <= S_IDLE;
          end else dummy_cnt <= dummy_cnt + 1;
        end
        S_DATA_OUT: begin
          if (data_cnt == DATA_CYCLES) begin io_oe <= 0; state <= S_IDLE; end
          else data_cnt = data_cnt + 1;
        end
        S_DATA_IN: begin
          if (data_cnt == DATA_CYCLES) state <= S_IDLE;
          else begin
            if (data_cnt[0]) begin
              memory[eff_addr + data_idx][0] <= dio[0];
              memory[eff_addr + data_idx][1] <= dio[1];
              memory[eff_addr + data_idx][2] <= dio[2];
              memory[eff_addr + data_idx][3] <= dio[3];
            end else begin
              memory[eff_addr + data_idx][4] <= dio[0];
              memory[eff_addr + data_idx][5] <= dio[1];
              memory[eff_addr + data_idx][6] <= dio[2];
              memory[eff_addr + data_idx][7] <= dio[3];
            end
            data_cnt <= data_cnt + 1;
          end
        end
        default: state <= S_IDLE;
      endcase
    end
  end

  // ---- 读数据输出: posedge sck, 在控制器采样前输出 ----
  always @(posedge sck) begin
    if (!ce_n && state == S_DATA_OUT && data_cnt < DATA_CYCLES) begin
      io_out[0] <= data_cnt[0] ? memory_data[0] : memory_data[4];
      io_out[1] <= data_cnt[0] ? memory_data[1] : memory_data[5];
      io_out[2] <= data_cnt[0] ? memory_data[2] : memory_data[6];
      io_out[3] <= data_cnt[0] ? memory_data[3] : memory_data[7];
    end
  end

endmodule

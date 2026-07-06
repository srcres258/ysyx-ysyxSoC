`timescale 1ns/1ps

/**
 * IS66WVS4M8ALL PSRAM 行为模型 — 全面测试平台 (iverilog 兼容版)
 *
 * 测试覆盖:
 *   Test 1: Quad IO Read (EBh) — 初始全零内存读取
 *   Test 2: Quad IO Write (38h) + Read back — 数据完整性验证 (多种数据模式)
 *   Test 3: 多地址读写 — 非重叠区域写入/回读，确保各地址独立
 *   Test 4: CE# 复位行为 — CE# 拉高后重新发起操作 / 中途拉高恢复
 *   Test 5: 边界地址 — addr=0 和最大地址
 *   Test 6: 背靠背操作 — 连续快速发起读写
 *
 * 注: 本 testbench 避免使用 unpacked array ports (iverilog 不支持),
 *     所有数据均通过 32-bit packed 向量传递.
 */
module tb_psram;

  // ============================================================
  // 参数定义
  // ============================================================
  localparam CMD_QUAD_IO_READ  = 8'hEB;
  localparam CMD_QUAD_IO_WRITE = 8'h38;
  localparam DUMMY_CLOCKS      = 6;
  localparam CLK_HALF_PERIOD   = 5;    // 10ns 周期 = 100MHz

  // ============================================================
  // 信号声明
  // ============================================================
  reg        sck;
  reg        ce_n;
  wire [3:0] dio;

  reg  [3:0] dio_drive;
  reg        dio_oe;

  assign dio = dio_oe ? dio_drive : 4'bzzzz;

  // ============================================================
  // DUT 实例化
  // ============================================================
  psram dut (
    .sck (sck),
    .ce_n(ce_n),
    .dio (dio)
  );

  // ============================================================
  // 时钟生成
  // ============================================================
  initial sck = 0;
  always #CLK_HALF_PERIOD sck = ~sck;

  // ============================================================
  // 测试变量
  // ============================================================
  reg [31:0] rdata32;     // 读回数据 (byte3, byte2, byte1, byte0)
  reg [31:0] exp32;       // 期望数据
  integer    test_pass;
  integer    test_fail;
  integer    i;

  // ============================================================
  // 底层协议任务
  // ============================================================
  //
  // 时序说明:
  //   PSRAM 模型在 SCK 上升沿采样输入, master 在下降沿切换后续数据.
  //
  // 各阶段消耗的 SCK 周期数 (含内部状态转移):
  //   send_cmd:        9 周期 (8 位命令 + 1 转移)
  //   send_addr:       7 周期 (6 四线地址拍 + 1 转移)
  //   wait_dummy(n):   n+1 周期 (n dummy + 1 转移到 S_DATA_OUT)
  //   recv_data:       9 周期 (8 四线数据拍 + 1 转移到 S_IDLE)
  //   send_data:       9 周期 (同上)
  //

  // --- 发送 8 位命令 (串行 SIO[0], MSB 优先) ---
  task psram_send_cmd;
    input [7:0] cmd;
    integer j;
    begin
      @(negedge sck);
      ce_n = 0;
      dio_oe = 1;
      dio_drive = {3'b000, cmd[7]};
      @(posedge sck);

      for (j = 6; j >= 0; j = j - 1) begin
        @(negedge sck);
        dio_drive = {3'b000, cmd[j]};
        @(posedge sck);
      end
    end
  endtask

  // --- 发送 24 位地址 (四线 SIO[3:0], MS nibble 优先) ---
  task psram_send_addr;
    input [23:0] addr;
    integer j;
      begin
      for (j = 5; j >= 0; j = j - 1) begin
        @(negedge sck);
        dio_drive = addr[j*4 +: 4];
        @(posedge sck);
      end
    end
  endtask

  // --- 等待 N 个 dummy 周期 (+1 转移周期), dio 高阻 ---
  task psram_wait_dummy;
    input [5:0] n;
    begin
      dio_oe = 0;
      repeat (n + 1) @(posedge sck);
    end
  endtask

  // --- 接收 4 字节读数据 (四线) → 32-bit packed ---
  //     字节映射: data[ 7: 0] = byte0, data[15: 8] = byte1,
  //               data[23:16] = byte2, data[31:24] = byte3
  task psram_recv_data;
    output [31:0] data;
    integer j;
    reg [31:0] tmp;
    reg [3:0]  nibble;
    begin
      dio_oe = 0;
      tmp = 0;
      for (j = 0; j < 4; j = j + 1) begin
        @(posedge sck);
        nibble = dio;               // 高半字节 [7:4]
        tmp[j*8+4 +: 4] = nibble;
        @(posedge sck);
        nibble = dio;               // 低半字节 [3:0]
        tmp[j*8 +: 4] = nibble;
      end
      @(posedge sck);    // 转移: PSRAM → S_IDLE
      data = tmp;
    end
  endtask

  // --- 发送 4 字节写数据 (四线) ---
  task psram_send_data;
    input [31:0] data;
    integer j;
    begin
      @(negedge sck);
      dio_oe = 1;
      dio_drive = data[7:4];
      @(posedge sck);
      for (j = 0; j < 4; j = j + 1) begin
        @(negedge sck);
        dio_drive = data[j*8 +: 4];
        @(posedge sck);
        if (j < 3) begin
          @(negedge sck);
          dio_drive = data[(j+1)*8+4 +: 4];
          @(posedge sck);
        end
      end
      @(negedge sck);
    end
  endtask

  // --- 结束操作: CE# 拉高, 释放 dio ---
  task psram_end_op;
    begin
      dio_oe = 0;
      ce_n = 1;
      @(posedge sck);    // tCPH ≥ 1 SCK
    end
  endtask

  // ============================================================
  // 高层测试原语
  // ============================================================

  // --- Quad IO Read + 自动比对 ---
  task test_quad_read;
    input [23:0]       addr;
    input [31:0]       exp;
    input [8*32-1:0]   test_name;
    integer j;
    reg    mismatch;
    begin
      $display("  [TEST] %0s: Quad IO Read @ addr=0x%06h", test_name, addr);
      psram_send_cmd(CMD_QUAD_IO_READ);
      psram_send_addr(addr);
      psram_wait_dummy(DUMMY_CLOCKS);
      psram_recv_data(rdata32);
      psram_end_op();

      if (rdata32 !== exp) begin
        mismatch = 1;
        for (j = 0; j < 4; j = j + 1) begin
          if (rdata32[j*8 +: 8] !== exp[j*8 +: 8]) begin
            $error("    FAIL: byte[%0d] read=0x%02h, expected=0x%02h",
                   j, rdata32[j*8 +: 8], exp[j*8 +: 8]);
          end
        end
        test_fail = test_fail + 1;
      end else begin
        $display("    PASS");
        test_pass = test_pass + 1;
      end
    end
  endtask

  // --- Quad IO Write + Read-back + 自动比对 ---
  task test_write_read;
    input [23:0]       addr;
    input [31:0]       wdata;
    input [8*32-1:0]   test_name;
    integer j;
    reg    mismatch;
    begin
      $display("  [TEST] %0s: Write-then-Read @ addr=0x%06h", test_name, addr);

      // Write
      psram_send_cmd(CMD_QUAD_IO_WRITE);
      psram_send_addr(addr);
      psram_send_data(wdata);
      psram_end_op();

      // Read back
      psram_send_cmd(CMD_QUAD_IO_READ);
      psram_send_addr(addr);
      psram_wait_dummy(DUMMY_CLOCKS);
      psram_recv_data(rdata32);
      psram_end_op();

      if (rdata32 !== wdata) begin
        mismatch = 1;
        for (j = 0; j < 4; j = j + 1) begin
          if (rdata32[j*8 +: 8] !== wdata[j*8 +: 8]) begin
            $error("    FAIL: byte[%0d] read=0x%02h, expected=0x%02h",
                   j, rdata32[j*8 +: 8], wdata[j*8 +: 8]);
          end
        end
        test_fail = test_fail + 1;
      end else begin
        $display("    PASS");
        test_pass = test_pass + 1;
      end
    end
  endtask

  // ============================================================
  // 主测试序列
  // ============================================================
  initial begin
    $dumpfile("tb_psram.vcd");
    $dumpvars(0, tb_psram);

    sck        = 0;
    ce_n       = 1;
    dio_oe     = 0;
    dio_drive  = 0;
    test_pass  = 0;
    test_fail  = 0;

    repeat(4) @(posedge sck);

    $display("==============================================");
    $display(" PSRAM Behavioral Model Test Suite");
    $display(" IS66WVS4M8ALL - SPI Quad IO (EBh/38h)");
    $display(" SCK = 100 MHz (10 ns period)");
    $display("==============================================");

    // --------------------------------------------------------
    // Test 1: 读取初始全零内存
    // --------------------------------------------------------
    $display("\n=== Test 1: Read default (all-zero) memory ===");
    exp32 = 32'h00000000;
    test_quad_read(24'h000000, exp32, "addr=0x000000");
    test_quad_read(24'h001000, exp32, "addr=0x001000");
    test_quad_read(24'h3FFFFC, exp32, "addr=0x3FFFFC (last 4B)");

    // --------------------------------------------------------
    // Test 2: 写入 + 读出验证 (多种数据模式)
    // --------------------------------------------------------
    $display("\n=== Test 2: Write + Read-back (data patterns) ===");

    // 2a: Walking ones (byte0=0x01, byte1=0x02, byte2=0x04, byte3=0x08)
    exp32 = {8'h08, 8'h04, 8'h02, 8'h01};
    test_write_read(24'h000100, exp32, "Walking ones");

    // 2b: 0x55 / 0xAA 交替
    exp32 = {8'hAA, 8'h55, 8'hAA, 8'h55};
    test_write_read(24'h000104, exp32, "0x55/0xAA alternating");

    // 2c: 全 1
    test_write_read(24'h000108, 32'hFFFFFFFF, "All-ones (0xFF)");

    // 2d: 全 0 覆盖
    test_write_read(24'h00010C, 32'h00000000, "All-zeros overwrite");

    // 2e: 递增
    exp32 = {8'hA3, 8'hA2, 8'hA1, 8'hA0};
    test_write_read(24'h000110, exp32, "Incrementing (0xA0-0xA3)");

    // 2f: 递减
    exp32 = {8'hED, 8'hEE, 8'hEF, 8'hF0};
    test_write_read(24'h000114, exp32, "Decrementing (0xF0-0xED)");

    // --------------------------------------------------------
    // Test 3: 多地址独立读写
    // --------------------------------------------------------
    $display("\n=== Test 3: Multi-address independence ===");

    // Write A
    exp32 = {8'hA3, 8'hA2, 8'hA1, 8'hA0};
    test_write_read(24'h002000, exp32, "Write+Read addr=0x2000");

    // Write B
    exp32 = {8'hB3, 8'hB2, 8'hB1, 8'hB0};
    test_write_read(24'h003000, exp32, "Write+Read addr=0x3000");

    // Write C (紧接 A 之后)
    exp32 = {8'hC3, 8'hC2, 8'hC1, 8'hC0};
    test_write_read(24'h002004, exp32, "Write+Read addr=0x2004");

    // 回读 A — 确保未被破坏
    exp32 = {8'hA3, 8'hA2, 8'hA1, 8'hA0};
    test_quad_read(24'h002000, exp32, "Re-read addr=0x2000 (A)");

    // 回读 B
    exp32 = {8'hB3, 8'hB2, 8'hB1, 8'hB0};
    test_quad_read(24'h003000, exp32, "Re-read addr=0x3000 (B)");

    // 回读 C
    exp32 = {8'hC3, 8'hC2, 8'hC1, 8'hC0};
    test_quad_read(24'h002004, exp32, "Re-read addr=0x2004 (C)");

    // --------------------------------------------------------
    // Test 4: CE# 复位行为
    // --------------------------------------------------------
    $display("\n=== Test 4: CE# reset behavior ===");

    // 4a: 正常 Write+Read, CE# 拉高后重新操作
    exp32 = {8'hDE, 8'hDD, 8'hDC, 8'hDB};
    test_write_read(24'h000400, exp32, "Normal CE# cycle");

    // 4b: 写入中途 CE# 拉高 (异常终止), 再重新写入
    begin
      $display("  [TEST] CE# abort during write, then re-write @ addr=0x000404");
      // 发起写操作
      psram_send_cmd(CMD_QUAD_IO_WRITE);
      psram_send_addr(24'h000404);
      // 写两个字节 (byte0=0x11, byte1=0x22)
      dio_oe = 1;
      dio_drive = 4'h1;  // byte0 upper, drive before @
      @(posedge sck);
      dio_drive = 4'h1;  // byte0 lower
      @(posedge sck);
      dio_drive = 4'h2;  // byte1 upper
      @(posedge sck);
      dio_drive = 4'h2;  // byte1 lower
      @(posedge sck);
      // 中途拉高 CE# (模拟异常)
      ce_n = 1;
      dio_oe = 0;
      repeat(3) @(posedge sck);

      // 重新完整写入
      exp32 = {8'h5D, 8'h5C, 8'h5B, 8'h5A};
      psram_send_cmd(CMD_QUAD_IO_WRITE);
      psram_send_addr(24'h000404);
      psram_send_data(exp32);
      psram_end_op();

      // 回读验证
      psram_send_cmd(CMD_QUAD_IO_READ);
      psram_send_addr(24'h000404);
      psram_wait_dummy(DUMMY_CLOCKS);
      psram_recv_data(rdata32);
      psram_end_op();

      if (rdata32 !== exp32) begin
        $error("    FAIL: CE# abort recovery mismatch");
        $display("      read=0x%08h, expected=0x%08h", rdata32, exp32);
        test_fail = test_fail + 1;
      end else begin
        $display("    PASS: CE# abort recovery OK");
        test_pass = test_pass + 1;
      end
    end

    // 4c: 快速 CE# 翻转 (模拟噪声/毛刺)
    begin
      $display("  [TEST] CE# glitch resilience @ addr=0x000500");
      exp32 = {8'hCE, 8'hCD, 8'hCC, 8'hCB};
      // 写入
      psram_send_cmd(CMD_QUAD_IO_WRITE);
      psram_send_addr(24'h000500);
      psram_send_data(exp32);
      psram_end_op();

      // 快速 CE# 翻转
      ce_n = 1; @(posedge sck); @(posedge sck);
      ce_n = 0; @(posedge sck); ce_n = 1; @(posedge sck);
      ce_n = 0; @(posedge sck); ce_n = 1;
      repeat(3) @(posedge sck);

      // 回读
      psram_send_cmd(CMD_QUAD_IO_READ);
      psram_send_addr(24'h000500);
      psram_wait_dummy(DUMMY_CLOCKS);
      psram_recv_data(rdata32);
      psram_end_op();

      if (rdata32 !== exp32) begin
        $error("    FAIL: data corrupted by CE# glitches");
        test_fail = test_fail + 1;
      end else begin
        $display("    PASS: data intact after CE# glitches");
        test_pass = test_pass + 1;
      end
    end

    // --------------------------------------------------------
    // Test 5: 边界地址
    // --------------------------------------------------------
    $display("\n=== Test 5: Boundary address tests ===");

    // 地址 0 — 之前未写入，仍为初始值 0x00
    exp32 = 32'h00000000;
    test_quad_read(24'h000000, exp32, "addr=0 (boundary, all-zero)");

    // 最大地址 (4MB - 4)
    exp32 = {8'hF0, 8'hEF, 8'hEE, 8'hED};
    test_write_read(24'h3FFFFC, exp32, "addr=0x3FFFFC (max-4B)");

    // --------------------------------------------------------
    // Test 6: 背靠背操作
    // --------------------------------------------------------
    $display("\n=== Test 6: Back-to-back operations ===");
    begin
      reg [31:0] w1, w2;
      w1 = {8'h14, 8'h13, 8'h12, 8'h11};
      w2 = {8'h24, 8'h23, 8'h22, 8'h21};

      // 连续两次 Write
      $display("  [TEST] Back-to-back writes @ addr=0x600 and 0x604");
      psram_send_cmd(CMD_QUAD_IO_WRITE);
      psram_send_addr(24'h000600);
      psram_send_data(w1);
      psram_end_op();

      psram_send_cmd(CMD_QUAD_IO_WRITE);
      psram_send_addr(24'h000604);
      psram_send_data(w2);
      psram_end_op();

      // 连续两次 Read
      $display("  [TEST] Back-to-back reads");
      psram_send_cmd(CMD_QUAD_IO_READ);
      psram_send_addr(24'h000600);
      psram_wait_dummy(DUMMY_CLOCKS);
      psram_recv_data(rdata32);
      psram_end_op();

      if (rdata32 !== w1) begin
        $error("    FAIL: Read A mismatch (0x%08h vs 0x%08h)", rdata32, w1);
        test_fail = test_fail + 1;
      end else begin
        $display("    Read A (0x600): PASS");
        test_pass = test_pass + 1;
      end

      psram_send_cmd(CMD_QUAD_IO_READ);
      psram_send_addr(24'h000604);
      psram_wait_dummy(DUMMY_CLOCKS);
      psram_recv_data(rdata32);
      psram_end_op();

      if (rdata32 !== w2) begin
        $error("    FAIL: Read B mismatch (0x%08h vs 0x%08h)", rdata32, w2);
        test_fail = test_fail + 1;
      end else begin
        $display("    Read B (0x604): PASS");
        test_pass = test_pass + 1;
      end

      // 背靠背 Read 然后 Write
      $display("  [TEST] Read-then-Write @ addr=0x600");
      psram_send_cmd(CMD_QUAD_IO_READ);
      psram_send_addr(24'h000600);
      psram_wait_dummy(DUMMY_CLOCKS);
      psram_recv_data(rdata32);
      psram_end_op();

      exp32 = {8'h34, 8'h33, 8'h32, 8'h31};
      psram_send_cmd(CMD_QUAD_IO_WRITE);
      psram_send_addr(24'h000600);
      psram_send_data(exp32);
      psram_end_op();

      test_quad_read(24'h000600, exp32, "Verify after read-then-write");
    end

    // ============================================================
    // 测试总结
    // ============================================================
    $display("\n==============================================");
    $display(" TEST SUMMARY");
    $display("   Total passed: %0d", test_pass);
    $display("   Total failed: %0d", test_fail);
    if (test_fail == 0) begin
      $display("   RESULT: ALL TESTS PASSED");
    end else begin
      $display("   RESULT: %0d TEST(S) FAILED!", test_fail);
    end
    $display("==============================================");

    #200;
    $finish;
  end

endmodule

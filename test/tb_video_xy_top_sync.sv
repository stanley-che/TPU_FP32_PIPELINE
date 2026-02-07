    // tb_video_xy_top_sync.sv
    /*
    iverilog -g2012 -Wall -I./src \
    -o ./vvp/tb_video_xy_top_sync.vvp \
    ./test/tb_video_xy_top_sync.sv

    vvp ./vvp/tb_video_xy_top_sync.vvp
    gtkwave ./vvp/tb_video_xy_top_sync.vcd
    */

    `include "./src/AMOLED/video_in_timing_if/video_xy_top_sync.sv"

    `timescale 1ns/1ps
    `default_nettype none

    module tb_video_xy_top_sync;

    // ------------------------------------------------------------
    // clock / reset
    // ------------------------------------------------------------
    logic clk = 1'b0;
    always #5 clk = ~clk; // 100 MHz
    logic rst;

    // ------------------------------------------------------------
    // raw sync inputs
    // ------------------------------------------------------------
    logic vsync, hsync, de;

    // ------------------------------------------------------------
    // DUT outputs
    // ------------------------------------------------------------
    localparam int unsigned ACTIVE_W = 16;
    localparam int unsigned ACTIVE_H = 6;

    logic vsync_c, hsync_c, de_c;
    logic in_frame, in_line, pix_valid;

    logic [10:0] x;
    logic [9:0]  y;

    logic frame_start_p, line_start_p, line_end_p;
    logic x_at_last, y_at_last, x_overflow, y_overflow;
    // ------------------------------------------------------------
    // TB-visible aliases for DUT outputs (so tasks can bind)
    // ------------------------------------------------------------
    logic vsync_o, hsync_o, de_o;
    logic vs_rise_o, vs_fall_o, hs_rise_o, hs_fall_o, de_rise_o, de_fall_o;
    localparam int unsigned TB_STABLE_CYCLES = 2; // 要跟你 DUT param 一樣
    logic de_rise_p, de_fall_p, vs_rise_p;

    logic de_rise_p_d1, de_fall_p_d1, vs_rise_p_d1;
    logic in_frame_d1;

    logic sol_exp, eol_exp, sof_exp;

    // 如果你的 DUT instance 叫 dut，就用 dut.xxx
    // 若不是 dut，改成你的 instance 名稱

    assign vsync_o = vsync_c;
    assign hsync_o = hsync_c;
    assign de_o    = de_c;



    // ------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------
    video_xy_top_sync #(
        // sanitizer
        .VS_INV(1'b0),
        .HS_INV(1'b0),
        .DE_INV(1'b0),
        .USE_DEGLITCH(1'b0),
        .STABLE_CYCLES(2),

        // tracker
        .FRAME_START_ON_VS_RISE(1'b1),
        .USE_HSYNC_FOR_EOL(1'b0),

        // xy
        .X_W(11),
        .Y_W(10),
        .ACTIVE_W(ACTIVE_W),
        .ACTIVE_H(ACTIVE_H),

        .Y_INC_ON_DE_RISE(1'b0),  // y++ on de_fall
        .X_LIMIT_MODE(2),
        .Y_LIMIT_MODE(2),
        .ENABLE_BOUNDS(1'b1),
        .GATE_BY_IN_FRAME(1'b1),
        .ENABLE_ASSERT(1'b0)
    ) dut (
        .clk(clk),
        .rst(rst),

        .vsync_i(vsync),
        .hsync_i(hsync),
        .de_i   (de),

        .vsync(vsync_c),
        .hsync(hsync_c),
        .de   (de_c),

        .in_frame(in_frame),
        .in_line (in_line),
        .pix_valid(pix_valid),

        .x(x),
        .y(y),

        .frame_start_p(frame_start_p),
        .line_start_p (line_start_p),
        .line_end_p   (line_end_p),

        .x_at_last(x_at_last),
        .y_at_last(y_at_last),
        .x_overflow(x_overflow),
        .y_overflow(y_overflow)
    );

    // ------------------------------------------------------------
    // TB helpers
    // ------------------------------------------------------------
    int unsigned err_cnt = 0;

    task automatic tb_err(input string msg);
        begin
        $display("[ERR] t=%0t %s", $time, msg);
        err_cnt++;
        end
    endtask

    task automatic drive_pix(input bit v, input bit h, input bit d);
        begin
        vsync <= v;
        hsync <= h;
        de    <= d;
        @(posedge clk);
        #1;
        check_one_cycle();
        end
    endtask

    task automatic idle(input int unsigned n);
        int unsigned i;
        begin
        for (i=0; i<n; i++) drive_pix(vsync, hsync, de);
        end
    endtask

    // alias (如果你習慣 drive_one_pixel / idle_cycles)
    task automatic drive_one_pixel(input bit v, input bit h, input bit d);
        begin
        drive_pix(v,h,d);
        end
    endtask
    task automatic idle_cycles(input int unsigned n);
        begin
        idle(n);
        end
    endtask

    // ------------------------------------------------------------
    // Scoreboard model
    // - 注意：sync_edge_top 可能會引入 latency
    // - 因此 model 直接用 DUT 的語意輸出 (in_frame/pix_valid/pulses) 來推 exp_x/exp_y
    //   而不是用 raw sync 自己算 edge 對齊
    // ------------------------------------------------------------
    int unsigned exp_x, exp_y;
    logic [9:0]  y_prev;

    function automatic int unsigned sat_inc(input int unsigned val, input int unsigned last);
        if (val < last) sat_inc = val + 1;
        else            sat_inc = last;
    endfunction

    // 這個用來處理「frame 被 vs_rise 腰斬 line」的情況：
    // 若 frame_start_p 發生時，當下 de_c=1（代表正在 active 中被切），則忽略下一個 line_end_p 的 y++
    logic suppress_next_line_end;

    task automatic model_step;
        begin
        // frame start：reset
        if (frame_start_p) begin
            exp_x = 0;
            exp_y = 0;
            suppress_next_line_end = de_c; // 若 frame start 時 de 仍高，代表 cut
        end else if (in_frame) begin
            // x:
            // line_start_p -> x=0
            if (line_start_p) begin
            exp_x = 0;
            end else if (pix_valid) begin
            exp_x = sat_inc(exp_x, (ACTIVE_W-1));
            end

            // y:
            // line_end_p -> y++
            if (line_end_p) begin
            if (suppress_next_line_end) begin
                suppress_next_line_end = 1'b0;
            end else begin
                exp_y = sat_inc(exp_y, (ACTIVE_H-1));
            end
            end
        end
        end
    endtask

    // ------------------------------------------------------------
    // Checks
    // ------------------------------------------------------------
    logic in_frame_q;
    always_ff @(posedge clk) begin
        if (rst) in_frame_q <= 1'b0;
        else     in_frame_q <= in_frame;
    end

    task automatic check_one_cycle;
        begin
        model_step();

        // x always check
        if (x !== exp_x[10:0]) begin
            tb_err($sformatf("x mismatch: got=%0d exp=%0d (pv=%0b ls=%0b le=%0b fs=%0b in_frame=%0b de_c=%0b)",
                            x, exp_x, pix_valid, line_start_p, line_end_p, frame_start_p, in_frame, de_c));
        end

        // y: in frame check value, out of frame hold
        if (in_frame_q) begin
            if (y !== exp_y[9:0]) begin
            tb_err($sformatf("y mismatch: got=%0d exp=%0d (pv=%0b ls=%0b le=%0b fs=%0b in_frame=%0b de_c=%0b)",
                            y, exp_y, pix_valid, line_start_p, line_end_p, frame_start_p, in_frame, de_c));
            end
        end else begin
            if (y !== y_prev) begin
            tb_err($sformatf("y changed while !in_frame_q: prev=%0d now=%0d", y_prev, y));
            end
        end
        y_prev = y;

        // bounds
        if (x > (ACTIVE_W-1)) tb_err($sformatf("x out of bounds: x=%0d", x));
        if (y > (ACTIVE_H-1)) tb_err($sformatf("y out of bounds: y=%0d", y));

        // semantic pulse checks (NOT phase-locked)
        if (frame_start_p) begin
            if (x !== 0 || y !== 0) tb_err("frame_start_p asserted but x/y not reset");
        end
        if (line_start_p) begin
            if (x !== 0) tb_err("line_start_p asserted but x not reset");
        end

        // tracker sanity
        if (!in_frame && in_line) tb_err("in_line must be 0 when !in_frame");
        if (pix_valid !== (in_frame_q && in_line_q && de_c_q))
            tb_err("pix_valid != (prev in_frame&in_line&de_c)");
        // flags
        if (x_at_last !== (x == (ACTIVE_W-1))) tb_err("x_at_last wrong");
        if (y_at_last !== (y == (ACTIVE_H-1))) tb_err("y_at_last wrong");
        end
    endtask

    // ------------------------------------------------------------
    // Pattern generators (raw)
    // ------------------------------------------------------------
    task automatic frame_start_pulse;
        begin
        drive_pix(1'b1, 1'b0, 1'b0);
        drive_pix(1'b0, 1'b0, 1'b0);
        end
    endtask

    task automatic line_hsync_pulse;
        begin
        drive_pix(vsync, 1'b1, 1'b0);
        drive_pix(vsync, 1'b0, 1'b0);
        end
    endtask

    task automatic drive_active_line(input int unsigned w);
        int unsigned i;
        begin
        line_hsync_pulse();
        for (i=0; i<w; i++) drive_pix(vsync, 1'b0, 1'b1);
        for (i=0; i<4; i++) drive_pix(vsync, 1'b0, 1'b0);
        end
    endtask

    task automatic drive_blank_line;
        int unsigned i;
        begin
        line_hsync_pulse();
        for (i=0; i<(ACTIVE_W+4); i++) drive_pix(vsync, 1'b0, 1'b0);
        end
    endtask

    // ------------------------------------------------------------
    // Test cases (same as your passing set)
    // ------------------------------------------------------------
    task automatic tc1_basic_frame;
        int unsigned j;
        begin
        $display("[TC1] basic frame counting");
        frame_start_pulse();
        for (j=0; j<ACTIVE_H; j++) drive_active_line(ACTIVE_W);
        drive_blank_line();
        end
    endtask

    task automatic tc2_de_glitch_outside_frame;
        begin
        $display("[TC2] de glitch outside frame");
        vsync = 1'b0; hsync = 1'b0; de = 1'b0;
        idle(5);
        drive_pix(1'b0, 1'b0, 1'b1);
        drive_pix(1'b0, 1'b0, 1'b0);
        idle(5);
        end
    endtask

    task automatic tc3_short_line;
        begin
        $display("[TC3] short active line (w=ACTIVE_W-3)");
        frame_start_pulse();
        drive_active_line(ACTIVE_W-3);
        drive_blank_line();
        end
    endtask

    task automatic tc4_long_line_saturate;
        begin
        $display("[TC4] long active line (w=ACTIVE_W+5) saturate x");
        frame_start_pulse();
        drive_active_line(ACTIVE_W+5);
        drive_blank_line();
        end
    endtask

    task automatic tc5_vsrise_mid_line;
        int unsigned i;
        begin
        $display("[TC5] vs_rise in middle of active line resets x/y");
        frame_start_pulse();
        line_hsync_pulse();

        for (i=0; i<5; i++) drive_pix(vsync, 1'b0, 1'b1);

        // abrupt new frame while de=1
        drive_pix(1'b1, 1'b0, 1'b1);
        drive_pix(1'b0, 1'b0, 1'b0);

        drive_active_line(ACTIVE_W);
        end
    endtask

    task automatic tc6_frame_end_while_de_high;
        int unsigned i;
        begin
        $display("[TC6] frame_end while de=1 must clear in_frame/in_line");
        frame_start_pulse();
        line_hsync_pulse();
        for (i=0; i<5; i++) drive_pix(vsync, 1'b0, 1'b1);
        // frame end while de=1
        drive_pix(1'b1, 1'b0, 1'b1);
        drive_pix(1'b0, 1'b0, 1'b1);
        drive_pix(1'b0, 1'b0, 1'b0);
        idle(2);
        end
    endtask

    task automatic tc7_de_stuck_high_outside_frame;
        int unsigned i;
        begin
        $display("[TC7] de stuck high outside frame should be ignored");
        vsync = 1'b0; hsync = 1'b0; de = 1'b0;
        idle(5);
        for (i=0; i<20; i++) drive_pix(1'b0, 1'b0, 1'b1);
        for (i=0; i<5; i++)  drive_pix(1'b0, 1'b0, 1'b0);
        end
    endtask

    task automatic tc8_many_short_de_segments_one_line;
        int unsigned k;
        begin
        $display("[TC8] many short de segments in one line");
        frame_start_pulse();
        line_hsync_pulse();
        for (k=0; k<5; k++) begin
            drive_pix(vsync, 1'b0, 1'b1);
            drive_pix(vsync, 1'b0, 1'b1);
            drive_pix(vsync, 1'b0, 1'b0);
        end
        repeat (5) drive_pix(vsync, 1'b0, 1'b0);
        drive_blank_line();
        end
    endtask

    task automatic tc9_vsrise_same_cycle_as_derise;
        begin
        $display("[TC9] vs_rise same cycle as de_rise");
        // same-cycle start
        drive_pix(1'b1, 1'b0, 1'b1);
        drive_pix(1'b0, 1'b0, 1'b1);
        drive_pix(1'b0, 1'b0, 1'b0);
        drive_pix(1'b1, 1'b0, 1'b0);
        drive_pix(1'b0, 1'b0, 1'b0);
        end
    endtask
    // ------------------------------------------------------------
    // TEST10: de 1-cycle glitch outside frame should not create in_frame/pix_valid
    // ------------------------------------------------------------
    task automatic test10_de_glitch_outside_frame_no_effect;
    begin
        $display("\n--- TEST10: de 1-cycle glitch outside frame no effect ---");
        // ensure out of frame
        drive_raw(0,0,0); drive_raw(0,0,0);

        // glitch
        drive_raw(0,0,1);
        drive_raw(0,0,0);

        chk_eq("in_frame stays 0", in_frame, 1'b0);
        chk_eq("pix_valid stays 0", pix_valid, 1'b0);
        chk_all_pulses_zero("after de glitch");
    end
    endtask

    // ------------------------------------------------------------
    // TEST11: stable DE high while still out_of_frame must NOT enter in_line
    // (because tracker gates line entry by in_frame)
    // ------------------------------------------------------------
    task automatic test11_de_stuck_high_outside_frame_ignored;
    int i;
    begin
        $display("\n--- TEST11: de stuck high outside frame ignored ---");
        drive_raw(0,0,0); drive_raw(0,0,0);
        for (i=0; i<10; i++) drive_raw(0,0,1);

        chk_eq("in_frame=0", in_frame, 1'b0);
        chk_eq("in_line=0",  in_line,  1'b0);
        chk_eq("pix_valid=0",pix_valid,1'b0);
    end
    endtask

    // ------------------------------------------------------------
    // TEST12: frame start then immediate de high should allow line entry
    // ------------------------------------------------------------
    task automatic frame_start_pulse_stable;
        int i;
        begin
            // vsync high stable
            for (i=0; i<TB_STABLE_CYCLES; i++) drive_raw(1,0,0);
            // vsync low stable
            for (i=0; i<TB_STABLE_CYCLES; i++) drive_raw(0,0,0);
        end
    endtask

    task automatic frame_start_level_hold;
        int i;
        begin
            // ensure low first
            for (i=0; i<TB_STABLE_CYCLES; i++) drive_raw(0,0,0);
            // rise and HOLD high (do not fall here)
            for (i=0; i<TB_STABLE_CYCLES; i++) drive_raw(1,0,0);
            // afterwards, test12 will keep using drive_raw(vsync,...) so vsync stays 1
        end
    endtask

    task automatic line_hsync_pulse_stable;
        int i;
        begin
            for (i=0; i<TB_STABLE_CYCLES; i++) drive_raw(vsync,1,0);
            for (i=0; i<TB_STABLE_CYCLES; i++) drive_raw(vsync,0,0);
        end
    endtask

    task automatic de_rise_stable;
        int i;
        begin
            for (i=0; i<TB_STABLE_CYCLES; i++) drive_raw(vsync,0,1);
        end
    endtask

    task automatic hold_raw(
        input bit v,
        input bit h,
        input bit d,
        input int unsigned n
    );
        int i;
        begin
            for (i=0; i<n; i++) drive_raw(v,h,d);
        end
    endtask

    // HSYNC: generate both edges with stable segments (but DO NOT force vsync to 0)
    task automatic hsync_pulse_both_edges_stable;
        int i;
        begin
            // ensure hsync=0 for a while
            for (i=0; i<TB_STABLE_CYCLES; i++) drive_raw(vsync,0,0);

            // 0->1 (rise)
            for (i=0; i<TB_STABLE_CYCLES; i++) drive_raw(vsync,1,0);

            // 1->0 (fall)
            for (i=0; i<TB_STABLE_CYCLES; i++) drive_raw(vsync,0,0);

            // extra rise (optional, keep it)
            for (i=0; i<TB_STABLE_CYCLES; i++) drive_raw(vsync,1,0);

            // back to 0
            for (i=0; i<TB_STABLE_CYCLES; i++) drive_raw(vsync,0,0);
        end
    endtask

    // ------------------------------------------------------------
    // Icarus-safe waiters (NO ref/input-sig freeze problem)
    // ------------------------------------------------------------
    task automatic wait_in_frame_1(input int unsigned max_cycles);
        int unsigned k;
        begin : WAITF
            for (k=0; k<max_cycles; k++) begin
                if (in_frame === 1'b1) begin
                    $display("[ OK ] in_frame=1 asserted @t=%0t (after %0d cycles)", $time, k);
                    disable WAITF;
                end
                drive_raw(vsync, hsync, de);
            end
            $display("[ERR] in_frame=1 did NOT assert within %0d cycles @t=%0t", max_cycles, $time);
            err_cnt++;
        end
    endtask

    task automatic wait_in_line_1(input int unsigned max_cycles);
        int unsigned k;
        begin : WAITL
            for (k=0; k<max_cycles; k++) begin
                if (in_line === 1'b1) begin
                    $display("[ OK ] in_line=1 asserted @t=%0t (after %0d cycles)", $time, k);
                    disable WAITL;
                end
                drive_raw(vsync, hsync, de);
            end
            $display("[ERR] in_line=1 did NOT assert within %0d cycles @t=%0t", max_cycles, $time);
            err_cnt++;
        end
    endtask

    task automatic wait_pix_valid_1(input int unsigned max_cycles);
    int unsigned k;
        begin : WAITP
            for (k=0; k<max_cycles; k++) begin
                if (pix_valid === 1'b1) begin
                    $display("[ OK ] pix_valid=1 asserted @t=%0t (after %0d cycles)", $time, k);
                    disable WAITP;
                end
                drive_raw(vsync, hsync, de);
            end
            $display("[ERR] pix_valid=1 did NOT assert within %0d cycles @t=%0t", max_cycles, $time);
            err_cnt++;
        end
    endtask

    // ------------------------------------------------------------
    // TEST12 (updated)
    // ------------------------------------------------------------
    task automatic test12_frame_then_de_enters_line;
        int unsigned MAXW;
        begin
            $display("\n--- TEST12: frame start then de enters line (latency-tolerant, HSYNC+DE) ---");
            MAXW = 400;

            // clean start
            hold_raw(0,0,0, TB_STABLE_CYCLES*6);

            // frame start and keep frame active (vsync held high with your current FRAME_START_ON_VS_RISE=1)
            frame_start_level_hold();
            wait_in_frame_1(MAXW);

            // produce a line sync edge
            hsync_pulse_both_edges_stable();

            // ensure de=0 is seen cleanly
            hold_raw(vsync,0,0, TB_STABLE_CYCLES*4);

            // DE 0->1 and hold
            hold_raw(vsync,0,1, TB_STABLE_CYCLES*10);

            // wait for in_line and pix_valid
            wait_in_line_1(MAXW);
            wait_pix_valid_1(MAXW);

            // hold a few more
            hold_raw(vsync,0,1, TB_STABLE_CYCLES*4);
        end
    endtask


    

    // ------------------------------------------------------------
    // TEST13: bouncing de inside frame should not break bounds (x/y stay within)
    // ------------------------------------------------------------
    task automatic frame_begin_hold_stable;
        int i;
        begin
            // ensure low stable
            for (i=0; i<TB_STABLE_CYCLES*2; i++) drive_raw(0,0,0);
            // rise and keep high (frame active)
            for (i=0; i<TB_STABLE_CYCLES*2; i++) drive_raw(1,0,0);
        end
    endtask

    task automatic frame_end_hold_stable;
        int i;
        begin
            // fall to end frame
            for (i=0; i<TB_STABLE_CYCLES*2; i++) drive_raw(0,0,0);
        end
    endtask

    task automatic test13_bouncing_de_inside_frame_bounds;
  int i;
  begin
    $display("\n--- TEST13: bouncing de inside frame bounds ---");

    // 用穩定的 frame 方式跑（vsync=1 期間才算 frame）
    frame_begin_hold_stable();

    // 先確保在 line 開始前 de=0 一段時間
    hold_raw(1,0,0, TB_STABLE_CYCLES*2);

    // 用 de_rise 觸發 line_start（因為你 USE_HSYNC_FOR_EOL=0，line 主要靠 DE）
    // bouncing: 0/1/0/1...
    for (i=0; i<10; i++) begin
      drive_raw(1,0,(i[0]));
    end

    // 收尾：de 拉回 0
    hold_raw(1,0,0, TB_STABLE_CYCLES*2);

    // frame end
    frame_end_hold_stable();

    // bounds check only
    if (x > (ACTIVE_W-1)) tb_err("x out of bounds during bounce");
    if (y > (ACTIVE_H-1)) tb_err("y out of bounds during bounce");
  end
    endtask

    // ------------------------------------------------------------
    // TEST14: simultaneous vsync rise and de high same cycle (extreme)
    // ------------------------------------------------------------
    task automatic test14_vsrise_same_cycle_as_de_high;
        begin
            $display("\n--- TEST14: vsync rise same cycle as de high ---");
            drive_raw(1,0,1);  // vsync=1 and de=1 same cycle
            drive_raw(0,0,1);
            drive_raw(0,0,0);

            // At least: x/y should be reset at some point when frame_start_p asserts
            // We don't hard-lock exact cycle due to sanitizer latency.
        end
    endtask

    // ------------------------------------------------------------
    // TEST15: reset while inputs are high => no spurious in_frame/in_line
    // ------------------------------------------------------------
    task automatic test15_reset_with_inputs_high;
    begin
        $display("\n--- TEST15: reset with inputs high ---");
        // drive high
        vsync <= 1'b1; hsync <= 1'b1; de <= 1'b1;
        @(posedge clk);

        // assert reset
        rst <= 1'b1;
        repeat(3) @(posedge clk);

        // release reset
        rst <= 1'b0;
        @(posedge clk);
        #1;

        // after reset release, we expect clean state (depends on sanitizer latency)
        chk_eq("in_line cleared", in_line, 1'b0);
    end
    endtask
    logic in_line_q;
    logic de_c_q;

    always_ff @(posedge clk) begin
    if (rst) begin
        in_frame_q <= 1'b0;
        in_line_q  <= 1'b0;
        de_c_q     <= 1'b0;
    end else begin
        in_frame_q <= in_frame;
        in_line_q  <= in_line;
        de_c_q     <= de_c;
    end
    end

    // ------------------------------------------------------------
    // TEST16: long random raw toggles should never violate pix_valid definition
    // pix_valid must always equal (in_frame & in_line & de_c)
    // ------------------------------------------------------------
    task automatic test16_random_sanity_pix_valid;
    int i;
    begin
        $display("\n--- TEST16: random pix_valid sanity ---");
        for (i=0; i<200; i++) begin
        drive_raw($random, $random, $random);
        if (pix_valid !== (in_frame_q && in_line_q && de_c_q))
            tb_err("pix_valid relation broken");
        end
    end
    endtask

    // ------------------------------------------------------------
    // TEST17: run an entire small frame quickly (smoke)
    // ------------------------------------------------------------
    

    task automatic test17_smoke_small_frame;
        begin
            $display("\n--- TEST17: smoke small frame (frame-hold wrapper, no tc1 change) ---");

            // 在呼叫 tc1 之前，先用 level-hold 方式把 frame 打開並保持
            frame_begin_hold_stable();

            // 直接跑你原本的 tc1（不修改 tc1）
            tc1_basic_frame();

            // tc1 跑完後，再把 frame 關掉
            frame_end_hold_stable();

            // 仍然做你原本的 smoke check
            chk_eq("end: x within", (x <= (ACTIVE_W-1)), 1'b1);
            chk_eq("end: y within", (y <= (ACTIVE_H-1)), 1'b1);
        end
    endtask

    // ------------------------------------------------------------
    // TEST18: (TB-level) sanitizer stable cycles param existence check
    // (we can't observe internal deglitch easily without exporting pulses;
    // just make sure TB_STABLE_CYCLES is usable)
    // ------------------------------------------------------------
    task automatic test18_tb_stable_cycles_defined;
    begin
        $display("\n--- TEST18: TB_STABLE_CYCLES defined = %0d ---", TB_STABLE_CYCLES);
        if (TB_STABLE_CYCLES < 1) tb_err("TB_STABLE_CYCLES must be >= 1");
    end
    endtask

    // ------------------------------------------------------------
    // Common helpers (must exist before being called)
    // ------------------------------------------------------------
    task automatic chk_eq(
    input string tag,
    input logic  got,
    input logic  exp
    );
    begin
        if (got !== exp) begin
        $display("[ERR] %s got=%0b exp=%0b @t=%0t", tag, got, exp, $time);
        err_cnt++;
        end else begin
        $display("[ OK ] %s got=%0b exp=%0b @t=%0t", tag, got, exp, $time);
        end
    end
    endtask

    task automatic drive_raw(
    input bit v,
    input bit h,
    input bit d
    );
    begin
        vsync <= v;
        hsync <= h;
        de    <= d;
        @(posedge clk);
        #1;
    end
    endtask

    task automatic chk_all_pulses_zero(input string tag);
    begin
        if (frame_start_p !== 1'b0) begin $display("[ERR] %s frame_start_p not zero @t=%0t", tag, $time); err_cnt++; end
        if (line_start_p  !== 1'b0) begin $display("[ERR] %s line_start_p not zero @t=%0t", tag, $time); err_cnt++; end
        if (line_end_p    !== 1'b0) begin $display("[ERR] %s line_end_p not zero @t=%0t", tag, $time); err_cnt++; end
    end
    endtask



    // ------------------------------------------------------------
    // Main
    // ------------------------------------------------------------
    initial begin
        $dumpfile("./vvp/tb_video_xy_top_sync.vcd");
        $dumpvars(0, tb_video_xy_top_sync);

        // init
        vsync = 1'b0;
        hsync = 1'b0;
        de    = 1'b0;

        exp_x  = 0;
        exp_y  = 0;
        y_prev = '0;
        suppress_next_line_end = 1'b0;

        // reset
        rst = 1'b1;
        repeat (5) @(posedge clk);
        rst = 1'b0;

        // run
        tc1_basic_frame();
        tc2_de_glitch_outside_frame();
        tc3_short_line();
        tc4_long_line_saturate();
        tc5_vsrise_mid_line();
        tc6_frame_end_while_de_high();
        tc7_de_stuck_high_outside_frame();
        tc8_many_short_de_segments_one_line();
        tc9_vsrise_same_cycle_as_derise();
        test10_de_glitch_outside_frame_no_effect();
        test11_de_stuck_high_outside_frame_ignored();
        test12_frame_then_de_enters_line();
        test13_bouncing_de_inside_frame_bounds();
        test14_vsrise_same_cycle_as_de_high();
        test15_reset_with_inputs_high();
        test16_random_sanity_pix_valid();
        test17_smoke_small_frame();
        test18_tb_stable_cycles_defined();


        if (err_cnt == 0) $display("[TB] PASS: no errors");
        else              $display("[TB] FAIL: err_cnt=%0d", err_cnt);

        $finish;
    end

    endmodule

    `default_nettype wire

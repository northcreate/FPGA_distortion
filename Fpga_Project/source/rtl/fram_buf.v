`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Meyesemi
// Engineer: Nill
// 
// Create Date: 15/03/23 14:17:29
// Design Name: 
// Module Name: fram_buf
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`define UD #1
module fram_buf #(
    parameter                     MEM_ROW_WIDTH        = 15    ,
    parameter                     MEM_COLUMN_WIDTH     = 10    ,
    parameter                     MEM_BANK_WIDTH       = 3     ,
    parameter                     CTRL_ADDR_WIDTH = MEM_ROW_WIDTH + MEM_BANK_WIDTH + MEM_COLUMN_WIDTH,
    parameter                     MEM_DQ_WIDTH         = 32    ,
    parameter                     H_NUM                = 12'd1920,//12'd1920,
    parameter                     V_NUM                = 12'd1080,//12'd1080,//12'd106,//
    parameter                     PIX_WIDTH            = 16,//24 
    parameter                     READ_TEST_MODE       = 1'b0,
    parameter                     READBACK_DISPLAY_MODE = 1'b0,
    parameter                     DISTORTION_ENABLE    = 1'b0,
    parameter integer             DIST_VERTICAL_RADIUS = 15,
    // Temporary screen diagnostic; set 0 to disable the magenta overlay.
    parameter                     DIST_CACHE_MISS_VISUAL = 1'b0,
    // Diagnostic only: add a 48-pixel wide prefetch progress bar at the left.
    parameter                     DIST_PREFETCH_PROGRESS_VISUAL = 1'b0
)(
    input                         vin_clk,
    input                         wr_fsync,
    input                         wr_en,
    input  [PIX_WIDTH- 1'b1 : 0]  wr_data,
    output reg                    init_done=0,
    output reg [2:0]              rd_test_vis_status,
    
    input                         ddr_clk,
    input                         ddr_rstn,
    
    input                         vout_clk,
    input                         rd_fsync,
    input                         rd_en,
    // Optional random-access 2x2 source-pixel request from the distortion mapper.
    input                         dist_rd_en,
    input  [15:0]                 dist_u_int,
    input  [15:0]                 dist_v_int,
    input  [15:0]                 dist_ref_v,          // ★ 新增诊断参考
    input                         dist_in_bounds,
    output                        dist_sample_valid,
    output [15:0]                 dist_p00,
    output [15:0]                 dist_p01,
    output [15:0]                 dist_p10,
    output [15:0]                 dist_p11,
    output                        vout_de,
    output [PIX_WIDTH- 1'b1 : 0]  vout_data,
    
    output [CTRL_ADDR_WIDTH-1:0]  axi_awaddr     ,
    output [3:0]                  axi_awid       ,
    output [3:0]                  axi_awlen      ,
    output [2:0]                  axi_awsize     ,
    output [1:0]                  axi_awburst    ,
    input                         axi_awready    ,
    output                        axi_awvalid    ,
                                                  
    output [MEM_DQ_WIDTH*8-1:0]   axi_wdata      ,
    output [MEM_DQ_WIDTH -1 :0]   axi_wstrb      ,
    input                         axi_wlast      ,
    output                        axi_wvalid     ,
    input                         axi_wready     ,
    input  [3 : 0]                axi_bid        ,                                      
                                                  
    output [CTRL_ADDR_WIDTH-1:0]  axi_araddr     ,
    output [3:0]                  axi_arid       ,
    output [3:0]                  axi_arlen      ,
    output [2:0]                  axi_arsize     ,
    output [1:0]                  axi_arburst    ,
    output                        axi_arvalid    ,
    input                         axi_arready    ,
                                                  
    output                        axi_rready     ,
    input  [MEM_DQ_WIDTH*8-1:0]   axi_rdata      ,
    input                         axi_rvalid     ,
    input                         axi_rlast      ,
    input  [3:0]                  axi_rid            
);
    // Clamp each address axis independently to break cross-axis timing paths.
    // Full geometry validity still gates the output; invalid pixels stay black.
    wire request_u_ok = (dist_u_int < H_NUM);
    wire request_v_ok = (dist_v_int < V_NUM);
    wire request_geometry_ok = dist_in_bounds && request_u_ok && request_v_ok;
    wire [15:0] safe_dist_u_int = request_u_ok ? dist_u_int : 16'd0;
    wire [15:0] safe_dist_v_int = request_v_ok ? dist_v_int : 16'd0;
    parameter LEN_WIDTH       = 32;
    parameter LINE_ADDR_WIDTH = 22;//19;//1440 * 1080 = 1 555 200 = 21'h17BB00
    parameter FRAME_CNT_WIDTH = CTRL_ADDR_WIDTH - LINE_ADDR_WIDTH;
    
    wire                        ddr_wreq;     
    wire [CTRL_ADDR_WIDTH- 1'b1 : 0] ddr_waddr;    
    wire [LEN_WIDTH- 1'b1 : 0]  ddr_wr_len;   
    wire                        ddr_wrdy;     
    wire                        ddr_wdone;    
    wire [8*MEM_DQ_WIDTH-1 : 0] ddr_wdata;    
    wire                        ddr_wdata_req;
    
    // Legacy rd_buf command wires and the optional phase-1 test reader.
    wire                        rd_buf_cmd_en;
    wire [CTRL_ADDR_WIDTH-1:0]  rd_buf_cmd_addr;
    wire [LEN_WIDTH- 1'b1:0]    rd_buf_cmd_len;

    wire                        rd_cmd_en;
    wire [CTRL_ADDR_WIDTH-1:0]  rd_cmd_addr;
    wire [LEN_WIDTH- 1'b1:0]    rd_cmd_len;
    wire                        rd_cmd_ready;
    wire                        rd_cmd_done;

    wire                        read_ready  = 1'b1;
    wire [PIX_WIDTH-1:0]        legacy_vout_data;

    // One full source row is first staged using the existing 256->32 dual-clock
    // RAM IP. The row is then copied into a 16-slot cache as 960 32-bit words.
    wire [31:0]                 rd_test_line_ram_data;
    wire [11:0]                 rd_test_line_rd_addr;
    wire [15:0]                 rd_test_line_pixel;
    reg  [1:0]                  vout_rstn_sync = 2'b00;
    reg  [1:0]                  row_cache_ready_sync_vout = 2'b00;
    reg                         line_start_toggle_vout = 1'b0;
    reg  [10:0]                 line_start_row_vout = 11'd0;
    reg                         frame_start_toggle_vout = 1'b0;
    reg  [1:0]                  line_start_toggle_sync_ddr = 2'b00;
    reg                         line_start_toggle_seen_ddr = 1'b0;
    reg  [10:0]                 line_start_row_meta_ddr = 11'd0;
    reg  [10:0]                 line_start_row_sync_ddr = 11'd0;
    reg  [1:0]                  frame_start_toggle_sync_ddr = 2'b00;
    reg                         frame_start_toggle_seen_ddr = 1'b0;
    // Remember output-frame VS even while a DDR row request/copy is busy.
    // The original code acknowledged VS globally but only acted on it from
    // S_WAIT_FRAME_START, so a late reader could miss an entire frame boundary.
    reg                         frame_start_pending_ddr = 1'b0;
    wire                        frame_start_event_ddr =
        frame_start_toggle_sync_ddr[1] != frame_start_toggle_seen_ddr;
    wire                        frame_restart_req_ddr =
        frame_start_pending_ddr || frame_start_event_ddr;
    (* keep = "true" *) reg [15:0] dbg_frame_start_busy_count = 16'd0;
    // Fixed 32-entry arrays keep elaboration compatible in both modes.
    // Only 16 entries are instantiated when distortion is disabled.
    wire [31:0]                 line_cache_word [0:31];
    wire [15:0]                 line_cache_even_pixel [0:31];
    wire [15:0]                 line_cache_odd_pixel [0:31];
    reg  [10:0]                 vout_x_count = 11'd0;
    reg  [11:0]                 vout_y_count = 12'd0;
    reg                         vout_rd_en_d1 = 1'b0;
    reg                         vout_fsync_d1 = 1'b0;
    reg                         vout_line_seen = 1'b0;
    reg  [4:0]                  vout_y_bank_d1 = 5'd0;
    reg                         vout_x_lsb_d1 = 1'b0;
    reg                         vout_x_lsb_d2 = 1'b0;              // ★ 新增

    reg  [11:0]                 row_cache_read_addr_d1 = 12'd0;   // ★ 新增

    reg  [31:0]                 line_cache_selected_word;
    wire [MEM_DQ_WIDTH*8-1:0]   read_rdata;
    wire                        read_en;
    wire                        ddr_wr_bac;

    // Phase-2 sequential row reader and row-cache fill controller.
    wire                        rd_test_req_ready;
    reg                         rd_test_req_valid = 1'b0;
    wire                        rd_test_cmd_en;
    wire [CTRL_ADDR_WIDTH-1:0]  rd_test_cmd_addr;
    wire [31:0]                 rd_test_cmd_len;
    wire                        rd_test_busy;
    wire                        rd_test_done;
    wire                        rd_test_error;
    wire [15:0]                 rd_test_beats_seen;
    wire [15:0]                 rd_test_completed_beats;
    wire                        rd_test_data_valid;
    wire [15:0]                 rd_test_data_index;
    wire [MEM_DQ_WIDTH*8-1:0]   rd_test_data;
    wire [MEM_DQ_WIDTH*8-1:0]   rd_test_first_data;
    wire [CTRL_ADDR_WIDTH-1:0]  rd_test_addr;
    localparam integer          RD_TEST_LINE_BEATS = (H_NUM * PIX_WIDTH) / (MEM_DQ_WIDTH * 8);
    localparam integer          DDR_WORDS_PER_ROW = (H_NUM * PIX_WIDTH) / 32;
    // Distortion mode holds a 32-row window; plain row-cache diagnostics keep 16 rows.
    // For K1=0.04, fx=fy=1400, 1080p, the estimated maximum vertical displacement
    // is about 13.4 rows. Radius=15 plus one row for bilinear v+1 fits in 32 slots.
    localparam integer          ROW_CACHE_LINES = (DISTORTION_ENABLE != 0) ? 32 : 16;
    localparam integer          CACHE_LOOKAHEAD = (DISTORTION_ENABLE != 0) ? (DIST_VERTICAL_RADIUS + 1) : 1;
    localparam integer          STREAM_START_ROW = ROW_CACHE_LINES - CACHE_LOOKAHEAD;
    // Diagnostic tag check follows the same two-cycle latency as pixel mux/parity.
    // It does not change pixel timing, DDR requests, cache scheduling or weights.
    reg                         dist_tags_match_d1 = 1'b0;
    reg                         dist_tags_match_d2 = 1'b0;
    // Diagnostic-only two-cycle color pipeline; aligned with tag_match and RAM data.
    reg [15:0]                  dist_tag_diag_color_d1 = 16'hF81F;
    reg [15:0]                  dist_tag_diag_color_d2 = 16'hF81F;
    reg  [4:0]                  dist_bank0_d1 = 5'd0;
    reg  [4:0]                  dist_bank1_d1 = 5'd1;
    reg                         dist_u_lsb_d1 = 1'b0;
    reg                         dist_bounds_d1 = 1'b0;
    reg                         dist_valid_d1 = 1'b0;
    wire [9:0]                  dist_word_addr0;       // floor(u/2)
    wire [9:0]                  dist_word_addr_even;   // even lane address, floor(u/2)+u[0]
    wire [9:0]                  cache_rd_addr_a;
    wire [9:0]                  cache_rd_addr_b;


    localparam [2:0] S_WAIT_INIT       = 3'd0;
    localparam [2:0] S_SEND_REQ        = 3'd1;
    localparam [2:0] S_WAIT_READ       = 3'd2;
    localparam [2:0] S_WAIT_COPY       = 3'd3;
    localparam [2:0] S_WAIT_LINE       = 3'd4;
    localparam [2:0] S_WAIT_FRAME_START= 3'd5;
    localparam [2:0] S_DONE            = 3'd6;
    localparam [2:0] S_ERROR           = 3'd7;
    localparam [1:0] PHASE_FRAME_PREFILL = 2'd0;
    localparam [1:0] PHASE_STREAM        = 2'd1;
    reg [2:0]                   row_state = S_WAIT_INIT;
    reg [1:0]                   row_scheduler_phase = PHASE_FRAME_PREFILL;
    reg [10:0]                  rd_test_row_index = 11'd0;
    reg                         row_cache_error = 1'b0;
    reg                         row_cache_all_ready = 1'b0;
    reg                         copy_req_toggle_ddr = 1'b0;
    reg [1:0]                   copy_done_sync_ddr = 2'b00;
    reg                         copy_done_seen_ddr = 1'b0;
    reg [1:0]                   copy_req_sync_pix = 2'b00;
    reg                         copy_req_seen_pix = 1'b0;
    reg [4:0]                   cache_fill_row_sync1 = 5'd0;
    reg [4:0]                   cache_fill_row_sync2 = 5'd0;
    reg [4:0]                   cache_fill_row_pix = 5'd0;
    // Full source-row identity tags for debug; these do not alter pixel data.
    reg [10:0]                  cache_fill_src_row_sync1 = 11'd0;
    reg [10:0]                  cache_fill_src_row_sync2 = 11'd0;
    reg [10:0]                  cache_fill_src_row_pix = 11'd0;
    reg [10:0]                  cache_row_tag [0:31];
    reg [31:0]                  cache_row_valid = 32'd0;
    reg [31:0]                  dist_cache_tag_miss_count = 32'd0;
    reg                         dist_cache_tag_miss_sticky = 1'b0;

    wire [4:0]                  dist_row_slot0_now = safe_dist_v_int[4:0];
    wire [4:0] dist_row_slot1_now = (safe_dist_v_int == V_NUM-1) ? safe_dist_v_int[4:0] : safe_dist_v_int[4:0]+5'd1;
    wire [10:0]                 dist_req_row0_now = safe_dist_v_int[10:0];
    wire [10:0] dist_req_row1_now = (safe_dist_v_int == V_NUM-1) ? safe_dist_v_int[10:0] : safe_dist_v_int[10:0]+11'd1;
    // Keep these signals for PDS/ILA to inspect source-row cache identity.
    (* keep = "true" *) wire [10:0] dbg_dist_req_row0 = dist_req_row0_now;
    (* keep = "true" *) wire [10:0] dbg_dist_req_row1 = dist_req_row1_now;
    (* keep = "true" *) wire [10:0] dbg_cache_tag_row0 = cache_row_tag[dist_row_slot0_now];
    (* keep = "true" *) wire [10:0] dbg_cache_tag_row1 = cache_row_tag[dist_row_slot1_now];
    (* keep = "true" *) wire        dbg_cache_valid_row0 = cache_row_valid[dist_row_slot0_now];
    (* keep = "true" *) wire        dbg_cache_valid_row1 = cache_row_valid[dist_row_slot1_now];
    (* keep = "true" *) wire        dbg_cache_tag_match_row0 = cache_row_valid[dist_row_slot0_now] &&
                                                               (cache_row_tag[dist_row_slot0_now] == dist_req_row0_now);
    (* keep = "true" *) wire        dbg_cache_tag_match_row1 = cache_row_valid[dist_row_slot1_now] &&
                                                               (cache_row_tag[dist_row_slot1_now] == dist_req_row1_now);
    (* keep = "true" *) wire        dbg_dist_cache_tag_miss = row_cache_ready_sync_vout[1] &&
                                                               dist_rd_en && dist_in_bounds &&
                                                               !(dbg_cache_tag_match_row0 && dbg_cache_tag_match_row1);
    // On-screen classification of a source-row tag miss. Compare source-row IDs,
    // not cache-slot indices. These indicate the direction of the mismatch;
    // an old-frame tag may also fall into the "greater than requested" category.
    wire tag_invalid_now = !(dbg_cache_valid_row0 && dbg_cache_valid_row1);
    wire tag_older_now = (dbg_cache_valid_row0 && (dbg_cache_tag_row0 < dbg_dist_req_row0)) ||
                         (dbg_cache_valid_row1 && (dbg_cache_tag_row1 < dbg_dist_req_row1));
    wire tag_newer_now = (dbg_cache_valid_row0 && (dbg_cache_tag_row0 > dbg_dist_req_row0)) ||
                         (dbg_cache_valid_row1 && (dbg_cache_tag_row1 > dbg_dist_req_row1));
    // Yellow = row being filled / never marked valid.
    // Cyan   = cached row number older than requested (possible prefetch lag).
    // Red    = cached row number newer than requested (overwrite or stale frame).
    // Magenta = other mismatch / fallback.
    wire [15:0] dist_tag_diag_color_now =
        tag_invalid_now ? 16'hFFE0 :
        tag_older_now   ? 16'h07FF :
        tag_newer_now   ? 16'hF800 : 16'hF81F;
    // Two-stage synchronizer on Gray-coded DDR prefetch row index. Diagnostic
    // only: do not use this bus for functional cache control.
    wire [10:0] diag_prefetch_gray_ddr =
        rd_test_row_index ^ (rd_test_row_index >> 1);
    reg  [10:0] diag_prefetch_gray_meta = 11'd0;
    reg  [10:0] diag_prefetch_gray_sync = 11'd0;
    reg  [1:0] diag_prefill_sync = 2'b11;
    reg  [1:0] diag_error_sync = 2'b00;
    function [10:0] diag_gray_to_bin;
        input [10:0] gray;
        integer b;
        begin
            diag_gray_to_bin[10] = gray[10];
            for (b = 9; b >= 0; b = b - 1)
                diag_gray_to_bin[b] = diag_gray_to_bin[b+1] ^ gray[b];
        end
    endfunction
    wire [10:0] diag_prefetch_row_pix =
        diag_gray_to_bin(diag_prefetch_gray_sync);
    always @(posedge vout_clk or negedge ddr_rstn) begin
        if (!ddr_rstn) begin
            diag_prefetch_gray_meta <= 11'd0;
            diag_prefetch_gray_sync <= 11'd0;
            diag_prefill_sync <= 2'b11;
            diag_error_sync <= 2'b00;
        end else begin
            diag_prefetch_gray_meta <= diag_prefetch_gray_ddr;
            diag_prefetch_gray_sync <= diag_prefetch_gray_meta;
            diag_prefill_sync <= {diag_prefill_sync[0],
                                  (row_scheduler_phase == PHASE_FRAME_PREFILL)};
            diag_error_sync <= {diag_error_sync[0], (row_state == S_ERROR)};
        end
    end

    // This overlay compares the requested source row (not only cache tags)
    // with the DDR prefetch command row. The command row may still be in flight.
    // The indicator therefore distinguishes large scheduling gaps, not
    // single-cycle completion accuracy.
    wire [11:0] diag_prefetch_plus_one = {1'b0, diag_prefetch_row_pix} + 12'd1;
    wire [11:0] diag_req_row12 = {1'b0, dist_req_row0_now};
    wire [11:0] diag_req_plus_33 = diag_req_row12 + 12'd33;

wire [11:0] dbg_display_row =
    {1'b0, line_start_row_vout};

wire [11:0] dbg_source_row =
    {1'b0, safe_dist_v_int[10:0]};


wire [15:0] dbg_src_color =
    (safe_dist_v_int < 16'd256)  ? 16'h001F : // 蓝
    (safe_dist_v_int < 16'd512)  ? 16'h07FF : // 青
    (safe_dist_v_int < 16'd768)  ? 16'h07E0 : // 绿
    (safe_dist_v_int < 16'd1024) ? 16'hFFE0 : // 黄
                              16'hF81F; // 品红

wire [15:0] dbg_display_color =
    (line_start_row_vout < 11'd256)  ? 16'h001F :
    (line_start_row_vout < 11'd512)  ? 16'h07FF :
    (line_start_row_vout < 11'd768)  ? 16'h07E0 :
    (line_start_row_vout < 11'd1024) ? 16'hFFE0 :
                                       16'hF81F;

// 左半屏：畸变计算输出的源行号
// 右半屏：行缓存显示端的行号
wire [16:0] dbg_src_row = {1'b0, safe_dist_v_int};
wire [16:0] dbg_ref_row = {1'b0, dist_ref_v};

wire [15:0] diag_progress_color_now =
    (dbg_src_row > dbg_ref_row + 17'd2)
        ? 16'hF800 :   // 红色：计算结果偏大
    (dbg_ref_row > dbg_src_row + 17'd2)
        ? 16'h001F :   // 蓝色：计算结果偏小
          16'h07E0;    // 绿色：计算正确或误差<=2


    reg [15:0] dist_progress_color_d1 = 16'h001F;
    reg [15:0] dist_progress_color_d2 = 16'h001F;
    reg        dist_progress_strip_d1 = 1'b0;
    reg        dist_progress_strip_d2 = 1'b0;
    reg                         copy_done_toggle_pix = 1'b0;
    reg                         copy_active_pix = 1'b0;
    reg [10:0]                  copy_word_index = 11'd0;
    wire                        cache_word_wr_en;
    wire [9:0]                  cache_word_wr_addr;
    // Registered row-cache write interface. Only distortion mode adds a cycle.
    wire                        cache_phys_wr_en;
    wire                        cache_phys_wr_last;
    wire [9:0]                  cache_phys_wr_addr;
    wire [4:0]                  cache_phys_wr_bank;
    wire [10:0]                 cache_phys_wr_src_row;
    wire [31:0]                 cache_phys_wr_data [0:3];
    wire [11:0]                 row_cache_read_addr;
    reg  [CTRL_ADDR_WIDTH-1:0]  read_frame_base = {CTRL_ADDR_WIDTH{1'b0}};
    wire [CTRL_ADDR_WIDTH-1:0]  write_bank_base;

    // All bank ownership stays in ddr_clk. The row scheduler, not the raw
    // HDMI VS edge, releases the previous bank after read/copy completion.
    wire frame_wirq;
    // Three DDR frame slots. All ownership decisions are in ddr_clk.
    // A slot is published only after V_NUM completed line writes.
    wire write_frame_start, display_frame_start;
    reg [1:0] write_bank = 2'd1, display_bank = 2'd0;
    reg [1:0] ready_bank = 2'd2;
    reg ready_valid = 1'b0;
    reg capture_accept = 1'b0;
    reg [12:0] completed_lines = 13'd0;
    reg write_req_q = 1'b0, write_done_q = 1'b0, read_req_q = 1'b0;
    reg write_busy = 1'b0;
    reg [15:0] read_beats_pending = 16'd0;
    wire write_request_event = ddr_wreq && !write_req_q;
    wire write_done_event = ddr_wdone && !write_done_q;
    wire read_request_event = rd_cmd_en && !read_req_q;
    reg [1:0] write_bank_n, display_bank_n, ready_bank_n;
    reg ready_valid_n, capture_accept_n, init_done_n, write_busy_n;
    reg [12:0] completed_lines_n;
    reg [15:0] read_beats_pending_n;

    always @* begin
        write_bank_n = write_bank;
        display_bank_n = display_bank;
        ready_bank_n = ready_bank;
        ready_valid_n = ready_valid;
        capture_accept_n = capture_accept;
        completed_lines_n = completed_lines;
        init_done_n = init_done;
        write_busy_n = write_busy;
        read_beats_pending_n = read_beats_pending;
        if (write_request_event) write_busy_n = 1'b1;
        if (write_done_event) write_busy_n = 1'b0;
        if (read_request_event)
            read_beats_pending_n = read_beats_pending_n + rd_cmd_len[15:0];
        if (read_en && read_beats_pending_n != 0)
            read_beats_pending_n = read_beats_pending_n - 1'b1;

        if (write_done_event && capture_accept && !write_frame_start) begin
            if (completed_lines == V_NUM-1) begin
                ready_bank_n = write_bank;
                ready_valid_n = 1'b1;
                init_done_n = 1'b1; // First complete capture allows cache startup.
                capture_accept_n = 1'b0;
                completed_lines_n = V_NUM;
            end else begin
                completed_lines_n = completed_lines + 1'b1;
            end
        end

        // Never change the displayed bank mid-frame or while old reads remain.
        if (display_frame_start && ready_valid_n &&
            read_beats_pending == 0 && !rd_cmd_en &&
            !(ready_bank_n == write_bank && (write_busy || ddr_wreq))) begin
            display_bank_n = ready_bank_n;
            ready_valid_n = 1'b0;
            init_done_n = 1'b1;
        end

        // Allocate at camera VSYNC. Preserve both display and newest ready frame.
        // If a previous line transaction overruns VSYNC, discard this input
        // frame, retain its private bank, and retry at the next camera VSYNC.
        if (write_frame_start) begin
            completed_lines_n = 0;
            if (!write_busy && !ddr_wreq) begin
                if (display_bank_n != 0 && (!ready_valid_n || ready_bank_n != 0))
                    write_bank_n = 0;
                else if (display_bank_n != 1 && (!ready_valid_n || ready_bank_n != 1))
                    write_bank_n = 1;
                else
                    write_bank_n = 2;
                capture_accept_n = 1'b1;
            end else begin
                capture_accept_n = 1'b0;
            end
        end
    end

    always @(posedge ddr_clk) begin
        if (!ddr_rstn) begin
            write_bank <= 1; display_bank <= 0; ready_bank <= 2;
            ready_valid <= 0; capture_accept <= 0; completed_lines <= 0;
            write_req_q <= 0; write_done_q <= 0; read_req_q <= 0;
            write_busy <= 0; read_beats_pending <= 0;
            init_done <= 0;
        end else begin
            write_bank <= write_bank_n; display_bank <= display_bank_n;
            ready_bank <= ready_bank_n; ready_valid <= ready_valid_n;
            capture_accept <= capture_accept_n; completed_lines <= completed_lines_n;
            write_req_q <= ddr_wreq; write_done_q <= ddr_wdone; read_req_q <= rd_cmd_en;
            write_busy <= write_busy_n; read_beats_pending <= read_beats_pending_n;
            init_done <= init_done_n;
        end
    end

    assign display_frame_start = frame_restart_req_ddr && (
        (row_state == S_WAIT_FRAME_START) || (row_state == S_SEND_REQ) ||
        (row_state == S_WAIT_LINE) ||
        ((row_state == S_WAIT_READ) && rd_test_done && !rd_test_error &&
         (rd_test_completed_beats == RD_TEST_LINE_BEATS)) ||
        ((row_state == S_WAIT_COPY) &&
         (copy_done_sync_ddr[1] != copy_done_seen_ddr)));

    wr_buf #(
        .ADDR_WIDTH       (  CTRL_ADDR_WIDTH  ),//parameter                     ADDR_WIDTH      = 6'd27,
        .ADDR_OFFSET      (  32'd0            ),//parameter                     ADDR_OFFSET     = 32'h0000_0000,
        .H_NUM            (  H_NUM            ),//parameter                     H_NUM           = 12'd1920,
        .V_NUM            (  V_NUM            ),//parameter                     V_NUM           = 12'd1080,
        .DQ_WIDTH         (  MEM_DQ_WIDTH     ),//parameter                     DQ_WIDTH        = 7'd32,
        .LEN_WIDTH        (  LEN_WIDTH        ),//parameter                     LEN_WIDTH       = 6'd16,
        .PIX_WIDTH        (  PIX_WIDTH        ),//parameter                     PIX_WIDTH       = 6'd24,
        .LINE_ADDR_WIDTH  (  LINE_ADDR_WIDTH  ),//parameter                     LINE_ADDR_WIDTH = 4'd19,
        .FRAME_CNT_WIDTH  (  FRAME_CNT_WIDTH  ) //parameter                     FRAME_CNT_WIDTH = 4'd8
    ) wr_buf (
        .frame_bank (write_bank),
        .frame_start(write_frame_start),                                       
        .ddr_clk          (  ddr_clk          ),//input                         ddr_clk,
        .ddr_rstn         (  ddr_rstn         ),//input                         ddr_rstn,
                                              
        .wr_clk           (  vin_clk          ),//input                         wr_clk,
        .wr_fsync         (  wr_fsync         ),//input                         wr_fsync,
        .wr_en            (  wr_en            ),//input                         wr_en,
        .wr_data          (  wr_data          ),//input  [PIX_WIDTH- 1'b1 : 0]  wr_data,
        
        .rd_bac           (  ddr_wr_bac       ),//input                         rd_bac,                                      
        .ddr_wreq         (  ddr_wreq         ),//output                        ddr_wreq,
        .ddr_waddr        (  ddr_waddr        ),//output [ADDR_WIDTH- 1'b1 : 0] ddr_waddr,
        .ddr_wr_len       (  ddr_wr_len       ),//output [LEN_WIDTH- 1'b1 : 0]  ddr_wr_len,
        .ddr_wrdy         (  ddr_wrdy         ),//input                         ddr_wrdy,
        .ddr_wdone        (  ddr_wdone        ),//input                         ddr_wdone,
        .ddr_wdata        (  ddr_wdata        ),//output [8*DQ_WIDTH- 1'b1 : 0] ddr_wdata,
        .ddr_wdata_req    (  ddr_wdata_req    ),//input                         ddr_wdata_req,
                                              
        .frame_wcnt       (                   ),//output [FRAME_CNT_WIDTH-1 :0] frame_wcnt,
        .frame_wirq       (  frame_wirq       ) //output                        frame_wirq
    );
    
    rd_buf #(
        .ADDR_WIDTH       (  CTRL_ADDR_WIDTH  ),//parameter                     ADDR_WIDTH      = 6'd27,
        .ADDR_OFFSET      (  32'h0000_0000    ),//parameter                     ADDR_OFFSET     = 32'h0000_0000,
        .H_NUM            (  H_NUM            ),//parameter                     H_NUM           = 12'd1920,
        .V_NUM            (  V_NUM            ),//parameter                     V_NUM           = 12'd1080,
        .DQ_WIDTH         (  MEM_DQ_WIDTH     ),//parameter                     DQ_WIDTH        = 7'd32,
        .LEN_WIDTH        (  LEN_WIDTH        ),//parameter                     LEN_WIDTH       = 6'd16,
        .PIX_WIDTH        (  PIX_WIDTH        ),//parameter                     PIX_WIDTH       = 6'd24,
        .LINE_ADDR_WIDTH  (  LINE_ADDR_WIDTH  ),//parameter                     LINE_ADDR_WIDTH = 4'd19,
        .FRAME_CNT_WIDTH  (  FRAME_CNT_WIDTH  ) //parameter                     FRAME_CNT_WIDTH = 4'd8
    ) rd_buf (
        .frame_bank (display_bank),
        .frame_start(),
        .ddr_clk         (  ddr_clk           ),//input                         ddr_clk,
        .ddr_rstn        (  ddr_rstn          ),//input                         ddr_rstn,

        .vout_clk        (  vout_clk          ),//input                         vout_clk,
        .rd_fsync        (  rd_fsync          ),//input                         rd_fsync,
        .rd_en           (  rd_en             ),//input                         rd_en,
        .vout_de         (  vout_de           ),//output                        vout_de,
        .vout_data       (  legacy_vout_data ),// original sequential readout
        
        .init_done       (  init_done         ),//input                         init_done,
      
        .ddr_rreq        (  rd_buf_cmd_en    ),//output                        ddr_rreq,
        .ddr_raddr       (  rd_buf_cmd_addr  ),//output [ADDR_WIDTH- 1'b1 : 0] ddr_raddr,
        .ddr_rd_len      (  rd_buf_cmd_len   ),//output [LEN_WIDTH- 1'b1 : 0]  ddr_rd_len,
        .ddr_rrdy        (  rd_cmd_ready      ),//input                         ddr_rrdy,
        .ddr_rdone       (  rd_cmd_done       ),//input                         ddr_rdone,
                                              
        .ddr_rdata       (  read_rdata        ),//input [8*DQ_WIDTH- 1'b1 : 0]  ddr_rdata,
        .ddr_rdata_en    (  read_en           ) //input                         ddr_rdata_en
    );
    
    // Select exactly one logical read-command source. Default mode 0 preserves
    // the original frame-buffer read path without behavior changes.
    generate
        if (READ_TEST_MODE != 0) begin : G_READ_TEST_MUX
            assign rd_cmd_en   = rd_test_cmd_en;
            assign rd_cmd_addr = rd_test_cmd_addr;
            assign rd_cmd_len  = rd_test_cmd_len;
        end else begin : G_LEGACY_READ_MUX
            assign rd_cmd_en   = rd_buf_cmd_en;
            assign rd_cmd_addr = rd_buf_cmd_addr;
            assign rd_cmd_len  = rd_buf_cmd_len;
        end
    endgenerate

    // Use the NEXT selected display bank on the same edge that starts row 0.
    // It stays reserved until the next safe scheduler frame restart.
    assign write_bank_base = {{(CTRL_ADDR_WIDTH-LINE_ADDR_WIDTH-2){1'b0}},
                              display_bank_n, {LINE_ADDR_WIDTH{1'b0}}};
    assign rd_test_addr = read_frame_base + (rd_test_row_index * DDR_WORDS_PER_ROW);

    // The row scheduler has two phases:
    //  1) On output vertical blank, prefill 16 rows in normal mode or 32 rows
    //     in distortion mode.
    //  2) During active video, prefetch source row y+CACHE_LOOKAHEAD into its
    //     modulo-ROW_CACHE_LINES slot, retaining the required source-row window.
    always @(posedge ddr_clk or negedge ddr_rstn) begin
        if (!ddr_rstn) begin
            rd_test_req_valid             <= 1'b0;
            rd_test_row_index             <= 11'd0;
            row_state                     <= S_WAIT_INIT;
            row_scheduler_phase           <= PHASE_FRAME_PREFILL;
            row_cache_error               <= 1'b0;
            row_cache_all_ready           <= 1'b0;
            read_frame_base               <= {CTRL_ADDR_WIDTH{1'b0}};
            copy_req_toggle_ddr           <= 1'b0;
            copy_done_sync_ddr            <= 2'b00;
            copy_done_seen_ddr            <= 1'b0;
            line_start_toggle_sync_ddr    <= 2'b00;
            line_start_toggle_seen_ddr    <= 1'b0;
            line_start_row_meta_ddr       <= 11'd0;
            line_start_row_sync_ddr       <= 11'd0;
            frame_start_toggle_sync_ddr   <= 2'b00;
            frame_start_toggle_seen_ddr   <= 1'b0;
            frame_start_pending_ddr       <= 1'b0;
            dbg_frame_start_busy_count    <= 16'd0;
            rd_test_vis_status            <= 3'd0;
        end else begin
            rd_test_req_valid           <= 1'b0;
            copy_done_sync_ddr          <= {copy_done_sync_ddr[0], copy_done_toggle_pix};
            line_start_toggle_sync_ddr  <= {line_start_toggle_sync_ddr[0], line_start_toggle_vout};
            line_start_row_meta_ddr     <= line_start_row_vout;
            line_start_row_sync_ddr     <= line_start_row_meta_ddr;
            frame_start_toggle_sync_ddr <= {frame_start_toggle_sync_ddr[0], frame_start_toggle_vout};

            // Consume each synchronized event once, even while the reader is
            // busy. The completion path below also catches up if an event
            // arrives near the end of a copy operation.
            if (line_start_toggle_sync_ddr[1] != line_start_toggle_seen_ddr)
                line_start_toggle_seen_ddr <= line_start_toggle_sync_ddr[1];
            if (frame_start_event_ddr) begin
                frame_start_toggle_seen_ddr <= frame_start_toggle_sync_ddr[1];
                frame_start_pending_ddr <= 1'b1;
                if (row_state != S_WAIT_FRAME_START &&
                    row_state != S_WAIT_INIT &&
                    dbg_frame_start_busy_count != 16'hFFFF)
                    dbg_frame_start_busy_count <= dbg_frame_start_busy_count + 1'b1;
            end

            case (row_state)
                S_WAIT_INIT: begin
                    rd_test_vis_status <= 3'd0; // waiting for a fully captured input frame
                    if ((READ_TEST_MODE != 0) && init_done && (READBACK_DISPLAY_MODE == 0)) begin
                        read_frame_base   <= write_bank_base;
                        rd_test_row_index <= 11'd0;
                        row_state         <= S_SEND_REQ;
                    end else if ((READ_TEST_MODE != 0) && init_done) begin
                        row_state <= S_WAIT_FRAME_START;
                    end
                end

                S_WAIT_FRAME_START: begin
                    rd_test_vis_status <= row_cache_all_ready ? 3'd2 : 3'd0;
                    // The rising VS edge starts vertical blank. Prefill the
                    // selected cache window before active source sampling begins.
                    // Select a completed frame from the shared three-slot owner.
                    if (frame_restart_req_ddr) begin
                        // Restart at a safe command/copy boundary, never leave
                        // a whole new output frame waiting for old row 1079.
                        frame_start_pending_ddr <= 1'b0;
                        read_frame_base     <= write_bank_base;
                        rd_test_row_index   <= 11'd0;
                        row_scheduler_phase <= PHASE_FRAME_PREFILL;
                        row_state           <= S_SEND_REQ;
                    end
                end

                S_SEND_REQ: begin
                    rd_test_vis_status <= 3'd1; // request row read
                    if (frame_restart_req_ddr) begin
                        // Restart at a safe command/copy boundary, never leave
                        // a whole new output frame waiting for old row 1079.
                        frame_start_pending_ddr <= 1'b0;
                        read_frame_base     <= write_bank_base;
                        rd_test_row_index   <= 11'd0;
                        row_scheduler_phase <= PHASE_FRAME_PREFILL;
                        row_state           <= S_SEND_REQ;
                    end else if (rd_test_req_ready) begin
                        rd_test_req_valid <= 1'b1;
                        row_state <= S_WAIT_READ;
                    end
                end

                S_WAIT_READ: begin
                    rd_test_vis_status <= 3'd3; // DDR row read in progress
                    if (rd_test_done) begin
                        if (rd_test_error || (rd_test_completed_beats != RD_TEST_LINE_BEATS)) begin
                            row_cache_error <= 1'b1;
                            row_state <= S_ERROR;
                        end else if (frame_restart_req_ddr) begin
                            // Discard the completed old-frame row before
                            // issuing any read for the new output frame.
                            frame_start_pending_ddr <= 1'b0;
                            read_frame_base     <= write_bank_base;
                            rd_test_row_index   <= 11'd0;
                            row_scheduler_phase <= PHASE_FRAME_PREFILL;
                            row_state           <= S_SEND_REQ;
                        end else if (READBACK_DISPLAY_MODE == 0) begin
                            // Keep the legacy solid-color diagnostic behavior.
                            row_cache_all_ready <= 1'b1;
                            row_state <= S_DONE;
                        end else begin
                            // The complete row is staged in rd_test_line_ram;
                            // serialize its 960 words into the selected cache slot.
                            copy_req_toggle_ddr <= ~copy_req_toggle_ddr;
                            row_state <= S_WAIT_COPY;
                        end
                    end
                end

                S_WAIT_COPY: begin
                    rd_test_vis_status <= 3'd3; // staging RAM -> row cache
                    if (copy_done_sync_ddr[1] != copy_done_seen_ddr) begin
                        copy_done_seen_ddr <= copy_done_sync_ddr[1];

                        if (frame_restart_req_ddr) begin
                            // The final BRAM write has completed: safe to
                            // abandon any remaining old-frame source rows.
                            frame_start_pending_ddr <= 1'b0;
                            read_frame_base     <= write_bank_base;
                            rd_test_row_index   <= 11'd0;
                            row_scheduler_phase <= PHASE_FRAME_PREFILL;
                            row_state           <= S_SEND_REQ;
                        end else if (row_scheduler_phase == PHASE_FRAME_PREFILL) begin
                            if (rd_test_row_index < ROW_CACHE_LINES-1) begin
                                rd_test_row_index <= rd_test_row_index + 1'b1;
                                row_state <= S_SEND_REQ;
                            end else begin
                                // Initial cache window is ready before active row 0.
                                row_cache_all_ready <= 1'b1;
                                row_scheduler_phase <= PHASE_STREAM;
                                row_state <= S_WAIT_LINE;
                            end
                        end else begin
                            // IMPORTANT: never jump over source rows. If DDR/copy
                            // finishes late, catch up sequentially (row N+1, then
                            // N+2...) so every modulo-cache slot receives its tag's
                            // actual pixel data. Jumping directly to y+LOOKAHEAD can
                            // leave a stale row in a slot and cause intermittent
                            // broken horizontal lines.
                            if (rd_test_row_index >= (V_NUM-1)) begin
                                row_state <= S_WAIT_FRAME_START;
                            end else if ((line_start_row_sync_ddr >= STREAM_START_ROW) &&
                                         (((line_start_row_sync_ddr + CACHE_LOOKAHEAD) >= V_NUM) ||
                                          ((line_start_row_sync_ddr + CACHE_LOOKAHEAD) > rd_test_row_index))) begin
                                rd_test_row_index <= rd_test_row_index + 11'd1;
                                row_state <= S_SEND_REQ;
                            end else begin
                                row_state <= S_WAIT_LINE;
                            end
                        end
                    end
                end

                S_WAIT_LINE: begin
                    rd_test_vis_status <= row_cache_all_ready ? 3'd2 : 3'd0;
                    if (frame_restart_req_ddr) begin
                        // Restart at a safe command/copy boundary, never leave
                        // a whole new output frame waiting for old row 1079.
                        frame_start_pending_ddr <= 1'b0;
                        read_frame_base     <= write_bank_base;
                        rd_test_row_index   <= 11'd0;
                        row_scheduler_phase <= PHASE_FRAME_PREFILL;
                        row_state           <= S_SEND_REQ;
                    end else if (line_start_toggle_sync_ddr[1] != line_start_toggle_seen_ddr) begin
                        // Continue filling one row at a time. Clamp the *window
                        // end* at the bottom of the frame, but do not stop until
                        // source row V_NUM-1 has actually been copied into BRAM.
                        if ((line_start_row_sync_ddr >= STREAM_START_ROW) &&
                            (rd_test_row_index < (V_NUM-1)) &&
                            (((line_start_row_sync_ddr + CACHE_LOOKAHEAD) >= V_NUM) ||
                             ((line_start_row_sync_ddr + CACHE_LOOKAHEAD) > rd_test_row_index))) begin
                            rd_test_row_index <= rd_test_row_index + 11'd1;
                            row_scheduler_phase <= PHASE_STREAM;
                            row_state <= S_SEND_REQ;
                        end else if (rd_test_row_index >= (V_NUM-1)) begin
                            row_state <= S_WAIT_FRAME_START;
                        end
                    end
                end

                S_DONE: begin
                    rd_test_vis_status <= 3'd2; // solid-color diagnostic pass
                end

                S_ERROR: begin
                    rd_test_vis_status <= 3'd7; // error
                end

                default: begin
                    row_cache_error <= 1'b1;
                    row_state <= S_ERROR;
                    rd_test_vis_status <= 3'd7;
                end
            endcase
        end
    end

    ddr3_reader #(
        .ADDR_WIDTH ( CTRL_ADDR_WIDTH ),
        .DATA_WIDTH ( MEM_DQ_WIDTH * 8 ),
        .COUNT_WIDTH( 16 )
    ) u_ddr3_reader_test (
        .clk             ( ddr_clk                  ),
        .rstn            ( ddr_rstn                 ),
        .req_valid       ( rd_test_req_valid       ),
        .req_ready       ( rd_test_req_ready       ),
        .req_addr        ( rd_test_addr            ),
        .req_len         ( RD_TEST_LINE_BEATS      ),
        .rd_cmd_en       ( rd_test_cmd_en          ),
        .rd_cmd_addr     ( rd_test_cmd_addr        ),
        .rd_cmd_len      ( rd_test_cmd_len         ),
        .rd_cmd_ready    ( rd_cmd_ready            ),
        .rd_cmd_done     ( rd_cmd_done             ),
        .rd_data         ( read_rdata              ),
        .rd_data_valid   ( read_en                 ),
        .data_valid      ( rd_test_data_valid      ),
        .data            ( rd_test_data            ),
        .data_index      ( rd_test_data_index      ),
        .busy            ( rd_test_busy            ),
        .done            ( rd_test_done            ),
        .error           ( rd_test_error           ),
        .beats_seen      ( rd_test_beats_seen      ),
        .completed_beats ( rd_test_completed_beats ),
        .first_data      ( rd_test_first_data      )
    );

    // Existing staging RAM: stores 120 x 256-bit DDR beats and exposes 960
    // 32-bit words to the pixel-clock-domain row-cache copier.
    generate
        if ((READ_TEST_MODE != 0) && (READBACK_DISPLAY_MODE != 0) && (PIX_WIDTH == 16)) begin : G_TEST_LINE_RAM
            rd_fram_buf rd_test_line_ram (
                .wr_data (rd_test_data),
                .wr_addr (rd_test_data_index[8:0]),
                .wr_en   (rd_test_data_valid && (rd_test_data_index < RD_TEST_LINE_BEATS)),
                .wr_clk  (ddr_clk),
                .wr_rst  (~ddr_rstn),
                .rd_addr (rd_test_line_rd_addr),
                .rd_data (rd_test_line_ram_data),
                .rd_clk  (vout_clk),
                .rd_rst  (~vout_rstn_sync[1])
            );
        end else begin : G_NO_TEST_LINE_RAM
            assign rd_test_line_ram_data = 32'd0;
        end
    endgenerate

    // Synchronizers for reset, the row-copy request/acknowledgement toggles,
    // and the row index (which remains stable until the copy is acknowledged).
    always @(posedge vout_clk or negedge ddr_rstn) begin
        if (!ddr_rstn) begin
            vout_rstn_sync        <= 2'b00;
            row_cache_ready_sync_vout <= 2'b00;
            copy_req_sync_pix     <= 2'b00;
            copy_req_seen_pix     <= 1'b0;
            cache_fill_row_sync1  <= 5'd0;
            cache_fill_row_sync2  <= 5'd0;
            cache_fill_row_pix    <= 5'd0;
            cache_fill_src_row_sync1 <= 11'd0;
            cache_fill_src_row_sync2 <= 11'd0;
            cache_fill_src_row_pix   <= 11'd0;
            cache_row_valid       <= 32'd0;
            copy_done_toggle_pix  <= 1'b0;
            copy_active_pix       <= 1'b0;
            copy_word_index       <= 11'd0;
        end else begin
            vout_rstn_sync       <= {vout_rstn_sync[0], 1'b1};
            row_cache_ready_sync_vout <= {row_cache_ready_sync_vout[0], row_cache_all_ready};
            copy_req_sync_pix    <= {copy_req_sync_pix[0], copy_req_toggle_ddr};
            cache_fill_row_sync1 <= (DISTORTION_ENABLE != 0) ? rd_test_row_index[4:0] : {1'b0, rd_test_row_index[3:0]};
            cache_fill_row_sync2 <= cache_fill_row_sync1;
            cache_fill_src_row_sync1 <= rd_test_row_index;
            cache_fill_src_row_sync2 <= cache_fill_src_row_sync1;

            // Row tags are identities within an output-frame epoch. Drop
            // previous-frame validity at the new VS; otherwise a stale row
            // number can accidentally pass a new-frame sample check.
            if (rd_fsync && !vout_fsync_d1)
                cache_row_valid <= 32'd0;

            // Mark a slot invalid throughout the row-copy write interval.
            // Otherwise a partially overwritten slot could still carry an old
            // tag and appear valid to readers of its previous source row.
            if ((DISTORTION_ENABLE != 0) && cache_phys_wr_en &&
                (cache_phys_wr_addr == 10'd0))
                cache_row_valid[cache_phys_wr_bank] <= 1'b0;

            // Commit the row identity on the edge of its final BRAM word write.
            if ((DISTORTION_ENABLE != 0) && cache_phys_wr_last) begin
                cache_row_tag[cache_phys_wr_bank] <= cache_phys_wr_src_row;
                cache_row_valid[cache_phys_wr_bank] <= 1'b1;
            end

            if (!copy_active_pix && (copy_req_sync_pix[1] != copy_req_seen_pix)) begin
                copy_req_seen_pix      <= copy_req_sync_pix[1];
                cache_fill_row_pix     <= cache_fill_row_sync2;
                cache_fill_src_row_pix <= cache_fill_src_row_sync2;
                copy_word_index        <= 11'd0;
                copy_active_pix        <= 1'b1;
            end else if (copy_active_pix) begin
                if (copy_word_index == 11'd960) begin
                    copy_active_pix <= 1'b0;
                    // In distortion mode the last word is still in the added
                    // write-data register. Acknowledge ONLY after its BRAM edge.
                    if (DISTORTION_ENABLE == 0)
                        copy_done_toggle_pix <= ~copy_done_toggle_pix;
                end else begin
                    copy_word_index <= copy_word_index + 1'b1;
                end
            end
            // At this edge the last staged word is written into the cache RAM.
            // The DDR reader can reuse the staging RAM only after this ack.
            if ((DISTORTION_ENABLE != 0) && cache_phys_wr_last)
                copy_done_toggle_pix <= ~copy_done_toggle_pix;
        end
    end

    // Diagnostic counter: does not gate or modify the pixel stream.
    always @(posedge vout_clk or negedge ddr_rstn) begin
        if (!ddr_rstn) begin
            dist_cache_tag_miss_count <= 32'd0;
            dist_cache_tag_miss_sticky <= 1'b0;
        end else if (dbg_dist_cache_tag_miss) begin
            if (dist_cache_tag_miss_count != 32'hFFFF_FFFF)
                dist_cache_tag_miss_count <= dist_cache_tag_miss_count + 1'b1;
            dist_cache_tag_miss_sticky <= 1'b1;
        end
    end

    (* keep = "true" *) wire [31:0] dbg_dist_cache_tag_miss_count = dist_cache_tag_miss_count;
    (* keep = "true" *) wire        dbg_dist_cache_tag_miss_sticky = dist_cache_tag_miss_sticky;
    (* keep = "true" *) wire [10:0] dbg_row_scheduler_next = rd_test_row_index;
    (* keep = "true" *) wire [2:0]  dbg_row_scheduler_state = row_state;
    (* keep = "true" *) wire        dbg_copy_active = copy_active_pix;
    (* keep = "true" *) wire [10:0] dbg_copy_word_index = copy_word_index;

    // rd_fram_buf has synchronous read latency. At copy_word_index=1 the
    // staging RAM output holds word 0, so cache address = index-1. Index 960
    // flushes the final staged word (address 959).
    assign rd_test_line_rd_addr = copy_active_pix ?
                                  ((copy_word_index < 11'd960) ? {1'b0, copy_word_index} : 12'd959) : 12'd0;
    assign cache_word_wr_en = copy_active_pix && (copy_word_index >= 11'd1) &&
                              (copy_word_index <= 11'd960);
    assign cache_word_wr_addr = copy_word_index[9:0] - 10'd1;

    // Timing cut for BRAM -> BRAM copy path.
    // The 256-to-32 staging RAM drives up to four small groups of flip-flops,
    // not all 64 cache RAM write-data pins. Each group's registered data drives
    // 8 source-row slots (16 physical RAMs: even/odd). Copy word, address, row
    // tag and bank are captured together. Non-distortion mode is unchanged.
    genvar cache_write_group;
    generate
        if (DISTORTION_ENABLE != 0) begin : G_CACHE_WRITE_PIPE
            reg        en_q       = 1'b0;
            reg        last_q     = 1'b0;
            reg [9:0]  addr_q     = 10'd0;
            reg [4:0]  bank_q     = 5'd0;
            reg [10:0] src_row_q  = 11'd0;
            always @(posedge vout_clk or negedge ddr_rstn) begin
                if (!ddr_rstn) begin
                    en_q      <= 1'b0;
                    last_q    <= 1'b0;
                    addr_q    <= 10'd0;
                    bank_q    <= 5'd0;
                    src_row_q <= 11'd0;
                end else begin
                    en_q   <= cache_word_wr_en;
                    last_q <= cache_word_wr_en && (copy_word_index == 11'd960);
                    if (cache_word_wr_en) begin
                        addr_q    <= cache_word_wr_addr;
                        bank_q    <= cache_fill_row_pix;
                        src_row_q <= cache_fill_src_row_pix;
                    end
                end
            end
            assign cache_phys_wr_en      = en_q;
            assign cache_phys_wr_last    = last_q;
            assign cache_phys_wr_addr    = addr_q;
            assign cache_phys_wr_bank    = bank_q;
            assign cache_phys_wr_src_row = src_row_q;
            for (cache_write_group = 0; cache_write_group < 4;
                 cache_write_group = cache_write_group + 1) begin : G_WRITE_GROUP
                // Deliberately registered locally: each group fans out to
                // eight slots rather than one staging BRAM feeding all 32.
                (* keep = "true" *) reg [31:0] wr_data_q;
                always @(posedge vout_clk) begin
                    if (cache_word_wr_en &&
                        (cache_fill_row_pix[4:3] == cache_write_group))
                        wr_data_q <= rd_test_line_ram_data;
                end
                assign cache_phys_wr_data[cache_write_group] = wr_data_q;
            end
        end else begin : G_CACHE_WRITE_LEGACY
            assign cache_phys_wr_en      = cache_word_wr_en;
            assign cache_phys_wr_last    = cache_word_wr_en && (copy_word_index == 11'd960);
            assign cache_phys_wr_addr    = cache_word_wr_addr;
            assign cache_phys_wr_bank    = cache_fill_row_pix;
            assign cache_phys_wr_src_row = cache_fill_src_row_pix;
            for (cache_write_group = 0; cache_write_group < 4;
                 cache_write_group = cache_write_group + 1) begin : G_WRITE_DIRECT
                assign cache_phys_wr_data[cache_write_group] = rd_test_line_ram_data;
            end
        end
    endgenerate

    // ★ 修改：row_cache_read_addr 改为延迟一拍输出
    // Count output coordinates and publish one stable row index for each line
    // start. The toggle avoids losing narrow pulses when crossing to ddr_clk.
    always @(posedge vout_clk or negedge ddr_rstn) begin
        if (!ddr_rstn) begin
            vout_x_count          <= 11'd0;
            vout_y_count          <= 12'd0;
            vout_rd_en_d1         <= 1'b0;
            vout_fsync_d1          <= 1'b0;
            vout_line_seen        <= 1'b0;
            vout_y_bank_d1        <= 5'd0;
            vout_x_lsb_d1         <= 1'b0;
            row_cache_read_addr_d1<= 12'd0;
            vout_x_lsb_d2         <= 1'b0;
            line_start_toggle_vout<= 1'b0;
            line_start_row_vout   <= 11'd0;
            frame_start_toggle_vout <= 1'b0;
            dist_bank0_d1 <= 5'd0;
            dist_bank1_d1 <= 5'd1;
            dist_u_lsb_d1 <= 1'b0;
            dist_bounds_d1 <= 1'b0;
            dist_valid_d1 <= 1'b0;
            dist_tags_match_d1 <= 1'b0;
            dist_tags_match_d2 <= 1'b0;
            dist_tag_diag_color_d1 <= 16'hF81F;
            dist_tag_diag_color_d2 <= 16'hF81F;
            dist_progress_color_d1 <= 16'h001F;
            dist_progress_color_d2 <= 16'h001F;
            dist_progress_strip_d1 <= 1'b0;
            dist_progress_strip_d2 <= 1'b0;
        end else if (!vout_rstn_sync[1]) begin
            vout_x_count          <= 11'd0;
            vout_y_count          <= 12'd0;
            vout_rd_en_d1         <= 1'b0;
            vout_fsync_d1          <= rd_fsync;
            vout_line_seen        <= 1'b0;
            vout_y_bank_d1        <= 5'd0;
            vout_x_lsb_d1         <= 1'b0;
            row_cache_read_addr_d1<= 12'd0;
            vout_x_lsb_d2         <= 1'b0;
            line_start_toggle_vout<= 1'b0;
            line_start_row_vout   <= 11'd0;
            frame_start_toggle_vout <= 1'b0;
            dist_bank0_d1 <= 5'd0;
            dist_bank1_d1 <= 5'd1;
            dist_u_lsb_d1 <= 1'b0;
            dist_bounds_d1 <= 1'b0;
            dist_valid_d1 <= 1'b0;
            dist_tags_match_d1 <= 1'b0;
            dist_tags_match_d2 <= 1'b0;
            dist_tag_diag_color_d1 <= 16'hF81F;
            dist_tag_diag_color_d2 <= 16'hF81F;
            dist_progress_color_d1 <= 16'h001F;
            dist_progress_color_d2 <= 16'h001F;
            dist_progress_strip_d1 <= 1'b0;
            dist_progress_strip_d2 <= 1'b0;
        end else begin
            dist_bank0_d1 <= safe_dist_v_int[4:0];
            dist_bank1_d1 <= dist_row_slot1_now;
            dist_u_lsb_d1 <= safe_dist_u_int[0];
            dist_bounds_d1 <= request_geometry_ok && row_cache_ready_sync_vout[1];
            dist_valid_d1 <= dist_rd_en;
            // Sample tag identity at the same edge as the BRAM read request.
            dist_tags_match_d1 <= dbg_cache_tag_match_row0 && dbg_cache_tag_match_row1 &&
                !(cache_phys_wr_en && ((cache_phys_wr_bank==dist_row_slot0_now) ||
                                          (cache_phys_wr_bank==dist_row_slot1_now)));
            dist_tags_match_d2 <= dist_tags_match_d1;
            dist_tag_diag_color_d1 <= dist_tag_diag_color_now;
            dist_tag_diag_color_d2 <= dist_tag_diag_color_d1;
            dist_progress_color_d1 <= diag_progress_color_now;
            dist_progress_color_d2 <= dist_progress_color_d1;
            dist_progress_strip_d1 <= (safe_dist_u_int < 16'd48);
            dist_progress_strip_d2 <= dist_progress_strip_d1;
            vout_rd_en_d1 <= rd_en;
            vout_fsync_d1 <= rd_fsync;

            row_cache_read_addr_d1 <= {1'b0, vout_x_count[10:1]};
            vout_x_lsb_d2 <= vout_x_lsb_d1;

            // sync_vg asserts VS at the beginning of the vertical blanking
            // interval. This event starts the next frame's cache prefill.
            if (rd_fsync && !vout_fsync_d1)
                frame_start_toggle_vout <= ~frame_start_toggle_vout;

            if (rd_fsync != vout_fsync_d1) begin
                vout_x_count   <= 11'd0;
                vout_y_count   <= 12'd0;
                vout_line_seen <= 1'b0;
                vout_y_bank_d1 <= 5'd0;
                vout_x_lsb_d1  <= 1'b0;
            end else if (rd_en && !vout_rd_en_d1) begin
                vout_x_count <= 11'd0;
                line_start_toggle_vout <= ~line_start_toggle_vout;
                if (!vout_line_seen) begin
                    vout_y_count        <= 12'd0;
                    line_start_row_vout <= 11'd0;
                    vout_line_seen      <= 1'b1;
                    vout_y_bank_d1      <= 5'd0;
                end else if (vout_y_count >= V_NUM-1) begin
                    vout_y_count        <= 12'd0;
                    line_start_row_vout <= 11'd0;
                    vout_y_bank_d1      <= 5'd0;
                end else begin
                    vout_y_count        <= vout_y_count + 1'b1;
                    line_start_row_vout <= vout_y_count[10:0] + 11'd1;
                    vout_y_bank_d1      <= (DISTORTION_ENABLE != 0) ?
                                               (vout_y_count[4:0] + 5'd1) :
                                               {1'b0, (vout_y_count[3:0] + 4'd1)};
                end
                vout_x_lsb_d1 <= 1'b0;
            end else if (rd_en) begin
                vout_x_count   <= vout_x_count + 1'b1;
                vout_y_bank_d1 <= (DISTORTION_ENABLE != 0) ? vout_y_count[4:0] : {1'b0, vout_y_count[3:0]};
                vout_x_lsb_d1  <= vout_x_count[0];
            end else begin
                vout_x_count <= 11'd0;
            end
        end
    end

    assign row_cache_read_addr = row_cache_read_addr_d1;

    // The distortion cache uses two 16-bit pixel lanes per source row:
    // even-x pixels are taken from the high half of staged RGB565 words and
    // odd-x pixels from the low half. This makes x and x+1 available in parallel
    // using one read address per lane, avoiding a duplicated full 32-bit row RAM.
    // Timing specialization for the verified 1920-pixel / 960-word line.
    // Build parity addresses directly from RAW u. Bounds mask is applied once,
    // in parallel with increment logic, rather than clamp -> compare -> add.
    generate
      if(H_NUM == 1920 && DDR_WORDS_PER_ROW == 960)begin:G_ADDR_1080P
        wire [9:0] raw_word=dist_u_int[10:1];
        wire bump=dist_u_int[0] && (raw_word != 10'd959);
        wire [9:0] even_word;
        genvar addr_bit;
        for(addr_bit=0;addr_bit<10;addr_bit=addr_bit+1)begin:G_INC_PREFIX
          if(addr_bit==0)begin
            assign even_word[addr_bit]=raw_word[addr_bit]^bump;
          end else begin
            assign even_word[addr_bit]=raw_word[addr_bit] ^
                (bump && (&raw_word[addr_bit-1:0]));
          end
        end
        assign dist_word_addr0=request_u_ok ? raw_word : 10'd0;
        assign dist_word_addr_even=request_u_ok ? even_word : 10'd0;
      end else begin:G_ADDR_GENERIC
        assign dist_word_addr0 = (safe_dist_u_int[10:1] < DDR_WORDS_PER_ROW) ? safe_dist_u_int[10:1] : 10'd0;
        assign dist_word_addr_even = ((safe_dist_u_int[0] != 1'b0) && (dist_word_addr0 < (DDR_WORDS_PER_ROW-1))) ?
                                     (dist_word_addr0 + 10'd1) : dist_word_addr0;
      end
    endgenerate
    assign cache_rd_addr_a = (DISTORTION_ENABLE != 0) ? dist_word_addr_even : row_cache_read_addr[9:0];
    assign cache_rd_addr_b = (DISTORTION_ENABLE != 0) ? dist_word_addr0      : row_cache_read_addr[9:0];

    genvar cache_bank;
    generate
        for (cache_bank = 0; cache_bank < 32; cache_bank = cache_bank + 1) begin : G_CACHE_ROWS
            if (cache_bank < ROW_CACHE_LINES) begin : G_ACTIVE_ROW
                if ((READ_TEST_MODE != 0) && (READBACK_DISPLAY_MODE != 0) && (PIX_WIDTH == 16)) begin : G_ROW_CACHE_ACTIVE
                    if (DISTORTION_ENABLE != 0) begin : G_DISTORTION_PARITY_BANKS
                        // One 1024x16 RAM per parity lane and row slot. Each is
                        // a synchronous read/write RAM; the two lanes provide
                        // p00/p01 (and p10/p11) without duplicating whole rows.
                        line_cache_ram16 u_even_pixel_ram (
                            .clk     (vout_clk),
                            .wr_en   (cache_phys_wr_en && (cache_phys_wr_bank == cache_bank)),
                            .wr_addr (cache_phys_wr_addr),
                            .wr_data (cache_phys_wr_data[cache_bank/8][31:16]),
                            .rd_addr (cache_rd_addr_a),
                            .rd_data (line_cache_even_pixel[cache_bank])
                        );
                        line_cache_ram16 u_odd_pixel_ram (
                            .clk     (vout_clk),
                            .wr_en   (cache_phys_wr_en && (cache_phys_wr_bank == cache_bank)),
                            .wr_addr (cache_phys_wr_addr),
                            .wr_data (cache_phys_wr_data[cache_bank/8][15:0]),
                            .rd_addr (cache_rd_addr_b),
                            .rd_data (line_cache_odd_pixel[cache_bank])
                        );
                        assign line_cache_word[cache_bank] = 32'd0;
                    end else begin : G_NORMAL_32BIT_ROW
                        line_cache_ram32 u_line_cache_ram (
                            .clk     (vout_clk),
                            .wr_en   (cache_phys_wr_en && (cache_phys_wr_bank == cache_bank)),
                            .wr_addr (cache_phys_wr_addr),
                            .wr_data (cache_phys_wr_data[cache_bank/8]),
                            .rd_addr (row_cache_read_addr[9:0]),
                            .rd_data (line_cache_word[cache_bank])
                        );
                        assign line_cache_even_pixel[cache_bank] = 16'd0;
                        assign line_cache_odd_pixel[cache_bank]  = 16'd0;
                    end
                end else begin : G_NO_ACTIVE_RAM
                    assign line_cache_word[cache_bank] = 32'd0;
                    assign line_cache_even_pixel[cache_bank] = 16'd0;
                    assign line_cache_odd_pixel[cache_bank]  = 16'd0;
                end
            end else begin : G_UNUSED_ROW
                assign line_cache_word[cache_bank] = 32'd0;
                assign line_cache_even_pixel[cache_bank] = 16'd0;
                assign line_cache_odd_pixel[cache_bank]  = 16'd0;
            end
        end
    endgenerate

    // Distortion-only timing cut: the 32-row read mux is factored into
    // four 8:1 selectors (registered), followed by a 4:1 selector in the
    // NEXT clock cycle. This avoids a combinational 32:1 mux between the
    // 1024x16 synchronous BRAM outputs and bilinear_interp_rgb565 stage 1.
    //
    // BRAM read for coordinate N and dist_bank*_d1 are latched together
    // at clock A. At clock B the selected group of 8 rows is registered;
    // the remaining 4:1 selection and interpolation stage 1 occur at C.
    // Pixel parity, in_bounds, valid, and top-level wx/wy all receive
    // exactly one matching extra cycle of delay.
    wire [15:0] even_row0_group [0:3];
    wire [15:0] odd_row0_group  [0:3];
    wire [15:0] even_row1_group [0:3];
    wire [15:0] odd_row1_group  [0:3];

    genvar mux_group;
    generate
        for (mux_group = 0; mux_group < 4; mux_group = mux_group + 1) begin : G_DIST_CACHE_GROUP
            reg [15:0] even0_mux, odd0_mux, even1_mux, odd1_mux;
            reg [15:0] even0_q, odd0_q, even1_q, odd1_q;
            always @* begin
                // The mux_group generate constant confines each select to
                // a physical subgroup of only eight row-cache memories.
                case (dist_bank0_d1[2:0])
                    3'd0: begin even0_mux = line_cache_even_pixel[mux_group*8+0]; odd0_mux = line_cache_odd_pixel[mux_group*8+0]; end
                    3'd1: begin even0_mux = line_cache_even_pixel[mux_group*8+1]; odd0_mux = line_cache_odd_pixel[mux_group*8+1]; end
                    3'd2: begin even0_mux = line_cache_even_pixel[mux_group*8+2]; odd0_mux = line_cache_odd_pixel[mux_group*8+2]; end
                    3'd3: begin even0_mux = line_cache_even_pixel[mux_group*8+3]; odd0_mux = line_cache_odd_pixel[mux_group*8+3]; end
                    3'd4: begin even0_mux = line_cache_even_pixel[mux_group*8+4]; odd0_mux = line_cache_odd_pixel[mux_group*8+4]; end
                    3'd5: begin even0_mux = line_cache_even_pixel[mux_group*8+5]; odd0_mux = line_cache_odd_pixel[mux_group*8+5]; end
                    3'd6: begin even0_mux = line_cache_even_pixel[mux_group*8+6]; odd0_mux = line_cache_odd_pixel[mux_group*8+6]; end
                    3'd7: begin even0_mux = line_cache_even_pixel[mux_group*8+7]; odd0_mux = line_cache_odd_pixel[mux_group*8+7]; end
                    default: begin even0_mux = 16'd0; odd0_mux = 16'd0; end
                endcase
                case (dist_bank1_d1[2:0])
                    3'd0: begin even1_mux = line_cache_even_pixel[mux_group*8+0]; odd1_mux = line_cache_odd_pixel[mux_group*8+0]; end
                    3'd1: begin even1_mux = line_cache_even_pixel[mux_group*8+1]; odd1_mux = line_cache_odd_pixel[mux_group*8+1]; end
                    3'd2: begin even1_mux = line_cache_even_pixel[mux_group*8+2]; odd1_mux = line_cache_odd_pixel[mux_group*8+2]; end
                    3'd3: begin even1_mux = line_cache_even_pixel[mux_group*8+3]; odd1_mux = line_cache_odd_pixel[mux_group*8+3]; end
                    3'd4: begin even1_mux = line_cache_even_pixel[mux_group*8+4]; odd1_mux = line_cache_odd_pixel[mux_group*8+4]; end
                    3'd5: begin even1_mux = line_cache_even_pixel[mux_group*8+5]; odd1_mux = line_cache_odd_pixel[mux_group*8+5]; end
                    3'd6: begin even1_mux = line_cache_even_pixel[mux_group*8+6]; odd1_mux = line_cache_odd_pixel[mux_group*8+6]; end
                    3'd7: begin even1_mux = line_cache_even_pixel[mux_group*8+7]; odd1_mux = line_cache_odd_pixel[mux_group*8+7]; end
                    default: begin even1_mux = 16'd0; odd1_mux = 16'd0; end
                endcase
            end
            always @(posedge vout_clk or negedge ddr_rstn) begin
                if (!ddr_rstn) begin
                    even0_q <= 16'd0;
                    odd0_q  <= 16'd0;
                    even1_q <= 16'd0;
                    odd1_q  <= 16'd0;
                end else begin
                    even0_q <= even0_mux;
                    odd0_q  <= odd0_mux;
                    even1_q <= even1_mux;
                    odd1_q  <= odd1_mux;
                end
            end
            assign even_row0_group[mux_group] = even0_q;
            assign odd_row0_group[mux_group]  = odd0_q;
            assign even_row1_group[mux_group] = even1_q;
            assign odd_row1_group[mux_group]  = odd1_q;
        end
    endgenerate

    reg dist_last_col_d1=0,dist_last_col_d2=0;
    always @(posedge vout_clk or negedge ddr_rstn)begin
        if(!ddr_rstn)begin dist_last_col_d1<=0;dist_last_col_d2<=0;end
        else begin dist_last_col_d1<=(safe_dist_u_int==H_NUM-1);dist_last_col_d2<=dist_last_col_d1;end
    end
    reg [1:0] dist_bank0_hi_d2 = 2'd0;
    reg [1:0] dist_bank1_hi_d2 = 2'd0;
    reg       dist_u_lsb_d2    = 1'b0;
    reg       dist_bounds_d2   = 1'b0;
    reg       dist_valid_d2    = 1'b0;
    always @(posedge vout_clk or negedge ddr_rstn) begin
        if (!ddr_rstn) begin
            dist_bank0_hi_d2 <= 2'd0;
            dist_bank1_hi_d2 <= 2'd0;
            dist_u_lsb_d2 <= 1'b0;
            dist_bounds_d2 <= 1'b0;
            dist_valid_d2 <= 1'b0;
        end else if (!vout_rstn_sync[1]) begin
            dist_bank0_hi_d2 <= 2'd0;
            dist_bank1_hi_d2 <= 2'd0;
            dist_u_lsb_d2 <= 1'b0;
            dist_bounds_d2 <= 1'b0;
            dist_valid_d2 <= 1'b0;
        end else begin
            dist_bank0_hi_d2 <= dist_bank0_d1[4:3];
            dist_bank1_hi_d2 <= dist_bank1_d1[4:3];
            dist_u_lsb_d2 <= dist_u_lsb_d1;
            dist_bounds_d2 <= dist_bounds_d1;
            dist_valid_d2 <= dist_valid_d1;
        end
    end

    wire [15:0] dist_even_row0 = even_row0_group[dist_bank0_hi_d2];
    wire [15:0] dist_odd_row0  = odd_row0_group[dist_bank0_hi_d2];
    wire [15:0] dist_even_row1 = even_row1_group[dist_bank1_hi_d2];
    wire [15:0] dist_odd_row1  = odd_row1_group[dist_bank1_hi_d2];

    assign dist_sample_valid = (DISTORTION_ENABLE != 0) ? dist_valid_d2 : 1'b0;
    // Visual-only diagnostic. With in-bounds source coordinates, color all
    // four interpolation inputs by the type of row tag mismatch.
    // The interpolated output therefore appears magenta at the defective
    // locations. When disabled, the original data route is bit-for-bit intact.
    wire dist_show_cache_miss = (DIST_CACHE_MISS_VISUAL != 0) &&
        (DISTORTION_ENABLE != 0) && dist_valid_d2 && dist_bounds_d2 &&
        !dist_tags_match_d2;
    // even lane address is k+u[0], odd lane address is k.
    // If u is even: p00=even[k], p01=odd[k]. If u is odd: p00=odd[k], p01=even[k+1].
    wire dist_show_progress = (DIST_PREFETCH_PROGRESS_VISUAL != 0) &&
                              (DISTORTION_ENABLE != 0) &&
                              dist_valid_d2 && dist_bounds_d2 &&
                              dist_progress_strip_d2;
    assign dist_p00 = (DISTORTION_ENABLE && dist_bounds_d2 && dist_tags_match_d2) ?
                      (dist_show_progress ? dist_progress_color_d2 :
                       dist_show_cache_miss ? dist_tag_diag_color_d2 : (dist_u_lsb_d2 ? dist_odd_row0 : dist_even_row0)) : 16'd0;
    assign dist_p01 = (DISTORTION_ENABLE && dist_bounds_d2 && dist_tags_match_d2) ?
                      (dist_show_progress ? dist_progress_color_d2 :
                       dist_show_cache_miss ? dist_tag_diag_color_d2 : (dist_last_col_d2 ? dist_odd_row0 : (dist_u_lsb_d2 ? dist_even_row0 : dist_odd_row0))) : 16'd0;
    assign dist_p10 = (DISTORTION_ENABLE && dist_bounds_d2 && dist_tags_match_d2) ?
                      (dist_show_progress ? dist_progress_color_d2 :
                       dist_show_cache_miss ? dist_tag_diag_color_d2 : (dist_u_lsb_d2 ? dist_odd_row1 : dist_even_row1)) : 16'd0;
    assign dist_p11 = (DISTORTION_ENABLE && dist_bounds_d2 && dist_tags_match_d2) ?
                      (dist_show_progress ? dist_progress_color_d2 :
                       dist_show_cache_miss ? dist_tag_diag_color_d2 : (dist_last_col_d2 ? dist_odd_row1 : (dist_u_lsb_d2 ? dist_even_row1 : dist_odd_row1))) : 16'd0;

    always @* begin
        case (vout_y_bank_d1)
            5'd0:  line_cache_selected_word = line_cache_word[0];
            5'd1:  line_cache_selected_word = line_cache_word[1];
            5'd2:  line_cache_selected_word = line_cache_word[2];
            5'd3:  line_cache_selected_word = line_cache_word[3];
            5'd4:  line_cache_selected_word = line_cache_word[4];
            5'd5:  line_cache_selected_word = line_cache_word[5];
            5'd6:  line_cache_selected_word = line_cache_word[6];
            5'd7:  line_cache_selected_word = line_cache_word[7];
            5'd8:  line_cache_selected_word = line_cache_word[8];
            5'd9:  line_cache_selected_word = line_cache_word[9];
            5'd10:  line_cache_selected_word = line_cache_word[10];
            5'd11:  line_cache_selected_word = line_cache_word[11];
            5'd12:  line_cache_selected_word = line_cache_word[12];
            5'd13:  line_cache_selected_word = line_cache_word[13];
            5'd14:  line_cache_selected_word = line_cache_word[14];
            5'd15:  line_cache_selected_word = line_cache_word[15];
            5'd16:  line_cache_selected_word = line_cache_word[16];
            5'd17:  line_cache_selected_word = line_cache_word[17];
            5'd18:  line_cache_selected_word = line_cache_word[18];
            5'd19:  line_cache_selected_word = line_cache_word[19];
            5'd20:  line_cache_selected_word = line_cache_word[20];
            5'd21:  line_cache_selected_word = line_cache_word[21];
            5'd22:  line_cache_selected_word = line_cache_word[22];
            5'd23:  line_cache_selected_word = line_cache_word[23];
            5'd24:  line_cache_selected_word = line_cache_word[24];
            5'd25:  line_cache_selected_word = line_cache_word[25];
            5'd26:  line_cache_selected_word = line_cache_word[26];
            5'd27:  line_cache_selected_word = line_cache_word[27];
            5'd28:  line_cache_selected_word = line_cache_word[28];
            5'd29:  line_cache_selected_word = line_cache_word[29];
            5'd30:  line_cache_selected_word = line_cache_word[30];
            5'd31:  line_cache_selected_word = line_cache_word[31];
            default: line_cache_selected_word = 32'd0;
        endcase
    end

    // ★ 修改：像素选择改用 vout_x_lsb_d2
    assign rd_test_line_pixel = vout_x_lsb_d2 ?
                                line_cache_selected_word[15:0] :
                                line_cache_selected_word[31:16];

    generate
        if ((READ_TEST_MODE != 0) && (READBACK_DISPLAY_MODE != 0) && (PIX_WIDTH == 16)) begin : G_VOUT_ROW_CACHE
            assign vout_data = row_cache_ready_sync_vout[1] ? rd_test_line_pixel : 16'd0;
        end else begin : G_VOUT_LEGACY
            assign vout_data = legacy_vout_data;
        end
    endgenerate

    wr_rd_ctrl_top#(
        .CTRL_ADDR_WIDTH  (  CTRL_ADDR_WIDTH  ),//parameter                    CTRL_ADDR_WIDTH      = 28,
        .MEM_DQ_WIDTH     (  MEM_DQ_WIDTH     ) //parameter                    MEM_DQ_WIDTH         = 16
    )wr_rd_ctrl_top (                         
        .clk              (  ddr_clk          ),//input                        clk            ,            
        .rstn             (  ddr_rstn         ),//input                        rstn           ,            
                                              
        .wr_cmd_en        (  ddr_wreq         ),//input                        wr_cmd_en   ,
        .wr_cmd_addr      (  ddr_waddr        ),//input  [CTRL_ADDR_WIDTH-1:0] wr_cmd_addr ,
        .wr_cmd_len       (  ddr_wr_len       ),//input  [31��0]               wr_cmd_len  ,
        .wr_cmd_ready     (  ddr_wrdy         ),//output                       wr_cmd_ready,
        .wr_cmd_done      (  ddr_wdone        ),//output                       wr_cmd_done,
        .wr_bac           (  ddr_wr_bac       ),//output                       wr_bac,                                     
        .wr_ctrl_data     (  ddr_wdata        ),//input  [MEM_DQ_WIDTH*8-1:0]  wr_ctrl_data,
        .wr_data_re       (  ddr_wdata_req    ),//output                       wr_data_re  ,
                                              
        .rd_cmd_en        (  rd_cmd_en        ),//input                        rd_cmd_en   ,
        .rd_cmd_addr      (  rd_cmd_addr      ),//input  [CTRL_ADDR_WIDTH-1:0] rd_cmd_addr ,
        .rd_cmd_len       (  rd_cmd_len       ),//input  [31��0]               rd_cmd_len  ,
        .rd_cmd_ready     (  rd_cmd_ready     ),//output                       rd_cmd_ready, 
        .rd_cmd_done      (  rd_cmd_done      ),//output                       rd_cmd_done,
                                              
        .read_ready       (  read_ready       ),//input                        read_ready  ,    
        .read_rdata       (  read_rdata       ),//output [MEM_DQ_WIDTH*8-1:0]  read_rdata  ,    
        .read_en          (  read_en          ),//output                       read_en     ,                                          
        // write channel                        
        .axi_awaddr       (  axi_awaddr       ),//output [CTRL_ADDR_WIDTH-1:0] axi_awaddr     ,  
        .axi_awid         (  axi_awid         ),//output [3:0]                 axi_awid       ,
        .axi_awlen        (  axi_awlen        ),//output [3:0]                 axi_awlen      ,
        .axi_awsize       (  axi_awsize       ),//output [2:0]                 axi_awsize     ,
        .axi_awburst      (  axi_awburst      ),//output [1:0]                 axi_awburst    , //only support 2'b01: INCR
        .axi_awready      (  axi_awready      ),//input                        axi_awready    ,
        .axi_awvalid      (  axi_awvalid      ),//output                       axi_awvalid    ,
                                              
        .axi_wdata        (  axi_wdata        ),//output [MEM_DQ_WIDTH*8-1:0]  axi_wdata      ,
        .axi_wstrb        (  axi_wstrb        ),//output [MEM_DQ_WIDTH -1 :0]  axi_wstrb      ,
        .axi_wlast        (  axi_wlast        ),//output                       axi_wlast      ,
        .axi_wvalid       (  axi_wvalid       ),//output                       axi_wvalid     ,
        .axi_wready       (  axi_wready       ),//input                        axi_wready     ,
        .axi_bid          (  4'd0             ),//input  [3 : 0]               axi_bid        , // Master Interface Write Response.
        .axi_bresp        (  2'd0             ),//input  [1 : 0]               axi_bresp      , // Write response. This signal indicates the status of the write transaction.
        .axi_bvalid       (  1'b0             ),//input                        axi_bvalid     , // Write response valid. This signal indicates that the channel is signaling a valid write response.
        .axi_bready       (                   ),//output                       axi_bready     ,
                                              
        // read channel                          
        .axi_araddr       (  axi_araddr       ),//output [CTRL_ADDR_WIDTH-1:0] axi_araddr     ,    
        .axi_arid         (  axi_arid         ),//output [3:0]                 axi_arid       ,
        .axi_arlen        (  axi_arlen        ),//output [3:0]                 axi_arlen      ,
        .axi_arsize       (  axi_arsize       ),//output [2:0]                 axi_arsize     ,
        .axi_arburst      (  axi_arburst      ),//output [1:0]                 axi_arburst    ,
        .axi_arvalid      (  axi_arvalid      ),//output                       axi_arvalid    , 
        .axi_arready      (  axi_arready      ),//input                        axi_arready    , //only support 2'b01: INCR
                                              
        .axi_rready       (  axi_rready       ),//output                       axi_rready     ,
        .axi_rdata        (  axi_rdata        ),//input  [MEM_DQ_WIDTH*8-1:0]  axi_rdata      ,
        .axi_rvalid       (  axi_rvalid       ),//input                        axi_rvalid     ,
        .axi_rlast        (  axi_rlast        ),//input                        axi_rlast      ,
        .axi_rid          (  axi_rid          ),//input  [3:0]                 axi_rid        ,
        .axi_rresp        (  2'd0             ) //input  [1:0]                 axi_rresp      
    );


endmodule
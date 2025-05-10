//-----------------------------------------------------------------------------
// scalar_engine.v
//-----------------------------------------------------------------------------
// Scalar SIMD Engine with AXI4-Full (1024-bit) slave interface,
// input/output FIFOs, and 32-bit scalar ALUs supporting multiple precisions:
//   0: FP32 (32-bit)
//   1: FP16 (16-bit)
//   2: INT8 (8-bit)
//   3: INT4 (4-bit)
// Uses Synopsys DesignWare FP IP for floating-point operations.
//-----------------------------------------------------------------------------
module scalar_engine #(
    parameter ADDR_WIDTH  = 32,
    parameter DATA_WIDTH  = 1024,
    parameter ID_WIDTH    = 4,
    parameter LEN_WIDTH   = 8,
    parameter FIFO_DEPTH  = 16
)(
    input  wire                       clk,
    input  wire                       rst_n,
    // AXI4-Full slave write address channel
    input  wire [ID_WIDTH-1:0]        s_awid,
    input  wire [ADDR_WIDTH-1:0]      s_awaddr,
    input  wire [1:0]                 s_awburst,
    input  wire [LEN_WIDTH-1:0]       s_awlen,
    input  wire [2:0]                 s_awsize,
    input  wire                       s_awvalid,
    output reg                        s_awready,
    // AXI4-Full slave write data channel
    input  wire [DATA_WIDTH-1:0]      s_wdata,
    input  wire [DATA_WIDTH/8-1:0]    s_wstrb,
    input  wire                       s_wlast,
    input  wire                       s_wvalid,
    output reg                        s_wready,
    // AXI4-Full slave write response channel
    output reg  [ID_WIDTH-1:0]        s_bid,
    output reg  [1:0]                 s_bresp,
    output reg                        s_bvalid,
    input  wire                       s_bready,
    // AXI4-Full slave read address channel
    input  wire [ID_WIDTH-1:0]        s_arid,
    input  wire [ADDR_WIDTH-1:0]      s_araddr,
    input  wire [1:0]                 s_arburst,
    input  wire [LEN_WIDTH-1:0]       s_arlen,
    input  wire [2:0]                 s_arsize,
    input  wire                       s_arvalid,
    output reg                        s_arready,
    // AXI4-Full slave read data channel
    output reg  [ID_WIDTH-1:0]        s_rid,
    output reg  [DATA_WIDTH-1:0]      s_rdata,
    output reg  [1:0]                 s_rresp,
    output reg                        s_rlast,
    output reg                        s_rvalid,
    input  wire                       s_rready
);

  // Derived constants
  localparam LANES_FP32 = DATA_WIDTH/32;
  localparam LANES_FP16 = DATA_WIDTH/16;
  localparam LANES_INT8 = DATA_WIDTH/8;
  localparam LANES_INT4 = DATA_WIDTH/4;
  localparam PTR_WIDTH  = $clog2(FIFO_DEPTH);

  // Control registers
  reg [1:0]  precision;  // 0=FP32,1=FP16,2=INT8,3=INT4
  reg [3:0]  opcode;     // 0:add,1:sub,2:mul,3:div
  reg        start;
  reg        done;

  // Input FIFOs for operand A and B
  reg [DATA_WIDTH-1:0] in_fifo_a [0:FIFO_DEPTH-1];
  reg [DATA_WIDTH-1:0] in_fifo_b [0:FIFO_DEPTH-1];
  reg [PTR_WIDTH-1:0]  wr_a, rd_a, wr_b, rd_b, wr_o, rd_o;
  reg [PTR_WIDTH:0]    cnt_a, cnt_b, cnt_o;

  // Output FIFO
  reg [DATA_WIDTH-1:0] out_fifo [0:FIFO_DEPTH-1];
  reg [PTR_WIDTH-1:0]  out_wr, out_rd;
  reg [PTR_WIDTH:0]    out_count;

  // AXI transaction state
  reg aw_active, ar_active;
  reg [ID_WIDTH-1:0]   awid_reg, arid_reg;
  reg [ADDR_WIDTH-1:0] awaddr_reg, araddr_reg;

  //-------------------------------------------------------------------------
  // SIMD lanes and DW IP instances
  genvar i;
  // FP32
  wire [31:0] fp32_a [0:LANES_FP32-1];
  wire [31:0] fp32_b [0:LANES_FP32-1];
  wire [31:0] fp32_add [0:LANES_FP32-1];
  wire [31:0] fp32_sub [0:LANES_FP32-1];
  wire [31:0] fp32_mul [0:LANES_FP32-1];  
  wire [31:0] fp32_div [0:LANES_FP32-1];
  wire [31:0] fp32_out [0:LANES_FP32-1];
  generate
    for (i=0; i<LANES_FP32; i=i+1) begin : fp32_pack
      assign fp32_a[i] = in_fifo_a[in_rd_a][32*i +:32];
      assign fp32_b[i] = in_fifo_b[in_rd_b][32*i +:32];
      DW_fp_add  #(.exp_width(8), .sig_width(23)) u_fp32_add(.a(fp32_a[i]), .b(fp32_b[i]), .rnd(3'b000), .z(fp32_add[i]), .status());
      DW_fp_sub  #(.exp_width(8), .sig_width(23)) u_fp32_sub(.a(fp32_a[i]), .b(fp32_b[i]), .rnd(3'b000), .z(fp32_sub[i]), .status());
      DW_fp_mult #(.exp_width(8), .sig_width(23)) u_fp32_mul(.a(fp32_a[i]), .b(fp32_b[i]), .rnd(3'b000), .z(fp32_mul[i]), .status());
      DW_fp_div  #(.exp_width(8), .sig_width(23)) u_fp32_div(.a(fp32_a[i]), .b(fp32_b[i]), .rnd(3'b000), .z(fp32_div[i]), .status());
      assign fp32_out[i] = (opcode==0) ? fp32_add[i] :
                           (opcode==1) ? fp32_sub[i] :
                           (opcode==2) ? fp32_mul[i] :
                           (opcode==3) ? fp32_div[i] : 32'd0;
    end
  endgenerate

  // FP16
  wire [15:0] fp16_a [0:LANES_FP16-1];
  wire [15:0] fp16_b [0:LANES_FP16-1];
  wire [15:0] fp16_add [0:LANES_FP16-1];
  wire [15:0] fp16_sub [0:LANES_FP16-1];
  wire [15:0] fp16_mul [0:LANES_FP16-1];
  wire [15:0] fp16_div [0:LANES_FP16-1];
  wire [15:0] fp16_out [0:LANES_FP16-1];
  generate
    for (i=0; i<LANES_FP16; i=i+1) begin : fp16_pack
      assign fp16_a[i] = in_fifo_a[in_rd_a][16*i +:16];
      assign fp16_b[i] = in_fifo_b[in_rd_b][16*i +:16];
      DW_fp_add  #(.exp_width(5), .sig_width(10)) u_fp16_add(.a(fp16_a[i]), .b(fp16_b[i]), .rnd(3'b000), .z(fp16_add[i]), .status());
      DW_fp_sub  #(.exp_width(5), .sig_width(10)) u_fp16_sub(.a(fp16_a[i]), .b(fp16_b[i]), .rnd(3'b000), .z(fp16_sub[i]), .status());
      DW_fp_mult #(.exp_width(5), .sig_width(10)) u_fp16_mul(.a(fp16_a[i]), .b(fp16_b[i]), .rnd(3'b000), .z(fp16_mul[i]), .status());
      DW_fp_div  #(.exp_width(5), .sig_width(10)) u_fp16_div(.a(fp16_a[i]), .b(fp16_b[i]), .rnd(3'b000), .z(fp16_div[i]), .status());
      assign fp16_out[i] = (opcode==0) ? fp16_add[i] :
                           (opcode==1) ? fp16_sub[i] : 16'd0;
    end
  endgenerate

  // INT8
  wire signed [7:0] int8_a [0:LANES_INT8-1];
  wire signed [7:0] int8_b [0:LANES_INT8-1];
  wire signed [7:0] int8_out[0:LANES_INT8-1];
  generate
    for (i=0; i<LANES_INT8; i=i+1) begin : int8_pack
      assign int8_a[i] = in_fifo_a[in_rd_a][8*i +:8];
      assign int8_b[i] = in_fifo_b[in_rd_b][8*i +:8];
      assign int8_out[i] = (opcode==0) ? int8_a[i]+int8_b[i] :
                           (opcode==1) ? int8_a[i]-int8_b[i] : 8'd0;
    end
  endgenerate

  // INT4
  wire [3:0] int4_a [0:LANES_INT4-1];
  wire [3:0] int4_b [0:LANES_INT4-1];
  wire [3:0] int4_out[0:LANES_INT4-1];
  generate
    for (i=0; i<LANES_INT4; i=i+1) begin : int4_pack
      assign int4_a[i] = in_fifo_a[in_rd_a][4*i +:4];
      assign int4_b[i] = in_fifo_b[in_rd_b][4*i +:4];
      assign int4_out[i] = (opcode==0) ? (int4_a[i]+int4_b[i]) & 4'hF :
                           (opcode==1) ? (int4_a[i]-int4_b[i]) & 4'hF : 4'd0;
    end
  endgenerate

  integer j;
  //-------------------------------------------------------------------------
  // Compute & start/done logic
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      done <= 1'b0;
      start<= 1'b0;
    end else if (start && in_count_a>0 && in_count_b>0 && out_count<FIFO_DEPTH) begin
      // Pop inputs
      in_rd_a <= in_rd_a + 1;
      in_rd_b <= in_rd_b + 1;
      in_count_a <= in_count_a - 1;
      in_count_b <= in_count_b - 1;
      // Pack outputs
      for (j=0; j<DATA_WIDTH/32; j=j+1) begin
        out_fifo[out_wr][32*j +:32] <= fp32_out[j];
      end
      out_wr <= out_wr + 1;
      out_count <= out_count + 1;
      done <= 1'b1;
      start<= 1'b0;
    end else begin
      done <= 1'b0;
    end
  end

  //-------------------------------------------------------------------------
  // AXI4-Full write AW/W/B
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin s_awready<=1; aw_active<=0; end
    else if (s_awvalid&&s_awready) begin aw_active<=1; s_awready<=0; awid_reg<=s_awid; awaddr_reg<=s_awaddr; end
    else if (s_bvalid&&s_bready) begin aw_active<=0; s_awready<=1; end
  end
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin s_wready<=0; s_bvalid<=0; in_wr_a<=0; in_wr_b<=0; in_count_a<=0; in_count_b<=0; end
    else begin
      if (aw_active&&!s_wready) s_wready<=1;
      if (s_wvalid&&s_wready) begin
        // control
        case (awaddr_reg[7:0])
          8'h00: precision <= s_wdata[1:0];
          8'h04: opcode    <= s_wdata[3:0];
          8'h08: start     <= 1;
          8'h20: if(in_count_a<FIFO_DEPTH) begin in_fifo_a[in_wr_a]<=s_wdata; in_wr_a<=in_wr_a+1; in_count_a<=in_count_a+1; end
          8'h40: if(in_count_b<FIFO_DEPTH) begin in_fifo_b[in_wr_b]<=s_wdata; in_wr_b<=in_wr_b+1; in_count_b<=in_count_b+1; end
        endcase
        s_bid<=awid_reg; s_bresp<=2'b00; s_bvalid<=1; s_wready<=0;
      end else if(s_bvalid&&s_bready) s_bvalid<=0;
    end
  end

  //-------------------------------------------------------------------------
  // AXI4-Full read AR/R
  reg [DATA_WIDTH-1:0] read_data;
  always @* begin
    read_data = {DATA_WIDTH{1'b0}};
    case (araddr_reg[7:0])
      8'h0C: read_data[0] = done;
      8'h10: begin // pop out
                if(out_count>0) read_data = out_fifo[out_rd];
              end
    endcase
  end
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin s_arready<=1; s_rvalid<=0; s_rlast<=0; out_rd<=0; out_count<=0; end
    else if(s_arvalid&&s_arready) begin s_arready<=0; s_rid<=s_arid; s_rresp<=2'b00; s_rvalid<=1; s_rlast<=1; s_rdata<=read_data; if(araddr_reg[7:0]==8'h10&&out_count>0) begin out_rd<=out_rd+1; out_count<=out_count-1; end end
    else if(s_rvalid&&s_rready) begin s_rvalid<=0; s_rlast<=0; s_arready<=1; end
  end
endmodule

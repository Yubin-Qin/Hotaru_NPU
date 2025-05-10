//-----------------------------------------------------------------------------
// scalar_engine.v
//-----------------------------------------------------------------------------
// Scalar ALU engine with SIMD-like support for multiple data precisions:
//  - FP32 (one 32-bit float)
//  - FP16 (two 16-bit floats)
//  - INT8 (four 8-bit integers)
//  - INT4 (eight 4-bit integers)
// Uses Synopsys DesignWare FP IP for FP operations
// Control/Register Map (offsets from base):
//   0x00: operand A [31:0]
//   0x04: operand B [31:0]
//   0x08: opcode   [3:0]  {0:add,1:sub,2:mul,3:div,4:and,5:or,6:xor,7:slt}
//   0x0C: precision[1:0]  {0:FP32,1:FP16,2:INT8,3:INT4}
//   0x10: start    [0]   write 1 to launch operation
//   0x14: result   [31:0]
//   0x18: status   [0]   done flag
//-----------------------------------------------------------------------------
module scalar_engine #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input  wire                  clk,
    input  wire                  rst_n,
    // AXI4-Lite slave interface
    input  wire [ADDR_WIDTH-1:0] s_awaddr,
    input  wire [2:0]            s_awprot,
    input  wire                  s_awvalid,
    output wire                  s_awready,

    input  wire [DATA_WIDTH-1:0] s_wdata,
    input  wire [DATA_WIDTH/8-1:0] s_wstrb,
    input  wire                  s_wvalid,
    output wire                  s_wready,

    output wire [1:0]            s_bresp,
    output wire                  s_bvalid,
    input  wire                  s_bready,

    input  wire [ADDR_WIDTH-1:0] s_araddr,
    input  wire [2:0]            s_arprot,
    input  wire                  s_arvalid,
    output wire                  s_arready,

    output wire [DATA_WIDTH-1:0] s_rdata,
    output wire [1:0]            s_rresp,
    output wire                  s_rvalid,
    input  wire                  s_rready
);

  // Internal state
  reg [DATA_WIDTH-1:0] op_a, op_b;
  reg [3:0]            opcode;
  reg [1:0]            precision;
  reg                  start, done;
  reg [DATA_WIDTH-1:0] result;

  // AXI4-Lite handshake
  reg aw_en, ar_en;
  reg [7:0] addr_l;
  assign s_awready = !aw_en;
  assign s_wready  = aw_en;
  assign s_bvalid  = aw_en;
  assign s_bresp   = 2'b00;
  assign s_arready = !ar_en;
  assign s_rvalid  = ar_en;
  assign s_rresp   = 2'b00;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) aw_en <= 1'b0;
    else if (s_awvalid && !aw_en) aw_en <= 1'b1;
    else if (s_bvalid && s_bready) aw_en <= 1'b0;
  end
  always @(posedge clk) if (s_awvalid && !aw_en) addr_l <= s_awaddr[7:0];

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) ar_en <= 1'b0;
    else if (s_arvalid && !ar_en) ar_en <= 1'b1;
    else if (s_rvalid && s_rready) ar_en <= 1'b0;
  end
  always @(posedge clk) if (s_arvalid && !ar_en) addr_l <= s_araddr[7:0];

  // Register writes
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      op_a      <= 0; op_b <= 0;
      opcode    <= 0; precision <= 0;
      start     <= 0; done <= 0; result <= 0;
    end else if (aw_en && s_wvalid) begin
      case (addr_l)
        8'h00: op_a      <= s_wdata;
        8'h04: op_b      <= s_wdata;
        8'h08: opcode    <= s_wdata[3:0];
        8'h0C: precision <= s_wdata[1:0];
        8'h10: begin start <= 1'b1; done <= 1'b0; end
      endcase
    end else if (done) begin start <= 1'b0; end
  end

  //----------------------------------------------------------------------------
  // Instantiate DesignWare FP IP for FP32 operations
  wire [31:0] fp32_add, fp32_sub, fp32_mul, fp32_div;
  wire [7:0]  status_add, status_sub, status_mul, status_div;
  DW_fp_add  #( .exp_width(8), .sig_width(23) ) u_fp32_add( .a(op_a), .b(op_b), .rnd(3'b000), .z(fp32_add), .status(status_add) );
  DW_fp_sub  #( .exp_width(8), .sig_width(23) ) u_fp32_sub( .a(op_a), .b(op_b), .rnd(3'b000), .z(fp32_sub), .status(status_sub) );
  DW_fp_mult #( .exp_width(8), .sig_width(23) ) u_fp32_mul( .a(op_a), .b(op_b), .rnd(3'b000), .z(fp32_mul), .status(status_mul) );
  DW_fp_div  #( .exp_width(8), .sig_width(23) ) u_fp32_div( .a(op_a), .b(op_b), .rnd(3'b000), .z(fp32_div), .status(status_div) );

  // Mux FP32 result
  wire [31:0] res_fp32 = (opcode==4'd0) ? fp32_add :
                         (opcode==4'd1) ? fp32_sub :
                         (opcode==4'd2) ? fp32_mul :
                         (opcode==4'd3) ? fp32_div : 32'd0;

  //----------------------------------------------------------------------------
  // Instantiate DesignWare FP IP for FP16 lanes
  wire [15:0] a16_l0 = op_a[15:0], a16_l1 = op_a[31:16];
  wire [15:0] b16_l0 = op_b[15:0], b16_l1 = op_b[31:16];
  wire [15:0] fp16_add0, fp16_add1, fp16_sub0, fp16_sub1;
  wire [15:0] fp16_mul0, fp16_mul1, fp16_div0, fp16_div1;
  wire [7:0]  st_add0, st_add1, st_sub0, st_sub1;
  // add
  DW_fp_add  #( .exp_width(5), .sig_width(10) ) u_fp16_add0( .a(a16_l0), .b(b16_l0), .rnd(3'b000), .z(fp16_add0), .status(st_add0) );
  DW_fp_add  #( .exp_width(5), .sig_width(10) ) u_fp16_add1( .a(a16_l1), .b(b16_l1), .rnd(3'b000), .z(fp16_add1), .status(st_add1) );
  // sub
  DW_fp_sub  #( .exp_width(5), .sig_width(10) ) u_fp16_sub0( .a(a16_l0), .b(b16_l0), .rnd(3'b000), .z(fp16_sub0), .status(st_sub0) );
  DW_fp_sub  #( .exp_width(5), .sig_width(10) ) u_fp16_sub1( .a(a16_l1), .b(b16_l1), .rnd(3'b000), .z(fp16_sub1), .status(st_sub1) );
  // mul
  DW_fp_mult #( .exp_width(5), .sig_width(10) ) u_fp16_mul0( .a(a16_l0), .b(b16_l0), .rnd(3'b000), .z(fp16_mul0), .status() );
  DW_fp_mult #( .exp_width(5), .sig_width(10) ) u_fp16_mul1( .a(a16_l1), .b(b16_l1), .rnd(3'b000), .z(fp16_mul1), .status() );
  // div
  DW_fp_div  #( .exp_width(5), .sig_width(10) ) u_fp16_div0( .a(a16_l0), .b(b16_l0), .rnd(3'b000), .z(fp16_div0), .status() );
  DW_fp_div  #( .exp_width(5), .sig_width(10) ) u_fp16_div1( .a(a16_l1), .b(b16_l1), .rnd(3'b000), .z(fp16_div1), .status() );

  // Mux FP16 results
  wire [15:0] r16_l0 = (opcode==4'd0) ? fp16_add0 :
                       (opcode==4'd1) ? fp16_sub0 :
                       (opcode==4'd2) ? fp16_mul0 :
                       (opcode==4'd3) ? fp16_div0 : 16'd0;
  wire [15:0] r16_l1 = (opcode==4'd0) ? fp16_add1 :
                       (opcode==4'd1) ? fp16_sub1 :
                       (opcode==4'd2) ? fp16_mul1 :
                       (opcode==4'd3) ? fp16_div1 : 16'd0;

  //----------------------------------------------------------------------------
  // INT8 and INT4 lanes unchanged
  wire signed [7:0]  a8 [3:0], b8 [3:0], r8 [3:0];
  genvar j;
  generate for (j=0; j<4; j=j+1) begin
    assign a8[j] = op_a[8*j +:8]; assign b8[j] = op_b[8*j +:8];
    assign r8[j] = (opcode==4'd0) ? a8[j] + b8[j] :
                   (opcode==4'd1) ? a8[j] - b8[j] : 8'd0;
  end endgenerate

  wire [3:0] a4 [7:0], b4 [7:0], r4 [7:0];
  generate for (j=0; j<8; j=j+1) begin
    assign a4[j] = op_a[4*j +:4]; assign b4[j] = op_b[4*j +:4];
    assign r4[j] = (opcode==4'd0) ? a4[j] + b4[j] :
                   (opcode==4'd1) ? a4[j] - b4[j] : 4'd0;
  end endgenerate

  //----------------------------------------------------------------------------
  // Launch and finalize
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin result <= 0; done <= 0; end
    else if (start) begin
      case (precision)
        2'b00: result <= res_fp32;
        2'b01: result <= {r16_l1, r16_l0};
        2'b10: result <= {r8[3],r8[2],r8[1],r8[0]};
        2'b11: result <= {r4[7],r4[6],r4[5],r4[4],r4[3],r4[2],r4[1],r4[0]};
      endcase
      done <= 1'b1;
    end
  end

  // Read data path
  reg [DATA_WIDTH-1:0] rdata_l;
  always @* begin
    case (addr_l)
      8'h00: rdata_l = op_a;
      8'h04: rdata_l = op_b;
      8'h08: rdata_l = {28'd0, opcode};
      8'h0C: rdata_l = {30'd0, precision};
      8'h14: rdata_l = result;
      8'h18: rdata_l = {31'd0, done};
      default: rdata_l = 0;
    endcase
  end
  assign s_rdata = rdata_l;

endmodule

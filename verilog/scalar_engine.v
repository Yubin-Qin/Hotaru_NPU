//-----------------------------------------------------------------------------
// scalar_engine.v
//-----------------------------------------------------------------------------
// Simple scalar ALU engine with AXI4-Lite slave interface for control
// Registers (offset by local base address):
// 0x00: operand A (32-bit)
// 0x04: operand B (32-bit)
// 0x08: opcode   (4-bit) {0:add,1:sub,2:mul,3:div,4:and,5:or,6:xor,7:slt}
// 0x0C: start    (write-only, 1 triggers operation)
// 0x10: result   (32-bit)
// 0x14: status   (bit0: done flag)
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

  // Internal registers
  reg [DATA_WIDTH-1:0] op_a;
  reg [DATA_WIDTH-1:0] op_b;
  reg [3:0]            opcode;
  reg                  done;
  reg [DATA_WIDTH-1:0] result;

  // Compute combinatorial result
  wire [DATA_WIDTH-1:0] result_comb;
  assign result_comb =
       (opcode == 4'd0) ? op_a + op_b :
       (opcode == 4'd1) ? op_a - op_b :
       (opcode == 4'd2) ? op_a * op_b :
       (opcode == 4'd3) ? op_b ? op_a / op_b : {DATA_WIDTH{1'b0}} :
       (opcode == 4'd4) ? op_a & op_b :
       (opcode == 4'd5) ? op_a | op_b :
       (opcode == 4'd6) ? op_a ^ op_b :
       (opcode == 4'd7) ? ($signed(op_a) < $signed(op_b) ? 32'd1 : 32'd0) :
       32'd0;

  // AXI4-Lite slave handshake latches
  reg aw_en;
  reg ar_en;
  reg [ADDR_WIDTH-1:0] awaddr_l;
  reg [ADDR_WIDTH-1:0] araddr_l;

  // AW channel
  assign s_awready = !aw_en;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) aw_en <= 1'b0;
    else if (s_awvalid && !aw_en) aw_en <= 1'b1;
    else if (s_bvalid && s_bready) aw_en <= 1'b0;
  end
  always @(posedge clk) if (s_awvalid && !aw_en) awaddr_l <= s_awaddr;

  // W channel
  assign s_wready = !aw_en;
  // Write response
  assign s_bvalid = aw_en;
  assign s_bresp  = 2'b00; // OKAY

  // AR channel
  assign s_arready = !ar_en;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) ar_en <= 1'b0;
    else if (s_arvalid && !ar_en) ar_en <= 1'b1;
    else if (s_rvalid && s_rready) ar_en <= 1'b0;
  end
  always @(posedge clk) if (s_arvalid && !ar_en) araddr_l <= s_araddr;

  // R channel
  reg [DATA_WIDTH-1:0] rdata_l;
  assign s_rdata = rdata_l;
  assign s_rvalid = ar_en;
  assign s_rresp  = 2'b00; // OKAY

  // Register write logic
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      op_a    <= {DATA_WIDTH{1'b0}};
      op_b    <= {DATA_WIDTH{1'b0}};
      opcode  <= 4'b0;
      result  <= {DATA_WIDTH{1'b0}};
      done    <= 1'b0;
    end else if (aw_en && s_wvalid) begin
      case (awaddr_l[7:0])
        8'h00: op_a   <= s_wdata;
        8'h04: op_b   <= s_wdata;
        8'h08: opcode <= s_wdata[3:0];
        8'h0C: begin
                 result <= result_comb;
                 done   <= 1'b1;
               end
        default: ;
      endcase
    end
  end

  // Register read logic
  always @* begin
    case (araddr_l[7:0])
      8'h00: rdata_l = op_a;
      8'h04: rdata_l = op_b;
      8'h08: rdata_l = {28'b0, opcode};
      8'h10: rdata_l = result;
      8'h14: rdata_l = {31'b0, done};
      default: rdata_l = {DATA_WIDTH{1'b0}};
    endcase
  end

endmodule
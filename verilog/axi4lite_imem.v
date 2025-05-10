//-----------------------------------------------------------------------------
// axi4lite_imem.v
//-----------------------------------------------------------------------------
// AXI4-Lite slave ROM (Instruction Memory) with 1MB capacity.
// Read-only: supports single-beat 32-bit reads. Ignoring writes.
//-----------------------------------------------------------------------------
module axi4lite_imem #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    // Depth in number of words (DATA_WIDTH/8 bytes each)
    parameter DEPTH      = 262144  // 1MB / 4 bytes
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // AXI4-Lite slave ports
    input  wire [ADDR_WIDTH-1:0]   s_awaddr,
    input  wire [2:0]              s_awprot,
    input  wire                    s_awvalid,
    output wire                    s_awready,

    input  wire [DATA_WIDTH-1:0]   s_wdata,
    input  wire [DATA_WIDTH/8-1:0] s_wstrb,
    input  wire                    s_wvalid,
    output wire                    s_wready,

    output wire [1:0]              s_bresp,
    output wire                    s_bvalid,
    input  wire                    s_bready,

    input  wire [ADDR_WIDTH-1:0]   s_araddr,
    input  wire [2:0]              s_arprot,
    input  wire                    s_arvalid,
    output wire                    s_arready,

    output wire [DATA_WIDTH-1:0]   s_rdata,
    output wire [1:0]              s_rresp,
    output wire                    s_rvalid,
    input  wire                    s_rready
);

  // Write channels disabled (read-only ROM)
  assign s_awready = 1'b0;
  assign s_wready  = 1'b0;
  assign s_bvalid  = 1'b0;
  assign s_bresp   = 2'b00;

  // Read address handshake
  reg               ar_en;
  reg [ADDR_WIDTH-1:2] araddr_l;

  assign s_arready = !ar_en;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      ar_en <= 1'b0;
    else if (s_arvalid && !ar_en)
      ar_en <= 1'b1;
    else if (s_rvalid && s_rready)
      ar_en <= 1'b0;
  end

  always @(posedge clk) begin
    if (s_arvalid && !ar_en)
      // word-aligned address, drop lower 2 bits
      araddr_l <= s_araddr[ADDR_WIDTH-1:2];
  end

  // ROM storage
  reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
  initial begin
    // Optionally initialize memory from a file:
    // $readmemh("imem_init.hex", mem);
  end

  // Read data path
  reg [DATA_WIDTH-1:0] rdata_l;
  assign s_rdata = rdata_l;
  assign s_rvalid = ar_en;
  assign s_rresp  = 2'b00; // OKAY

  always @(posedge clk) begin
    if (ar_en)
      rdata_l <= mem[araddr_l];
  end

endmodule

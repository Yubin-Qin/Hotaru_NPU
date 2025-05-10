//-----------------------------------------------------------------------------
// axi4lite_ram.v
//-----------------------------------------------------------------------------
// AXI4-Lite slave RAM (Data Memory) with parameterizable depth (1MB).
// Supports single-beat 32-bit reads and writes.
//-----------------------------------------------------------------------------
module axi4lite_ram #(
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

  // Internal memory
  reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

  // Write handshake signals
  reg aw_en;
  reg [ADDR_WIDTH-1:2] awaddr_l;
  reg wdata_en;
  reg [DATA_WIDTH-1:0] wdata_l;
  reg [DATA_WIDTH/8-1:0] wstrb_l;
  reg bvalid_l;

  // Read handshake signals
  reg ar_en;
  reg [ADDR_WIDTH-1:2] araddr_l;
  reg [DATA_WIDTH-1:0] rdata_l;
  reg rvalid_l;

  //----------------------------------------------------------------------
  // Write address channel
  assign s_awready = !aw_en;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      aw_en <= 1'b0;
    end else if (s_awvalid && s_awready) begin
      aw_en    <= 1'b1;
      awaddr_l <= s_awaddr[ADDR_WIDTH-1:2];
    end else if (bvalid_l && s_bready) begin
      aw_en <= 1'b0;
    end
  end

  // Write data channel
  assign s_wready = aw_en && !wdata_en;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wdata_en <= 1'b0;
    end else if (s_wvalid && s_wready) begin
      wdata_en <= 1'b1;
      wdata_l  <= s_wdata;
      wstrb_l  <= s_wstrb;
    end else if (bvalid_l && s_bready) begin
      wdata_en <= 1'b0;
    end
  end

  // Perform write and generate response
  always @(posedge clk) begin
    if (aw_en && wdata_en) begin
      integer i;
      for (i = 0; i < DATA_WIDTH/8; i = i + 1) begin
        if (wstrb_l[i])
          mem[awaddr_l][8*i +: 8] <= wdata_l[8*i +: 8];
      end
    end
  end

  // Write response channel
  assign s_bvalid = bvalid_l;
  assign s_bresp  = 2'b00; // OKAY
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bvalid_l <= 1'b0;
    end else if (aw_en && wdata_en) begin
      bvalid_l <= 1'b1;
    end else if (bvalid_l && s_bready) begin
      bvalid_l <= 1'b0;
    end
  end

  //----------------------------------------------------------------------
  // Read address channel
  assign s_arready = !ar_en;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ar_en <= 1'b0;
    end else if (s_arvalid && s_arready) begin
      ar_en     <= 1'b1;
      araddr_l  <= s_araddr[ADDR_WIDTH-1:2];
    end else if (rvalid_l && s_rready) begin
      ar_en <= 1'b0;
    end
  end

  // Read data and response
  always @(posedge clk) begin
    if (ar_en) begin
      rdata_l  <= mem[araddr_l];
      rvalid_l <= 1'b1;
    end else if (rvalid_l && s_rready) begin
      rvalid_l <= 1'b0;
    end
  end

  assign s_rvalid = rvalid_l;
  assign s_rdata  = rdata_l;
  assign s_rresp  = 2'b00; // OKAY

endmodule

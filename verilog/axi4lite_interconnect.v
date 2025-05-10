//-----------------------------------------------------------------------------
// axi4lite_interconnect.v
//-----------------------------------------------------------------------------
// Simple AXI4-Lite interconnect: one master to three slaves
//----------------------------------------------------------------------------- 
module axi4lite_interconnect #(
  parameter ADDR_WIDTH   = 32,
  parameter DATA_WIDTH   = 32,
  // Slave 0 (e.g. IMEM)
  parameter [ADDR_WIDTH-1:0] SLAVE0_BASE = 32'h0000_0000,
  parameter [31:0]           SLAVE0_SIZE = 32'h0001_0000,
  // Slave 1 (e.g. DMEM)
  parameter [ADDR_WIDTH-1:0] SLAVE1_BASE = 32'h1000_0000,
  parameter [31:0]           SLAVE1_SIZE = 32'h0001_0000,
  // Slave 2 (e.g. CSR registers)
  parameter [ADDR_WIDTH-1:0] SLAVE2_BASE = 32'h2000_0000,
  parameter [31:0]           SLAVE2_SIZE = 32'h0000_1000
)(
  input  wire                    clk,
  input  wire                    rst_n,

  // Master port (CPU)
  input  wire [ADDR_WIDTH-1:0]   m_awaddr,
  input  wire [2:0]              m_awprot,
  input  wire                    m_awvalid,
  output wire                    m_awready,

  input  wire [DATA_WIDTH-1:0]   m_wdata,
  input  wire [DATA_WIDTH/8-1:0] m_wstrb,
  input  wire                    m_wvalid,
  output wire                    m_wready,

  output wire [1:0]              m_bresp,
  output wire                    m_bvalid,
  input  wire                    m_bready,

  input  wire [ADDR_WIDTH-1:0]   m_araddr,
  input  wire [2:0]              m_arprot,
  input  wire                    m_arvalid,
  output wire                    m_arready,

  output wire [DATA_WIDTH-1:0]   m_rdata,
  output wire [1:0]              m_rresp,
  output wire                    m_rvalid,
  input  wire                    m_rready,

  // Slave0 port
  output wire [ADDR_WIDTH-1:0]   s0_awaddr,
  output wire [2:0]              s0_awprot,
  output wire                    s0_awvalid,
  input  wire                    s0_awready,

  output wire [DATA_WIDTH-1:0]   s0_wdata,
  output wire [DATA_WIDTH/8-1:0] s0_wstrb,
  output wire                    s0_wvalid,
  input  wire                    s0_wready,

  input  wire [1:0]              s0_bresp,
  input  wire                    s0_bvalid,
  output wire                    s0_bready,

  output wire [ADDR_WIDTH-1:0]   s0_araddr,
  output wire [2:0]              s0_arprot,
  output wire                    s0_arvalid,
  input  wire                    s0_arready,

  input  wire [DATA_WIDTH-1:0]   s0_rdata,
  input  wire [1:0]              s0_rresp,
  input  wire                    s0_rvalid,
  output wire                    s0_rready,

  // Slave1 port (same as Slave0)
  output wire [ADDR_WIDTH-1:0]   s1_awaddr,
  output wire [2:0]              s1_awprot,
  output wire                    s1_awvalid,
  input  wire                    s1_awready,

  output wire [DATA_WIDTH-1:0]   s1_wdata,
  output wire [DATA_WIDTH/8-1:0] s1_wstrb,
  output wire                    s1_wvalid,
  input  wire                    s1_wready,

  input  wire [1:0]              s1_bresp,
  input  wire                    s1_bvalid,
  output wire                    s1_bready,

  output wire [ADDR_WIDTH-1:0]   s1_araddr,
  output wire [2:0]              s1_arprot,
  output wire                    s1_arvalid,
  input  wire                    s1_arready,

  input  wire [DATA_WIDTH-1:0]   s1_rdata,
  input  wire [1:0]              s1_rresp,
  input  wire                    s1_rvalid,
  output wire                    s1_rready,

  // Slave2 port (same as Slave0)
  output wire [ADDR_WIDTH-1:0]   s2_awaddr,
  output wire [2:0]              s2_awprot,
  output wire                    s2_awvalid,
  input  wire                    s2_awready,

  output wire [DATA_WIDTH-1:0]   s2_wdata,
  output wire [DATA_WIDTH/8-1:0] s2_wstrb,
  output wire                    s2_wvalid,
  input  wire                    s2_wready,

  input  wire [1:0]              s2_bresp,
  input  wire                    s2_bvalid,
  output wire                    s2_bready,

  output wire [ADDR_WIDTH-1:0]   s2_araddr,
  output wire [2:0]              s2_arprot,
  output wire                    s2_arvalid,
  input  wire                    s2_arready,

  input  wire [DATA_WIDTH-1:0]   s2_rdata,
  input  wire [1:0]              s2_rresp,
  input  wire                    s2_rvalid,
  output wire                    s2_rready
);

  // Address decode for AW/AR channels
  wire sel0_aw = (m_awaddr >= SLAVE0_BASE) && (m_awaddr < SLAVE0_BASE + SLAVE0_SIZE);
  wire sel1_aw = (m_awaddr >= SLAVE1_BASE) && (m_awaddr < SLAVE1_BASE + SLAVE1_SIZE);
  wire sel2_aw = (m_awaddr >= SLAVE2_BASE) && (m_awaddr < SLAVE2_BASE + SLAVE2_SIZE);
  wire sel0_ar = (m_araddr >= SLAVE0_BASE) && (m_araddr < SLAVE0_BASE + SLAVE0_SIZE);
  wire sel1_ar = (m_araddr >= SLAVE1_BASE) && (m_araddr < SLAVE1_BASE + SLAVE1_SIZE);
  wire sel2_ar = (m_araddr >= SLAVE2_BASE) && (m_araddr < SLAVE2_BASE + SLAVE2_SIZE);

  // Latch selection for write data & response
  reg [2:0] aw_sel_reg;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) aw_sel_reg <= 3'b000;
    else if (m_awvalid && m_awready) begin
      if (sel0_aw) aw_sel_reg <= 3'b001;
      else if (sel1_aw) aw_sel_reg <= 3'b010;
      else if (sel2_aw) aw_sel_reg <= 3'b100;
      else aw_sel_reg <= 3'b000;
    end
  end

  // Latch selection for read data & response
  reg [2:0] ar_sel_reg;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) ar_sel_reg <= 3'b000;
    else if (m_arvalid && m_arready) begin
      if (sel0_ar) ar_sel_reg <= 3'b001;
      else if (sel1_ar) ar_sel_reg <= 3'b010;
      else if (sel2_ar) ar_sel_reg <= 3'b100;
      else ar_sel_reg <= 3'b000;
    end
  end

  // --------------------------------------------------------------------------
  // AW channel routing
  assign s0_awaddr  = m_awaddr;
  assign s0_awprot  = m_awprot;
  assign s0_awvalid = m_awvalid && sel0_aw;
  assign s1_awaddr  = m_awaddr;
  assign s1_awprot  = m_awprot;
  assign s1_awvalid = m_awvalid && sel1_aw;
  assign s2_awaddr  = m_awaddr;
  assign s2_awprot  = m_awprot;
  assign s2_awvalid = m_awvalid && sel2_aw;

  assign m_awready = (s0_awready && sel0_aw) |
                      (s1_awready && sel1_aw) |
                      (s2_awready && sel2_aw);

  // W channel routing
  assign s0_wdata  = m_wdata;
  assign s0_wstrb  = m_wstrb;
  assign s0_wvalid = m_wvalid && aw_sel_reg[0];
  assign s1_wdata  = m_wdata;
  assign s1_wstrb  = m_wstrb;
  assign s1_wvalid = m_wvalid && aw_sel_reg[1];
  assign s2_wdata  = m_wdata;
  assign s2_wstrb  = m_wstrb;
  assign s2_wvalid = m_wvalid && aw_sel_reg[2];

  assign m_wready = (s0_wready && aw_sel_reg[0]) |
                     (s1_wready && aw_sel_reg[1]) |
                     (s2_wready && aw_sel_reg[2]);

  // B channel merging
  assign s0_bready = m_bready && aw_sel_reg[0];
  assign s1_bready = m_bready && aw_sel_reg[1];
  assign s2_bready = m_bready && aw_sel_reg[2];

  assign m_bvalid = (s0_bvalid && aw_sel_reg[0]) |
                      (s1_bvalid && aw_sel_reg[1]) |
                      (s2_bvalid && aw_sel_reg[2]);
  assign m_bresp  = s0_bvalid && aw_sel_reg[0] ? s0_bresp  :
                      s1_bvalid && aw_sel_reg[1] ? s1_bresp  :
                      s2_bvalid && aw_sel_reg[2] ? s2_bresp  : 2'b00;

  // --------------------------------------------------------------------------
  // AR channel routing
  assign s0_araddr  = m_araddr;
  assign s0_arprot  = m_arprot;
  assign s0_arvalid = m_arvalid && sel0_ar;
  assign s1_araddr  = m_araddr;
  assign s1_arprot  = m_arprot;
  assign s1_arvalid = m_arvalid && sel1_ar;
  assign s2_araddr  = m_araddr;
  assign s2_arprot  = m_arprot;
  assign s2_arvalid = m_arvalid && sel2_ar;

  assign m_arready = (s0_arready && sel0_ar) |
                      (s1_arready && sel1_ar) |
                      (s2_arready && sel2_ar);

  // R channel merging
  assign s0_rready = m_rready && ar_sel_reg[0];
  assign s1_rready = m_rready && ar_sel_reg[1];
  assign s2_rready = m_rready && ar_sel_reg[2];

  assign m_rvalid = (s0_rvalid && ar_sel_reg[0]) |
                      (s1_rvalid && ar_sel_reg[1]) |
                      (s2_rvalid && ar_sel_reg[2]);
  assign m_rdata  = s0_rvalid && ar_sel_reg[0] ? s0_rdata  :
                      s1_rvalid && ar_sel_reg[1] ? s1_rdata  :
                      s2_rvalid && ar_sel_reg[2] ? s2_rdata  : {DATA_WIDTH{1'b0}};
  assign m_rresp  = s0_rvalid && ar_sel_reg[0] ? s0_rresp  :
                      s1_rvalid && ar_sel_reg[1] ? s1_rresp  :
                      s2_rvalid && ar_sel_reg[2] ? s2_rresp  : 2'b00;

endmodule

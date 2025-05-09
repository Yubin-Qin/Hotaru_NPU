//-----------------------------------------------------------------------------
// npu.v
// Top-level NPU integrating CPU controller, AXI4-Lite interconnect,
// IMEM, DMEM, scalar engine, and placeholders for KV PIM and Sparse GEMM engines.
//----------------------------------------------------------------------------- 
module npu #(
    parameter ADDR_WIDTH     = 32,
    parameter DATA_WIDTH     = 32,
    // Memory map
    parameter [ADDR_WIDTH-1:0] IMEM_BASE   = 32'h0000_0000,
    parameter [31:0]           IMEM_SIZE   = 32'h0001_0000,
    parameter [ADDR_WIDTH-1:0] DMEM_BASE   = 32'h1000_0000,
    parameter [31:0]           DMEM_SIZE   = 32'h0001_0000,
    parameter [ADDR_WIDTH-1:0] SCALAR_BASE = 32'h2000_0000,
    parameter [31:0]           SCALAR_SIZE = 32'h0000_1000
)(
    input  wire clk,
    input  wire rst_n
);

  // ---------------------------------------------------------------------------
  // CPU controller instantiation (PicoRV32 + AXI4-Lite adapter)
  // ---------------------------------------------------------------------------
  wire [ADDR_WIDTH-1:0] cpu_awaddr;
  wire [2:0]            cpu_awprot;
  wire                  cpu_awvalid;
  wire                  cpu_awready;
  wire [DATA_WIDTH-1:0] cpu_wdata;
  wire [DATA_WIDTH/8-1:0] cpu_wstrb;
  wire                  cpu_wvalid;
  wire                  cpu_wready;
  wire [1:0]            cpu_bresp;
  wire                  cpu_bvalid;
  wire                  cpu_bready;
  wire [ADDR_WIDTH-1:0] cpu_araddr;
  wire [2:0]            cpu_arprot;
  wire                  cpu_arvalid;
  wire                  cpu_arready;
  wire [DATA_WIDTH-1:0] cpu_rdata;
  wire [1:0]            cpu_rresp;
  wire                  cpu_rvalid;
  wire                  cpu_rready;

  npu_cpu_controller u_cpu (
    .clk         (clk),
    .rst_n       (rst_n),
    .m_axi_awaddr(cpu_awaddr),
    .m_axi_awprot(cpu_awprot),
    .m_axi_awvalid(cpu_awvalid),
    .m_axi_awready(cpu_awready),
    .m_axi_wdata (cpu_wdata),
    .m_axi_wstrb (cpu_wstrb),
    .m_axi_wvalid(cpu_wvalid),
    .m_axi_wready(cpu_wready),
    .m_axi_bresp (cpu_bresp),
    .m_axi_bvalid(cpu_bvalid),
    .m_axi_bready(cpu_bready),
    .m_axi_araddr(cpu_araddr),
    .m_axi_arprot(cpu_arprot),
    .m_axi_arvalid(cpu_arvalid),
    .m_axi_arready(cpu_arready),
    .m_axi_rdata (cpu_rdata),
    .m_axi_rresp (cpu_rresp),
    .m_axi_rvalid(cpu_rvalid),
    .m_axi_rready(cpu_rready)
  );

  // ---------------------------------------------------------------------------
  // AXI4-Lite interconnect: CPU master -> 3 slaves
  // ---------------------------------------------------------------------------
  // Slave ports: 0=IMEM, 1=DMEM, 2=Scalar engine (CSRs)
  wire [ADDR_WIDTH-1:0] s0_awaddr, s1_awaddr, s2_awaddr;
  wire [2:0]            s0_awprot, s1_awprot, s2_awprot;
  wire                  s0_awvalid, s1_awvalid, s2_awvalid;
  wire                  s0_awready, s1_awready, s2_awready;
  wire [DATA_WIDTH-1:0] s0_wdata, s1_wdata, s2_wdata;
  wire [DATA_WIDTH/8-1:0] s0_wstrb, s1_wstrb, s2_wstrb;
  wire                  s0_wvalid, s1_wvalid, s2_wvalid;
  wire                  s0_wready, s1_wready, s2_wready;
  wire [1:0]            s0_bresp, s1_bresp, s2_bresp;
  wire                  s0_bvalid, s1_bvalid, s2_bvalid;
  wire                  s0_bready, s1_bready, s2_bready;
  wire [ADDR_WIDTH-1:0] s0_araddr, s1_araddr, s2_araddr;
  wire [2:0]            s0_arprot, s1_arprot, s2_arprot;
  wire                  s0_arvalid, s1_arvalid, s2_arvalid;
  wire                  s0_arready, s1_arready, s2_arready;
  wire [DATA_WIDTH-1:0] s0_rdata, s1_rdata, s2_rdata;
  wire [1:0]            s0_rresp, s1_rresp, s2_rresp;
  wire                  s0_rvalid, s1_rvalid, s2_rvalid;
  wire                  s0_rready, s1_rready, s2_rready;

  axi4lite_interconnect #(
    .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH),
    .SLAVE0_BASE(IMEM_BASE),   .SLAVE0_SIZE(IMEM_SIZE),
    .SLAVE1_BASE(DMEM_BASE),   .SLAVE1_SIZE(DMEM_SIZE),
    .SLAVE2_BASE(SCALAR_BASE), .SLAVE2_SIZE(SCALAR_SIZE)
  ) u_intercon (
    .clk       (clk), .rst_n     (rst_n),
    .m_awaddr  (cpu_awaddr),  .m_awprot(cpu_awprot),  .m_awvalid(cpu_awvalid),  .m_awready(cpu_awready),
    .m_wdata   (cpu_wdata),   .m_wstrb (cpu_wstrb),   .m_wvalid(cpu_wvalid),  .m_wready(cpu_wready),
    .m_bresp   (cpu_bresp),   .m_bvalid(cpu_bvalid), .m_bready(cpu_bready),
    .m_araddr  (cpu_araddr),  .m_arprot(cpu_arprot),  .m_arvalid(cpu_arvalid),        .m_arready(cpu_arready),
    .m_rdata   (cpu_rdata),   .m_rresp (cpu_rresp),   .m_rvalid(cpu_rvalid),        .m_rready(cpu_rready),
    .s0_awaddr (s0_awaddr),   .s0_awprot(s0_awprot), .s0_awvalid(s0_awvalid), .s0_awready(s0_awready),
    .s0_wdata  (s0_wdata),    .s0_wstrb(s0_wstrb),   .s0_wvalid(s0_wvalid),   .s0_wready(s0_wready),
    .s0_bresp  (s0_bresp),    .s0_bvalid(s0_bvalid), .s0_bready(s0_bready),
    .s0_araddr (s0_araddr),   .s0_arprot(s0_arprot), .s0_arvalid(s0_arvalid), .s0_arready(s0_arready),
    .s0_rdata  (s0_rdata),    .s0_rresp(s0_rresp),   .s0_rvalid(s0_rvalid),   .s0_rready(s0_rready),
    .s1_awaddr (s1_awaddr),   .s1_awprot(s1_awprot), .s1_awvalid(s1_awvalid), .s1_awready(s1_awready),
    .s1_wdata  (s1_wdata),    .s1_wstrb(s1_wstrb),   .s1_wvalid(s1_wvalid),   .s1_wready(s1_wready),
    .s1_bresp  (s1_bresp),    .s1_bvalid(s1_bvalid), .s1_bready(s1_bready),
    .s1_araddr (s1_araddr),   .s1_arprot(s1_arprot), .s1_arvalid(s1_arvalid), .s1_arready(s1_arready),
    .s1_rdata  (s1_rdata),    .s1_rresp(s1_rresp),   .s1_rvalid(s1_rvalid),   .s1_rready(s1_rready),
    .s2_awaddr (s2_awaddr),   .s2_awprot(s2_awprot), .s2_awvalid(s2_awvalid), .s2_awready(s2_awready),
    .s2_wdata  (s2_wdata),    .s2_wstrb(s2_wstrb),   .s2_wvalid(s2_wvalid),   .s2_wready(s2_wready),
    .s2_bresp  (s2_bresp),    .s2_bvalid(s2_bvalid), .s2_bready(s2_bready),
    .s2_araddr (s2_araddr),   .s2_arprot(s2_arprot), .s2_arvalid(s2_arvalid), .s2_arready(s2_arready),
    .s2_rdata  (s2_rdata),    .s2_rresp(s2_rresp),   .s2_rvalid(s2_rvalid),   .s2_rready(s2_rready)
  );

  // ---------------------------------------------------------------------------
  // Instruction Memory (IMEM) - AXI4-Lite read-only
  // TODO: replace with your AXI4-Lite slave ROM implementation
  axi4lite_imem #(
    .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(IMEM_SIZE/4)
  ) u_imem (
    .clk       (clk), .rst_n      (rst_n),
    .s_awaddr  (s0_awaddr),  .s_awprot(s0_awprot),  .s_awvalid(s0_awvalid),  .s_awready(s0_awready),
    .s_wdata   (s0_wdata),   .s_wstrb (s0_wstrb),   .s_wvalid(s0_wvalid),   .s_wready(s0_wready),
    .s_bresp   (s0_bresp),   .s_bvalid(s0_bvalid), .s_bready(s0_bready),
    .s_araddr  (s0_araddr),  .s_arprot(s0_arprot),  .s_arvalid(s0_arvalid),  .s_arready(s0_arready),
    .s_rdata   (s0_rdata),   .s_rresp (s0_rresp),   .s_rvalid(s0_rvalid),   .s_rready(s0_rready)
  );

  // ---------------------------------------------------------------------------
  // Data Memory (DMEM) - AXI4-Lite read/write RAM
  // TODO: replace with your AXI4-Lite slave RAM implementation
  axi4lite_ram #(
    .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(DMEM_SIZE/4)
  ) u_dmem (
    .clk       (clk), .rst_n      (rst_n),
    .s_awaddr  (s1_awaddr),  .s_awprot(s1_awprot),  .s_awvalid(s1_awvalid),  .s_awready(s1_awready),
    .s_wdata   (s1_wdata),   .s_wstrb (s1_wstrb),   .s_wvalid(s1_wvalid),   .s_wready(s1_wready),
    .s_bresp   (s1_bresp),   .s_bvalid(s1_bvalid), .s_bready(s1_bready),
    .s_araddr  (s1_araddr),  .s_arprot(s1_arprot),  .s_arvalid(s1_arvalid),  .s_arready(s1_arready),
    .s_rdata   (s1_rdata),   .s_rresp (s1_rresp),   .s_rvalid(s1_rvalid),   .s_rready(s1_rready)
  );

  // ---------------------------------------------------------------------------
  // Scalar Engine (slave port 2)
  scalar_engine #(
    .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH)
  ) u_scalar (
    .clk       (clk), .rst_n     (rst_n),
    .s_awaddr  (s2_awaddr),  .s_awprot(s2_awprot),  .s_awvalid(s2_awvalid),  .s_awready(s2_awready),
    .s_wdata   (s2_wdata),   .s_wstrb (s2_wstrb),   .s_wvalid(s2_wvalid),   .s_wready(s2_wready),
    .s_bresp   (s2_bresp),   .s_bvalid(s2_bvalid), .s_bready(s2_bready),
    .s_araddr  (s2_araddr),  .s_arprot(s2_arprot),  .s_arvalid(s2_arvalid),  .s_arready(s2_arready),
    .s_rdata   (s2_rdata),   .s_rresp (s2_rresp),   .s_rvalid(s2_rvalid),   .s_rready(s2_rready)
  );

  // ---------------------------------------------------------------------------
  // TODO: Instantiate Tensor and Vector engine.
  // ---------------------------------------------------------------------------

endmodule

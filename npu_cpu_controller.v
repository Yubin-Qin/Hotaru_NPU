//-----------------------------------------------------------------------------
// npu_cpu_controller.v
//-----------------------------------------------------------------------------
// PicoRV32 + AXI4-Lite adapter wrapper
//-----------------------------------------------------------------------------
module npu_cpu_controller #(
    parameter AXI_ADDR_WIDTH = 32,
    parameter AXI_DATA_WIDTH = 32
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // --- AXI4-Lite master interface to your interconnect ---
    output wire [AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [2:0]                m_axi_awprot,
    output wire                      m_axi_awvalid,
    input  wire                      m_axi_awready,

    output wire [AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output wire [AXI_DATA_WIDTH/8-1:0] m_axi_wstrb,
    output wire                      m_axi_wvalid,
    input  wire                      m_axi_wready,

    input  wire [1:0]                m_axi_bresp,
    input  wire                      m_axi_bvalid,
    output wire                      m_axi_bready,

    output wire [AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [2:0]                m_axi_arprot,
    output wire                      m_axi_arvalid,
    input  wire                      m_axi_arready,

    input  wire [AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire [1:0]                m_axi_rresp,
    input  wire                      m_axi_rvalid,
    output wire                      m_axi_rready
);

  //--------------------------------------------------------------------------
  // Internal “mem” interface for PicoRV32
  //--------------------------------------------------------------------------
  wire                     mem_valid;
  wire                     mem_instr;
  wire                     mem_ready;
  wire [31:0]              mem_addr;
  wire [31:0]              mem_wdata;
  wire [3:0]               mem_wstrb;
  wire [31:0]              mem_rdata;

  //--------------------------------------------------------------------------
  // 1) Instantiate PicoRV32 core
  //--------------------------------------------------------------------------
  picoRV32 #(
    .ENABLE_MUL          (0),
    .ENABLE_COUNTERS     (1),
    .ENABLE_COUNTERS64   (1),
    .ENABLE_REGS_DUALPORT(1),
    .STACKADDR           (32'hFFFF_F000)
  ) cpu_core (
    .clk         (clk),
    .resetn      (rst_n),
    .trap        (),            // unused here
    .mem_valid   (mem_valid),
    .mem_instr   (mem_instr),
    .mem_ready   (mem_ready),
    .mem_addr    (mem_addr),
    .mem_wdata   (mem_wdata),
    .mem_wstrb   (mem_wstrb),
    .mem_rdata   (mem_rdata),
    // “fast” interface not used
    .mem_la_read (),
    .mem_la_write(),
    .mem_la_addr (),
    .mem_la_wdata(),
    .mem_la_wstrb(),
    // No PCPI, no interrupts for now
    .pcpi_valid  (),
    .pcpi_insn   (),
    .pcpi_rs1    (),
    .pcpi_rs2    (),
    .pcpi_wr     (),
    .pcpi_rd     (),
    .pcpi_wait   (),
    .pcpi_ready  (),
    .irq         (0),
    .eoi         ()
  );

  //--------------------------------------------------------------------------
  // 2) AXI4-Lite adapter: converts PicoRV32’s simple mem_* bus
  //    into an AXI4-Lite master. We’ll flesh out the FSM in a later step.
  //--------------------------------------------------------------------------
  mem_axi4lite_adapter #(
    .ADDR_WIDTH (AXI_ADDR_WIDTH),
    .DATA_WIDTH (AXI_DATA_WIDTH)
  ) axi_adapter (
    .clk           (clk),
    .rst_n         (rst_n),

    // PicoRV32 side
    .mem_valid     (mem_valid),
    .mem_instr     (mem_instr),
    .mem_ready     (mem_ready),
    .mem_addr      (mem_addr),
    .mem_wdata     (mem_wdata),
    .mem_wstrb     (mem_wstrb),
    .mem_rdata     (mem_rdata),

    // AXI4-Lite master side
    .m_axi_awaddr  (m_axi_awaddr),
    .m_axi_awprot  (m_axi_awprot),
    .m_axi_awvalid (m_axi_awvalid),
    .m_axi_awready (m_axi_awready),

    .m_axi_wdata   (m_axi_wdata),
    .m_axi_wstrb   (m_axi_wstrb),
    .m_axi_wvalid  (m_axi_wvalid),
    .m_axi_wready  (m_axi_wready),

    .m_axi_bresp   (m_axi_bresp),
    .m_axi_bvalid  (m_axi_bvalid),
    .m_axi_bready  (m_axi_bready),

    .m_axi_araddr  (m_axi_araddr),
    .m_axi_arprot  (m_axi_arprot),
    .m_axi_arvalid (m_axi_arvalid),
    .m_axi_arready (m_axi_arready),

    .m_axi_rdata   (m_axi_rdata),
    .m_axi_rresp   (m_axi_rresp),
    .m_axi_rvalid  (m_axi_rvalid),
    .m_axi_rready  (m_axi_rready)
  );

endmodule

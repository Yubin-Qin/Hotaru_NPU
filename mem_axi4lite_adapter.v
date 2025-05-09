//-----------------------------------------------------------------------------
// mem_axi4lite_adapter.v
//-----------------------------------------------------------------------------
// Convert PicoRV32 mem_* bus into AXI4-Lite master (reads & writes).
//-----------------------------------------------------------------------------
module mem_axi4lite_adapter #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // PicoRV32 “mem” interface
    input  wire                    mem_valid,
    input  wire                    mem_instr,      // unused here – read/write is decided by wstrb
    output reg                     mem_ready,
    input  wire [ADDR_WIDTH-1:0]   mem_addr,
    input  wire [DATA_WIDTH-1:0]   mem_wdata,
    input  wire [DATA_WIDTH/8-1:0] mem_wstrb,
    output reg  [DATA_WIDTH-1:0]   mem_rdata,

    // AXI4-Lite master interface
    output reg  [ADDR_WIDTH-1:0]   m_axi_awaddr,
    output reg  [2:0]              m_axi_awprot,
    output reg                     m_axi_awvalid,
    input  wire                    m_axi_awready,

    output reg  [DATA_WIDTH-1:0]   m_axi_wdata,
    output reg  [DATA_WIDTH/8-1:0] m_axi_wstrb,
    output reg                     m_axi_wvalid,
    input  wire                    m_axi_wready,

    input  wire [1:0]              m_axi_bresp,
    input  wire                    m_axi_bvalid,
    output reg                     m_axi_bready,

    output reg  [ADDR_WIDTH-1:0]   m_axi_araddr,
    output reg  [2:0]              m_axi_arprot,
    output reg                     m_axi_arvalid,
    input  wire                    m_axi_arready,

    input  wire [DATA_WIDTH-1:0]   m_axi_rdata,
    input  wire [1:0]              m_axi_rresp,
    input  wire                    m_axi_rvalid,
    output reg                     m_axi_rready
);

  // State encoding
  localparam IDLE       = 3'd0,
             WRITE      = 3'd1,
             WRITE_RESP = 3'd2,
             READ       = 3'd3,
             READ_RESP  = 3'd4;

  reg [2:0] state, next_state;

  // Latch incoming request parameters
  reg [ADDR_WIDTH-1:0]   lat_addr;
  reg [DATA_WIDTH-1:0]   lat_wdata;
  reg [DATA_WIDTH/8-1:0] lat_wstrb;

  // Sequential: state & latches
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state     <= IDLE;
      lat_addr  <= {ADDR_WIDTH{1'b0}};
      lat_wdata <= {DATA_WIDTH{1'b0}};
      lat_wstrb <= {DATA_WIDTH/8{1'b0}};
    end else begin
      state <= next_state;
      if (state == IDLE && mem_valid) begin
        lat_addr  <= mem_addr;
        lat_wdata <= mem_wdata;
        lat_wstrb <= mem_wstrb;
      end
    end
  end

  // Combinational: next state logic
  always @* begin
    next_state = state;
    case (state)
      IDLE: if (mem_valid) begin
        // write if any write‐strobes set, else read
        if (lat_wstrb != 0)
          next_state = WRITE;
        else
          next_state = READ;
      end

      WRITE:      if (m_axi_awvalid && m_axi_awready && m_axi_wvalid && m_axi_wready)
                     next_state = WRITE_RESP;
      WRITE_RESP: if (m_axi_bvalid && m_axi_bready)
                     next_state = IDLE;

      READ:       if (m_axi_arvalid && m_axi_arready)
                     next_state = READ_RESP;
      READ_RESP:  if (m_axi_rvalid && m_axi_rready)
                     next_state = IDLE;
    endcase
  end

  // Combinational: outputs & handshakes
  always @* begin
    // defaults
    m_axi_awaddr  = {ADDR_WIDTH{1'b0}};
    m_axi_awprot  = 3'b000;
    m_axi_awvalid = 1'b0;

    m_axi_wdata   = {DATA_WIDTH{1'b0}};
    m_axi_wstrb   = {DATA_WIDTH/8{1'b0}};
    m_axi_wvalid  = 1'b0;

    m_axi_bready  = 1'b0;

    m_axi_araddr  = {ADDR_WIDTH{1'b0}};
    m_axi_arprot  = 3'b000;
    m_axi_arvalid = 1'b0;

    m_axi_rready  = 1'b0;

    mem_ready     = 1'b0;
    mem_rdata     = {DATA_WIDTH{1'b0}};

    case (state)
      IDLE: begin
        // nothing—wait for next_state to pick WRITE or READ
      end

      // Issue both AW and W together (AXI-Lite allows concurrent issue)
      WRITE: begin
        m_axi_awaddr  = lat_addr;
        m_axi_awprot  = 3'b000;
        m_axi_awvalid = 1'b1;

        m_axi_wdata   = lat_wdata;
        m_axi_wstrb   = lat_wstrb;
        m_axi_wvalid  = 1'b1;
      end

      WRITE_RESP: begin
        // accept write response
        m_axi_bready = 1'b1;
        if (m_axi_bvalid)
          mem_ready = 1'b1;
      end

      READ: begin
        m_axi_araddr  = lat_addr;
        m_axi_arprot  = 3'b000;
        m_axi_arvalid = 1'b1;
      end

      READ_RESP: begin
        // accept read data
        m_axi_rready = 1'b1;
        if (m_axi_rvalid) begin
          mem_ready = 1'b1;
          mem_rdata = m_axi_rdata;
        end
      end
    endcase
  end

endmodule

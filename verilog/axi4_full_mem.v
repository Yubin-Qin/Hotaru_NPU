//-----------------------------------------------------------------------------
// axi4_full_ram.v
//-----------------------------------------------------------------------------
// AXI4-Full slave RAM (Data Memory) with 1 MiB capacity and 1024-bit data width.
// Supports burst/incrementing reads and writes.
//-----------------------------------------------------------------------------
module axi4_full_ram #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 1024,
    parameter ID_WIDTH   = 4,
    parameter LEN_WIDTH  = 8,
    // Depth = 1MiB / (DATA_WIDTH/8) bytes per beat
    parameter DEPTH      = 8192
)(
    input  wire                        clk,
    input  wire                        rst_n,

    // Write address channel
    input  wire [ID_WIDTH-1:0]         s_awid,
    input  wire [ADDR_WIDTH-1:0]       s_awaddr,
    input  wire [1:0]                  s_awburst,
    input  wire [LEN_WIDTH-1:0]        s_awlen,
    input  wire [2:0]                  s_awsize,
    input  wire                        s_awvalid,
    output reg                         s_awready,

    // Write data channel
    input  wire [DATA_WIDTH-1:0]       s_wdata,
    input  wire [DATA_WIDTH/8-1:0]     s_wstrb,
    input  wire                        s_wlast,
    input  wire                        s_wvalid,
    output reg                         s_wready,

    // Write response channel
    output reg  [ID_WIDTH-1:0]         s_bid,
    output reg  [1:0]                  s_bresp,
    output reg                         s_bvalid,
    input  wire                        s_bready,

    // Read address channel
    input  wire [ID_WIDTH-1:0]         s_arid,
    input  wire [ADDR_WIDTH-1:0]       s_araddr,
    input  wire [1:0]                  s_arburst,
    input  wire [LEN_WIDTH-1:0]        s_arlen,
    input  wire [2:0]                  s_arsize,
    input  wire                        s_arvalid,
    output reg                         s_arready,

    // Read data channel
    output reg  [ID_WIDTH-1:0]         s_rid,
    output reg  [DATA_WIDTH-1:0]       s_rdata,
    output reg  [1:0]                  s_rresp,
    output reg                         s_rlast,
    output reg                         s_rvalid,
    input  wire                        s_rready
);

  // Internal RAM storage
  reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

  // Internal write transaction registers
  reg aw_active;
  reg [ID_WIDTH-1:0] awid_reg;
  reg [ADDR_WIDTH-1:0] awaddr_reg;
  reg [LEN_WIDTH-1:0] awlen_reg;
  reg [1:0] awburst_reg;
  reg [2:0] awsize_reg;
  reg [LEN_WIDTH:0] write_cnt;

  // Internal read transaction registers
  reg ar_active;
  reg [ID_WIDTH-1:0] arid_reg;
  reg [ADDR_WIDTH-1:0] araddr_reg;
  reg [LEN_WIDTH-1:0] arlen_reg;
  reg [1:0] arburst_reg;
  reg [2:0] arsize_reg;
  reg [LEN_WIDTH:0] read_cnt;

  //---------------------------------------------------------------------------
  // Write address handshake
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s_awready  <= 1'b1;
      aw_active  <= 1'b0;
    end else if (s_awvalid && s_awready) begin
      // capture write address info
      s_awready   <= 1'b0;
      aw_active   <= 1'b1;
      awid_reg    <= s_awid;
      awaddr_reg  <= s_awaddr;
      awlen_reg   <= s_awlen;
      awburst_reg <= s_awburst;
      awsize_reg  <= s_awsize;
      write_cnt   <= 0;
    end else if (s_bvalid && s_bready) begin
      // response accepted, ready for next
      s_awready  <= 1'b1;
      aw_active  <= 1'b0;
    end
  end

  // Write data handshake and memory write
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s_wready <= 1'b0;
      s_bvalid <= 1'b0;
    end else begin
      // accept data beats when address phase done
      if (aw_active && !s_wready) begin
        s_wready <= 1'b1;
      end
      if (s_wvalid && s_wready) begin
        // calculate word index (drop lower awsize bits)
        integer byte_index = awaddr_reg[ADDR_WIDTH-1:awsize_reg];
        integer i;
        for (i = 0; i < DATA_WIDTH/8; i = i + 1) begin
          if (s_wstrb[i]) mem[byte_index][8*i +: 8] <= s_wdata[8*i +: 8];
        end
        // increment for next beat
        awaddr_reg <= (awburst_reg == 2'b01) ? awaddr_reg + (1<<awsize_reg) : awaddr_reg;
        write_cnt  <= write_cnt + 1;
        s_wready   <= s_wlast ? 1'b0 : 1'b1;
        // on last beat, drive response
        if (s_wlast) begin
          s_bid    <= awid_reg;
          s_bresp  <= 2'b00;
          s_bvalid <= 1'b1;
        end
      end
      if (s_bvalid && s_bready) begin
        s_bvalid <= 1'b0;
      end
    end
  end

  //---------------------------------------------------------------------------
  // Read address handshake
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s_arready <= 1'b1;
      ar_active <= 1'b0;
    end else if (s_arvalid && s_arready) begin
      // capture read address info
      s_arready   <= 1'b0;
      ar_active   <= 1'b1;
      arid_reg    <= s_arid;
      araddr_reg  <= s_araddr;
      arlen_reg   <= s_arlen;
      arburst_reg <= s_arburst;
      arsize_reg  <= s_arsize;
      read_cnt    <= 0;
    end else if (s_rvalid && s_rready && s_rlast) begin
      // last read accepted
      s_arready <= 1'b1;
      ar_active <= 1'b0;
    end
  end

  // Read data channel
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s_rvalid <= 1'b0;
      s_rlast  <= 1'b0;
    end else if (ar_active && !s_rvalid) begin
      // issue first data beat
      s_rvalid  <= 1'b1;
      s_rid     <= arid_reg;
      s_rresp   <= 2'b00;
      integer idx = araddr_reg[ADDR_WIDTH-1:arsize_reg];
      s_rdata   <= mem[idx];
      s_rlast   <= (read_cnt == arlen_reg);
      // prepare next
      araddr_reg <= (arburst_reg == 2'b01) ? araddr_reg + (1<<arsize_reg) : araddr_reg;
      read_cnt   <= read_cnt + 1;
    end else if (s_rvalid && s_rready) begin
      if (read_cnt <= arlen_reg) begin
        integer idx2 = araddr_reg[ADDR_WIDTH-1:arsize_reg];
        s_rdata  <= mem[idx2];
        s_rlast  <= (read_cnt == arlen_reg);
        araddr_reg<= (arburst_reg == 2'b01) ? araddr_reg + (1<<arsize_reg) : araddr_reg;
        read_cnt <= read_cnt + 1;
      end else begin
        s_rvalid <= 1'b0;
        s_rlast  <= 1'b0;
      end
    end
  end

endmodule

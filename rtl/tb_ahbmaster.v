`include "ahbmaster.v"
module tb_ahbmaster #(
    parameter AW         = 32,
    parameter DATA_WIDTH = 32
)();

// Clock and reset
reg                      HCLK;
reg                      HRESETn;
// Arbiter grant, from arbiter
reg                      HGRANTx;
// Transfer response, from slave
reg                      HREADY;
reg [1:0]                HRESP;
// Data, from slave
reg [DATA_WIDTH - 1:0]   HRDATA;
// Arbiter
wire                     HBUSREQx;
wire                     HLOCKx;
// Transfer type
wire [1:0]               HTRANS;
// Address and control
wire [AW - 1: 0]         HADDR;
wire                     HWRITE;
wire [2:0]               HSIZE;
wire [2:0]               HBURST;
wire [3:0]               HPROT;
// Data
wire [DATA_WIDTH - 1:0]  HWDATA;

localparam TIME_HCLK = 10;

ahbmaster u_ahbmaster (
    .HCLK(HCLK),
    .HRESETn(HRESETn),
    .HGRANTx(HGRANTx),
    .HREADY(HREADY),
    .HRESP(HRESP),
    .HRDATA(HRDATA),
    .HBUSREQx(HBUSREQx),
    .HLOCKx(HLOCKx),
    .HTRANS(HTRANS),
    .HADDR(HADDR),
    .HWRITE(HWRITE),
    .HSIZE(HSIZE),
    .HBURST(HBURST),
    .HPROT(HPROT),
    .HWDATA(HWDATA)
);

always #(TIME_HCLK/2) HCLK = ~HCLK;

// Simulate the functionality of the arbiter through code
always @(HBUSREQx) begin
    HGRANTx = 0;
    if (HBUSREQx == 1) begin
        #1 HGRANTx = 1;
    end else begin
        #1 HGRANTx = 0;
    end
end

// Simulate the functionality of the slave through code
integer i;
reg [DATA_WIDTH - 1:0] memory [127 : 0];
reg [AW - 1: 0] HADDR_r;
reg             HWRITE_r;
reg [1:0]       HTRANS_r;
reg             HREADY_r;
// For read operations, the address needs to be stored for one cycle
always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        HADDR_r <= 0;
    end else begin
        HADDR_r <= HADDR;
    end
end
always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        HWRITE_r <= 0;
    end else begin
        HWRITE_r <= HWRITE;
    end
end
always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        HTRANS_r <= 0;
    end else begin
        HTRANS_r <= HTRANS;
    end
end
always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        HREADY_r <= 0;
    end else begin
        HREADY_r <= HREADY;
    end
end
// WRITE in memory
always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        for (i = 0; i < 8; i++) begin
            memory[i] <= 0;
        end
    end else if (HWRITE_r && HREADY_r && ((HTRANS_r == `NONSEQ) || (HTRANS_r == `SEQ))) begin
        memory[HADDR_r] <= HWDATA;
    end
end
// READ from memory
always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        HRDATA <= 0;
    end else if (!HWRITE && HREADY && ((HTRANS == `NONSEQ) || (HTRANS == `SEQ))) begin
        HRDATA <= memory[HADDR];
    end
end

initial begin
    #0 begin
        HCLK = 0;
        HRESETn = 0;
        HREADY = 0;
        HRESP = 0;
    end
    #4 begin
        HRESETn = 1;
    end
    #40 begin
        @(posedge HCLK)
        HREADY = 1;
        HRESP = `OKAY;
    end
    // HREADY = 0，ERROR test
    #80 begin
        @(posedge HCLK)
        #1
        HREADY = 0;
        HRESP = `ERROR;
        @(posedge HCLK)
        #1
        HREADY = 1;
        HRESP = `OKAY;
    end
    // #1 begin
    //     @(posedge HCLK)
    //     #1
    //     HREADY = 0;
    //     HRESP = `ERROR;
    //     @(posedge HCLK)
    //     #1
    //     HREADY = 1;
    //     HRESP = `OKAY;
    // end
    // HREADY = 0，ERROR test
    #70 begin
        @(posedge HCLK)
        #1
        HREADY = 0;
        HRESP = `ERROR;
        @(posedge HCLK)
        #1
        HREADY = 1;
        HRESP = `OKAY;
    end
    #600
    $finish;
end

initial begin
    $dumpfile("wave_ahbmaster.vcd");
    $dumpvars(0,tb_ahbmaster);
end
    
endmodule
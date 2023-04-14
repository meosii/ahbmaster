`ifndef AHBMASTER
`define AHBMASTER
`include "one_burst.v"
`include "ahb_defs.v"
module ahbmaster #(
    parameter AW         = 32,
    parameter DATA_WIDTH = 32 // Here, we only transfer 32 bits of data
)(
    // Clock and reset
    input wire                      HCLK,
    input wire                      HRESETn,
    // Arbiter grant, from arbiter
    input wire                      HGRANTx,
    // Transfer response, from slave
    input wire                      HREADY,
    input wire [1:0]                HRESP,
    // Data, from slave
    input wire [DATA_WIDTH - 1:0]   HRDATA,
    // Arbiter
    output reg                      HBUSREQx,
    output reg                      HLOCKx,
    // Transfer type
    output reg [1:0]                HTRANS,
    // Address and control
    output reg [AW - 1: 0]          HADDR,
    output reg                      HWRITE,
    output reg [2:0]                HSIZE,
    output reg [2:0]                HBURST,
    output reg [3:0]                HPROT,
    // Data
    output reg [DATA_WIDTH - 1:0]   HWDATA
);

/* 
 Transmission type
 Transmission 1: 0x0 ‐> 0x8, INCR   TRANS_W1 and TRANS_R1
 Transmission 2: 0x10‐>   ?, INCR4  TRANS_W2 and TRANS_R2
 Transmission 3: 0x28‐>   ?, WRAP8  TRANS_W3 and TRANS_R3
*/
localparam NO_TRANS   = 3'b000;
localparam TRANS_W1   = 3'b001;
localparam TRANS_W2   = 3'b010;
localparam TRANS_W3   = 3'b011;
localparam TRANS_R1   = 3'b100;
localparam TRANS_R2   = 3'b101;
localparam TRANS_R3   = 3'b110;

// Here we have six types of transfers
localparam TRANS_NUM     = 6;
// `trans_en` indicates that the type of current transfer
reg [TRANS_NUM - 1 : 0] trans_en;
// trans_en[TRANS_NUM - 1:0] = 
// {trans_en[R3_NUM], trans_en[R2_NUM], trans_en[R1_NUM], 
//  trans_en[W3_NUM], trans_en[W2_NUM], trans_en[W1_NUM]};

localparam W1_NUM        = 0; //trans_en[0] is the w1 enable 
localparam W2_NUM        = 1;
localparam W3_NUM        = 2;
localparam R1_NUM        = 3;
localparam R2_NUM        = 4;
localparam R3_NUM        = 5;
localparam TRANS_EN_NONE = 6'b000_000;
localparam TRANS_EN_W1   = 6'b000_001; // The w1 transport type is selected
localparam TRANS_EN_W2   = 6'b000_010;
localparam TRANS_EN_W3   = 6'b000_100;
localparam TRANS_EN_R1   = 6'b001_000;
localparam TRANS_EN_R2   = 6'b010_000;
localparam TRANS_EN_R3   = 6'b100_000;

// "1" indicates a new transmission start
reg new_trans;
reg new_trans_r;

always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        new_trans_r <= 0;
    end else begin
        new_trans_r <= new_trans;
    end
end

// `trans_finish` comes from one_burst
wire trans_w1_finish;
wire trans_w2_finish;
wire trans_w3_finish;
wire trans_r1_finish;
wire trans_r2_finish;
wire trans_r3_finish;

reg [2:0] c_trans_type;
reg [2:0] n_trans_type;

always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        c_trans_type <= NO_TRANS;
    end else begin
        c_trans_type <= n_trans_type;
    end
end

always @(*) begin
    n_trans_type = NO_TRANS;
    case (c_trans_type)
        NO_TRANS: begin
            n_trans_type = TRANS_W1;
            new_trans    = 1;
            trans_en     = TRANS_EN_NONE;
        end
        TRANS_W1: begin
            trans_en = TRANS_EN_W1;
            if (trans_w1_finish == 1) begin
                n_trans_type = TRANS_W2;
                new_trans    = 1;
            end else begin
                n_trans_type = TRANS_W1;
                new_trans    = 0;
            end
        end
        TRANS_W2: begin
            trans_en = TRANS_EN_W2;
            if (trans_w2_finish == 1) begin
                n_trans_type = TRANS_W3;
                new_trans    = 1;
            end else begin
                n_trans_type = TRANS_W2;
                new_trans    = 0;
            end
        end
        TRANS_W3: begin
            trans_en = TRANS_EN_W3;
            if (trans_w3_finish == 1) begin
                n_trans_type = TRANS_R1;
                new_trans    = 1;
            end else begin
                n_trans_type = TRANS_W3;
                new_trans    = 0;
            end
        end
        TRANS_R1: begin
            trans_en = TRANS_EN_R1;
            if (trans_r1_finish == 1) begin
                n_trans_type = TRANS_R2;
                new_trans    = 1;
            end else begin
                n_trans_type = TRANS_R1;
                new_trans    = 0;
            end
        end
        TRANS_R2: begin
            trans_en = TRANS_EN_R2;
            if (trans_r2_finish == 1) begin
                n_trans_type = TRANS_R3;
                new_trans    = 1;
            end else begin
                n_trans_type = TRANS_R2;
                new_trans    = 0;
            end
        end
        TRANS_R3: begin
            trans_en = TRANS_EN_R3;
            if (trans_r3_finish == 1) begin
                n_trans_type = NO_TRANS;
                new_trans    = 1;
            end else begin
                n_trans_type = TRANS_R3;
                new_trans    = 0;
            end
        end
        default: begin
            n_trans_type = NO_TRANS;
            new_trans    = 0;
            trans_en     = TRANS_EN_NONE;
        end
    endcase
end

// Judge the current transfer type and select the correct HBUSREQx to master
wire w1_HBUSREQx; 
wire w2_HBUSREQx;
wire w3_HBUSREQx;
wire r1_HBUSREQx;
wire r2_HBUSREQx;
wire r3_HBUSREQx;

// Judge the current transfer type and select the correct HBUSLOCKx to master
wire w1_HLOCKx; 
wire w2_HLOCKx;
wire w3_HLOCKx;
wire r1_HLOCKx;
wire r2_HLOCKx;
wire r3_HLOCKx;

// Judge the current transfer type and select the correct HTRANS to master
wire [1:0] w1_HTRANS; 
wire [1:0] w2_HTRANS;
wire [1:0] w3_HTRANS;
wire [1:0] r1_HTRANS;
wire [1:0] r2_HTRANS;
wire [1:0] r3_HTRANS;

// Judge the current transfer type and select the correct HADDR to master
wire [AW - 1: 0] w1_HADDR; 
wire [AW - 1: 0] w2_HADDR;
wire [AW - 1: 0] w3_HADDR;
wire [AW - 1: 0] r1_HADDR;
wire [AW - 1: 0] r2_HADDR;
wire [AW - 1: 0] r3_HADDR;

// Judge the current transfer type and select the correct HWRITE to master
wire w1_HWRITE; 
wire w2_HWRITE;
wire w3_HWRITE;
wire r1_HWRITE;
wire r2_HWRITE;
wire r3_HWRITE;

// Judge the current transfer type and select the correct HSIZE to master
wire [2:0] w1_HSIZE; 
wire [2:0] w2_HSIZE;
wire [2:0] w3_HSIZE;
wire [2:0] r1_HSIZE;
wire [2:0] r2_HSIZE;
wire [2:0] r3_HSIZE;

// Judge the current transfer type and select the correct HBURST to master
wire [2:0] w1_HBURST; 
wire [2:0] w2_HBURST;
wire [2:0] w3_HBURST;
wire [2:0] r1_HBURST;
wire [2:0] r2_HBURST;
wire [2:0] r3_HBURST;

// Judge the current transfer type and select the correct HPROT to master
wire [3:0] w1_HPROT; 
wire [3:0] w2_HPROT;
wire [3:0] w3_HPROT;
wire [3:0] r1_HPROT;
wire [3:0] r2_HPROT;
wire [3:0] r3_HPROT;

// Judge the current transfer type and select the correct HWDATA to master
wire [DATA_WIDTH - 1:0] w1_HWDATA; 
wire [DATA_WIDTH - 1:0] w2_HWDATA;
wire [DATA_WIDTH - 1:0] w3_HWDATA;
wire [DATA_WIDTH - 1:0] r1_HWDATA;
wire [DATA_WIDTH - 1:0] r2_HWDATA;
wire [DATA_WIDTH - 1:0] r3_HWDATA;

always @(*) begin
    case (trans_en)
        TRANS_EN_W1: begin
            HBUSREQx = w1_HBUSREQx;
            HLOCKx   = w1_HLOCKx;
            HTRANS   = w1_HTRANS;
            HADDR    = w1_HADDR;
            HWRITE   = w1_HWRITE;
            HSIZE    = w1_HSIZE;
            HBURST   = w1_HBURST;
            HPROT    = w1_HPROT;
            HWDATA   = w1_HWDATA;
        end
        TRANS_EN_W2: begin
            HBUSREQx = w2_HBUSREQx;
            HLOCKx   = w2_HLOCKx;
            HTRANS   = w2_HTRANS;
            HADDR    = w2_HADDR;
            HWRITE   = w2_HWRITE;
            HSIZE    = w2_HSIZE;
            HBURST   = w2_HBURST;
            HPROT    = w2_HPROT;
            HWDATA   = w2_HWDATA;
        end
        TRANS_EN_W3: begin
            HBUSREQx = w3_HBUSREQx;
            HLOCKx   = w3_HLOCKx;
            HTRANS   = w3_HTRANS;
            HADDR    = w3_HADDR;
            HWRITE   = w3_HWRITE;
            HSIZE    = w3_HSIZE;
            HBURST   = w3_HBURST;
            HPROT    = w3_HPROT;
            HWDATA   = w3_HWDATA;
        end
        TRANS_EN_R1: begin
            HBUSREQx = r1_HBUSREQx;
            HLOCKx   = r1_HLOCKx;
            HTRANS   = r1_HTRANS;
            HADDR    = r1_HADDR;
            HWRITE   = r1_HWRITE;
            HSIZE    = r1_HSIZE;
            HBURST   = r1_HBURST;
            HPROT    = r1_HPROT;
            HWDATA   = r1_HWDATA;
        end
        TRANS_EN_R2: begin
            HBUSREQx = r2_HBUSREQx;
            HLOCKx   = r2_HLOCKx;
            HTRANS   = r2_HTRANS;
            HADDR    = r2_HADDR;
            HWRITE   = r2_HWRITE;
            HSIZE    = r2_HSIZE;
            HBURST   = r2_HBURST;
            HPROT    = r2_HPROT;
            HWDATA   = r2_HWDATA;
        end
        TRANS_EN_R3: begin
            HBUSREQx = r3_HBUSREQx;
            HLOCKx   = r3_HLOCKx;
            HTRANS   = r3_HTRANS;
            HADDR    = r3_HADDR;
            HWRITE   = r3_HWRITE;
            HSIZE    = r3_HSIZE;
            HBURST   = r3_HBURST;
            HPROT    = r3_HPROT;
            HWDATA   = r3_HWDATA;
        end
        default: begin // include TRANS_EN_NONE
            HBUSREQx = 1'b0;
            HLOCKx   = 1'b0;
            HTRANS   = `IDLE;
            HADDR    = 32'b0;
            HWRITE   = `READ;
            HSIZE    = `BYTE;
            HBURST   = `SINGLE;
            HPROT    = 4'b0; 
            HWDATA   = 32'b0;
        end
    endcase
end

one_burst w1_one_burst (
    .trans_en(trans_en[W1_NUM]),
    .new_trans(new_trans_r),
    .hclk(HCLK),
    .hresetn(HRESETn),
    .hgrantx(HGRANTx),
    .haddr({`HADDR_SLAVE0,29'b0}),
    .hwrite(`WRITE), 
    .hsize(`WORD),
    .hburst(`INCR8),
    .hwdata0(`W1_HWDATA_0),
    .hwdata1(`W1_HWDATA_1),
    .hwdata2(`W1_HWDATA_2),
    .hwdata3(`W1_HWDATA_3),
    .hwdata4(`W1_HWDATA_4),
    .hwdata5(`W1_HWDATA_5),
    .hwdata6(`W1_HWDATA_6),
    .hwdata7(`W1_HWDATA_7),
    .hready(HREADY),
    .hresp(HRESP),
    .HBUSREQx(w1_HBUSREQx),
    .HLOCKx(w1_HLOCKx),
    .HTRANS(w1_HTRANS),
    .HADDR(w1_HADDR),
    .HWRITE(w1_HWRITE),
    .HSIZE(w1_HSIZE),
    .HBURST(w1_HBURST),
    .HPROT(w1_HPROT),
    .HWDATA(w1_HWDATA),
    .trans_finish(trans_w1_finish)
);

one_burst w2_one_burst (
    .trans_en(trans_en[W2_NUM]),
    .new_trans(new_trans_r),
    .hclk(HCLK),
    .hresetn(HRESETn),
    .hgrantx(HGRANTx),
    .haddr({`HADDR_SLAVE0,29'h10}),
    .hwrite(`WRITE), 
    .hsize(`WORD),
    .hburst(`INCR4),
    .hwdata0(`W2_HWDATA_0),
    .hwdata1(`W2_HWDATA_1),
    .hwdata2(`W2_HWDATA_2),
    .hwdata3(`W2_HWDATA_3),
    .hwdata4(0),
    .hwdata5(0),
    .hwdata6(0),
    .hwdata7(0),
    .hready(HREADY),
    .hresp(HRESP),
    .HBUSREQx(w2_HBUSREQx),
    .HLOCKx(w2_HLOCKx),
    .HTRANS(w2_HTRANS),
    .HADDR(w2_HADDR),
    .HWRITE(w2_HWRITE),
    .HSIZE(w2_HSIZE),
    .HBURST(w2_HBURST),
    .HPROT(w2_HPROT),
    .HWDATA(w2_HWDATA),
    .trans_finish(trans_w2_finish)
);

one_burst w3_one_burst (
    .trans_en(trans_en[W3_NUM]),
    .new_trans(new_trans_r),
    .hclk(HCLK),
    .hresetn(HRESETn),
    .hgrantx(HGRANTx),
    .haddr({`HADDR_SLAVE0,29'h28}),
    .hwrite(`WRITE), 
    .hsize(`WORD),
    .hburst(`WRAP8),
    .hwdata0(`W3_HWDATA_0),
    .hwdata1(`W3_HWDATA_1),
    .hwdata2(`W3_HWDATA_2),
    .hwdata3(`W3_HWDATA_3),
    .hwdata4(`W3_HWDATA_4),
    .hwdata5(`W3_HWDATA_5),
    .hwdata6(`W3_HWDATA_6),
    .hwdata7(`W3_HWDATA_7),
    .hready(HREADY),
    .hresp(HRESP),
    .HBUSREQx(w3_HBUSREQx),
    .HLOCKx(w3_HLOCKx),
    .HTRANS(w3_HTRANS),
    .HADDR(w3_HADDR),
    .HWRITE(w3_HWRITE),
    .HSIZE(w3_HSIZE),
    .HBURST(w3_HBURST),
    .HPROT(w3_HPROT),
    .HWDATA(w3_HWDATA),
    .trans_finish(trans_w3_finish)
);

one_burst r1_one_burst (
    .trans_en(trans_en[R1_NUM]),
    .new_trans(new_trans_r),
    .hclk(HCLK),
    .hresetn(HRESETn),
    .hgrantx(HGRANTx),
    .haddr({`HADDR_SLAVE0,29'b0}),
    .hwrite(`READ), 
    .hsize(`WORD),
    .hburst(`INCR8),
    .hwdata0(0),
    .hwdata1(0),
    .hwdata2(0),
    .hwdata3(0),
    .hwdata4(0),
    .hwdata5(0),
    .hwdata6(0),
    .hwdata7(0),
    .hready(HREADY),
    .hresp(HRESP),
    .HBUSREQx(r1_HBUSREQx),
    .HLOCKx(r1_HLOCKx),
    .HTRANS(r1_HTRANS),
    .HADDR(r1_HADDR),
    .HWRITE(r1_HWRITE),
    .HSIZE(r1_HSIZE),
    .HBURST(r1_HBURST),
    .HPROT(r1_HPROT),
    .HWDATA(r1_HWDATA),
    .trans_finish(trans_r1_finish)
);

one_burst r2_one_burst (
    .trans_en(trans_en[R2_NUM]),
    .new_trans(new_trans_r),
    .hclk(HCLK),
    .hresetn(HRESETn),
    .hgrantx(HGRANTx),
    .haddr({`HADDR_SLAVE0,29'h10}),
    .hwrite(`READ), 
    .hsize(`WORD),
    .hburst(`INCR4),
    .hwdata0(0),
    .hwdata1(0),
    .hwdata2(0),
    .hwdata3(0),
    .hwdata4(0),
    .hwdata5(0),
    .hwdata6(0),
    .hwdata7(0),
    .hready(HREADY),
    .hresp(HRESP),
    .HBUSREQx(r2_HBUSREQx),
    .HLOCKx(r2_HLOCKx),
    .HTRANS(r2_HTRANS),
    .HADDR(r2_HADDR),
    .HWRITE(r2_HWRITE),
    .HSIZE(r2_HSIZE),
    .HBURST(r2_HBURST),
    .HPROT(r2_HPROT),
    .HWDATA(r2_HWDATA),
    .trans_finish(trans_r2_finish)
);

one_burst r3_one_burst (
    .trans_en(trans_en[R3_NUM]),
    .new_trans(new_trans_r),
    .hclk(HCLK),
    .hresetn(HRESETn),
    .hgrantx(HGRANTx),
    .haddr({`HADDR_SLAVE0,29'h28}),
    .hwrite(`READ), 
    .hsize(`WORD),
    .hburst(`WRAP8),
    .hwdata0(0),
    .hwdata1(0),
    .hwdata2(0),
    .hwdata3(0),
    .hwdata4(0),
    .hwdata5(0),
    .hwdata6(0),
    .hwdata7(0),
    .hready(HREADY),
    .hresp(HRESP),
    .HBUSREQx(r3_HBUSREQx),
    .HLOCKx(r3_HLOCKx),
    .HTRANS(r3_HTRANS),
    .HADDR(r3_HADDR),
    .HWRITE(r3_HWRITE),
    .HSIZE(r3_HSIZE),
    .HBURST(r3_HBURST),
    .HPROT(r3_HPROT),
    .HWDATA(r3_HWDATA),
    .trans_finish(trans_r3_finish)
);

endmodule
`endif
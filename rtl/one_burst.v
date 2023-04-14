`ifndef ONE_BURST
`define ONE_BURST
`include "ahb_defs.v"
module one_burst #(
    parameter AW         = 32,
    parameter DATA_WIDTH = 32,
    parameter MAX_CNT    = 100 // Maximum number of transmissions of indefinite length (INCR)
)(
    // `trans_en` indicates that this module transfer type is being transferred.
    input wire                       trans_en, 
    // `new_trans` indicates that a new transmission has just started.
    input wire                       new_trans,

    // Clock and reset
    input wire                       hclk,
    input wire                       hresetn,
    // This master is be selected
    input wire                       hgrantx,
    /* 
    Here we take haddr[31:29] as the address of each slave, 
    provided to the decoder as a slice selection.
    `haddr` is the address of the first data.
    `hwrite` `hsize` `hburst` will remain in one burst.
    */
    input wire [AW - 1: 0]           haddr,
    input wire                       hwrite, 
    input wire [2:0]                 hsize,
    input wire [2:0]                 hburst,
    // Here, we only provide 8 sets of data, they will remain in one burst.
    input wire [DATA_WIDTH - 1:0]    hwdata0,
    input wire [DATA_WIDTH - 1:0]    hwdata1,
    input wire [DATA_WIDTH - 1:0]    hwdata2,
    input wire [DATA_WIDTH - 1:0]    hwdata3,
    input wire [DATA_WIDTH - 1:0]    hwdata4,
    input wire [DATA_WIDTH - 1:0]    hwdata5,
    input wire [DATA_WIDTH - 1:0]    hwdata6,
    input wire [DATA_WIDTH - 1:0]    hwdata7,
    // `hready` and `hresp` are obtained from slave
    input wire                       hready,
    input wire [1:0]                 hresp,

    output reg                      HBUSREQx,
    output reg                      HLOCKx,
    output reg [1:0]                HTRANS,
    output reg [AW - 1: 0]          HADDR,
    output reg                      HWRITE,
    output reg [2:0]                HSIZE,
    output reg [2:0]                HBURST,
    output reg [3:0]                HPROT,
    output reg [DATA_WIDTH - 1:0]   HWDATA,

    // `trans_finish` indicates that one burst is finished
    output reg                      trans_finish
);
    
/*
 transmission state
*/
localparam NONE     = 3'b000; // No new transmission has started
localparam REQ      = 3'b001; // A new transmission begins and the master sends a request signal
localparam GRANT    = 3'b010; // The arbiter agrees to this master transfer, and the slave has completed its last transfer
localparam TRANSMIT = 3'b011; // In transmitting
localparam DONE     = 3'b100; // Transmission is complete

reg [2:0] c_trans_state;
reg [2:0] n_trans_state;

reg [$clog2(MAX_CNT) :0] already_cnt; // The number of data that has been transferred
reg [$clog2(MAX_CNT) :0] need_cnt; // The total number of data to be transferred

always @(hburst) begin
    case (hburst)
        `SINGLE: need_cnt = 1;
        `INCR:   need_cnt = 100;
        `WRAP4:  need_cnt = 4;
        `INCR4:  need_cnt = 4;
        `WRAP8:  need_cnt = 8;
        `INCR8:  need_cnt = 8;
        `WRAP16: need_cnt = 16;
        `INCR16: need_cnt = 16;
        default: need_cnt = 0;
    endcase
end

always @(posedge hclk or negedge hresetn) begin
    if (!hresetn) begin
        c_trans_state <= NONE;
    end else if (!trans_en) begin
        c_trans_state <= NONE;
    end else begin
        c_trans_state <= n_trans_state;
    end
end

always @(*) begin
    trans_finish  = 0;
    n_trans_state = NONE;
    case (c_trans_state)
        NONE: begin
            if (new_trans == 1) begin
                n_trans_state = REQ;
            end else begin
                n_trans_state = NONE;
            end
        end
        REQ: begin
            if (hgrantx == 1) begin
                n_trans_state = GRANT;
            end else begin
                n_trans_state = REQ;
            end
        end
        GRANT: begin
            if ((hresp == `SPLIT) || (hresp == `RETRY)) begin
                trans_finish  = 1;
                n_trans_state = DONE;
            end else if ((hgrantx == 1) && (hready == 1)) begin // Granting access with no wait states
                if (hburst != `SINGLE) begin
                    n_trans_state = TRANSMIT;
                    trans_finish  = 0;
                end else begin
                    n_trans_state = DONE;
                    trans_finish  = 1;
                end
            end else begin
                n_trans_state = GRANT; // Granting access with wait states
                trans_finish  = 0;
            end
        end
        TRANSMIT: begin
            if ((already_cnt == need_cnt) || (hresp == `SPLIT) || (hresp == `RETRY)) begin
                trans_finish  = 1;
                n_trans_state = DONE;
            end else begin
                trans_finish  = 0;
                n_trans_state = TRANSMIT;
            end
        end
        DONE: begin
            n_trans_state = NONE;
            trans_finish  = 1;
        end
    default: begin
            n_trans_state = NONE;
            trans_finish  = 0;
        end
    endcase
end

/* 
    'already_cnt' corresponds to the sequence number of the data being transferred in the second cycle

    REQ:        Only send request, not granted by arbiter, no data transfer;

    GRANT:      Transfer the first data, the read and write control signal for the first data is sent,
                but the transmission has not been completed;

    TRANSMIT:   AHB transmission is a two-stage pipeline transmission,
                the address and control signal of the first data 
                are given in the GRANT phase, so the TRANSMIT phase is the 
                second cycle for the first data, and we only needs to judge 
                the HREADY of the second cycle to obtain whether the 
                current data transmission is completed.
*/
always @(posedge hclk or negedge hresetn) begin
    if (!hresetn) begin
        already_cnt <= 0;
    end else if ((c_trans_state == NONE) || (c_trans_state == REQ) || (c_trans_state == GRANT)) begin
        already_cnt <= 0;
    end else if ((c_trans_state == TRANSMIT)) begin
        if ((hready == 1) && (hresp == `OKAY) && (already_cnt < need_cnt)) begin
            already_cnt <= already_cnt + 1;
        end else begin
            already_cnt <= already_cnt;
        end
    end else if (c_trans_state == DONE) begin
        already_cnt <= already_cnt;
    end
end

// `HWDATA` is given at the second clock edge
always @* begin
    case (c_trans_state)
        NONE:   HWDATA <= 0;
        REQ:    HWDATA <= 0;
        GRANT:  HWDATA <= 0;
        TRANSMIT:
            if (already_cnt == 0) begin
                HWDATA <= 0;
            end else if (already_cnt == 1) begin // Give the value of the first data
                HWDATA <= hwdata0;
            end else if (already_cnt == 2) begin
                HWDATA <= hwdata1;
            end else if (already_cnt == 3) begin
                HWDATA <= hwdata2;
            end else if (already_cnt == 4) begin
                HWDATA <= hwdata3;
            end else if (already_cnt == 5) begin
                HWDATA <= hwdata4;
            end else if (already_cnt == 6) begin
                HWDATA <= hwdata5;
            end else if (already_cnt == 7) begin
                HWDATA <= hwdata6;
            end else if (already_cnt == 8) begin
                HWDATA <= hwdata7;
            end
        DONE:     HWDATA <= 0;
        default:  HWDATA <= 0;
    endcase
end

always @(posedge hclk or negedge hresetn) begin
    if (!hresetn) begin
        HBUSREQx    <= 0;
        HLOCKx      <= 0;
        HTRANS      <= 0;
        HWRITE      <= 0;
        HSIZE       <= 0;
        HBURST      <= 0;
        HPROT       <= 0;
    end else begin
        case (c_trans_state)
        NONE: begin
            HBUSREQx    <= 0;
            HLOCKx      <= 0;
            HTRANS      <= `IDLE;
            HWRITE      <= 0;
            HSIZE       <= 0;
            HBURST      <= 0;
            HPROT       <= 0;
        end
        REQ: begin // to arbiter
            HBUSREQx    <= 1;
            HLOCKx      <= 1;
        end
        GRANT: begin // Transfer the first data
            HTRANS      <= `NONSEQ;
            HWRITE      <= hwrite;
            HSIZE       <= hsize;
            HBURST      <= hburst;
        end
        TRANSMIT: begin
            // The control signal ends when the last `HADDR` receives `HREADY` valid
            if ((already_cnt >= (need_cnt - 1)) && (hready == 1)) begin
                HWRITE   <= `READ; // When the transfer is complete, it defaults to READ
                HTRANS   <= `IDLE;
                HBUSREQx <= 0;
                HLOCKx   <= 0;
                HSIZE    <= 0;
                HBURST   <= 0;
                HPROT    <= 0;
            end else begin
                HWRITE   <= hwrite;
                HBUSREQx <= 1;
                // For the SPLIT and RETRY response the transfer must be cancelled
                if ((hresp == `RETRY) || (hresp == `SPLIT)) begin
                    HTRANS <= `IDLE;
                end else begin
                    HTRANS <= `SEQ;
                end
            end
        end
        DONE: begin
            HBUSREQx    <= 0;
            HLOCKx      <= 0;
            HTRANS      <= `IDLE;
            HWRITE      <= 0;
            HSIZE       <= 0;
            HBURST      <= 0;
            HPROT       <= 0;
        end
    endcase
    end
end

// Address generation part
always @(posedge hclk or negedge hresetn) begin
    if (!hresetn) begin
        HADDR       <= 0;
    end else begin
        case (c_trans_state)
            NONE:  HADDR <= 0;
            REQ:   HADDR <= 0;
            GRANT: HADDR <= haddr;
            TRANSMIT: 
                // When the `HREADY` sampled at the second cycle is valid,
                // indicating that this current data is transmitted.
                if ((hready == 1) && (hresp == `OKAY) && (already_cnt < (need_cnt - 1))) begin
                    // The address is obtained based on the `HBURST` and `HSIZE`
                    case (HBURST)
                        `INCR: begin
                            case (HSIZE)
                            `BYTE:                 HADDR <= HADDR + 1;
                            `HALFWORD:             HADDR <= HADDR + 2;
                            `WORD:                 HADDR <= HADDR + 4; 
                            `TWO_WORD_LINE:        HADDR <= HADDR + 8;  
                            `FOUR_WORD_LINE:       HADDR <= HADDR + 16;
                            `EIGHT_WORD_LINE:      HADDR <= HADDR + 32; 
                            `SIXTEEN_WORD_LINE:    HADDR <= HADDR + 64;
                            `THIRTY_TWO_WORD_LINE: HADDR <= HADDR + 128;
                            default:               HADDR <= HADDR;
                            endcase 
                        end
                        `INCR4: begin
                            case (HSIZE)
                            `BYTE:                 HADDR <= HADDR + 1;
                            `HALFWORD:             HADDR <= HADDR + 2;
                            `WORD:                 HADDR <= HADDR + 4; 
                            `TWO_WORD_LINE:        HADDR <= HADDR + 8;  
                            `FOUR_WORD_LINE:       HADDR <= HADDR + 16;
                            `EIGHT_WORD_LINE:      HADDR <= HADDR + 32; 
                            `SIXTEEN_WORD_LINE:    HADDR <= HADDR + 64;
                            `THIRTY_TWO_WORD_LINE: HADDR <= HADDR + 128;
                            default:               HADDR <= HADDR;
                            endcase 
                        end
                        `INCR8: begin
                            case (HSIZE)
                            `BYTE:                 HADDR <= HADDR + 1;
                            `HALFWORD:             HADDR <= HADDR + 2;
                            `WORD:                 HADDR <= HADDR + 4; 
                            `TWO_WORD_LINE:        HADDR <= HADDR + 8;  
                            `FOUR_WORD_LINE:       HADDR <= HADDR + 16;
                            `EIGHT_WORD_LINE:      HADDR <= HADDR + 32; 
                            `SIXTEEN_WORD_LINE:    HADDR <= HADDR + 64;
                            `THIRTY_TWO_WORD_LINE: HADDR <= HADDR + 128;
                            default:               HADDR <= HADDR;
                            endcase 
                        end
                        `INCR16: begin
                            case (HSIZE)
                            `BYTE:                 HADDR <= HADDR + 1;
                            `HALFWORD:             HADDR <= HADDR + 2;
                            `WORD:                 HADDR <= HADDR + 4; 
                            `TWO_WORD_LINE:        HADDR <= HADDR + 8;  
                            `FOUR_WORD_LINE:       HADDR <= HADDR + 16;
                            `EIGHT_WORD_LINE:      HADDR <= HADDR + 32; 
                            `SIXTEEN_WORD_LINE:    HADDR <= HADDR + 64;
                            `THIRTY_TWO_WORD_LINE: HADDR <= HADDR + 128;
                            default:               HADDR <= HADDR;
                            endcase 
                        end
                        `WRAP4: begin // We need to judge whether the address reaches the boundary.
                            case (HSIZE)
                            `BYTE: 
                                    if ((HADDR[1:0] == 2'b11) && (HADDR[28:0] != 0)) begin // next HADDR[1:0] == 2'b00
                                        HADDR <= HADDR - (4 - 1);
                                    end else begin
                                        HADDR <= HADDR + 1;
                                    end
                            `HALFWORD:
                                    if ((HADDR[2:0] == 3'b110) && (HADDR[28:0] != 0)) begin // next HADDR[2:0] == 3'b000
                                        HADDR <= HADDR - (8 - 2);
                                    end else begin
                                        HADDR <= HADDR + 2;
                                    end
                            `WORD:
                                    if ((HADDR[3:0] == 4'b1100) && (HADDR[28:0] != 0)) begin // next HADDR[3:0] == 4'b0000
                                        HADDR <= HADDR - (16 - 4);
                                    end else begin
                                        HADDR <= HADDR + 4;
                                    end
                            `TWO_WORD_LINE:
                                    if ((HADDR[4:0] == 5'b11000) && (HADDR[28:0] != 0)) begin // next HADDR[4:0] == 5'b00000
                                        HADDR <= HADDR - (32 - 8);
                                    end else begin
                                        HADDR <= HADDR + 8;
                                    end
                            `FOUR_WORD_LINE:
                                    if ((HADDR[5:0] == 6'b110000) && (HADDR[28:0] != 0)) begin // next HADDR[5:0] == 6'b000000
                                        HADDR <= HADDR - (64 - 16);
                                    end else begin
                                        HADDR <= HADDR + 16;
                                    end
                            `EIGHT_WORD_LINE:
                                    if ((HADDR[6:0] == 7'b1100000) && (HADDR[28:0] != 0)) begin // next HADDR[6:0] == 7'b0000000
                                        HADDR <= HADDR - (128 - 32);
                                    end else begin
                                        HADDR <= HADDR + 32;
                                    end 
                            `SIXTEEN_WORD_LINE:
                                    if ((HADDR[7:0] == 8'b11000000) && (HADDR[28:0] != 0)) begin // next HADDR[7:0] == 8'b00000000
                                        HADDR <= HADDR - (256 - 64);
                                    end else begin
                                        HADDR <= HADDR + 64;
                                    end 
                            `THIRTY_TWO_WORD_LINE:
                                    if ((HADDR[8:0] == 9'b110000000) && (HADDR[28:0] != 0)) begin // next HADDR[8:0] == 9'b000000000
                                        HADDR <= HADDR - (512 - 128);
                                    end else begin
                                        HADDR <= HADDR + 128;
                                    end 
                            default: HADDR <= HADDR;
                            endcase 
                        end
                        `WRAP8:
                            case (HSIZE)
                                `BYTE: 
                                    if ((HADDR[2:0] == 3'b111) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (8 - 1);
                                    end else begin
                                        HADDR <= HADDR + 1;
                                    end
                                `HALFWORD:
                                    if ((HADDR[3:0] == 4'b1110) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (16 - 2);
                                    end else begin
                                        HADDR <= HADDR + 2;
                                    end
                                `WORD:
                                    if ((HADDR[4:0] == 5'b11100) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (32 - 4);
                                    end else begin
                                        HADDR <= HADDR + 4;
                                    end
                                `TWO_WORD_LINE:
                                    if ((HADDR[5:0] == 6'b111000) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (64 - 8);
                                    end else begin
                                        HADDR <= HADDR + 8;
                                    end
                                `FOUR_WORD_LINE:
                                    if ((HADDR[6:0] == 7'b1110000) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (128 - 16);
                                    end else begin
                                        HADDR <= HADDR + 16;
                                    end
                                `EIGHT_WORD_LINE:
                                    if ((HADDR[7:0] == 8'b11100000) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (256 - 32);
                                    end else begin
                                        HADDR <= HADDR + 32;
                                    end 
                                `SIXTEEN_WORD_LINE:
                                    if ((HADDR[8:0] == 9'b111000000) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (512 - 64);
                                    end else begin
                                        HADDR <= HADDR + 64;
                                    end 
                                `THIRTY_TWO_WORD_LINE:
                                    if ((HADDR[9:0] == 10'b1110000000) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (1024 - 128);
                                    end else begin
                                        HADDR <= HADDR + 128;
                                    end 
                                default: HADDR <= HADDR;
                            endcase
                        `WRAP16:
                            case (HSIZE)
                                `BYTE: 
                                    if ((HADDR[3:0] == 4'b1111) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (16 - 1);
                                    end else begin
                                        HADDR <= HADDR + 1;
                                    end
                                `HALFWORD:
                                    if ((HADDR[4:0] == 5'b11110) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (32 - 2);
                                    end else begin
                                        HADDR <= HADDR + 2;
                                    end
                                `WORD:
                                    if ((HADDR[5:0] == 6'b111100) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (64 - 4);
                                    end else begin
                                        HADDR <= HADDR + 4;
                                    end
                                `TWO_WORD_LINE:
                                    if ((HADDR[6:0] == 7'b1111000) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (128 - 8);
                                    end else begin
                                        HADDR <= HADDR + 8;
                                    end
                                `FOUR_WORD_LINE:
                                    if ((HADDR[7:0] == 8'b11110000) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (256 - 16);
                                    end else begin
                                        HADDR <= HADDR + 16;
                                    end
                                `EIGHT_WORD_LINE:
                                    if ((HADDR[8:0] == 9'b111100000) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (512 - 32);
                                    end else begin
                                        HADDR <= HADDR + 32;
                                    end 
                                `SIXTEEN_WORD_LINE:
                                    if ((HADDR[9:0] == 10'b1111000000) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (1024 - 64);
                                    end else begin
                                        HADDR <= HADDR + 64;
                                    end 
                                `THIRTY_TWO_WORD_LINE:
                                    if ((HADDR[10:0] == 11'b11110000000) && (HADDR[28:0] != 0)) begin
                                        HADDR <= HADDR - (2048 - 128);
                                    end else begin
                                        HADDR <= HADDR + 128;
                                    end 
                                default: HADDR <= HADDR;
                            endcase
                        default: HADDR <= HADDR;
                    endcase
                end else if ((hready == 1) && (hresp == `OKAY) && (already_cnt == (need_cnt - 1))) begin
                    HADDR <= 0; // When the last data would be finished, the address is reset to 0
                end else begin
                    HADDR <= HADDR;
                end
            DONE: HADDR <= 0;
        endcase
    end
end

endmodule

`endif
`ifndef AHB_DEFS
`define AHB_DEFS

// HWRITE
`define WRITE 1'b1
`define READ  1'b0

// HTRANS[1:0]
`define IDLE    2'b00 // No data transfer is required
`define BUSY    2'b01 // The next transfer cannot take place immediately
`define NONSEQ  2'b10 // The first transfer of a burst or a single transfer
`define SEQ     2'b11 // Remaining transfers in a burst are SEQUENTIAL

// HRESP[1:0]
`define OKAY    2'b00
`define ERROR   2'b01
`define RETRY   2'b10
`define SPLIT   2'b11

// HSIZE[2:0]
`define BYTE                    3'b000 // 8 bits
`define HALFWORD                3'b001 // 16 bits
`define WORD                    3'b010 // 32 bits
`define TWO_WORD_LINE           3'b011 // 64 bits
`define FOUR_WORD_LINE          3'b100 // 128 bits
`define EIGHT_WORD_LINE         3'b101 // 256 bits
`define SIXTEEN_WORD_LINE       3'b110 // 512 bits
`define THIRTY_TWO_WORD_LINE    3'b111 // 1024 bits

// HBURST[2:0]
`define SINGLE  3'b000 // Single transfer
`define INCR    3'b001 // Incrementing burst of unspecified length
`define WRAP4   3'b010 // 4-beat wrapping burst
`define INCR4   3'b011 // 4-beat incrementing burst
`define WRAP8   3'b100 // 8-beat wrapping burst
`define INCR8   3'b101 // 8-beat incrementing burst
`define WRAP16  3'b110 // 16-beat wrapping burst
`define INCR16  3'b111 //16-beat incrementing burst

// HADDR[31:29]
`define HADDR_SLAVE0 3'b000 // sram slave
`define HADDR_SLAVE1 3'b001 

// wdata we want to write
`define W1_HWDATA_0 32'haa111111
`define W1_HWDATA_1 32'haa121212
`define W1_HWDATA_2 32'haa131313
`define W1_HWDATA_3 32'haa141414
`define W1_HWDATA_4 32'haa151515
`define W1_HWDATA_5 32'haa161616
`define W1_HWDATA_6 32'haa171717
`define W1_HWDATA_7 32'haa181818

`define W2_HWDATA_0 32'hbb212121
`define W2_HWDATA_1 32'hbb222222
`define W2_HWDATA_2 32'hbb232323
`define W2_HWDATA_3 32'hbb242424

`define W3_HWDATA_0 32'hcc313131
`define W3_HWDATA_1 32'hcc323232
`define W3_HWDATA_2 32'hcc333333
`define W3_HWDATA_3 32'hcc343434
`define W3_HWDATA_4 32'hcc353535
`define W3_HWDATA_5 32'hcc363636
`define W3_HWDATA_6 32'hcc373737
`define W3_HWDATA_7 32'hcc383838

`endif
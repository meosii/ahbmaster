# AHB接口的特定传输操作MASTER模块

## 设计内容：

针对给定的AMBA AHB SRAM Slave接口模块，设计特定传输操作 Master 模块

## 设计指标：

AMBA AHB2.0 接口

先写入数据，后读出数据确认

传输要求1：0x0 ‐> 0x8, INCR

传输要求2：0x10‐> ？, INCR4

传输要求2：0x28‐> ？, WRAP8

## 设计思路

在 AHB 传输中，MASTER 需要发送符合 AHB 传输规范的信号，这些信号包括地址、控制、数据和响应信号。我们需要根据 AHB 传输的特点和要求生成这些信号。

首先，我们需要了解 AHB 传输的基本流程和特点。AHB 是两级流水传输，在第一个时钟沿给出地址控制信号，在第二个有效时钟沿（从设备准备好收发数据）给出写数据或接收读数据。这样可以提高传输效率，减少等待时间。地址和数据的对于总线的流水线性质是基本的，并且允许高性能操作，同时仍然为从 SLAVE 提供足够的时间来提供对传输的响应。

在一次突发传输中，控制信号（HWRITE, HBURST, HSIZE, HTRANS）是用来指定传输的方向、长度、大小和类型的，它们在每次突发传输的开始时给出有效值，在此后保持不变，直到一次突发传输结束。

请求信号（HBUSREQx,HLOCKx）在每次突发传输开始时给出有效值，至少保持至 ARBITER 给出同意信号。

地址信号（HADDR）是用来指定从设备的地址空间的，它应该在每个时钟周期内保持稳定，并且在每次突发传输的开始时给出有效值。在此，我们将高位作为从设备的片选地址，低位作为从设备内的地址，该地址在每一周期需要根据数据位宽和突发类型进行自递增或回环运算。

传输类型（HTRANS）指明当前 MASTER 的传输类型（如当前传输第一个数据，当前继续一次突发传输，当前主机忙无数据传输，当前传输结束为空）。最后还需发出该 MASTER 的请求信号和锁定信号，在接收到 ARBITER 的同意信号时，才开始一次传输。

数据信号（HWDATA, HRDATA）是用来传送写数据或读数据的，它们应该在第二个有效时钟沿给出或接收，并且与地址信号对齐。

响应信号（HREADY, HRESP）是用来表示从设备的状态和结果，它们应该在每个时钟周期内给出，并且与数据信号对齐。

在此我们首先设计一个突发传输模块，而 MASTER 由多个突发传输构成。

### 模块 ONE_BURST 的设计

该模块的输入参数包括：

- hclk，hresetn：时钟和复位信号；
- trans_en: 在模块实例化时，对于不同的信号输入，该模块代表不同类型的突发传输，在主机中可以选择不同类型的传输，因此需要一个使能信号告知该模块，当前类型传输是否被选中；
- new_trans：告知该模块开始一次新的传输，用来发出请求信号；
- haddr，hwrite，hsize，hburst，hwdata：由 MASTER 提供，用来告知此次突发传输的类型：读/写，突发个数，数据位宽，起始地址，写数据；
- hgrantx：从仲裁器得到的同意信号；
- hready，hresp：从机的反馈信号。


该模块的输出参数包括：

- HBUSREQx，HLOCKx：主机的请求信号；
- HWRITE，HSIZE，HBURST：用来告知此次突发传输的类型；
- HTRANS：主机传输阶段，告知从机当前总线上传输的类型；
- HADDR：利用 HBURST 和 HSIZE 生成每一周期所需要的地址；
- HWDATA：通过计数器输出当前传输的数据值；
- trans_finish：告知一次突发传输完成，MASTER 可以开始下一个传输类型。

通过状态机控制一次突发传输，将其分为五个阶段：

- NONE： 无传输。这是初始状态，表示没有任何突发传输正在进行或即将进行。
- REQ：  MASTER 发送请求（HBUSREQx）。这是当 MASTER 需要进行一次突发传输时，向 ARBITER 发出请求信号的状态。在这个状态下，MASTER 需要保持 HBUSREQx 信号为高电平，直到收到 ARBITER 的 HGRANT 信号为止。
- GRANT：MASTER 接收到 ARBITER 的 HGRANT 信号，同时 SLAVE 完成上一次传输并且准备好进行当前传输，则由 REQ 进入 GRANT 阶段，该阶段为突发传输的第一个数据阶段，在该阶段 MASTER 需要发出第一个数据的地址（HADDR）以及控制信号（HWRITE, HBURST, HSIZE, HTRANS）。如果 HREADY 为高电平，则只需保持一个周期。
- TRANSMIT：如果传输不是 SINGLE 类型，且从机能继续接收数据（非 SPLIT 或 RETRY），继续一次突发传输，直到数据传输完成，可以通过计数器记录当前传输完成的数据个数。在该阶段，MASTER 需要根据 HBURST 和 HSIZE 的值来确定下一个数据的地址，并且保持 HTRANS 为 NONSEQ 或 SEQ。如果 HREADY 为低电平，则需要等待从机的响应。
- DONE：数据传输结束。这是当突发传输完成后，MASTER 释放 HBUSREQx 信号，并且等待 ARBITER 取消 HGRANT 信号的状态。在这个状态下，MASTER 可以准备下一次突发传输，或者返回 NONE 状态。

``` verilog
// 传输阶段
localparam NONE     = 3'b000; // No new transmission has started
localparam REQ      = 3'b001; // A new transmission begins and the master sends a request signal
localparam GRANT    = 3'b010; // The arbiter agrees to this master transfer, and the slave has completed its last transfer
localparam TRANSMIT = 3'b011; // In transmitting
localparam DONE     = 3'b100; // Transmission is complete

// 传输阶段的转换
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
```

1. 要实现状态的转换，我们需要设计一个计数器，判断当前传输的是第几个数据：

因为传输是两级流水完成的，在第二级流水采集到 HREADY 为高时，将计数值 + 1，如果 HREADY 为低，当前传输的 HADDR 和 HWDATA 都要保持，计数值也要保持，也就是说，计数值等于当前传输的 wdata 在一串突发传输中的排列序数，计数值和 HWDATA 同步变化。

``` verilog
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
```
2. 当已知当前状态后，设计数据输出模块，以下为写数据的分配模块

由计数器已经产生了当前传输的数据个数，利用组合电路将对应数据输出：

``` verilog
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
```

3. 突发传输的控制信号输出

控制信号是 AHB 总线上用于指示和控制突发传输的一组信号，控制信号可以根据当前的 trans_state 给出，其中 HBUSREQx，HLOCKx，HWRITE，HSIZE，HBURST 需要在整个突发传输过程中保持不变；HTRANS 需要接收从设备的反馈给出当前状态；在此不考虑 HPORT。

``` verilog
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
```

4. 地址产生器

在突发传输中，地址可以根据首地址以及传输的突发类型、数据位数直接算出。

首先，我们来看 WRAP4，HSIZE = WORD 的情况。这里 WRAP4 表示一次突发传输 4 个数据，HSIZE = WORD 表示每个数据是 32 位的。在内存中，每个地址空间可以存放 8 位的值，因此一个 32 位的值需要占据 4 个地址空间。假如起始地址为 0x34，那么下一个地址应该是 0x34 + 4 = 0x38，依次类推。WRAP 是一种回环传输，它的目的是为了保证在缓存中每次只能取一条缓存行，不能跨过边界。因此当地址遇到边界时，需要返回到该行的起始。由于 WRAP4，HSIZE = WORD 的情况下，一条缓存行有 4 * 32 = 128 位，也就是 16 个地址空间，每个地址空间存放 8 位的值。那么如果起始地址为 0x34，需要的地址依次为：0x34，0x38，0x3c，0x30。

接下来，我们来看 WRAP8，HSIZE = WORD 的情况。这里 WRAP8 表示一次突发传输 8 个数据，HSIZE = WORD 表示每个数据是 32 位的。在内存中，每个地址空间可以存放 8 位的值，因此一个 32 位的值需要占据 4 个地址空间。假如起始地址为 0x34，那么下一个地址应该是 0x34 + 4 = 0x38，依次类推。WRAP 是一种回环传输，它的目的是为了保证在缓存中每次只能取一条缓存行，不能跨过边界。因此当地址遇到边界时，需要返回到该行的起始。由于 WRAP8，HSIZE = WORD 的情况下，一条缓存行有 8 * 32 = 256 位，也就是 32 个地址空间，每个地址空间存放 8 位的值。那么如果起始地址为 0x34，需要的地址依次为：0x34，0x38，0x3c，0x20，0x24，0x28，0x2c，0x30。

最后，我们来看 WRAP8，HSIZE = BYTE 的情况。这里 WRAP8 表示一次突发传输 8 个数据，HSIZE = BYTE 表示每个数据是 8 位的。在内存中，每个地址空间可以存放一个数据（即一个字节），因此一个数据不需要占据多个地址空间。假设起始地址为 0x34，那么下一个地址应该是 0x34 + 1 = 0x35，此时一条缓存行有 有 8 * 8 = 64 位，也就是 8 个地址空间，每个地址中存放 8 位值，那么需要的地址依次为： 0x34，0x35，0x36，0x37，0x30，0x31，0x32，0x33。

那么如何计算这些地址呢？

对于 INRC，只需要判断 HSIZE，即可得到递增的地址值，如 

- BYTE：    +1
- HALFWORD：+2
- WORD：    +4
- TWO_WORD_LINE： +8
- ···

对于 WRAP，需要判断什么时候到达边界，需要将地址减去 line 的总地址个数，如

1. WRAP4
- BYTE：4 * 8，一行的地址数为 4，当地址可以被 4 整除，将这个地址减去 4，但是需要注意的，这个能被整除的地址是不能出现的，即 0x32，0x33，(0x34) -> 0x30，0x31 ，括号里的地址在算出来的同时就要被去掉，因此，要判断的是（能被 4 整除 - 1）的这个数，由这个数减去 （4 - 1），得到该行的第一个地址。（0，1，2，3）
- HALFWORD：4 * 16 = 8 * 8，一行的地址数为 8，判断（能被 8 整除 - 2）的这个数，由这个数减去 （8 - 2），得到该行的第一个地址，0x34，0x36，（0x38）-> 0x30，0x32。（0，2，4，6）
- WORD：    4 * 32 = 16 * 8，一行的地址数为 16，判断（能被 16 整除 - 4）的这个数，由这个数减去 （16 - 4），得到该行的第一个地址，0x34，0x36，0x38，0x3a，0x3c，0x3e，（0x40）-> 0x30，0x32。(0，4，8，c)
- TWO_WORD_LINE： 4 * 64 = 32 * 8，一行的地址数为 32，判断当前地址为（能被 32 整除 - 8）的这个数，由这个数减去 （32 - 8），得到该行的第一个地址，0x34，0x36，0x38，0x3a，0x3c，0x3e，（0x40）-> 0x30，0x32。(0，8，0x10，0x18)
- ···

2. WRAP8
同理

如何判断是否被 4， 8， 16 整除：低位为 2'b00， 3'b000， 4'b0000，因此 (4-1)， (8-2)， (16-4) 分别对应 2'b11， 3'b110， 4'b1100。

``` verilog
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
                        `INCR4:  ...
                        `INCR8:  ...
                        `INCR16: ...

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
                        `WRAP8:  ...
                        `WRAP16: ...
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
```

### AHBMASTER 的设计

有了 one_burst 模块后，我们可以简化 burst 传输的设计和实现，得到 MASTER 模块。one_burst 模块是一个通用的 burst 传输控制器，它可以根据输入的参数（haddr，hwrite，hsize，hburst，hwdata）生成相应的 AHB 信号（HBUSREQx，HLOCKx，HTRANS，HADDR，HWRITE，HSIZE，HBURST，HPROT，HWDATA）。我们可以为每一种 burst 类型（INCR，WRAP，SINGLE）实例化一个 one_burst 模块，并通过一个状态机来切换当前的 burst 类型。状态机的输入是 burst 类型的选择信号（trans_en），输出各个 one_burst 模块的结束信号（trans_finish）。状态机根据 trans_en 的值来决定当前激活哪一个 one_burst 模块，并将其输出通过一个多路选择器（mux）作为 MASTER 的输出。ARBITER 和 SLAVE 的反馈信号（HREADY，HRDATA，HRESP）则统一输入给每一个 one_burst 模块，以便于它们根据反馈信号来更新自己的状态。这样，我们就可以用一组 one_burst 模块来实现不同类型的 burst 传输，而不需要为每一种类型单独设计一个控制器。

``` verilog
/* 
 Transmission type
 Transmission 1: 0x0 ‐> 0x8, INCR
 Transmission 2: 0x10‐>   ?, INCR4
 Transmission 3: 0x28‐>   ?, WRAP8
*/
localparam NO_TRANS   = 3'b000;
localparam TRANS_W1   = 3'b001;
localparam TRANS_W2   = 3'b010;
localparam TRANS_W3   = 3'b011;
localparam TRANS_R1   = 3'b100;
localparam TRANS_R2   = 3'b101;
localparam TRANS_R3   = 3'b110;

always @(*) begin
    n_trans_type = NO_TRANS;
    case (c_trans_type)
        NO_TRANS: begin
            n_trans_type = TRANS_W1;
            new_trans    = 1;
            trans_en = TRANS_EN_NONE;
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
            ···
        end
        TRANS_W3: begin
            ···
        end
        TRANS_R1: begin
            ···
        end
        TRANS_R2: begin
            ···
        end
        TRANS_R3: begin
            ···
        end
        default: begin
            n_trans_type = NO_TRANS;
            new_trans    = 0;
            trans_en     = TRANS_EN_NONE;
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
```

## 测试

要完成一次完整的 AHB 传输，需要 MASTER，ARBITER，SLAVE 共同作用，在此测试模块模拟 ARBITER 与 SLAVE，如下所示：

- ARBITER: 假设仲裁器接收到请求信号后，便输出同意信号；

``` verilog
always @(HBUSREQx) begin
    HGRANTx = 0;
    if (HBUSREQx == 1) begin
        #1 HGRANTx = 1;
    end else begin
        #1 HGRANTx = 0;
    end
end
```

- SLAVE: 在测试模块加一个存储模块，对其进行读写操作，从机反馈的 HREADY 与 HRESP 默认为 1 与 OKAY；也可在 initial 块中进行更改赋值。对于 SLAVE 的考虑，在 HREADY 为 0 的时钟周期，地址阶段的所有控制信号都记得要打一拍。

``` verilog
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
```
- MASTER: 实例化本设计模块

``` verilog
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
```

通过给出时钟复位信号，从机的反馈信号 HREADY 与 HRESP 默认为 1 与 OKAY，即可开始传输，六种类型的突发传输如下：

1. Eight-beat incrementing burst

如图所示为 INCR8 传输，起始地址为 0x00，由于 HSIZE = WORD，传输数据为 32 bits，占据 4 个地址空间，所以每次地址递增 + 4，当 HTRANS 为 NONSEQ 时，开始传输第 1 个数据的地址，若此时 HREADY 为高，则在第二的时钟周期传输第 1 个数据值 HWDATA，为了保证 SRAM 的从设备可以正确读到数据，在 SRAM 中需要将地址打一拍，从而在一个时钟沿采样地址和数据。当传输到第 8 个数据，第一拍给出地址信号，采样第二拍时钟沿的 HREADY，如果此时为高，则当前传输可判断为执行完成，输出 finish 信号，第二个时钟上升沿即可拉低控制信号，同时给出最后一个数据的 HWDATA。当第二个时钟周期结束，下一个时钟沿采集到 finish 信号，将传输交给下一类型。

[![ppxhjvq.png](https://s1.ax1x.com/2023/04/13/ppxhjvq.png)](https://imgse.com/i/ppxhjvq)

下图同为第一类型的突发传输，只是在最后一个传输数据的地址阶段拉低 HREADY，此时将保持地址和数据（注意，此时的数据是倒数第二个数据值），直到 HREADY 为高，完成传输。

[![ppx4rzn.png](https://s1.ax1x.com/2023/04/13/ppx4rzn.png)](https://imgse.com/i/ppx4rzn)

2. Four-beat wrapping burst

如图所示为 WRAP4 传输，起始地址为 0x10，由于 HSIZE = WORD，传输数据为 32 bits，占据 4 个地址空间，所以每次地址递增 + 4，WRAP 传输到达边界会将地址回环，此处恰巧首地址由边界开始，未到达边界。当 HTRANS 为 NONSEQ 时，开始传输第 1 个数据的地址，此时 HREADY 为高，则在第二的时钟周期传输第 1 个数据值 HWDATA。当传输到第 4 个数据，第一拍给出地址信号，采样第二拍时钟沿的 HREADY，如果此时为高，则当前传输可判断为执行完成，输出 finish 信号，第二个时钟上升沿即可拉低控制信号，同时给出最后一个数据的 HWDATA。当第二个时钟周期结束，下一个时钟沿采集到 finish 信号，将传输交给下一类型。

[![ppx5ZWj.png](https://s1.ax1x.com/2023/04/13/ppx5ZWj.png)](https://imgse.com/i/ppx5ZWj)

同理在最后一个传输数据的地址阶段拉低 HREADY，此时将保持地址和数据（注意，此时的数据是倒数第二个数据值），直到 HREADY 为高，完成传输。

[![ppx5uyq.png](https://s1.ax1x.com/2023/04/13/ppx5uyq.png)](https://imgse.com/i/ppx5uyq)

3. Eight-beat wrapping burst

如图所示为 WRAP8 传输，起始地址为 0x28，由于 HSIZE = WORD，传输数据为 32 bits，占据 4 个地址空间，所以每次地址递增 + 4，WRAP 传输到达边界会将地址回环，其中 ADDR7 地址 0x3c 继续 + 4 为 0x40，可以被 （8 * 4 = 32）整除，因此判断其即将到达边界，对 0x3c - （32 - 4），即得到下一地址 0x20。

[![ppxISNF.png](https://s1.ax1x.com/2023/04/13/ppxISNF.png)](https://imgse.com/i/ppxISNF)

4. Eight-beat incrementing burst

该传输为传输类型1 的读操作，注意，传输类型2 的地址覆盖了传输类型1，此时读传输类型1 时，正确的输出 HRDATA 应该为前 4 个数为传输类型1 的输入，后四个数为传输类型2 的输入。

[![ppxH1JS.png](https://s1.ax1x.com/2023/04/13/ppxH1JS.png)](https://imgse.com/i/ppxH1JS)

5. 
传输类型2 的输出如下：

[![ppxHYss.png](https://s1.ax1x.com/2023/04/14/ppxHYss.png)](https://imgse.com/i/ppxHYss)

6. 
传输类型3 的输出如下：

[![ppxH8zQ.png](https://s1.ax1x.com/2023/04/14/ppxH8zQ.png)](https://imgse.com/i/ppxH8zQ)


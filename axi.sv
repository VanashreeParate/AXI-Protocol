`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11.08.2025 19:13:19
// Design Name: 
// Module Name: axi
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

//axi3 Full slave considered to be memory
module axi_slave( 
//global signals
input clk,  
input resetn,

//write address signals
input awvalid,
input [3:0] awid,
input [31:0] awaddr,
input [3:0] awlen,
input [2:0] awsize,
input [1:0] awburst,
output reg awready,

//write data channel
input wvalid,
input [31:0] wdata,
input [3:0] wid,
input [3:0] wstrb,
input wlast,
output reg wready,

//write response channel
input bready,
output reg bvalid,
output reg [1:0] bresp,
output reg [3:0] bid,

//read address channel
input arvalid,
input [3:0] arid,
input [31:0] araddr,
input [3:0] arlen,
input [2:0] arsize,
input [1:0] arburst,
output reg arready,

//read data/response channel
input rready,
output reg rvalid,
output reg [31:0] rdata,
output reg rlast,
output reg [1:0] rresp,
output reg [3:0] rid
    );
    
    reg [7:0] mem [0:127]; //memory has 128 locations of 8 bits each
    
    /////////////////////////RESET DECODER//////////////////////////
    //reset decoder
    always_ff@(posedge clk)
    begin
        if(resetn==0) 
        begin
            for(int i=0; i<128; i++) mem[i]<=i;
        end
    end 
    
    
    
    //consider different fsm for each channel
    
    
    /////////////////////////////WRITE ADDRESS CHANNEL//////////////////////////////
    //states for write address channel
    
    typedef enum bit [1:0] {aw_idle=0, aw_start=1, aw_ready=2} aw_state_type; 
    aw_state_type aw_state;
    
    reg [31:0] awaddr_temp; //temporary register to store awaddr
    
    //fsm for write address channel
    always_ff@(posedge clk)
    begin
        if(resetn==0)
        begin
            aw_state <= aw_idle;
        end
        else
        begin
            case(aw_state)
            aw_idle : begin
                awready <=1'b0;
                if(awvalid==1)
                begin
                    aw_state <= aw_start;
                end
                else
                begin
                    aw_state <= aw_idle;
                end
            end
            
            aw_start : begin
                awready = 0;
                if(awvalid) 
                begin
                    awaddr_temp <= awaddr;
                    aw_state <= aw_ready;
                end
                else
                    aw_state <= aw_start;
            end
            
            aw_ready : begin
                awready <= 1'b1;
                aw_state <= aw_idle;
            end
            
            default : aw_state <= aw_idle; 
    //dont need default since gives error (if sequential reset decoder and combinational blocks are different)
    //that aw_state is driven multiple times. here and in reset decoder
            endcase
        end
        
    end
    
    
    
    ///////////////////////////////WRITE DATA CHANNEL/////////////////////////
    //states for write data channel
    
    typedef enum bit[1:0] {w_idle=0, w_start=1, w_store=2, w_ready_deassert=3} w_state_type;
    w_state_type w_state; 
    
    reg [31:0] wdata_temp; //temporary register to store wdata
    reg [31:0] write_addr, w_return_addr; //current and next adresses
    reg first; //for first beat awaddr is current address
    reg [7:0] boundary; //for boundary in wrap burst
    reg [4:0] burst_len; //to track number of beats
    
    //fsm for write data channel
    always_ff@(posedge clk)
    begin
        if(resetn==0)
        begin
            w_state <= w_idle;
        end
        else
        begin
            case(w_state)
            w_idle : begin
                wready <= 0;
                first <= 0;
                burst_len <= 0;
                w_state <= w_start;
            end
            
            w_start : begin
                if(wvalid) 
                begin
                    wready <= 1'b0;
                    wdata_temp <= wdata;
                    burst_len <= burst_len + 1;
                    if(first==0)
                    begin
                        first = 1;
                        write_addr <= awaddr_temp;
                    end
                    else
                    begin
                        write_addr <= w_return_addr;
                    end
                    w_state <= w_store;
                end
                else
                begin
                    w_state <= w_start; //in same state till wvalid is asserted
                end
            end
            
            w_store : begin
                wready <= 1'b1;
                w_state <= w_ready_deassert;
                case(awburst)
                2'b00 : begin //fixed
                    w_return_addr <= data_wr_fixed(awaddr, wstrb);
                end
                
                2'b01 : begin //incr
                    w_return_addr <= data_wr_incr(write_addr, wstrb);
                end
                
                2'b10 : begin //wrap
                    boundary = wrap_boundary(awsize, awlen);
                    w_return_addr = data_wr_wrap(write_addr, boundary, wstrb);
                end
                endcase
            end
            
            w_ready_deassert : begin
                wready <= 1'b0;
                if(wlast==0 || burst_len != (awlen+1))
                begin
                    w_state <= w_start;
                end
                else
                begin
                    w_state <= w_idle; //if end of burst goes to w_idle state
                end
            end
            
            default : w_state <= w_idle;
            endcase  
        end
    end
    
    
    
    /////////////////////////////WRITE RESPONSE CHANNEL////////////////////////
    //states for write response channel

    typedef enum bit[1:0] {b_idle=0, b_detect_last=1, b_start=2, b_wait=3} b_state_types;
    b_state_types b_state;
    
    //fsm for write response channel
    always_ff@(posedge clk)
    begin
        if(resetn==0)
        begin
            b_state <= b_idle;
        end
        else
        begin
            case(b_state)
            b_idle : begin
                bvalid <= 0;
                bresp <= 0;
                bid <= 0;
                b_state <= b_detect_last;
            end
            
            b_detect_last : begin
                if(wlast || (burst_len==awlen+1)) //waits till last beat is detected
                    b_state <= b_start;
                else
                    b_state <= b_detect_last;
            end
            
            b_start : begin
                bvalid <= 1;
                bid <= awid;
                if((awaddr < 128) && (awsize<=3))
                    bresp <= 2'b00;
                else if(awaddr > 127) 
                    bresp <= 2'b11;
                else
                    bresp <= 2'b11;
                b_state <= b_wait;
            end
            
            b_wait : begin
                if(bready) //waits till bready is asserted
                    b_state <= b_idle;
                else
                    b_state <= b_wait;
            end
            
            default : b_state <= b_idle;
            endcase
        end
    end
    
    
    
    //////////////////////////////READ ADDRESS CHANNEL/////////////////////////////
    //states for read address channel

    typedef enum bit[1:0] {ar_idle=0, ar_start=1, ar_ready=2} ar_state_type;
    ar_state_type ar_state;
    
    reg [31:0] araddr_temp; //temporary register to store read address
    
    //fsm for read address channel
    always_ff@(posedge clk)
    begin
        if(resetn==0)
        begin
            ar_state <= ar_idle;
        end
        else
        begin
            case(ar_state)
            ar_idle : begin
                arready <= 0;
                ar_state <= ar_start;
            end
            
            ar_start : begin
                if(arvalid==1) 
                begin
                    araddr_temp <= araddr;
                    ar_state <= ar_ready;
                end
                else
                    ar_state <= ar_start;
            end
            
            ar_ready : begin
                arready <= 1'b1;
                ar_state <= ar_idle;
            end
            
            default : ar_state <= ar_idle;
            endcase
        end
    end
    
    


    ///////////////////////////READ DATA CHANNEL////////////////////////
    //states for read data channel
    typedef enum bit [2:0] {r_idle=0, r_start=1, r_load=2, r_wait=3, r_valid_deassert=4} r_state_type;
    r_state_type r_state;
    
    reg [4:0] r_burst_len; //to track beats of burst
    reg r_first; //first beat current address is araddr else is calculated return addr
    reg [31:0] read_addr, r_return_addr; //current and next address 
    reg [7:0] r_boundary; //boundary for wrap burst
    
    always_ff@(posedge clk) begin
        if(resetn == 0) begin
            r_state <= r_idle;
        end
        else begin
            case(r_state)
            r_idle : begin
                rvalid <= 0;
                rresp <= 0;
                rid <= 0;
                rdata <= 0;
                rlast <= 0;
                r_first <= 0;
                r_burst_len <= 0;
                if(arvalid == 1 && arready == 1)
                    r_state <= r_start;
                else
                    r_state <= r_idle;
            end
            
            r_start : begin
                if((araddr_temp < 128) && (arsize <= 3'b010)) begin //if all parameters in bound
                    rresp <= 2'b00; //response = OKAY
                    rid <= arid;
                    r_burst_len <= r_burst_len + 1;                  
                    if(r_first == 0) begin
                        r_first <= 1;
                        read_addr <= araddr_temp;
                    end
                    else begin
                        read_addr <= r_return_addr;
                    end                    
                end
                else if((araddr_temp >= 128) && (arsize <= 3'b010)) begin 
                    rresp <= 2'b11; //rresp = DECERR
                    rid <= arid;
                    r_burst_len <= r_burst_len + 1;
                end
                else begin
                    rresp <= 2'b11; //rresp = DECERR
                    rid <= arid;
                    r_burst_len <= r_burst_len + 1;
                end
                r_state <= r_load;
            end
            
            r_load : begin
                rvalid <= 1'b1;
                if(r_burst_len == (arlen+1))
                    rlast <= 1;
                else
                    rlast <= 0;
                if(rresp==2'b00)
                begin
                    case(arburst)
                    2'b00 : begin //fixed
                        r_return_addr <= r_data_fixed(read_addr, arsize);
                    end
                    2'b01 : begin //incr
                        r_return_addr <= r_data_incr(read_addr, arsize);
                    end
                    2'b10 : begin //wrap
                        r_boundary = wrap_boundary(arsize, arlen);
                        r_return_addr = r_data_wrap(read_addr, r_boundary, arsize);
                    end
                    endcase
                end
                r_state <= r_wait;
            end
            
            r_wait : begin
                if(rready) 
                begin
                    r_state <= r_valid_deassert;
                end
                else
                    r_state <= r_wait;
            end
            
            r_valid_deassert : begin
                rvalid <= 0;
                rlast <= 0;
                if(r_burst_len == (arlen + 1))
                    r_state <= r_idle;
                else
                    r_state <= r_start;
            end 
            endcase
        end
    end

    
    
    ///////////////////////////////BURST FUNCTIONS//////////////////////////////
    
    //write data fixed burst
    function bit [31:0] data_wr_fixed(input bit [31:0] addr, input [3:0] w_strb);
        unique case(w_strb)
        4'b0001 : begin
            mem[addr] = wdata_temp[7:0];
        end
        
        4'b0010 : begin
            mem[addr] = wdata_temp[15:8];
        end
        
        4'b0011: begin
            mem[addr] = wdata_temp[7:0];
            mem[addr+1] = wdata_temp[15:8];
        end
        
        4'b0100 : begin
            mem[addr] = wdata_temp[23:16];
        end
        
        4'b0101 : begin
            mem[addr] = wdata_temp[7:0];
            mem[addr+1] = wdata_temp[23:16];
        end
        
        4'b0110 : begin
            mem[addr] = wdata_temp[15:8];
            mem[addr+1] = wdata_temp[23:16];
        end
        
        4'b0111 : begin
            mem[addr] = wdata_temp[7:0];
            mem[addr+1] = wdata_temp[15:8];
            mem[addr+2] = wdata_temp[23:16];
        end
        
        4'b1000 : begin
            mem[addr] = wdata_temp[31:24];
        end
        
        4'b1001 : begin
            mem[addr] = wdata_temp[7:0];
            mem[addr+1] = wdata_temp[31:24];
        end
        
        4'b1010 : begin
            mem[addr] = wdata_temp[15:8];
            mem[addr+1] = wdata_temp[31:24];
        end
        
        4'b1011 : begin
            mem[addr] = wdata_temp[7:0];
            mem[addr+1] = wdata_temp[15:8];
            mem[addr+2] = wdata_temp[31:24];
        end
        
        4'b1100 : begin
            mem[addr] = wdata_temp[23:16];
            mem[addr+1] = wdata_temp[31:24];
        end
        
        4'b1101 : begin
            mem[addr] = wdata_temp[7:0];
            mem[addr+1] = wdata_temp[23:16];
            mem[addr+2] = wdata_temp[31:24];
        end
        
        4'b1110 : begin
            mem[addr] = wdata_temp[15:8];
            mem[addr+1] = wdata_temp[23:16];
            mem[addr+2] = wdata_temp[31:24];
        end
        
        4'b1111 : begin
            mem[addr] = wdata_temp[7:0];
            mem[addr+1] = wdata_temp[15:8];
            mem[addr+2] = wdata_temp[23:16];
            mem[addr+3] = wdata_temp[31:24];
        end
        endcase
        
        return addr; //returns same address
    endfunction
    
    
    
    //write data incr burst
    function bit [31:0] data_wr_incr(input bit [31:0] addr, input bit [3:0] w_strb);
        unique case(w_strb)
        4'b0001 : begin
            mem[addr] = wdata_temp[7:0];
            return (addr+1);
        end
        
        4'b0010 : begin
            mem[addr] = wdata_temp[15:8];
            return (addr+1);
        end
        
        4'b0011 : begin
            mem[addr] = wdata_temp[7:0];
            mem[addr+1] = wdata_temp[15:8];
            return (addr+2);
        end
        
        4'b0100 : begin
            mem[addr] = wdata_temp[23:16];
            return (addr+1);
        end
        
        4'b0101 : begin
            mem[addr] = wdata_temp[7:0];
            mem[addr+1] = wdata_temp[23:16];
            return (addr+2);
        end
        
        4'b0110 : begin
            mem[addr] = wdata_temp[15:8];
            mem[addr+1] = wdata_temp[23:16];
            return (addr+2);
        end
        
        4'b0111 : begin
            mem[addr] = wdata_temp[7:0];
            mem[addr+1] = wdata_temp[15:8];
            mem[addr+2] = wdata_temp[23:16];
            return (addr+3);
        end
        
        4'b1000 : begin
            mem[addr] = wdata_temp[31:24];
            return (addr+1);
        end
        
        4'b1001 : begin
            mem[addr] = wdata_temp[7:0];
            mem[addr+1] = wdata_temp[31:24];
            return (addr+2);
        end
        
        4'b1010 : begin
            mem[addr] = wdata_temp[15:8];
            mem[addr+1] = wdata_temp[31:24];
            return (addr+2);
        end
        
        4'b1011 : begin
            mem[addr] = wdata_temp[7:0];
            mem[addr+1] = wdata_temp[15:8];
            mem[addr+2] = wdata_temp[31:24];
            return (addr+3);
        end
        
        4'b1100 : begin
            mem[addr] = wdata_temp[23:16];
            mem[addr+1] = wdata_temp[31:24];
            return (addr+2);
        end
        
        4'b1101 : begin
            mem[addr] = wdata_temp[7:0];
            mem[addr+1] = wdata_temp[23:16];
            mem[addr+2] = wdata_temp[31:24];
            return (addr+3);
        end
        
        4'b1110 : begin
            mem[addr] = wdata_temp[15:8];
            mem[addr+1] = wdata_temp[23:16];
            mem[addr+2] = wdata_temp[31:24];
            return (addr+3);
        end
        
        4'b1111 : begin
            mem[addr] = wdata_temp[7:0];
            mem[addr+1] = wdata_temp[15:8];
            mem[addr+2] = wdata_temp[23:16];
            mem[addr+3] = wdata_temp[31:24];
            return (addr+4);
        end
        endcase
    endfunction
    
    
    
    //boundary function
    function bit [7:0] wrap_boundary(input bit [2:0] size, input bit [3:0] len);
        bit [7:0] boundary;
        unique case(len)
        4'b0001 : begin
            unique case(size)
                3'b00 : boundary = 2 * 1;
                3'b01 : boundary = 2 * 2;
                3'b10 : boundary = 2 * 4;
            endcase
        end
        
        4'b0011 : begin
            unique case(size)
                3'b00 : boundary = 4 * 1;
                3'b01 : boundary = 4 * 2;
                3'b10 : boundary = 4 * 4;
            endcase
        end
        
        4'b0111 : begin
            unique case(size)
                3'b00 : boundary = 8 * 1;
                3'b01 : boundary = 8 * 2;
                3'b10 : boundary = 8 * 4;
            endcase
        end 
        
        4'b1111 : begin
            unique case(size)
                3'b00 : boundary = 16 * 1;
                3'b01 : boundary = 16 * 2;
                3'b10 : boundary = 16 * 4;
            endcase
        end
        endcase
        
        return boundary;
    endfunction
    
    
    //write data wrap burst
    function bit [31:0] data_wr_wrap(input bit [31:0] addr, input bit [7:0] boundary, input bit [3:0] w_strb);
        bit [31:0] addr1, addr2, addr3, addr4;
        unique case(w_strb)
        4'b0001 : begin
            mem[addr] = wdata_temp[7:0];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = addr+1;
            return addr1;
        end
        
        4'b0010 : begin
            mem[addr] = wdata_temp[15:8];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = addr+1;
            return addr1;
        end
        
        4'b0011 : begin
            mem[addr] = wdata_temp[7:0];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = addr+1;
            mem[addr1] = wdata_temp[15:8];
            if((addr1+1) % boundary == 0)
                addr2 = (addr1+1) - boundary;
            else
                addr2 = addr1+1;
            return addr2;
        end
        
        4'b0100 : begin
            mem[addr] = wdata_temp[23:16];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = addr+1;
            return addr1;
        end
        
        4'b0101 : begin
            mem[addr] = wdata_temp[7:0];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = addr+1;
            mem[addr1] = wdata_temp[23:16];
            if((addr1+1) % boundary == 0)
                addr2 = (addr1+1) - boundary;
            else
                addr2 = addr1+1;
            return addr2;
        end
        
        4'b0110 : begin
            mem[addr] = wdata_temp[15:8];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = addr+1;
            mem[addr1] = wdata_temp[23:16];
            if((addr1+1) % boundary == 0)
                addr2 = (addr1+1) - boundary;
            else
                addr2 = addr1+1;
            return addr2;
        end
        
        4'b0111 : begin
            mem[addr] = wdata_temp[7:0];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = addr+1;
            mem[addr1] = wdata_temp[15:8];
            if((addr1+1) % boundary == 0)
                addr2 = (addr1+1) - boundary;
            else
                addr2 = addr1+1;
            mem[addr2] = wdata_temp[23:16];
            if((addr2+1) % boundary == 0)
                addr3 = (addr2+1) - boundary;
            else
                addr3 = addr2+1;
            return addr3;
        end
        
        4'b1000 : begin
            mem[addr] = wdata_temp[31:24];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = addr+1;
            return addr1;
        end
        
        4'b1001 : begin
            mem[addr] = wdata_temp[7:0];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = addr+1;
             mem[addr1] = wdata_temp[31:24];
             if((addr1+1) % boundary == 0)
                addr2 = (addr1+1) - boundary;
            else
                addr2 = addr1+1;
            return addr2;
        end
        
        4'b1010 : begin
            mem[addr] = wdata_temp[15:8];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = addr+1;
            mem[addr1] = wdata_temp[31:24];
             if((addr1+1) % boundary == 0)
                addr2 = (addr1+1) - boundary;
            else
                addr2 = addr1+1;
            return addr2;
        end
        
        4'b1011 : begin
            mem[addr] = wdata_temp[7:0];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = addr+1;
            mem[addr1] = wdata_temp[15:8];
            if((addr1+1) % boundary == 0)
                addr2 = (addr1+1) - boundary;
            else
                addr2 = addr1+1;
            mem[addr2] = wdata_temp[31:24];
            if((addr2+1) % boundary == 0)
                addr3 = (addr2+1) - boundary;
            else
                addr3 = addr2+1;
            return addr3;
        end
        
        4'b1100 : begin
            mem[addr] = wdata_temp[23:16];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = addr+1;
            mem[addr1] = wdata_temp[31:24];
            if((addr1+1) % boundary == 0)
                addr2 = (addr1+1) - boundary;
            else
                addr2 = addr1+1;
            return addr2;
        end
        
        4'b1101 : begin
            mem[addr] = wdata_temp[7:0];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = addr+1;
            mem[addr1] = wdata_temp[23:16];
            if((addr1+1) % boundary == 0)
                addr2 = (addr1+1) - boundary;
            else
                addr2 = addr1+1;
            mem[addr2] = wdata_temp[31:24];
            if((addr2+1) % boundary == 0)
                addr3 = (addr2+1) - boundary;
            else
                addr3 = addr2+1;
            return addr3;
        end
        
        4'b1110 : begin
            mem[addr] = wdata_temp[15:8];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = addr+1;
            mem[addr1] = wdata_temp[23:16];
            if((addr1+1) % boundary == 0)
                addr2 = (addr1+1) - boundary;
            else
                addr2 = addr1+1;
            mem[addr2] = wdata_temp[31:24];
            if((addr2+1) % boundary == 0)
                addr3 = (addr2+1) - boundary;
            else
                addr3 = addr2+1;
            return addr3;
        end
        
        4'b1111 : begin
            mem[addr] = wdata_temp[7:0];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = addr+1;
            mem[addr1] = wdata_temp[15:8];
            if((addr1+1) % boundary == 0)
                addr2 = (addr1+1) - boundary;
            else
                addr2 = addr1+1;
            mem[addr2] = wdata_temp[23:16];
            if((addr2+1) % boundary == 0)
                addr3 = (addr2+1) - boundary;
            else
                addr3 = addr2+1;
            mem[addr3] = wdata_temp[31:24];
            if((addr3+1) % boundary == 0)
                addr4 = (addr3+1) - boundary;
            else
                addr4 = addr3+1;
            return addr4;
        end
        endcase
    endfunction
    
    
    
    //read data fixed
    function bit [31:0] r_data_fixed(input bit [31:0] addr, input bit [2:0] size);
        unique case(size)
        3'b000 : begin
            rdata[7:0] = mem[addr];
            rdata[15:8] = 0;
            rdata[23:16] = 0;
            rdata[31:24] = 0;
        end
        
        3'b001 : begin
            rdata[7:0] = mem[addr];
            rdata[15:8] = mem[addr+1];
            rdata[23:16] = 0;
            rdata[31:24] = 0;
        end
        
        3'b010 : begin
            rdata[7:0] = mem[addr];
            rdata[15:8] = mem[addr+1];
            rdata[23:16] = mem[addr+2];
            rdata[31:24] = mem[addr+3];
        end
        endcase
        return addr; //returns same address
    endfunction
    
    
    //read data incr burst
    function bit [31:0] r_data_incr(input bit [31:0] addr, input bit [2:0] size);
        unique case(size)
        3'b000 : begin
            rdata[7:0] = mem[addr];
            rdata[15:8] = 0;
            rdata[23:16] = 0;
            rdata[31:24] = 0;
            return (addr+1);
        end
        
        3'b001 : begin
            rdata[7:0] = mem[addr];
            rdata[15:8] = mem[addr+1];
            rdata[23:16] = 0;
            rdata[31:24] = 0;
            return (addr+2);
        end
        
        3'b010 : begin
            rdata[7:0] = mem[addr];
            rdata[15:8] = mem[addr+1];
            rdata[23:16] = mem[addr+2];
            rdata[31:24] = mem[addr+3];
            return (addr+4);
        end
        endcase
    endfunction
    
    
    //read data wrap burst
    function bit [31:0] r_data_wrap(input bit [31:0] addr, input bit [7:0] boundary, input bit [2:0] size);
        bit [31:0] addr1, addr2, addr3, addr4;
        unique case(size)
        3'b000 : begin
            rdata[7:0] = mem[addr];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = (addr+1);
            rdata[15:8] = 0;
            rdata[23:16] = 0;
            rdata[31:24] = 0;
            return addr1;
        end
        
        3'b001 : begin
            rdata[7:0] = mem[addr];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = (addr+1);
            rdata[15:8] = mem[addr1];
            if((addr1+1) % boundary == 0)
                addr2 = (addr1+1) - boundary;
            else
                addr2 = addr1+1;
            rdata[23:16] = 0;
            rdata[31:24] = 0;
            return addr2;
        end
        
        3'b010 : begin
            rdata[7:0] = mem[addr];
            if((addr+1) % boundary == 0)
                addr1 = (addr+1) - boundary;
            else
                addr1 = (addr+1);
            rdata[15:8] = mem[addr1];
            if((addr1+1) % boundary == 0)
                addr2 = (addr1+1) - boundary;
            else
                addr2 = addr1+1;
            rdata[23:16] = mem[addr2];
            if((addr2+1) % boundary == 0)
                addr3 = (addr2+1) - boundary;
            else
                addr3 = (addr2+1);
            rdata[31:24] = mem[addr3];
            if((addr3+1) % boundary == 0)
                addr4 = (addr3+1) - boundary;
            else
                addr4 = addr3+1;
            return addr4;
        end
        endcase
    endfunction

endmodule

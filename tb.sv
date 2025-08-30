`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 14.08.2025 11:57:29
// Design Name: 
// Module Name: tb
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


//interface
interface axi_slave_if;
    //global signals
    logic clk; 
    logic resetn;
    
    //write address signals
    logic awvalid;
    logic [3:0] awid;
    logic [31:0] awaddr;
    logic [3:0] awlen;
    logic [2:0] awsize;
    logic [1:0] awburst;
    logic awready;
    
    //write data channel
    logic wvalid;
    logic [31:0] wdata;
    logic [3:0] wid;
    logic [3:0] wstrb;
    logic wlast;
    logic wready;
    
    //write response channel
    logic bready;
    logic bvalid;
    logic [1:0] bresp;
    logic [3:0] bid;
    
    //read address channel
    logic arvalid;
    logic [3:0] arid;
    logic [31:0] araddr;
    logic [3:0] arlen;
    logic [2:0] arsize;
    logic [1:0] arburst;
    logic arready;
    
    //read data/response channel
    logic rready;
    logic rvalid;
    logic [31:0] rdata;
    logic rlast;
    logic [1:0] rresp;
    logic [3:0] rid;
    
    logic [31:0] write_addr;
    logic [31:0] read_addr;
    
endinterface


class transaction;

    rand bit op; //1=read; 0=write
    
    bit clk;
    bit resetn;
    
    rand bit awvalid;
    rand bit [31:0] awaddr;
    rand bit [3:0] awid;
    rand bit [3:0] awlen;
    rand bit [2:0] awsize;
    rand bit [1:0] awburst;
    bit awready;
    bit[31:0] write_addr;
    
    bit wvalid;
    rand bit [31:0] wdata;
    bit [3:0] wid;
    bit wlast;
    rand bit [3:0] wstrb;
    bit wready;
    
    bit bready;
    bit bvalid;
    bit [3:0] bid;
    bit [1:0] bresp;
    
    rand bit arvalid;
    rand bit [31:0] araddr;
    rand bit arid;
    rand bit [3:0] arlen;
    rand bit [2:0] arsize;
    rand bit [1:0] arburst;
    bit arready;
    bit [31:0] read_addr;
    
    bit rready;
    bit rvalid;
    bit [31:0] rdata;
    bit rlast;
    bit [1:0] rresp;
    bit [3:0] rid; 
    
    constraint operation_c{
    op dist {0:=50, 1:=50};
    }
    constraint aw_addr_c{
//    awaddr > 140;
    awaddr<128;
    }
    constraint ar_addr_c{
//    araddr==130;
     araddr>=0; araddr<128;
    }
    constraint awsize_c{
    awsize>=0; awsize<=2;
//    awsize>4;
    }
    constraint arsize_c{
    arsize>=0; arsize<=2;
//    arsize == 4;
    }
    constraint awburst_c{
    awburst<3;
    }
    constraint arburst_c{arburst<3;}
    constraint awlen_c{
    awlen dist {1:=25, 3:= 25, 7:=25, 15:=25};
    }
    constraint arlen_c{arlen dist {1:=25, 3:= 25, 7:=25, 15:=25};}
    constraint wstrb_c{
        if(awsize==0)
        {
            wstrb inside {4'b0001, 4'b0010, 4'b0100, 4'b1000};
        }
        else if(awsize==1)
        {
            wstrb inside {4'b0011, 4'b0101, 4'b0110, 4'b1001, 4'b1010, 4'b1100};
        }
        else if(awsize==2)
        {
            wstrb inside {4'b1111};
        }
    }
endclass


class generator;
    mailbox#(transaction) mbxgd;
    mailbox#(bit) mbxgm; //for sending op
    transaction tr;
    int iterations = 0;
    event drvnext, sconext;
    event done;
    
    function new(mailbox #(transaction) mbxgd, mailbox #(bit) mbxgm);
        this.mbxgd = mbxgd;
        this.mbxgm = mbxgm;
        tr = new();
    endfunction
    
    task run();
        for (int i=1; i<=iterations; i++)
        begin
            assert(tr.randomize) else $display("Randomization Failed");
            $display("-------------------------------------------------------------");
            $display("%0d/%0d", i, iterations);
            if(tr.op==0)
            begin
                tr.awvalid=1;
                tr.arvalid=0;
                $display("[GEN] : awvalid=%0d, arvalid=%0d, awaddr=%0h, awburst=%0d, awsize=%0d, awlen=%0d, wdata=%0h", 
                    tr.awvalid, tr.arvalid, tr.awaddr, tr.awburst, tr.awsize, tr.awlen, tr.wdata);
            end
            else if(tr.op==1)
            begin
                tr.awvalid=0;
                tr.arvalid=1;
                $display("[GEN] : arvalid = %0d, awvalid = %0d, araddr = %0h, arburst = %0d, arsize = %0d, arlen = %0d", 
                    tr.arvalid, tr.awvalid, tr.araddr, tr.arburst, tr.arsize, tr.arlen);
            end
            mbxgd.put(tr);
            mbxgm.put(tr.op);
            @(sconext);
            @(drvnext);
        end
        -> done;
    endtask
endclass

class driver;
    transaction tr;
    mailbox #(transaction) mbxgd;
    virtual axi_slave_if vif;
    event drvnext;
    int len=0;
    
    function new(mailbox #(transaction) mbxgd);
        this.mbxgd = mbxgd;
    endfunction
    
    task reset();
        @(posedge vif.clk);
        vif.resetn <= 0;
        
        vif.awvalid <= 0;
        vif.awid <= 0;
        vif.awaddr <= 0;
        vif.awlen <= 0;
        vif.awsize <= 0;
        vif.awburst <= 0;
        
        vif.wvalid <= 0;
        vif.wdata <= 0;
        vif.wid <= 0;
        vif.wlast <= 0;
        vif.wstrb <= 0;
        
        vif.bready <= 0;
        
        vif.arvalid <= 0;
        vif.arid <= 0;
        vif.araddr <= 0;
        vif.awlen <= 0;
        vif.arsize <= 0;
        vif.arburst <= 0;
        
        vif.rready <= 0;
        repeat(5) @(posedge vif.clk);
        vif.resetn <= 1;
        repeat(5) @(posedge vif.clk);
        $display("[DRV] : Reset done");
    endtask
    
    task write();
        if(tr.awburst==2'b00) $display("[DRV] : Write Fixed Burst"); 
        else if(tr.awburst==2'b01) $display("[DRV] : Write Incr Burst");
        else if(tr.awburst==2'b10)$display("[DRV] : Write Wrap Burst"); 
        @(posedge vif.clk);
        len <= (tr.awlen)+1;
        vif.resetn <= 1;
        vif.awvalid <= 1;
        vif.arvalid <= 0;
        vif.awaddr <= tr.awaddr;
        vif.awid <= tr.awid;
        vif.awlen <= tr.awlen;
        vif.awsize <= tr.awsize;
        vif.awburst <= tr.awburst;
        @(posedge vif.awready);
        $display("[DRV] : awvalid=%0d, awaddr=%0d, awlen=%0d, awsize=%0d, awburst=%0d, bready==%0d",
            vif.awvalid, vif.awaddr, vif.awlen, vif.awsize, vif.awburst, vif.bready);
        @(negedge vif.awready);
        vif.awvalid <= 0;
        
        for(int i = 1; i<=len; i++)
        begin
            @(posedge vif.clk);
            vif.wvalid <= 1;
            vif.wstrb <= tr.wstrb;
            vif.wdata <= $urandom();
            vif.wid <= tr.awid;
            if(i==len) vif.wlast <= 1;
            @(posedge vif.wready);
            $display("[DRV] : i=%0d, wvalid=%d, wdata=%0h, wstrb=%0b",i, vif.wvalid, vif.wdata, vif.wstrb);
            @(negedge vif.wready);
            vif.wvalid <= 0;
            vif.wlast <= 0;
        end
        
        @(posedge vif.clk);
        vif.awvalid <= 0;
        vif.awaddr <= 0;
        vif.awid <= 0;
        vif.awlen <= 0;
        vif.awsize <= 0;
        vif.awburst <= 0;
        vif.wvalid <= 0;
        vif.wid <= 0;
        vif.wstrb <= 0;
        vif.wdata <= 0;
        vif.bready <= 1;
        @(negedge vif.bvalid);
        vif.bready <= 0;
        if(tr.awburst==2'b00) $display("[DRV] : Write Fixed Burst DONE"); 
        else if(tr.awburst==2'b01) $display("[DRV] : Write Incr Burst DONE");
        else if(tr.awburst==2'b10)$display("[DRV] : Write Wrap Burst DONE"); 
        -> drvnext;
        $display("[DRV] : ->drvnext");
    endtask
    
    task read();
        if(tr.arburst==2'b00) $display("[DRV] : Read Fixed Burst"); 
        else if(tr.arburst==2'b01) $display("[DRV] : Read Incr Burst");
        else if(tr.arburst==2'b10)$display("[DRV] : Read Wrap Burst");
        @(posedge vif.clk);
        @(posedge vif.clk);
        len <= tr.arlen + 1;
        vif.resetn <= 1;
        vif.arvalid <= 1;
        vif.awvalid <= 0;
        vif.arid <= tr.arid;
        vif.araddr <= tr.araddr;
        vif.arlen <= tr.arlen;
        vif.arsize <= tr.arsize;
        vif.arburst <= tr.arburst;
        vif.rready <= 1;
        @(posedge vif.arready);
        $display("[DRV] : arvalid=%0d, araddr=%0d, arlen=%0d, arsize=%0d, arburst=%0d",
            vif.arvalid, vif.araddr, vif.arlen, vif.arsize, vif.arburst);
        @(negedge vif.arready);
        vif.arvalid <= 0;
        @(negedge vif.rlast);
        @(posedge vif.clk);
        vif.rready <= 0;
        if(tr.arburst==2'b00) $display("[DRV] : Read Fixed Burst DONE"); 
        else if(tr.arburst==2'b01) $display("[DRV] : Read Incr Burst DONE");
        else if(tr.arburst==2'b10)$display("[DRV] : Read Wrap Burst DONE");
        -> drvnext;
        $display("[DRV] : ->drvnext");
    endtask
    
    task run();
        forever
        begin
            mbxgd.get(tr);
            if(tr.op==0)
            begin
                write();
            end
            else if(tr.op==1)
            begin
                read();
            end
        end
    endtask
endclass

class monitor;
    virtual axi_slave_if vif;
    mailbox #(transaction) mbxms;
    mailbox #(transaction) mbxgm;
    transaction tr;
    bit trref;
    int len = 0;
    event monnext;
    
    function new(mailbox #(transaction) mbxms, mailbox #(transaction) mbxgm);
        this.mbxms = mbxms;
        this.mbxgm = mbxgm;
    endfunction
    
    task run();
        tr = new();
        forever begin
            mbxgm.get(trref);
            @(posedge vif.clk);
            if(trref.op == 0) //write operation
            begin
                @(posedge vif.awready);
                tr.op = 0;
                len = vif.awlen + 1;
                tr.awvalid = vif.awvalid;
                tr.arvalid = vif.arvalid;
                tr.awburst = vif.awburst;
                tr.awaddr = vif.awaddr;
                tr.awlen = vif.awlen;
                tr.awsize = vif.awsize;
                $display("[MON] : awvalid=%0d, awaddr=%0d, awburst=%0d, awlen=%0d, awsize=%0d", 
                    vif.awvalid, vif.awaddr, vif.awburst, vif.awlen, vif.awsize);
                
                for(int i=1; i<=len; i++)
                begin
                    @(posedge vif.wready);
                    @(posedge vif.clk);
                    tr.wdata = vif.wdata;
                    tr.write_addr = vif.write_addr;
                    tr.wstrb = vif.wstrb;
                    tr.wid = vif.wid;
                    tr.wlast = vif.wlast;
                    tr.bresp = vif.bresp;
                    tr.bid = vif.bid;
                    mbxms.put(tr);
                    $display("[MON] : i=%0d, wdata=%0h, write_addr=%0d, wlast=%0d", 
                        i, tr.wdata, tr.write_addr, tr.wlast);
                    -> monnext;
                end
            end
            else if(trref.op == 1) //read operation
            begin
                @(posedge vif.arready);
                tr.op = 1;
                len = vif.arlen+1;
                tr.arvalid = vif.arvalid;
                tr.awvalid = vif.awvalid;
                tr.arburst = vif.arburst;
                tr.arid = vif.arid;
                tr.arlen = vif.arlen;
                tr.arsize = vif.arsize;
                $display("[MON] : arvalid=%0d, arburst=%0d, arlen=%0d, arsize=%0d", 
                    vif.arvalid, vif.arburst, vif.arlen, vif.arsize);
                for(int i=1; i<=len; i++)
                begin
                    @(posedge vif.rvalid);
                    @(posedge vif.clk);
                    tr.read_addr = vif.read_addr;
                    tr.rdata = vif.rdata;
                    tr.rid = vif.rid;
                    tr.rresp = vif.rresp;
                    tr.rlast = vif.rlast;
                    mbxms.put(tr);
                    $display("[MON] : i=%0d, rdata=%0h, read_addr=%0d, rlast=%0d", 
                        i, vif.rdata, vif.read_addr, vif.rlast);
                    -> monnext;
                end
            end
        end
    endtask
endclass

class scoreboard;
    mailbox #(transaction) mbxms;
    transaction tr;
    event sconext;
    event monnext;
    int len = 0;
    bit [7:0] mem [128]; // = '{default:0};
    bit [31:0] temp;
    bit [7:0] boundary;
    
    
    function new(mailbox #(transaction) mbxms);
        this.mbxms = mbxms;
        for(int i=0; i<128; i++) mem[i] = i;
    endfunction
    
    task run();
        forever
        begin
            bit [31:0] addr1, addr2, addr3, addr4;
            @(monnext);
            mbxms.get(tr);
            if(tr.op==0) //write
            begin
                $display("[SCO] : awvalid=%0d, awaddr=%0d, awburst=%0d, awlen=%0d, awsize=%0d",
                    tr.awvalid, tr.awaddr, tr.awburst, tr.awlen, tr.awsize);
                len=len+1;
                if(tr.awburst==2'b00 || tr.awburst==2'b01) //for fixed and incr burst
                begin
                    case(tr.wstrb)
                    4'b0001 : begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                    end
                    
                    4'b0010 : begin
                        mem[tr.write_addr] = tr.wdata[15:8];
                    end
                    
                    4'b0011: begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                        mem[tr.write_addr+1] = tr.wdata[15:8];
                    end
                    
                    4'b0100 : begin
                        mem[tr.write_addr] = tr.wdata[23:16];
                    end
                    
                    4'b0101 : begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                        mem[tr.write_addr+1] = tr.wdata[23:16];
                    end
                    
                    4'b0110 : begin
                        mem[tr.write_addr] = tr.wdata[15:8];
                        mem[tr.write_addr+1] = tr.wdata[23:16];
                    end
                    
                    4'b0111 : begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                        mem[tr.write_addr+1] = tr.wdata[15:8];
                        mem[tr.write_addr+2] = tr.wdata[23:16];
                    end
                    
                    4'b1000 : begin
                        mem[tr.write_addr] = tr.wdata[31:24];
                    end
                    
                    4'b1001 : begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                        mem[tr.write_addr+1] = tr.wdata[31:24];
                    end
                    
                    4'b1010 : begin
                        mem[tr.write_addr] = tr.wdata[15:8];
                        mem[tr.write_addr+1] = tr.wdata[31:24];
                    end
                    
                    4'b1011 : begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                        mem[tr.write_addr+1] = tr.wdata[15:8];
                        mem[tr.write_addr+2] = tr.wdata[31:24];
                    end
                    
                    4'b1100 : begin
                        mem[tr.write_addr] = tr.wdata[23:16];
                        mem[tr.write_addr+1] = tr.wdata[31:24];
                    end
                    
                    4'b1101 : begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                        mem[tr.write_addr+1] = tr.wdata[23:16];
                        mem[tr.write_addr+2] = tr.wdata[31:24];
                    end
                    
                    4'b1110 : begin
                        mem[tr.write_addr] = tr.wdata[15:8];
                        mem[tr.write_addr+1] = tr.wdata[23:16];
                        mem[tr.write_addr+2] = tr.wdata[31:24];
                    end
                    
                    4'b1111 : begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                        mem[tr.write_addr+1] = tr.wdata[15:8];
                        mem[tr.write_addr+2] = tr.wdata[23:16];
                        mem[tr.write_addr+3] = tr.wdata[31:24];
                    end
                    endcase
                    $display("[SCO] : mem[%0d]=%0h, mem[%0d]=%0h, mem[%0d]=%0h, mem[%0d]=%0h", 
                    tr.write_addr, mem[tr.write_addr], tr.write_addr+1, mem[tr.write_addr+1], tr.write_addr+2, 
                    mem[tr.write_addr+2], tr.write_addr+3, mem[tr.write_addr+3]);
                end
                else if(tr.awburst==2'b10) //wrap burst
                begin
                    case(tr.awsize)
                    3'b000 : boundary = (tr.awlen+1) * 1;
                    3'b001 : boundary = (tr.awlen+1) * 2;
                    3'b010 : boundary = (tr.awlen+1) * 4;
                    endcase
                    unique case(tr.wstrb)
                    4'b0001 : begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                        if((tr.write_addr+1) % boundary == 0)
                            addr1 = (tr.write_addr+1) - boundary;
                        else
                            addr1 = tr.write_addr+1;
                    end
                    
                    4'b0010 : begin
                        mem[tr.write_addr] = tr.wdata[15:8];
                        if((tr.write_addr+1) % boundary == 0)
                            addr1 = (tr.write_addr+1) - boundary;
                        else
                            addr1 = tr.write_addr+1;
                    end
                    
                    4'b0011 : begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                        if((tr.write_addr+1) % boundary == 0)
                            addr1 = (tr.write_addr+1) - boundary;
                        else
                            addr1 = tr.write_addr+1;
                        mem[addr1] = tr.wdata[15:8];
                        if((addr1+1) % boundary == 0)
                            addr2 = (addr1+1) - boundary;
                        else
                            addr2 = addr1+1;
                    end
                    
                    4'b0100 : begin
                        mem[tr.write_addr] = tr.wdata[23:16];
                        if((tr.write_addr+1) % boundary == 0)
                            addr1 = (tr.write_addr+1) - boundary;
                        else
                            addr1 = tr.write_addr+1;
                    end
                    
                    4'b0101 : begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                        if((tr.write_addr+1) % boundary == 0)
                            addr1 = (tr.write_addr+1) - boundary;
                        else
                            addr1 = tr.write_addr+1;
                        mem[addr1] = tr.wdata[23:16];
                        if((addr1+1) % boundary == 0)
                            addr2 = (addr1+1) - boundary;
                        else
                            addr2 = addr1+1;
                    end
                    
                    4'b0110 : begin
                        mem[tr.write_addr] = tr.wdata[15:8];
                        if((tr.write_addr+1) % boundary == 0)
                            addr1 = (tr.write_addr+1) - boundary;
                        else
                            addr1 = tr.write_addr+1;
                        mem[addr1] = tr.wdata[23:16];
                        if((addr1+1) % boundary == 0)
                            addr2 = (addr1+1) - boundary;
                        else
                            addr2 = addr1+1;
                    end
                    
                    4'b0111 : begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                        if((tr.write_addr+1) % boundary == 0)
                            addr1 = (tr.write_addr+1) - boundary;
                        else
                            addr1 = tr.write_addr+1;
                        mem[addr1] = tr.wdata[15:8];
                        if((addr1+1) % boundary == 0)
                            addr2 = (addr1+1) - boundary;
                        else
                            addr2 = addr1+1;
                        mem[addr2] = tr.wdata[23:16];
                        if((addr2+1) % boundary == 0)
                            addr3 = (addr2+1) - boundary;
                        else
                            addr3 = addr2+1;
                    end
                    
                    4'b1000 : begin
                        mem[tr.write_addr] = tr.wdata[31:24];
                        if((tr.write_addr+1) % boundary == 0)
                            addr1 = (tr.write_addr+1) - boundary;
                        else
                            addr1 = tr.write_addr+1;
                    end
                    
                    4'b1001 : begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                        if((tr.write_addr+1) % boundary == 0)
                            addr1 = (tr.write_addr+1) - boundary;
                        else
                            addr1 = tr.write_addr+1;
                         mem[addr1] = tr.wdata[31:24];
                         if((addr1+1) % boundary == 0)
                            addr2 = (addr1+1) - boundary;
                        else
                            addr2 = addr1+1;
                    end
                    
                    4'b1010 : begin
                        mem[tr.write_addr] = tr.wdata[15:8];
                        if((tr.write_addr+1) % boundary == 0)
                            addr1 = (tr.write_addr+1) - boundary;
                        else
                            addr1 = tr.write_addr+1;
                        mem[addr1] = tr.wdata[31:24];
                         if((addr1+1) % boundary == 0)
                            addr2 = (addr1+1) - boundary;
                        else
                            addr2 = addr1+1;
                    end
                    
                    4'b1011 : begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                        if((tr.write_addr+1) % boundary == 0)
                            addr1 = (tr.write_addr+1) - boundary;
                        else
                            addr1 = tr.write_addr+1;
                        mem[addr1] = tr.wdata[15:8];
                        if((addr1+1) % boundary == 0)
                            addr2 = (addr1+1) - boundary;
                        else
                            addr2 = addr1+1;
                        mem[addr2] = tr.wdata[31:24];
                        if((addr2+1) % boundary == 0)
                            addr3 = (addr2+1) - boundary;
                        else
                            addr3 = addr2+1;
                    end
                    
                    4'b1100 : begin
                        mem[tr.write_addr] = tr.wdata[23:16];
                        if((tr.write_addr+1) % boundary == 0)
                            addr1 = (tr.write_addr+1) - boundary;
                        else
                            addr1 = tr.write_addr+1;
                        mem[addr1] = tr.wdata[31:24];
                        if((addr1+1) % boundary == 0)
                            addr2 = (addr1+1) - boundary;
                        else
                            addr2 = addr1+1;
                    end
                    
                    4'b1101 : begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                        if((tr.write_addr+1) % boundary == 0)
                            addr1 = (tr.write_addr+1) - boundary;
                        else
                            addr1 = tr.write_addr+1;
                        mem[addr1] = tr.wdata[23:16];
                        if((addr1+1) % boundary == 0)
                            addr2 = (addr1+1) - boundary;
                        else
                            addr2 = addr1+1;
                        mem[addr2] = tr.wdata[31:24];
                        if((addr2+1) % boundary == 0)
                            addr3 = (addr2+1) - boundary;
                        else
                            addr3 = addr2+1;
                    end
                    
                    4'b1110 : begin
                        mem[tr.write_addr] = tr.wdata[15:8];
                        if((tr.write_addr+1) % boundary == 0)
                            addr1 = (tr.write_addr+1) - boundary;
                        else
                            addr1 = tr.write_addr+1;
                        mem[addr1] = tr.wdata[23:16];
                        if((addr1+1) % boundary == 0)
                            addr2 = (addr1+1) - boundary;
                        else
                            addr2 = addr1+1;
                        mem[addr2] = tr.wdata[31:24];
                        if((addr2+1) % boundary == 0)
                            addr3 = (addr2+1) - boundary;
                        else
                            addr3 = addr2+1;
                    end
                    
                    4'b1111 : begin
                        mem[tr.write_addr] = tr.wdata[7:0];
                        if((tr.write_addr+1) % boundary == 0)
                            addr1 = (tr.write_addr+1) - boundary;
                        else
                            addr1 = tr.write_addr+1;
                        mem[addr1] = tr.wdata[15:8];
                        if((addr1+1) % boundary == 0)
                            addr2 = (addr1+1) - boundary;
                        else
                            addr2 = addr1+1;
                        mem[addr2] = tr.wdata[23:16];
                        if((addr2+1) % boundary == 0)
                            addr3 = (addr2+1) - boundary;
                        else
                            addr3 = addr2+1;
                        mem[addr3] = tr.wdata[31:24];
                        if((addr3+1) % boundary == 0)
                            addr4 = (addr3+1) - boundary;
                        else
                            addr4 = addr3+1;
                    end
                    endcase
                    $display("[SCO] : boundary=%0d, mem[%0d]=%0h, mem[%0d]=%0h, mem[%0d]=%0h, mem[%0d]=%0h", 
                        boundary, tr.write_addr, mem[tr.write_addr], addr1, mem[addr1], addr2, mem[addr2], addr3, mem[addr3]);
                end
                
                $display("[SCO] : DATA WRITTEN wdata=%0h, write_addr=%0d, wstrb=%0b, wlast=%0d",
                    tr.wdata, tr.write_addr, tr.wstrb, tr.wlast);
                $display("[SCO] : bid=%0d, bresp=%0d", tr.bid, tr.bresp);
                if(len==(tr.awlen+1))
                begin
                    ->sconext;
                    $display("[SCO] : ->sconext");
                    len=0;
                end
                else
                begin
                    $display("[SCO] : len=%0d, awlen=%0d, arlen=%0d", len, tr.awlen, tr.arlen);
                end
            end
            
            
            else if(tr.op==1) //read
            begin
                $display("[SCO] : arvalid=%0d, araddr=%0d, arburst=%0d, arlen=%0d, arsize=%0d",
                    tr.arvalid, tr.araddr, tr.arburst, tr.arlen, tr.arsize);
                len=len+1;
                if(tr.arburst==2'b00 || tr.arburst==2'b01) //fixed and incr burst
                begin
                    case(tr.arsize)
                    3'b000 : begin
                        temp[7:0] = mem[tr.read_addr];
                    end
                    
                    3'b001 : begin
                        temp[7:0] = mem[tr.read_addr];
                        temp[15:8] = mem[tr.read_addr+1];
                    end
                    
                    3'b010 : begin
                        temp[7:0] = mem[tr.read_addr];
                        temp[15:8] = mem[tr.read_addr+1];
                        temp[23:16] = mem[tr.read_addr+2];
                        temp[31:24] = mem[tr.read_addr+3];
                    end
                    endcase
                    $display("[SCO] : mem[%0d]=%0h, mem[%0d]=%0h, mem[%0d]=%0h, mem[%0d]=%0h", 
                        tr.read_addr, mem[tr.read_addr], tr.read_addr+1, mem[tr.read_addr+1], 
                        tr.read_addr+2, mem[tr.read_addr+2], tr.read_addr+3, mem[tr.read_addr+3]);
                end
                else if(tr.arburst==2'b10) //wrap burst
                begin
                    case(tr.arsize)
                    3'b000 : boundary = (tr.arlen+1) * 1;
                    3'b001 : boundary = (tr.arlen+1) * 2;
                    3'b010 : boundary = (tr.arlen+1) * 4;
                    endcase
                    unique case(tr.arsize)
                    3'b000 : begin
                        temp[7:0] = mem[tr.read_addr];
                        if((tr.read_addr+1) % boundary == 0)
                            addr1 = (tr.read_addr+1) - boundary;
                        else
                            addr1 = (tr.read_addr+1);
                    end
                    
                    3'b001 : begin
                        temp[7:0] = mem[tr.read_addr];
                        if((tr.read_addr+1) % boundary == 0)
                            addr1 = (tr.read_addr+1) - boundary;
                        else
                            addr1 = (tr.read_addr+1);
                        temp[15:8] = mem[addr1];
                        if((addr1+1) % boundary == 0)
                            addr2 = (addr1+1) - boundary;
                        else
                            addr2 = addr1+1;
                    end
                    
                    3'b010 : begin
                        temp[7:0] = mem[tr.read_addr];
                        if((tr.read_addr+1) % boundary == 0)
                            addr1 = (tr.read_addr+1) - boundary;
                        else
                            addr1 = (tr.read_addr+1);
                        temp[15:8] = mem[addr1];
                        if((addr1+1) % boundary == 0)
                            addr2 = (addr1+1) - boundary;
                        else
                            addr2 = addr1+1;
                        temp[23:16] = mem[addr2];
                        if((addr2+1) % boundary == 0)
                            addr3 = (addr2+1) - boundary;
                        else
                            addr3 = (addr2+1);
                        temp[31:24] = mem[addr3];
                        if((addr3+1) % boundary == 0)
                            addr4 = (addr3+1) - boundary;
                        else
                            addr4 = addr3+1;
                    end
                    endcase
                    $display("[SCO] : boundary=%0d, mem[%0d]=%0h, mem[%0d]=%0h, mem[%0d]=%0h, mem[%0d]=%0h", 
                        boundary, tr.read_addr, mem[tr.read_addr], addr1, mem[addr1], addr2, mem[addr2], addr3, mem[addr3]);
                end
                
                $display("[SCO] : read_addr=%0d, temp=%0h, rdata=%0h, rlast=%0d",
                    tr.read_addr, temp, tr.rdata, tr.rlast);
                if(temp==tr.rdata)
                    $display("[SCO] : Match");
                else
                    $display("[SCO] : Mismatch");
                $display("[SCO] : rid=%0d, rresp=%0d", tr.rid, tr.rresp);
                if(len==(tr.arlen+1))
                begin
                    ->sconext;
                    $display("[SCO] : ->sconext");
                    len=0;
                end
                else
                begin
                    $display("[SCO] : len=%0d, arlen=%0d, awlen=%0d", len, tr.arlen, tr.awlen);
                end
            end
            $display("-------------------------------------------------------------");
        end
    endtask
endclass

class environment;
    generator g;
    driver d;
    monitor m;
    scoreboard s;
    
    mailbox #(transaction) mbxgd;
    mailbox #(transaction) mbxms;
    mailbox #(bit) mbxgm;
    
    virtual axi_slave_if vif;
    
    function new(virtual axi_slave_if vif);
        mbxgd = new();
        mbxms = new();
        mbxgm = new();
        g = new(mbxgd, mbxgm);
        d = new(mbxgd);
        m = new(mbxms, mbxgm);
        s = new(mbxms);
        
        d.drvnext = g.drvnext;
        m.monnext = s.monnext;
        s.sconext = g.sconext;
        
        d.vif = vif;
        m.vif = vif;
    endfunction
    
    task pre_test();
        d.reset();
    endtask
    
    task test();
        fork
            g.run();
            d.run();
            m.run();
            s.run();
        join_any
    endtask
    
    task post_test();
        wait(g.done.triggered);
        $finish;
    endtask 
    
    task run();
        pre_test();
        test();
        post_test();
    endtask
endclass


module tb();
    axi_slave_if vif();
    axi_slave dut(vif.clk, vif.resetn, vif.awvalid, vif.awid, vif.awaddr, vif.awlen, vif.awsize, vif.awburst, vif.awready, 
        vif.wvalid, vif.wdata, vif.wid, vif.wstrb, vif.wlast, vif.wready, vif.bready, vif.bvalid, vif.bresp, vif.bid, 
        vif.arvalid, vif.arid, vif.araddr, vif.arlen, vif.arsize, vif.arburst, vif.arready, vif.rready, vif.rvalid,
        vif.rdata, vif.rlast, vif.rresp, vif.rid);
    assign vif.write_addr = dut.write_addr;
    assign vif.read_addr = dut.read_addr;
    
    environment env;
    
    initial vif.clk = 0;
    always #10 vif.clk = ~vif.clk;
    
    initial
    begin
        env = new(vif);
        env.g.iterations = 5;
        env.run();
    end

endmodule

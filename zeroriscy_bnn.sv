// Copyright 2018 tom01h
// Copyright 2017 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

import zeroriscy_defines::*;

module zeroriscy_bnn
  (
   input logic         clk,
   input logic         rst_n,

   // signals from ex stage
   input logic         bnn_en_i,
   input logic [2:0]   bnn_operator_i,
   input logic [31:0]  bnn_addr_i,
   input logic [31:0]  bnn_data_i,

   output logic [31:0] bnn_result_o,
   output logic        bnn_ready_o
   );


   reg [1023:0]        param;

   wire [15:0]         param_addr = (bnn_operator_i != 3'b101) ? bnn_addr_i[15:0] : {1'b0,bnn_addr_i[7:4]};
   
   ram0 ram0 (clk, param_addr[15:0], param);

   reg                 bnn_en_1;
   reg [2:0]           com_1;
   reg [31:0]          addr_1;
   reg [31:0]          data_1;
   reg                 busy;
// input stage
   always_ff @(posedge clk)begin
      addr_1[31:0] <= bnn_addr_i;
      data_1[31:0] <= bnn_data_i;
      if(bnn_en_i&~busy)begin
         bnn_en_1 <= 1'b1;
         com_1[2:0] <= bnn_operator_i;
      end else begin
         bnn_en_1 <= 1'b0;
         com_1[2:0] <= 3'b100;
      end
   end

// 1st stage for IP8
   reg [2:0]           com_2;
   reg [23:0]          inA_2;
   reg [23:0]          inB0_2;
   reg [23:0]          inB1_2;
   reg [31:0]          inC_2;
   always_ff @(posedge clk)begin
      com_2[2:0]  <= com_1[2:0];
      inA_2[23:0] <= addr_1[31:8];
      inC_2[31:0] <= data_1[31:0];
      case(addr_1[3:0])
        4'h0 : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+992:8+992], param[31+960:8+960]};
        4'h1 : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+928:8+928], param[31+896:8+896]};
        4'h2 : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+864:8+864], param[31+832:8+832]};
        4'h3 : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+800:8+800], param[31+768:8+768]};
        4'h4 : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+736:8+736], param[31+704:8+704]};
        4'h5 : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+672:8+672], param[31+640:8+640]};
        4'h6 : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+608:8+608], param[31+576:8+576]};
        4'h7 : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+544:8+544], param[31+512:8+512]};
        4'h8 : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+480:8+480], param[31+448:8+448]};
        4'h9 : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+416:8+416], param[31+384:8+384]};
        4'ha : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+352:8+352], param[31+320:8+320]};
        4'hb : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+288:8+288], param[31+256:8+256]};
        4'hc : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+224:8+224], param[31+192:8+192]};
        4'hd : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+160:8+160], param[31+128:8+128]};
        4'he : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+ 96:8+ 96], param[31+ 64:8+ 64]};
        4'hf : {inB0_2[23:0], inB1_2[23:0]} <= {param[31+ 32:8+ 32], param[31+  0:8+  0]};
      endcase
   end

// 2nd stage for IP8
   wire signed [16:0]  ip0 = ($signed(inA_2[23:16])*$signed(inB0_2[23:16]) +
                              $signed(inA_2[15:8] )*$signed(inB0_2[15:8])  +
                              $signed(inA_2[7:0]  )*$signed(inB0_2[7:0] )   );
   wire signed [16:0]  ip1 = ($signed(inA_2[23:16])*$signed(inB1_2[23:16]) +
                              $signed(inA_2[15:8] )*$signed(inB1_2[15:8])  +
                              $signed(inA_2[7:0]  )*$signed(inB1_2[7:0] )   );
   reg [2:0]           com_3;
   reg [15:0]          IP0_3;
   reg [15:0]          IP1_3;
   always_ff @(posedge clk)begin
      com_3[2:0] <= com_2[2:0];
      IP0_3 <= ip0[16:1]+inC_2[31:16];
      IP1_3 <= ip1[16:1]+inC_2[15:0];
   end

// 3rd stage for IP8
   logic [31:0] bnn_result;
   assign bnn_result_o = (com_3[2:0] != 3'b101) ? bnn_result : {IP0_3, IP1_3};

   assign bnn_ready_o = (~bnn_en_1|(com_1!=3'b100)&(com_1!=3'b101))&(com_2!=3'b101);

   always_ff @(posedge clk)begin
      if((bnn_en_i&(bnn_operator_i[2:0]==3'b100))|
         (bnn_en_i&(bnn_operator_i[2:0]==3'b101))|
         (bnn_en_1&         (com_1[2:0]==3'b100))|
         (bnn_en_1&         (com_1[2:0]==3'b101))|
         (                  (com_2[2:0]==3'b101))  )
        busy <= 1'b1;
      else
        busy <= 1'b0;
   end

   genvar               g;
   generate begin
      for(g=0;g<32;g=g+1) begin : estimate_block
         estimate_core core
            (.clk(clk), .com_1(com_1[2:0]), .seten_1({(g==addr_1),(g==addr_1+1)}),
             .data_1(data_1[31:0]), .param(param[32*(31-g)+31:32*(31-g)+0]),
             .activ(bnn_result[g])
             );
      end : estimate_block
   end
   endgenerate

endmodule

module estimate_core
  (
   input wire        clk,
   input wire [2:0]  com_1,
   input wire [1:0]  seten_1,
   input wire [31:0] data_1,
   input wire [31:0] param,
   output wire       activ
   );

   integer           i;

   reg signed [15:0] acc;
   reg signed [15:0] pool;

   reg [2:0]         com_2;
   reg [31:0]        data_2;
   reg [1:0]         seten_2;

// 1st stage
   always_ff @(posedge clk)begin
      com_2[2:0] <= com_1;
      case(com_1)
        3'd0 : begin //ini
           data_2 <= data_1;
        end
        3'd1 : begin //acc
           data_2 <= ~(data_1^param);
        end
        3'd2 : begin //pool
           data_2 <= data_1;
        end
        3'd3 : begin //norm
           data_2 <= param;
        end
        3'd6 : begin //seten
           seten_2 <= seten_1;
           data_2 <= data_1;
        end
        3'd7 : begin //norm8
           data_2 <= param;
        end
      endcase
   end
// 2nd stage
   reg [15:0] sum;
   always_comb begin
      sum = 0;
      for(i=0; i<32;i=i+1)begin
         sum = sum + {1'b0,data_2[i],1'b0};
      end
   end
   always_ff @(posedge clk)begin
      case(com_2)
        3'd0 : begin //ini
           acc[15:0] <= data_2[15:0];
           pool[15:0] <= 16'h8000;
        end
        3'd1 : begin //acc
           acc <= acc + sum;
        end
        3'd2 : begin //pool
           if(acc>pool)begin
              pool[15:0] <= acc[15:0];
           end
           acc[15:0] <= data_2[15:0];
        end
        3'd3 : begin //norm
           pool[15:0] <= {pool[15:0],3'h0} - data_2[15:0];
        end
//        3'd4 : begin //activ
//           activ <= pool[15];
//        end
        3'd6 : begin //seten
           if(seten_2==2'b10)
             acc[15:0] <= data_2[31:16];
           if(seten_2==2'b01)
             acc[15:0] <= data_2[15:0];
           if(seten_2==2'b11)
             acc[15:0] <= 16'hxxxx;
        end
        3'd7 : begin //norm8
           pool[15:0] <= pool[15:0] - data_2[15:0];
        end
      endcase
   end
   assign activ = pool[15];
endmodule

module ram0 (clk, addr, dout);
   input clk;
   input [15:0] addr;
   output [1023:0] dout;
`include "param0.v"
   reg [1023:0]    dout;

   always @(posedge clk)
     begin
        dout <= mem[addr];
     end
endmodule

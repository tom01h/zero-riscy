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
   input logic [6:0]   bnn_param_i,
   input logic [31:0]  bnn_addr_i,
   input logic [31:0]  bnn_data_i,

   output logic [31:0] bnn_result_o,
   output logic        bnn_ready_o
   );


   reg [63:0]          param;

   wire [15:0]         param_addr = {1'b0,bnn_addr_i[7:0]}+{bnn_param_i[4:0],4'h0};
   
   ram0 ram0 (clk, param_addr[15:0], param);

   reg                 bnn_en_1;
   reg [2:0]           com_1;
   reg [1:0]           sft_1;
   reg [31:0]          addr_1;
   reg [31:0]          data_1;
   reg                 busy;
   wire                bnn_en = bnn_en_i&~busy;

// input stage
   always_ff @(posedge clk)begin
      addr_1[31:0] <= bnn_addr_i;
      data_1[31:0] <= bnn_data_i;
      sft_1[1:0]   <= bnn_param_i[6:5];
      if(bnn_en)begin
         bnn_en_1 <= 1'b1;
         com_1[2:0] <= bnn_operator_i;
      end else begin
         bnn_en_1 <= 1'b0;
         com_1[2:0] <= 3'b100;
      end
   end

// 1st stage for IP8
   reg [2:0]           com_2;
   reg [1:0]           sft_2;
   reg [23:0]          inA_2;
   reg [23:0]          inB0_2;
   reg [23:0]          inB1_2;
   reg [31:0]          inC_2;
   always_ff @(posedge clk)begin
      com_2[2:0]  <= com_1[2:0];
      sft_2[1:0]  <= sft_1[1:0];
      inA_2[23:0] <= addr_1[31:8];
      inC_2[31:0] <= data_1[31:0];
      inB0_2[23:0] <= param[31+ 32:8+ 32];
      inB1_2[23:0] <= param[31+  0:8+  0];
   end

// 2nd stage for IP8
   wire signed [19:0]  ip0 = ($signed(inA_2[23:16])*$signed(inB0_2[23:16]) +
                              $signed(inA_2[15:8] )*$signed(inB0_2[15:8])  +
                              $signed(inA_2[7:0]  )*$signed(inB0_2[7:0] )   );
   wire signed [19:0]  ip1 = ($signed(inA_2[23:16])*$signed(inB1_2[23:16]) +
                              $signed(inA_2[15:8] )*$signed(inB1_2[15:8])  +
                              $signed(inA_2[7:0]  )*$signed(inB1_2[7:0] )   );
   reg [2:0]           com_3;
   reg [15:0]          IP0_3;
   reg [15:0]          IP1_3;
   always_ff @(posedge clk)begin
      com_3[2:0] <= com_2[2:0];
      case(sft_2)
        2'b00:begin
           IP0_3 <= ip0[15:0]+inC_2[31:16];
           IP1_3 <= ip1[15:0]+inC_2[15:0];
        end
        2'b01:begin
           IP0_3 <= ip0[16:1]+inC_2[31:16];
           IP1_3 <= ip1[16:1]+inC_2[15:0];
        end
        2'b10:begin
           IP0_3 <= ip0[17:2]+inC_2[31:16];
           IP1_3 <= ip1[17:2]+inC_2[15:0];
        end
        2'b11:begin
           IP0_3 <= ip0[19:4]+inC_2[31:16];
           IP1_3 <= ip1[19:4]+inC_2[15:0];
        end
      endcase
   end

// 3rd stage for IP8
   logic [31:0] bnn_result;
   assign bnn_result_o = {IP0_3, IP1_3};

   assign bnn_ready_o = ~bnn_en_1&(com_2!=3'b101);

   always_ff @(posedge clk)begin
      if((bnn_en  &(bnn_operator_i[2:0]==3'b101))|
         (bnn_en_1&         (com_1[2:0]==3'b101))|
         (                  (com_2[2:0]==3'b101))  )
        busy <= 1'b1;
      else
        busy <= 1'b0;
   end

endmodule

module ram0 (clk, addr, dout);
   input clk;
   input [15:0] addr;
   output [63:0] dout;
`include "param0.v"
   reg [63:0]    dout;

   always @(posedge clk)
     begin
        dout <= mem[addr];
     end
endmodule

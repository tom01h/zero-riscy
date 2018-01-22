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
   
   ram0 ram0 (clk, bnn_addr_i[15:0], param);

   reg [2:0]            com_1;
   reg [31:0]           data_1;
   reg                  busy;
// input stage
   always_ff @(posedge clk)begin
      data_1[31:0] <= bnn_data_i;
      if(bnn_en_i&~busy)
        com_1[2:0] <= bnn_operator_i;
      else
        com_1[2:0] <= 3'b111;
      if((bnn_en_i&(bnn_operator_i[2:0]==3'b100))|
         (com_1[2:0]==3'b100))
        busy <= 1'b1;
      else
        busy <= 1'b0;
   end

//   assign bnn_ready_o = (bnn_operator_i!=3'b100)&(com_1!=3'b100);
   assign bnn_ready_o = (com_1!=3'b100);

   genvar               g;
   generate begin
      for(g=0;g<32;g=g+1) begin : estimate_block
         estimate_core core
            (.clk(clk), .com_1(com_1[2:0]), .data_1(data_1[31:0]),
             .param(param[32*(31-g)+31:32*(31-g)+0]),
             .activ(bnn_result_o[g])
             );
      end : estimate_block
   end
   endgenerate

endmodule

module estimate_core
  (
   input wire        clk,
   input wire [2:0]  com_1,
   input wire [31:0] data_1,
   input wire [31:0] param,
   output wire       activ
   );

   integer           i;

   reg signed [15:0] acc;
   reg signed [15:0] pool;

   reg [2:0]         com_2;
   reg [31:0]        data_2;
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
           pool[15:0] <= {pool[15:0],6'h00} - data_2[15:0];
        end
//        3'd4 : begin //activ
//           activ <= pool[15];
//        end
      endcase
   end
   assign activ = pool[15];
endmodule

module ram0 (clk, addr, dout);
   input clk;
   input [15:0] addr;
   output [1023:0] dout;
`include "param0"
   reg [1024:0]    dout;

   always @(posedge clk)
     begin
        dout <= mem[addr];
     end
endmodule

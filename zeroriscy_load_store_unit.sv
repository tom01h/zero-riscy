// Copyright 2017 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Igor Loi - igor.loi@unibo.it                               //
//                                                                            //
// Additional contributions by:                                               //
//                 Andreas Traber - atraber@iis.ee.ethz.ch                    //
//                 Markus Wegmann - markus.wegmann@technokrat.ch              //
//                 Davide Schiavone - pschiavo@iis.ee.ethz.ch                 //
//                                                                            //
// Design Name:    Load Store Unit                                            //
// Project Name:   zero-riscy                                                 //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Load Store Unit, used to eliminate multiple access during  //
//                 processor stalls, and to align bytes and halfwords         //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

`include "zeroriscy_config.sv"

module zeroriscy_load_store_unit
(
    input  logic         clk,
    input  logic         rst_n,

    // output to data memory
    output logic         data_req_o,
    input  logic         data_gnt_i,
    input  logic         data_rvalid_i,
    input  logic         data_err_i,

    output logic [31:0]  data_addr_o,
    output logic         data_we_o,
    output logic [3:0]   data_be_o,
    output logic [31:0]  data_wdata_o,
    input  logic [31:0]  data_rdata_i,

    // signals from ex stage
    input  logic         data_we_ex_i,         // write enable                      -> from ex stage
    input  logic [1:0]   data_type_ex_i,       // Data type word, halfword, byte    -> from ex stage
    input  logic [31:0]  data_wdata_ex_i,      // data to write to memory           -> from ex stage
    input  logic [1:0]   data_reg_offset_ex_i, // offset inside register for stores -> from ex stage
    input  logic         data_sign_ext_ex_i,   // sign extension                    -> from ex stage

    output logic [31:0]  data_rdata_ex_o,      // requested data                    -> to ex stage
    input  logic         data_req_ex_i,        // data request                      -> from ex stage

    input  logic [31:0]  adder_result_ex_i,

    // exception signals
    output logic         load_err_o,
    output logic         store_err_o,

    // stall signal
    output logic         data_valid_o,

    output logic         busy_o
);

  logic [31:0]  data_addr_int;
  assign data_addr_int = adder_result_ex_i;

  // registers for data_rdata alignment and sign extension
  logic [1:0]   data_type_q;
  logic [1:0]   rdata_offset_q;
  logic         data_sign_ext_q;

  logic [3:0]   data_be;
  logic [31:0]  data_wdata;

  enum logic [1:0]  { IDLE, WAIT_GNT, WAIT_RVALID } CS, NS;

  ///////////////////////////////// BE generation ////////////////////////////////
  always_comb
  begin
    case (data_type_ex_i) // Data type 00 Word, 01 Half word, 11,10 byte
      2'b00:
      begin // Writing a word
        data_wdata = data_wdata_ex_i[31:0];
        data_be = 4'b1111;
      end

      2'b01:
      begin // Writing a half word
        data_wdata = {2{data_wdata_ex_i[15:0]}};
        case (data_addr_int[1])
          1'b0: data_be = 4'b0011;
          1'b1: data_be = 4'b1100;
        endcase; // case (data_addr_int[1:0])
      end

      2'b10,
      2'b11: begin // Writing a byte
        data_wdata = {4{data_wdata_ex_i[7:0]}};
        case (data_addr_int[1:0])
          2'b00: data_be = 4'b0001;
          2'b01: data_be = 4'b0010;
          2'b10: data_be = 4'b0100;
          2'b11: data_be = 4'b1000;
        endcase; // case (data_addr_int[1:0])
      end
    endcase; // case (data_type_ex_i)
  end


  // FF for rdata alignment and sign-extension
  always_ff @(posedge clk, negedge rst_n)
  begin
    if(rst_n == 1'b0)
    begin
      data_type_q     <= '0;
      rdata_offset_q  <= '0;
      data_sign_ext_q <= '0;
    end
    else if (data_gnt_i == 1'b1) // request was granted, we wait for rvalid and can continue to WB
    begin
      data_type_q     <= data_type_ex_i;
      rdata_offset_q  <= data_addr_int[1:0];
      data_sign_ext_q <= data_sign_ext_ex_i;
    end
  end


  ////////////////////////////////////////////////////////////////////////
  //  ____  _               _____      _                 _              //
  // / ___|(_) __ _ _ __   | ____|_  _| |_ ___ _ __  ___(_) ___  _ __   //
  // \___ \| |/ _` | '_ \  |  _| \ \/ / __/ _ \ '_ \/ __| |/ _ \| '_ \  //
  //  ___) | | (_| | | | | | |___ >  <| ||  __/ | | \__ \ | (_) | | | | //
  // |____/|_|\__, |_| |_| |_____/_/\_\\__\___|_| |_|___/_|\___/|_| |_| //
  //          |___/                                                     //
  ////////////////////////////////////////////////////////////////////////

  logic [31:0] data_rdata_ext;

  logic [31:0] rdata_w_ext; // sign extension for words
  logic [31:0] rdata_h_ext; // sign extension for half words
  logic [31:0] rdata_b_ext; // sign extension for bytes

  assign rdata_w_ext = data_rdata_i[31:0];

  // sign extension for half words
  always_comb
  begin
    case (rdata_offset_q[1])
      1'b0:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_h_ext = {16'h0000, data_rdata_i[15:0]};
        else
          rdata_h_ext = {{16{data_rdata_i[15]}}, data_rdata_i[15:0]};
      end

      1'b1:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_h_ext = {16'h0000, data_rdata_i[31:16]};
        else
          rdata_h_ext = {{16{data_rdata_i[31]}}, data_rdata_i[31:16]};
      end

    endcase // case (rdata_offset_q)
  end

  // sign extension for bytes
  always_comb
  begin
    case (rdata_offset_q)
      2'b00:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_b_ext = {24'h00_0000, data_rdata_i[7:0]};
        else
          rdata_b_ext = {{24{data_rdata_i[7]}}, data_rdata_i[7:0]};
      end

      2'b01: begin
        if (data_sign_ext_q == 1'b0)
          rdata_b_ext = {24'h00_0000, data_rdata_i[15:8]};
        else
          rdata_b_ext = {{24{data_rdata_i[15]}}, data_rdata_i[15:8]};
      end

      2'b10:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_b_ext = {24'h00_0000, data_rdata_i[23:16]};
        else
          rdata_b_ext = {{24{data_rdata_i[23]}}, data_rdata_i[23:16]};
      end

      2'b11:
      begin
        if (data_sign_ext_q == 1'b0)
          rdata_b_ext = {24'h00_0000, data_rdata_i[31:24]};
        else
          rdata_b_ext = {{24{data_rdata_i[31]}}, data_rdata_i[31:24]};
      end
    endcase // case (rdata_offset_q)
  end

  // select word, half word or byte sign extended version
  always_comb
  begin
    case (data_type_q)
      2'b00:       data_rdata_ext = rdata_w_ext;
      2'b01:       data_rdata_ext = rdata_h_ext;
      2'b10,2'b11: data_rdata_ext = rdata_b_ext;
    endcase //~case(rdata_type_q)
  end



  always_ff @(posedge clk, negedge rst_n)
  begin
    if(rst_n == 1'b0)
      CS            <= IDLE;
    else
      CS            <= NS;
  end

  // output to register file
  assign data_rdata_ex_o = data_rdata_ext;

  // output to data interface
  assign data_addr_o   = data_addr_int;
  assign data_wdata_o  = data_wdata;
  assign data_we_o     = data_we_ex_i;
  assign data_be_o     = data_be;

  assign load_err_o    = 1'b0;
  assign store_err_o   = 1'b0;

  // FSM
  always_comb
  begin
    NS             = CS;
    data_req_o     = 1'b0;
    data_valid_o   = 1'b0;

    case(CS)
      // starts from not active and stays in IDLE until request was granted
      IDLE:
      begin
        if (data_req_ex_i) begin
          data_req_o     = data_req_ex_i;
          if(data_gnt_i)
            NS = WAIT_RVALID;
          else
            NS = WAIT_GNT;
        end
      end //~ IDLE

      WAIT_GNT:
      begin
        //useful in case [1]
        data_req_o        = 1'b1;
        if(data_gnt_i) begin
          NS = WAIT_RVALID;
        end
      end //~ WAIT_GNT

      WAIT_RVALID:
      begin
        if(data_rvalid_i) begin
          data_valid_o = 1'b1;
          NS           = IDLE;
        end
      end //~ WAIT_RVALID


      default: begin
      end
    endcase
  end

  assign busy_o = (CS == WAIT_RVALID) || (data_req_o == 1'b1);

endmodule

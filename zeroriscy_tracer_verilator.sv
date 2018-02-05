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
// Engineer:       Andreas Traber - atraber@iis.ee.ethz.ch                    //
//                                                                            //
// Additional contributions by:                                               //
//                 Davide Schiavone - pschiavo@iis.ee.ethz.ch                 //
//                                                                            //
// Design Name:    RISC-V Tracer                                              //
// Project Name:   zero-riscy                                                 //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Traces the executed instructions                           //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

`ifdef VERILATOR


`include "zeroriscy_config.sv"

import zeroriscy_defines::*;
import zeroriscy_tracer_defines::*;


// Source/Destination register instruction index
`define REG_S1 19:15
`define REG_S2 24:20
`define REG_S3 29:25
`define REG_D  11:07


module zeroriscy_tracer
  #(
    parameter REG_ADDR_WIDTH      = 5
    )
   (
    // Clock and Reset
    input logic                        clk,
    input logic                        rst_n,

    input logic                        fetch_enable,
    input logic [3:0]                  core_id,
    input logic [5:0]                  cluster_id,

    input logic [31:0]                 pc,
    input logic [31:0]                 instr,
    input logic                        compressed,
    input logic                        id_valid,
    input logic                        is_decoding,
    input logic                        is_branch,
    input logic                        branch_taken,
    input logic                        pipe_flush,
    input logic                        mret_insn,
    input logic                        ecall_insn,
    input logic                        ebrk_insn,
    input logic                        csr_status,
    input logic [31:0]                 rs1_value,
    input logic [31:0]                 rs2_value,
    input logic [31:0]                 lsu_value,

    input logic [(REG_ADDR_WIDTH-1):0] ex_reg_addr,
    input logic                        ex_reg_we,
    input logic [31:0]                 ex_reg_wdata,
    input logic                        data_valid_lsu,
    input logic                        ex_data_req,
    input logic                        ex_data_gnt,
    input logic                        ex_data_we,
    input logic [31:0]                 ex_data_addr,
    input logic [31:0]                 ex_data_wdata,

    input logic [31:0]                 lsu_reg_wdata,

    input logic [31:0]                 imm_u_type,
    input logic [31:0]                 imm_uj_type,
    input logic [31:0]                 imm_i_type,
    input logic [11:0]                 imm_iz_type,
    input logic [31:0]                 imm_z_type,
    input logic [31:0]                 imm_s_type,
    input logic [31:0]                 imm_sb_type
    );

   integer                             f;
   string                              fn;
   integer                             cycles;
   logic [ 4:0]                        rd, rs1, rs2, rs3;

   typedef struct                      {
      logic [(REG_ADDR_WIDTH-1):0]     addr;
      logic [31:0]                     value;
   } reg_t;

   typedef struct                      {
      logic [31:0]                     addr;
      logic                            en;
   } mem_acc_t;

   // cycle counter
   always_ff @(posedge clk, negedge rst_n)
     begin
        if (rst_n == 1'b0)
          cycles = 0;
        else
          cycles = cycles + 1;
     end

   // open/close output file for writing
   initial
     begin
        $sformat(fn, "trace_core.log");
        $display("[TRACER] Output filename is: %s", fn);
        f = $fopen(fn, "w");
        $fwrite(f, "         Cycles PC       Instr    Mnemonic                             | \n");

     end

   final
     begin
        $fclose(f);
     end

   assign rd  = instr[`REG_D];
   assign rs1 = instr[`REG_S1];
   assign rs2 = instr[`REG_S2];
   assign rs3 = instr[`REG_S3];

   reg_t reg_rs1;
   reg_t reg_rs2;
   reg_t reg_rs3;
   reg_t reg_rd;
   mem_acc_t mem_acc;

   string       str;

   function void printInstrTrace
     (
      input integer      f,
      input integer      cycles,
      input logic [31:0] pc,
      input logic [31:0] instr,
      input string       str
      );
      begin
         $fwrite(f, "%15d %h %h %36s  ",
                 cycles,
                 pc,
                 instr,
                 str);

         if (reg_rd.addr != 0)
           $fwrite(f, " x%02d=%08x", reg_rd.addr, reg_rd.value);
         else
           $fwrite(f, "             ");
         if (reg_rs1.addr != 0)
           $fwrite(f, " x%02d:%08x", reg_rs1.addr, reg_rs1.value);
         else
           $fwrite(f, "             ");
         if (reg_rs2.addr != 0)
           $fwrite(f, " x%02d:%08x", reg_rs2.addr, reg_rs2.value);
         else
           $fwrite(f, "             ");

         if (mem_acc.en)
           $fwrite(f, " PA:%08x", mem_acc.addr);
         else
           $fwrite(f, "            ");

         $fwrite(f, "\n");
      end

   endfunction

   function void printMnemonic(input string mnemonic);
      begin
         str = mnemonic;
      end
   endfunction

   function void printRInstr(input string mnemonic);
      begin
         reg_rd.addr = rd;
         reg_rd.value = ex_reg_wdata;
         reg_rs1.addr = rs1;
         reg_rs1.value = rs1_value;
         reg_rs2.addr = rs2;
         reg_rs2.value = rs2_value;
         str = $sformatf("%16s x%02d, x%02d, x%02d      ", mnemonic, rd, rs1, rs2);
      end

   endfunction

   function void printIInstr(input string mnemonic);
      begin
         reg_rd.addr = rd;
         reg_rd.value = ex_reg_wdata;
         reg_rs1.addr = rs1;
         reg_rs1.value = rs1_value;
         str = $sformatf("%16s x%02d, x%02d, %9d", mnemonic, rd, rs1, $signed(imm_i_type));
      end

   endfunction

   function void printIuInstr(input string mnemonic);
      begin
         reg_rd.addr = rd;
         reg_rd.value = ex_reg_wdata;
         reg_rs1.addr = rs1;
         reg_rs1.value = rs1_value;
         str = $sformatf("%16s x%02h, x%02h, 0x%07h", mnemonic, rd, rs1, imm_i_type[27:0]);
      end
   endfunction

   function void printUInstr(input string mnemonic);
      begin
         reg_rd.addr = rd;
         reg_rd.value = ex_reg_wdata;
         str = $sformatf("%16s x%02d, 0x%08h    ", mnemonic, rd, {imm_u_type[31:12], 12'h000});
      end
   endfunction

   function void printUJInstr(input string mnemonic);
      begin
         reg_rd.addr = rd;
         reg_rd.value = ex_reg_wdata;
         str =  $sformatf("%16s x%02d, %14d", mnemonic, rd, $signed(imm_uj_type));
      end
    endfunction

   function void printSBInstr(input string mnemonic);
      begin
         reg_rs1.addr = rs1;
         reg_rs1.value = rs1_value;
         reg_rs2.addr = rs2;
         reg_rs2.value = rs2_value;
         str =  $sformatf("%16s x%02d, x%02d, %9d", mnemonic, rs1, rs2, $signed(imm_sb_type));
      end
   endfunction

   function void printCSRInstr(input string mnemonic);
      logic [11:0] csr;
      begin
         csr = instr[31:20];

         reg_rd.addr = rd;
         reg_rd.value = ex_reg_wdata;

         if (instr[14] == 1'b0) begin
            reg_rs1.addr = rs1;
            reg_rs1.value = rs1_value;
            str = $sformatf("%16s x%02d, x%02d, 0x%03h    ", mnemonic, rd, rs1, csr);
         end else begin
            str = $sformatf("%16s x%02d, 0x%05h, 0x%03h", mnemonic, rd, imm_z_type[19:0], csr);
         end
      end
   endfunction

   function void printLoadInstr();
      string mnemonic;
      logic [2:0] size;
      begin
         // detect reg-reg load and find size
         size = instr[14:12];
         if (instr[14:12] == 3'b111)
           size = instr[30:28];

         case (size)
           3'b000: mnemonic = "lb              ";
           3'b001: mnemonic = "lh              ";
           3'b010: mnemonic = "lw              ";
           3'b100: mnemonic = "lbu             ";
           3'b101: mnemonic = "lhu             ";
           3'b110: mnemonic = "p.elw           ";
           3'b011,
             3'b111: begin
                printMnemonic("INVALID                             ");
                return;
             end
         endcase

         reg_rd.addr = rd;
         reg_rd.value = lsu_reg_wdata;

         if (instr[14:12] != 3'b111) begin
            // regular load
            reg_rs1.addr = rs1;
            reg_rs1.value = rs1_value;
            mem_acc.en=1;
            mem_acc.addr=ex_data_addr;
            str = $sformatf("%16s x%02d, %9d(x%02d)", mnemonic, rd, $signed(imm_i_type), rs1);
         end else begin
            printMnemonic("INVALID                             ");
         end
      end
   endfunction

   function void printStoreInstr();
      string mnemonic;
      begin
         case (instr[13:12])
           2'b00:  mnemonic = "sb              ";
           2'b01:  mnemonic = "sh              ";
           2'b10:  mnemonic = "sw              ";
           2'b11: begin
              printMnemonic("INVALID                             ");
              return;
           end
         endcase

         if (instr[14] == 1'b0) begin
            // regular store
            reg_rs1.addr = rs1;
            reg_rs1.value = rs1_value;
            reg_rs2.addr = rs2;
            reg_rs2.value = rs2_value;
            mem_acc.en=1;
            mem_acc.addr=ex_data_addr;
            str = $sformatf("%16s x%02d, %9d(x%02d)", mnemonic, rs2, $signed(imm_s_type), rs1);
         end else begin
              printMnemonic("INVALID                             ");
         end
      end
   endfunction

   // log execution
   always @(negedge clk)
     begin
        // special case for WFI because we don't wait for unstalling there
        if ( (id_valid || mret_insn || ecall_insn || pipe_flush || ebrk_insn || csr_status || ex_data_req) && is_decoding)
          begin
             reg_rs1.addr=0;
             reg_rs2.addr=0;
             reg_rs3.addr=0;
             reg_rd.addr=0;
             mem_acc.en=0;

             // use casex instead of case inside due to ModelSim bug
             casex (instr)
               // Aliases
               32'h00_00_00_13:  printMnemonic("nop                                 ");
                // Regular opcodes
               INSTR_LUI:        printUInstr("lui             ");
               INSTR_AUIPC:      printUInstr("auipc           ");
               INSTR_JAL:        printUJInstr("jal             ");

               INSTR_JALR:       printIInstr("jalr            ");
               // BRANCH
               INSTR_BEQ:        printSBInstr("beq             ");
               INSTR_BNE:        printSBInstr("bne             ");
               INSTR_BLT:        printSBInstr("blt             ");
               INSTR_BGE:        printSBInstr("bge             ");
               INSTR_BLTU:       printSBInstr("bltu            ");
               INSTR_BGEU:       printSBInstr("bgeu            ");
               // OPIMM
               INSTR_ADDI:       printIInstr("addi            ");
               INSTR_SLTI:       printIInstr("slti            ");
               INSTR_SLTIU:      printIInstr("sltiu           ");
               INSTR_XORI:       printIInstr("xori            ");
               INSTR_ORI:        printIInstr("ori             ");
               INSTR_ANDI:       printIInstr("andi            ");
               INSTR_SLLI:       printIuInstr("slli            ");
               INSTR_SRLI:       printIuInstr("srli            ");
               INSTR_SRAI:       printIuInstr("srai            ");
               // OP
               INSTR_ADD:        printRInstr("add             ");
               INSTR_SUB:        printRInstr("sub             ");
               INSTR_SLL:        printRInstr("sll             ");
               INSTR_SLT:        printRInstr("slt             ");
               INSTR_SLTU:       printRInstr("sltu            ");
               INSTR_XOR:        printRInstr("xor             ");
               INSTR_SRL:        printRInstr("srl             ");
               INSTR_SRA:        printRInstr("sra             ");
               INSTR_OR:         printRInstr("or              ");
               INSTR_AND:        printRInstr("and             ");
               // SYSTEM (CSR manipulation)
               INSTR_CSRRW:      printCSRInstr("csrrw           ");
               INSTR_CSRRS:      printCSRInstr("csrrs           ");
               INSTR_CSRRC:      printCSRInstr("csrrc           ");
               INSTR_CSRRWI:     printCSRInstr("csrrwi          ");
               INSTR_CSRRSI:     printCSRInstr("csrrsi          ");
               INSTR_CSRRCI:     printCSRInstr("csrrci          ");
               // SYSTEM (others)
               INSTR_ECALL:      printMnemonic("ecall                               ");
               INSTR_EBREAK:     printMnemonic("ebreak                              ");
               INSTR_MRET:       printMnemonic("mret                                ");
               INSTR_WFI:        printMnemonic("wfi                                 ");
               // RV32M
               INSTR_PMUL:       printRInstr("mul             ");
               INSTR_PMUH:       printRInstr("mulh            ");
               INSTR_PMULHSU:    printRInstr("mulhsu          ");
               INSTR_PMULHU:     printRInstr("mulhu           ");
               INSTR_DIV:        printRInstr("div             ");
               INSTR_DIVU:       printRInstr("divu            ");
               INSTR_REM:        printRInstr("rem             ");
               INSTR_REMU:       printRInstr("remu            ");
               {25'b?, OPCODE_LOAD}:  printLoadInstr();
               {25'b?, OPCODE_STORE}: printStoreInstr();
               default:          printMnemonic("INVALID                             ");
             endcase // unique case (instr)

             if(~ex_data_req|data_valid_lsu)
               printInstrTrace(f, cycles, pc, instr, str);

          end
     end // always @ (posedge clk)

endmodule
`endif

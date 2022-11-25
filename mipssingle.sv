// mipssingle.sv

// Single-cycle MIPS processor

module mips(input  logic        clk, reset,
            output logic [31:0] pc,
            input  logic [31:0] instr,
            output logic        memwrite,
            output logic [31:0] aluout, writedata,
            input  logic [31:0] readdata);

  logic       regdst, regwrite, pcsrc, zero, AvsB;
  logic [1:0] memtoreg, alusrc, jump;
  logic [2:0] alucontrol;

   controller c(instr[31:26], instr[5:0],
                instr[28:26], zero, AvsB,
                memwrite, pcsrc, regdst, regwrite,
                memtoreg, alusrc, jump, alucontrol);

    datapath dp(clk, reset, pcsrc, regdst, regwrite, 
                memtoreg, alusrc, jump, alucontrol, 
                instr, readdata, pc, aluout, writedata, 
                zero, AvsB);
endmodule

module controller(input  logic [5:0] op, funct, 
                  input  logic [2:0] imFunct,
                  input  logic       zero, AvsB,
                  output logic       memwrite,
                  output logic       pcsrc, 
                  output logic       regdst, regwrite, 
                  output logic [1:0] memtoreg, alusrc, jump, 
                  output logic [2:0] alucontrol);

  logic [1:0] aluop, branch;

  maindec md(op, funct, memwrite, regdst, 
             regwrite, memtoreg, branch, 
             aluop, alusrc, jump);

  aludec ad(funct, aluop, imFunct, alucontrol);

  assign pcsrc = (branch[1] & AvsB) | (branch[0] & zero);
endmodule

module maindec(input  logic [5:0] op, funct, 
               output logic       memwrite,
               output logic       regdst, regwrite,
               output logic [1:0] memtoreg, branch,
               output logic [1:0] aluop, alusrc, jump);

  logic [12:0] controls;

  assign {regwrite, regdst, alusrc, branch, memwrite, 
          memtoreg, jump, aluop} = controls;

  always_comb
    case(op)
      6'b000000: begin
        if (funct == 5'b001000)
          controls <= 13'b0000000001000; // JR
        else
          controls <= 13'b1100000000010; // RTYPE
      end
      6'b100011: controls <= 13'b1001000010000; // LW
      6'b101011: controls <= 13'b0001001000000; // SW
      6'b000100: controls <= 13'b0000010000001; // BEQ
      6'b001000: controls <= 13'b1001000000000; // ADDI
      6'b000010: controls <= 13'b0000000000100; // J
      6'b000111: controls <= 13'b0000100000000; // BGTZ
      6'b100001: controls <= 13'b1001000100000; // LH
      6'b001110: controls <= 13'b1010000000011; // XORI
      6'b100100: controls <= 13'b1001000110000; // LHU
      6'b001100: controls <= 13'b1010000000011; // ANDI
      default:   controls <= 13'bxxxxxxxxxxxxx; // illegal op
    endcase
endmodule

module aludec(input  logic [5:0] funct,
              input  logic [1:0] aluop, 
              input  logic [2:0] imFunct,
              output logic [2:0] alucontrol);

  always_comb
    case(aluop)
      2'b00: alucontrol <= 3'b010; // add (for lw/sw/addi)
      2'b01: alucontrol <= 3'b110; // sub (for beq)
      2'b10: case(funct)           // Functions (R-type instructions)
            6'b100000: alucontrol <= 3'b010; // add
            6'b100010: alucontrol <= 3'b110; // sub
            6'b100100: alucontrol <= 3'b000; // and
            6'b100101: alucontrol <= 3'b001; // or
            6'b101010: alucontrol <= 3'b111; // slt
            6'b000010: alucontrol <= 3'b011; // srl
            default:   alucontrol <= 3'bxxx; // ???
        endcase
      2'b11: case(imFunct)                 // Immediate Functions
            3'b100:  alucontrol <= 3'b000; // and immediate
            3'b110:  alucontrol <= 3'b101; // xor immediate
            default: alucontrol <= 3'bxxx; // ???
        endcase
      default: alucontrol <= 3'bxxx; // ???
    endcase
endmodule

module datapath(input  logic         clk, reset,
                input  logic         pcsrc, regdst,
                input  logic         regwrite, 
                input  logic [1:0]   memtoreg, alusrc, jump, 
                input  logic [2:0]   alucontrol, 
                input  logic [31:0]  instr, readdata,
                output logic [31:0]  pc, 
                output logic [31:0]  aluout, writedata,
                output logic         zero, AvsB);

  logic [4:0]  writereg;
  logic [31:0] pcnext, pcnextbr, pcplus4, pcbranch;
  logic [31:0] signimm, signimmsh, zeroimm;
  logic [31:0] srca, srcb;
  logic [31:0] result, readHalf, readByte;
  logic [31:0] EMPTY;

  // next PC logic
  flopr #(32) pcreg(clk, reset, pcnext, pc);
  adder       pcadd1(pc, 32'b100, pcplus4);
  sl2         immsh(signimm, signimmsh);
  adder       pcadd2(pcplus4, signimmsh, pcbranch);
  mux2 #(32)  pcbrmux(pcplus4, pcbranch, pcsrc, pcnextbr);
  mux4 #(32)  pcmux(pcnextbr, {pcplus4[31:28],instr[25:0],2'b00}, result, EMPTY, jump, pcnext);

  // register file logic
  regfile   rf(clk, regwrite, instr[25:21], instr[20:16], writereg, result, srca, writedata);
  mux2 #(5) wrmux(instr[20:16], instr[15:11], regdst, writereg);

  assign readHalf = 32'(signed'(readdata[15:0]));
  assign readByte = {24'b000000000000000000000000, readdata[7:0]};
  mux4 #(32) resmux(aluout, readdata, readHalf, readByte, memtoreg, result);
  
  signext    se(instr[15:0], signimm);
  zeroext    ze(instr[15:0], zeroimm);

  // ALU logic
  mux4 #(32) srcbmux(writedata, signimm, zeroimm, EMPTY, alusrc, srcb);
  alu        alu(srca, srcb, alucontrol, instr[10:6], aluout, zero, AvsB);
endmodule

module regfile(input  logic clk,
               input  logic we3,
               input  logic [4:0] ra1, ra2, wa3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally
  // write third port on rising edge of clk
  // register 0 hardwired to 0
  // note: for pipelined processor, write third port
  // on falling edge of clk

  always_ff @(posedge clk)
    if (we3) rf[wa3] <= wd3;
    
  assign rd1 = (ra1 != 0) ? rf[ra1] : 0;
  assign rd2 = (ra2 != 0) ? rf[ra2] : 0;
endmodule

module adder(input  logic [31:0] a, b, 
             output logic [31:0] y);

  assign y = a + b;
endmodule

module sl2(input  logic [31:0] a,
           output logic [31:0] y);

  // shift left by 2
  assign y = {a[29:0], 2'b00};
endmodule

module signext(input  logic [15:0] a, 
               output logic [31:0] y);

  assign y = {{16{a[15]}}, a};
endmodule

module zeroext(input  logic [15:0] a, 
               output logic [31:0] y);

  assign y = {16'b0000000000000000, a};
endmodule

module flopr #(parameter WIDTH = 8) 
              (input  logic             clk, reset, 
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module mux2 #(parameter WIDTH = 8) 
             (input  logic [WIDTH-1:0] d0, d1,  
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0;
endmodule

module mux4 #(parameter WIDTH = 8) 
             (input  logic [WIDTH-1:0] d0, d1, d2, d3, 
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  always_comb
    case (s)
        2'b00: y <= d0;
        2'b01: y <= d1;
        2'b10: y <= d2;
        2'b11: y <= d3;
        default: y <= d0;
    endcase
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [2:0]  alucontrol, 
           input  logic [4:0]  shamt,
           output logic [31:0] result, 
           output logic        zero, AvsB);

  logic [31:0] condinvb, sum;

  assign condinvb = alucontrol[2] ? ~b : b;
  assign sum = a + condinvb + alucontrol[2];

  always_comb
    casez (alucontrol)
      3'bz00: result <= a & condinvb;
      3'b001: result <= a | condinvb;
      3'bz10: result <= sum;
      3'b111: result <= sum[31];
      3'b101: result <= a ^ b;
      3'b011: result <= condinvb >> shamt;
    endcase

  assign zero = (result == 32'b0);
  assign AvsB = a > b;
endmodule

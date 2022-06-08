
//`#start header` -- edit after this line, do not edit this line
// ========================================
//
// Robert Johnson
// Copyright UCSC, 2022
// All Rights Reserved
// UNPUBLISHED, LICENSED SOFTWARE.
//
// CONFIDENTIAL AND PROPRIETARY INFORMATION
// WHICH IS THE PROPERTY OF UCSC.
//
// Logic state machine to control the readout of the external SAR ADCs
//
// ========================================
`include "cypress.v"
//`#end` -- edit above this line, do not edit this line
// Generated on 11/17/2021 at 10:05
// Component: ADCshiftCtrl
module ADCshiftCtrl (
	output  ClkOut,
	output  Done,
	input   Clock,
	input   Reset,
	input   Start
);

//`#start body` -- edit after this line, do not edit this line

reg Done2, ClkOut2;
assign Done = Done2;
assign ClkOut = ClkOut2;

parameter [1:0] Wait = 2'b00;   // Wait for the Start signal
parameter [1:0] Doit = 2'b01;   // Output the 6 MHz readout clock high
parameter [1:0] Clow = 2'b10;   // Output the 6 MHz readout clock low 
parameter [1:0] Fini = 2'b11;   // Transfer the shift register contents to the output buffer and notify the CPU

reg [1:0] State, NextState;
reg [3:0] Cnt;

always @ (State or Start or Cnt) begin
    case (State) 
      Wait: begin
                if (Start) NextState = Doit;
                else NextState = Wait;
                Done2 = 1'b0;
                ClkOut2 = 1'b0;
            end
      Doit: begin
                NextState = Clow;
                Done2 = 1'b0;
                ClkOut2 = 1'b1;
            end
      Clow: begin
                if (Cnt == 11) NextState = Fini;
                else NextState = Doit;
                Done2 = 1'b0;
                ClkOut2 = 1'b0;
            end
      Fini: begin
                NextState = Wait;
                Done2 = 1'b1;
                ClkOut2 = 1'b0;
            end
    endcase
end

always @ (posedge Clock) begin
    if (Reset) begin
        State <= Wait;
    end else begin
        State <= NextState;
        case (State)
          Wait: begin
                    Cnt <= 0;
                end
          Clow: begin
                    Cnt <= Cnt + 1;
                end
        endcase
    end
end

//`#end` -- edit above this line, do not edit this line
endmodule
//`#start footer` -- edit after this line, do not edit this line
//`#end` -- edit above this line, do not edit this line

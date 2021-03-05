
//`#start header` -- edit after this line, do not edit this line
// ========================================
//
// State machine to look for trigger coincidences. This version makes use
// of an external 7-bit down counter to provide the timing. That uses fewer
// PSOC resources compared with implementing a 7-bit counter in Verilog.
//
// Copyright UCSC, 2020
// All Rights Reserved
// UNPUBLISHED, LICENSED SOFTWARE.
//
// CONFIDENTIAL AND PROPRIETARY INFORMATION
// WHICH IS THE PROPERTY OF UCSC.
//
// ========================================
`include "cypress.v"
//`#end` -- edit above this line, do not edit this line
// Generated on 03/11/2020 at 14:57
// Component: coincidence_v2
module coincidence_v2 (
	output  RstCnt,
	output  RstPeak,
	output  Y,
	input   A,
	input   B,
	input   Clk,
	input  [6:0] Cnt,
	input   eoc,
	input   Reset,
	input   tc
);
	parameter Delay = 48;
	parameter Window = 58;

//`#start body` -- edit after this line, do not edit this line
//
// RstCnt resets the 7-bit counter
// RstPeak resets the analog peak detectors
// Y starts the ADC conversion
// A and B are the inputs from the two trigger discriminators
// Clk is a 12 MHz clock
// Cnt is the state of the 7-bit counter
// eoc is the end-of-conversion signal from the ADC
// Reset is supplied by a control register operated by the CPU; puts machine into the Wait state
// tc is the termination count of the 7-bit counter
//
// Enumeration of the states of the state machine
parameter [2:0] Null = 3'b000;   // Reset counter and then go to Clrp 
parameter [2:0] Wait = 3'b001;   // Wait for either input to go high
parameter [2:0] Look = 3'b010;   // Look for a coincidence within the window
parameter [2:0] Clrp = 3'b011;   // Send signal to clear the peak detector
parameter [2:0] Dela = 3'b100;   // Delay before signaling to start the ADC
parameter [2:0] Outp = 3'b101;   // Output a signal to start the ADC
parameter [2:0] Weoc = 3'b110;   // Wait for the ADC to finish
parameter [2:0] Done = 3'b111;   // Wait until both signals go low before looking for a new coincidence

reg [2:0] State, NextState;

reg Y2;
assign Y = Y2;

reg C2;
assign RstPeak = C2;

reg rstC;
assign RstCnt = rstC;

// Combinatorial logic for the state machine
always @ (State or A or B or Cnt) begin
    case (State)
        Wait: begin
		         if (A & B) NextState = Dela;
                 else if (A | B) NextState = Look;
                 else NextState = Wait;
                 C2 = 1'b0;
                 Y2 = 1'b0;
                 rstC = 1'b1;
              end
        Look: begin
                 if (A & B) NextState = Dela;
				 else if (Cnt == Window) NextState = Clrp;
                 else NextState = Look;
                 C2 = 1'b0;
                 Y2 = 1'b0;
                 rstC = 1'b0;
              end
        Clrp: begin
                 if (tc) NextState = Done;
                 else NextState = Clrp;
                 C2 = 1'b1;
                 Y2 = 1'b0;
                 rstC = 1'b0;
              end
		Dela: begin
		         if (Cnt == Delay) NextState = Outp;
				 else NextState = Dela;
                 C2 = 1'b0;
                 Y2 = 1'b0;
                 rstC = 1'b0;
		      end
		Outp: begin
		         NextState = Weoc;
                 C2 = 1'b0;
                 Y2 = 1'b1;
                 rstC = 1'b1;
		      end
        Weoc: begin
                 if (eoc) begin
                     NextState = Clrp;
                 end else if (tc) begin // Sometime eoc doesn't come, but why?
                     NextState = Null;
                 end else begin
                     NextState = Weoc;
                 end
                 C2 = 1'b0;
                 Y2 = 1'b0;
                 rstC = 1'b0;
              end
        Done: begin
                 if (!(A | B)) NextState = Wait;
                 else NextState = Done;
                 C2 = 1'b0;
                 Y2 = 1'b0;
                 rstC = 1'b1;
              end
        Null: begin  
                 NextState = Clrp;
                 C2 = 1'b0;
                 Y2 = 1'b0;
                 rstC = 1'b1;
              end
    endcase
end

// Synchronous block of the state machine
always @ (posedge Clk) begin
    if (Reset) begin
        State <= Wait;
    end else begin
	    State <= NextState;
    end     
end

//`#end` -- edit above this line, do not edit this line
endmodule
//`#start footer` -- edit after this line, do not edit this line
//`#end` -- edit above this line, do not edit this line

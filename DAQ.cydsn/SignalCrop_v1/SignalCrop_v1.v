
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
// Set default coincidence time to 6 counts (0.5 us), longer than the 330ns peaking time. 7/9/22  RJ
// ========================================
`include "cypress.v"
//`#end` -- edit above this line, do not edit this line
// Generated on 07/09/2022 at 17:35
// Component: SignalCrop_v1
module SignalCrop_v1 (
	output  RstCtr,
	output  Y,
	input   A,
	input   Clk,
	input   Reset,
	input   tc
);
	parameter CoincWindow = 6;

//`#start body` -- edit after this line, do not edit this line

parameter [1:0] Wait = 2'b00;  // Wait for a rising edge at the input
parameter [1:0] Cont = 2'b01;  // Count down to hold the output high for the specified trigger window
parameter [1:0] Done = 2'b10;  // Wait for the input to go low
parameter [1:0] WtAt = 2'b11;  // Wait for another down-counter period, to avoid second pulses

reg [1:0] State, NextState;

reg Y2, RstCtr2;
assign Y = Y2;
assign RstCtr = RstCtr2;
reg [2:0] ctr;
always @ (State or A or tc) begin
    case (State) 
	    Wait: begin
		          if (A) NextState = Cont;
				  else NextState = Wait;
                  Y2 = 1'b0;
                  RstCtr2 = 1'b1;
		      end
	    Cont: begin
		          if (ctr == CoincWindow) NextState = Done;
				  else NextState = Cont;
                  Y2 = 1'b1;
                  RstCtr2 = 1'b1;
		      end
		Done: begin
		          if (!A) NextState = WtAt;
				  else NextState = Done;
                  Y2 = 1'b0;
                  RstCtr2 = 1'b1;
		      end
		WtAt: begin
		          if (tc) NextState = Wait;
                  else NextState = WtAt;
                  Y2 = 1'b0;
                  RstCtr2 = 1'b0;
		      end
	endcase
end

always @ (posedge Clk) begin
    if (Reset) begin
	    State <= Wait;
	end else begin
	    State <= NextState;
        if (State == Cont) ctr <= ctr + 1;
        else ctr <= 0;       
	end
end
//`#end` -- edit above this line, do not edit this line
endmodule
//`#start footer` -- edit after this line, do not edit this line
//`#end` -- edit above this line, do not edit this line


//`#start header` -- edit after this line, do not edit this line
// ========================================
//
// Copyright YOUR COMPANY, THE YEAR
// All Rights Reserved
// UNPUBLISHED, LICENSED SOFTWARE.
//
// CONFIDENTIAL AND PROPRIETARY INFORMATION
// WHICH IS THE PROPERTY OF your company.
//
// ========================================
`include "cypress.v"
//`#end` -- edit above this line, do not edit this line
// Generated on 01/14/2021 at 13:35
// Component: SignalCrop_v1
module SignalCrop_v1 (
	output  RstCtr,
	output  Y,
	input   A,
	input   Clk,
	input   Reset,
	input   tc
);

//`#start body` -- edit after this line, do not edit this line

parameter [1:0] Wait = 2'b00;  // Wait for a rising edge at the input
parameter [1:0] Cont = 2'b01;  // Count down to hold the output high for the specified trigger window
parameter [1:0] Done = 2'b10;  // Wait for the input to go low
parameter [1:0] WtAt = 2'b11;  // Wait for another down-counter period, to avoid second pulses

reg [1:0] State, NextState;

assign Y = (State == Cont);
assign RstCtr = (State == Wait || State == Done);
always @ (State or A or tc) begin
    case (State) 
	    Wait: begin
		          if (A) NextState = Cont;
				  else NextState = Wait;
		      end
	    Cont: begin
		          if (tc) NextState = Done;
				  else NextState = Cont;
		      end
		Done: begin
		          if (!A) NextState = WtAt;
				  else NextState = Done;
		      end
		WtAt: begin
		          if (tc) NextState = Wait;
                  else NextState = WtAt;
		      end
	endcase
end

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

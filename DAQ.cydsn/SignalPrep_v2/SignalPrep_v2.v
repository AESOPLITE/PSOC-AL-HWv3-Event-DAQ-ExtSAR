
//`#start header` -- edit after this line, do not edit this line
// ========================================
//
// Copyright UCSC, 2022
// All Rights Reserved
// UNPUBLISHED, LICENSED SOFTWARE.
//
// CONFIDENTIAL AND PROPRIETARY INFORMATION
// WHICH IS THE PROPERTY OF UCSC.
//
// ========================================
`include "cypress.v"
//`#end` -- edit above this line, do not edit this line
// Generated on 09/29/2020 at 22:26
// Component: SignalPrep_v2
module SignalPrep_v2 (
	output  Y,
	input   A,
	input   Clk,
	input   Reset
);
	parameter pulseWidth = 7;

//`#start body` -- edit after this line, do not edit this line

// Look for a leading edge. Once found, look for the trailing edge.
// Once the trailing edge if found, wait pulseWidth clock cycles
// before looking for a leading edge again. The point is to avoid
// secondary pulses on the output.
parameter [1:0] Wait = 2'b00;
parameter [1:0] Cont = 2'b01;
parameter [1:0] Done = 2'b10;
parameter [1:0] Null = 2'b11;

reg [1:0] State, NextState;
reg [3:0] cnt;

assign Y = (State == Cont);
always @ (State or A or cnt) begin
    case (State) 
	    Wait: begin
		          if (A) NextState = Cont;
				  else NextState = Wait;
		      end
	    Cont: begin
		          if (A) NextState = Cont;
				  else NextState = Done;
		      end
		Done: begin
		          if (cnt == pulseWidth) NextState = Wait;
				  else NextState = Done;
		      end
		Null: begin
		          NextState = Wait;
		      end
	endcase
end

always @ (posedge Clk) begin
    if (Reset) begin
	    State <= Wait;
	end else begin
	    State <= NextState;
        case (State)
            Cont: begin
                      cnt <= 0;
                  end
            Done: begin
                      cnt <= cnt + 1;
                  end
        endcase
	end
end
//`#end` -- edit above this line, do not edit this line
endmodule
//`#start footer` -- edit after this line, do not edit this line
//`#end` -- edit above this line, do not edit this line

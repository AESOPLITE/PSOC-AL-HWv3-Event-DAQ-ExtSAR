
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
// Generated on 02/02/2021 at 16:34
// Component: EdgeDet_V1
module EdgeDet_V1 (
	output  Q,
	input   A,
	input   Clock
);

//`#start body` -- edit after this line, do not edit this line

//        Your code goes here
    parameter [1:0] Wait = 2'b00;
    parameter [1:0] Outp = 2'b01;
    parameter [1:0] Look = 2'b10;
    parameter [1:0] Noop = 2'b11;
    
    reg [1:0] State, NextState;
    
    assign Q = (State == Outp);
    always @ (State or A) begin
        case (State) 
    	    Wait: begin
    		          if (A) NextState = Outp;
    				  else NextState = Wait;
    		      end
    	    Outp: begin
    		          NextState = Look;
    		      end
    		Look: begin
    		          if (!A) NextState = Wait;
    				  else NextState = Look;
    		      end
    		Noop: begin
    		          NextState = Wait;
    		      end
    	endcase
    end
    
    always @ (posedge Clock) begin
	    State <= NextState;
	end

//`#end` -- edit above this line, do not edit this line
endmodule
//`#start footer` -- edit after this line, do not edit this line
//`#end` -- edit above this line, do not edit this line

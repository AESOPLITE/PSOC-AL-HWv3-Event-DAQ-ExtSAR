
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
// Generated on 01/25/2021 at 19:33
// Component: Stretch4_V1
module Stretch4_V1 (
	output  Q,
	input   A,
	input   Clock
);

//`#start body` -- edit after this line, do not edit this line

//        Your code goes here
parameter Wait = 1'b0;
parameter Cont = 1'b1;

reg State, NextState;
reg [1:0] Cnt;
assign Q = (State == Cont);

always @ (State or A) begin
    if (State == Wait) begin
        if (A) NextState = Cont;
        else NextState = Wait;
    end else begin
        if (Cnt == 2'b11) NextState = Wait;
        else NextState = Cont;
    end
end

always @ (posedge Clock) begin
    State <= NextState;
    if (State == Wait) begin
        Cnt <= 2'b00;
    end else begin
        Cnt <= Cnt + 1;
    end
end
//`#end` -- edit above this line, do not edit this line
endmodule
//`#start footer` -- edit after this line, do not edit this line
//`#end` -- edit above this line, do not edit this line

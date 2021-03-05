
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
// Generated on 03/27/2020 at 10:55
// Component: Cntr8Bit_V1
module Cntr8Bit_V1 (
	output  tc,
	input   A,
	input   Clk,
	input   Reset
);

//`#start body` -- edit after this line, do not edit this line

//        Your code goes here
reg tc2;
assign tc = tc2;
reg [7:0] cnt;

parameter Wait = 1'b0;   // Wait for a rising edge on A
parameter Cnts = 1'b1;   // Wait for A to go back down

reg State, NextState;

always @ (State or A) begin
    if (A) NextState = Cnts;
    else NextState = Wait;
end

always @ (posedge Clk) begin
    if (Reset) begin
        cnt <= 0;
    end else begin
        State = NextState;
        if (State == Wait && A) begin
            cnt <= cnt + 1;
            if (cnt == 8'hff) tc2 <= 1'b1;
        end else begin
            tc2 <= 1'b0;
        end
    end
end

//`#end` -- edit above this line, do not edit this line
endmodule
//`#start footer` -- edit after this line, do not edit this line
//`#end` -- edit above this line, do not edit this line

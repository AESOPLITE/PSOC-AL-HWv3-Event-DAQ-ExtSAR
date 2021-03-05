
//`#start header` -- edit after this line, do not edit this line
// ========================================
//
// State Machine to sequence signals into the SAR ADC and control the peak-detector reset
// Robert Johnson March 2020
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
// Generated on 03/26/2020 at 10:57
// Component: ACD_CTRL_v1
module ACD_CTRL_v1 (
	output  Done,
	output [1:0] MUX,
	output  RstCt,
	output  RstPk1,
	output  SOC,
	input   ChOR,
	input   CLK,
	input   EOC1,
	input   EOC2,
	input   GO,
	input   NRQ1,
	input   NRQ2,
	input   Reset,
	input   tc
);

//`#start body` -- edit after this line, do not edit this line
// Arguments:
//    Done  - signal for the status register, to indicate that the ADC has been read out twice
//    MUX   - control signal for the analog multiplexer
//    RstCt - reset signal for the external Count7 counter
//    SOC   - signal to the SAR ADC to start a conversion
//    ChOR  - OR of the two channel outputs
//    CLK   - system clock
//    EOCn  - signal that the SAR ADC has completed a conversion
//    GO    - AND of the two channel outputs
//    NRQn  - signal that the DMA transfer of the ADC results has completed
//    Reset - reset the state machine
//    tc    - signal indicating termination of the Count7 counter
//    RstPk - reset signal for the peak detectors
reg SOC2, RstCt2, Done2;
reg [1:0] MUX2;
assign MUX = MUX2;
assign SOC = SOC2;
assign RstCt = RstCt2;
assign Done = Done2;

reg [3:0] State, NextState;
reg [1:0] nEOC, nNRQ;
reg [2:0] cnt;

parameter [3:0] Wait = 4'b0000;  // Wait for a signal from the OR of all 5 comparators
parameter [3:0] Dlay = 4'b0001;  // Delay to look for the GO and for the peak detector signal to stabilize
parameter [3:0] Soc0 = 4'b0010;  // Send first start of conversion signal
parameter [3:0] Eoc0 = 4'b0011;  // Wait for both ADCs to finish their first conversion
parameter [3:0] Soc1 = 4'b0100;  // Send second start of conversion signal
parameter [3:0] Eoc1 = 4'b0101;  // Wait for both ADCs to finish their second conversion
parameter [3:0] Soc2 = 4'b0110;  // Send third start of conversion signal
parameter [3:0] Eoc2 = 4'b0111;  // Wait for the first ADC to finish its third conversion
parameter [3:0] Wnrq = 4'b1000;  // Wait for the last DMA transfer to complete from both ADC channels, and set status register
parameter [3:0] Down = 4'b1001;  // Wait for the trigger OR to go back to zero
parameter [3:0] Fini = 4'b1010;  // Send out the done signal to reset peak detectors after OR signal is gone

assign RstPk1 = (State == Fini);

// Note: the external Count7 time must be longer than the longest conversion time.
reg GOlatch;
always @ (State or GO or GOlatch or ChOR or EOC1 or EOC2 or NRQ1 or NRQ2 or nEOC or nNRQ or tc or cnt) begin
    case (State) 
      Wait: begin
                if (ChOR) NextState = Dlay;
                else NextState = Wait;
                MUX2 = 2'b00;
                SOC2 = 1'b0;
                RstCt2 = 1'b1;
                Done2 = 1'b0;
            end
      Dlay: begin
                if (tc) begin
                    if (GOlatch) NextState = Soc0;
                    else NextState = Fini;
                end else NextState = Dlay;
                MUX2 = 2'b00;
                SOC2 = 1'b0;
                RstCt2 = 1'b0;
                Done2 = 1'b0;
            end
      Soc0: begin
                NextState = Eoc0;
                MUX2 = 2'b00;
                SOC2 = 1'b1;
                RstCt2 = 1'b1;
                Done2 = 1'b0;
            end
      Eoc0: begin
                if (nEOC > 1) NextState = Soc1;
                //else if (tc) NextState = Fini; // This is to prevent getting stuck forever here
                else NextState = Eoc0;
                MUX2 = 2'b00;
                SOC2 = 1'b0;
                RstCt2 = 1'b0;
                Done2 = 1'b0;
            end
      Soc1: begin
                if (cnt == 3'b111) begin
                    SOC2 = 1'b1;
                    NextState = Eoc1;
                end else begin
                    NextState = Soc1;
                    SOC2 = 1'b0;
                end
                MUX2 = 2'b01;
                RstCt2 = 1'b1;
                Done2 = 1'b0;
            end
      Eoc1: begin
                if (nEOC > 1) begin
                    NextState = Soc2;
                //end else if (tc) begin
                //    NextState = Fini; // This is to prevent getting stuck forever here                    
                end else begin
                    NextState = Eoc1;
                end
                RstCt2 = 1'b0;
                MUX2 = 2'b01;
                SOC2 = 1'b0;               
                Done2 = 1'b0;
            end
      Soc2: begin
                if (cnt == 3'b111) begin
                    SOC2 = 1'b1;
                    NextState = Eoc2;
                end else begin
                    NextState = Soc2;
                    SOC2 = 1'b0;
                end
                MUX2 = 2'b10;
                RstCt2 = 1'b1;
                Done2 = 1'b0;
            end
      Eoc2: begin
                if (nEOC > 1) begin
                    NextState = Wnrq;
                    RstCt2 = 1'b1;
                //end else if (tc) begin
                //    NextState = Fini; // This is to prevent getting stuck forever here
                //    RstCt2 = 1'b0;
                end else begin
                    NextState = Eoc2;
                    RstCt2 = 1'b0;
                end
                MUX2 = 2'b10;
                SOC2 = 1'b0;               
                Done2 = 1'b0;
            end
      Wnrq: begin
                if (nNRQ > 1) begin  
                    NextState = Down;
                end else if (tc) begin
                    NextState = Down;
                end else begin
                    NextState = Wnrq;
                end
                Done2 = 1'b0;
                RstCt2 = 1'b0;
                MUX2 = 2'b00;
                SOC2 = 1'b0;               
            end
      Down: begin
                if (!ChOR) NextState = Fini;
                else NextState = Down;
                MUX2 = 2'b00;
                SOC2 = 1'b0;
                RstCt2 = 1'b1;
                Done2 = 1'b1;
            end
      Fini: begin
                if (tc) NextState = Wait;
                else NextState = Fini;
                MUX2 = 2'b00;
                SOC2 = 1'b0;
                RstCt2 = 1'b0;
                Done2 = 1'b0;
            end
      default: begin
                 NextState = Wait;
                 MUX2 = 2'b00;
                 SOC2 = 1'b0;
                 RstCt2 = 1'b0;
                 Done2 = 1'b0;
               end
    endcase
end

always @ (posedge CLK) begin
    if (Reset) begin
        State <= Wait;
    end else begin
        State <= NextState;
        if (State == Soc0 || State == Soc1 || State == Soc2) begin
            nEOC <= 0;
            nNRQ <= 0;
            cnt <= cnt + 1;
        end else begin
            cnt <= 0;
            if (EOC1 & EOC2) nEOC <= nEOC + 2;
            else if (EOC1 | EOC2) nEOC <= nEOC + 1; 
            if (NRQ1 & NRQ2) nNRQ <= nNRQ + 2;
            else if (NRQ1 | NRQ2) nNRQ <= nNRQ + 1;
        end
        if (State == Fini) begin  // Capture the GO signal if and when it arrives.
            GOlatch <= 1'b0;          
        end else begin
            if (GO) GOlatch <= 1'b1;
        end
    end
end
//`#end` -- edit above this line, do not edit this line
endmodule
//`#start footer` -- edit after this line, do not edit this line
//`#end` -- edit above this line, do not edit this line

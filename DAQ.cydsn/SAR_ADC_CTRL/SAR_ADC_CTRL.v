
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
// Logic to control the peak detectors and to start the external SAR ADCs
// at the right moment.
//
// ========================================
`include "cypress.v"
//`#end` -- edit above this line, do not edit this line
// Generated on 06/08/2022 at 14:22
// Component: SAR_ADC_CTRL
module SAR_ADC_CTRL (
	output  CONVSTB,
	output  DONE,
	output  GO1en,
	output  GOen,
	output  RstCnt,
	output  RstPk,
	input   ChOR,
	input   CLK,
	input   GO,
	input   RST,
	input   TC
);

//`#start body` -- edit after this line, do not edit this line

// Logic to control the PHA digitization with external SAR ADCs
// Arguments:
//	output  CONVSTB,   // Start the SAR ADC conversion for all 5 channels
//	output  DONE,      // Tell the CPU that the SAR ADCs are ready to read out
//	output  RstCnt,    // Reset the external Count7 counter
//	output  RstPk,     // Reset the peak detectors on all the channels
//	input   ChOR,      // OR of all 5 comparator outputs
//	input   CLK,       // System clock (24 MHz)
//	input   GO,        // AESOP-Lite trigger GO signal
//	input   RST,       // Logic reset
//	input   TC         // Termination Count from the Count7 external counter

reg ConvStb2, Done2, RstCt2;
assign CONVSTB = ConvStb2;
assign DONE = Done2;
assign RstCnt = RstCt2;

parameter [2:0] Wait = 3'b000;  // Wait for a signal from the OR of all 5 comparators
parameter [2:0] Dlay = 3'b001;  // Delay to look for the GO and for the peak detector signal to stabilize
parameter [2:0] Soc0 = 3'b011;  // Send start of conversion signal
parameter [2:0] Eoc0 = 3'b010;  // Wait for all ADCs to finish their conversion
parameter [2:0] Read = 3'b110;  // Signal the CPU to start reading the ADCs
parameter [2:0] Down = 3'b100;  // Wait for the trigger OR to go back to zero
parameter [2:0] Fini = 3'b101;  // Send out the signal to reset peak detectors, holding it long enough for full reset

reg [2:0] State, NextState;
reg GOlatch;

assign RstPk = (State == Fini);
assign GOen = (State == Wait || State == Dlay);
assign GO1en = ~GOlatch;

// Note: the external Count7 time must be longer than the conversion time.
//       It also determines the time waiting for the peak detector to settle 
//       and the time needed to fully reset the peak detectors.
always @ (State or GOlatch or ChOR or TC) begin
    case (State) 
      Wait: begin
                if (ChOR) NextState = Dlay;  // Wait for a PMT signal to cross threshold
                else NextState = Wait;       // or for a tracker trigger
                ConvStb2 = 1'b1;
                RstCt2 = 1'b1;
                Done2 = 1'b0;
            end
      Dlay: begin
                if (TC) begin        // Hold here while the peak detector stabilizes
                    if (GOlatch) NextState = Soc0;  // and while waiting for the GO
                    else NextState = Down;
                end else NextState = Dlay;
                ConvStb2 = 1'b1;
                RstCt2 = 1'b0;       // Release of the reset allows the timer to count down
                Done2 = 1'b0;
            end
      Soc0: begin
                NextState = Eoc0;
                ConvStb2 = 1'b0;     // This starts the ADC conversion
                RstCt2 = 1'b1;
                Done2 = 1'b0;
            end
      Eoc0: begin
                if (TC) NextState = Read;  // Hold here long enough for the ADC conversion to finish
                else NextState = Eoc0;
                ConvStb2 = 1'b1;
                RstCt2 = 1'b0;       // Release of the reset allows the timer to count down
                Done2 = 1'b0;
            end
      Read: begin
                NextState = Down;
                ConvStb2 = 1'b1;
                RstCt2 = 1'b1;
                Done2 = 1'b1;        // The DONE signal is checked by the CPU during readout;
            end
      Down: begin
                if (!ChOR) NextState = Fini;   // Wait here if the Channel-OR is still high
                else NextState = Down;
                ConvStb2 = 1'b1;
                RstCt2 = 1'b1;
                Done2 = 1'b0;
            end
      Fini: begin
                if (TC) NextState = Wait;  // Hold here while the peak detectors are resetting
                else NextState = Fini;
                ConvStb2 = 1'b1;
                RstCt2 = 1'b0;       // Release of the reset allows the timer to count down
                Done2 = 1'b0;
            end
      default: begin
                 NextState = Wait;
                 ConvStb2 = 1'b1;
                 RstCt2 = 1'b1;
                 Done2 = 1'b0;
               end
    endcase
end

always @ (posedge CLK) begin
    if (RST) begin
        State <= Wait;
    end else begin
        State <= NextState;
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

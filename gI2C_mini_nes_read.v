//  ------------------------------------------------------------------------------
//
//  gI2C_mini_nes_read.v -- I2C comm with Nintendo Class Mini controller
//
//  Copyright (C) 2020 Michael Gansler
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
//  ------------------------------------------------------------------------------
//
//  Module:       gI2C_mini_nes_read.v
//
//  Objective:    Performs basic I2C Master operations to communicate with
//                a Nintendo Class Mini controller and read all buttons.
//
//
//  Assumptions:  - 40 MHz input clock
//                - Development done on Terasic DE10-Lite board with 
//                  Altera MAX10 FPGA
//                - No clock domain crossings accomodated.  If this module 
//                  were used in an actual situation with clock domain 
//                  crossings or asynchronous inputs, more attention 
//                  (i.e. code changes) would be necessary to manage the 
//                  situation.
//  Notes:
//
//  This module communicates as a Master over an I2C bus and decodes 
//  button press status from a NES Classi Mini retro controller.  This is 
//  the sort of controller that looks like an original NES controller
//  (up/down/left/right/select/start/A/B), but has a completely different
//  interface than the original.  This updated retro controller has a Wii
//  style connector and an I2C bus.
//
//  The high-level module in this file coordinates the sequences of starts,
//  stops, transmits and reads for a full transaction with the controller.  
//  This obtains the status of all of the buttons: up, down, left, right,
//  select, start, A, and B.  Since this controller is a rendition of the 
//  original NES controller, this is the full set of buttons on the device.
//
//  This module also instantiates a lower level I2c module which handles 
//  the bit by bit signaling of each I2C operation.  The sequence of these
//  operations is coordinated by this upper level module, sending a
//  number of commands to the lower level driver for each operation, 
//  resulting in one full transaction with the device.
//
//  Once transaction consists of the following operations:
//
//   - generate start condition
//   - transmit address to slave, write mode
//   - write 0x00 to slave
//   - generate stop condition
//   - generate start condition
//   - transmit address to slave, read mode
//   - read 6 bytes from slave (the last two bytes contain the button data)
//   - generate a stop condition
//
//  This complete transaction requires less than 1.5 msec, allowing high
//  polling rates of the buttons, if needed.
//
//  The input clock used for testing and development was 40 MHz since this 
//  module was intended for a VGA project that has a 40 MHz pixel clock.  
//
//  Parameterization was done to allow other clock frequencies.  Comments are
//  included describing areas to be mindful of when changing the clock. 
//  The main concern is the counter in the lower-level module (ctr) that 
//  regulates SCL bit timing.  Make sure it has sufficient width if a 
//  faster input clock is used.
//
`default_nettype none                    // Require all nets to be declared before used.
                                         // --> typo'd net names get trapped

module gI2C_mini_nes_read
#( 
   parameter SLAVE_ADDR = 7'h52,         // This is the I2C address of the NES mini controller
   
   parameter TICKS_PER_I2C_CLK = 400     // For 40 MHz clock -> 400.   
                                         // 10 is useful for simulation.
                                         // Note: standard I2C is 100 KHz which means 10 usec/bit.  
                                         // Therefore, if input clk is 40 MHz, then:
                                         // TICKS_PER_I2C_CLK = 40MHz*10usec = 400 counts.
)
(
   //
   // INPUTS 
   //
   input  wire          clk_40,          // Input clock.  Tested with 40 MHz clock
   
   input  wire          rst,             // Active high reset line.
   
   input  wire          request_data,    // Pulse this high to initiate full data transfer sequence
                                         // which includes all Starts, Stops, Master Writes, and Master Reads
                                         // to obtain data from the controller.
   
   //
   // OUTPUTS
   //
   output reg           data_valid,      // Pulse that indicates controller_data was updated with new data.
   
   output reg           busy,            // Module is busy, i.e. transaction in progress

   output reg           btn_none,        // 
   output reg           btn_up,          //
   output reg           btn_down,        //
   output reg           btn_left,        //   Button states decoded from the
   output reg           btn_right,       //   NES mini controller I2C data (bytes 4 and 5).
   output reg           btn_select,      //
   output reg           btn_start,       //
   output reg           btn_B,           //            
   output reg           btn_A,           // 
   
   output reg  [8:0]    buttons,         // Same button state info as above, but as a packed word
   
   //
   // BIDIRECTIONAL
   //
   inout wire           io_sda,          // SDA line to outside world (implements a tristate driver)
   
   inout wire           io_scl           // SCL line to outside world (implements a tristate driver)
);


//
// Legal values for main FSM (finite state machine) state variable.
//
localparam IDLE             = 4'b0000;  // 0
localparam TX_START_COND_1  = 4'b0001;  // 1
localparam TX_ADDR_WR       = 4'b0010;  // 2
localparam TX_BYTE          = 4'b0011;  // 3
localparam TX_STOP_COND_1   = 4'b0100;  // 4
localparam TX_START_COND_2  = 4'b0101;  // 5
localparam TX_ADDR_RD       = 4'b0110;  // 6
localparam RX_BYTE          = 4'b0111;  // 7
localparam TX_STOP_COND_2   = 4'b1000;  // 8
localparam DONE             = 4'b1001;  // 9

//
// Legal values for commands sent to low level I2C driver module
//
localparam LL_CMD_NONE        = 3'd0;
localparam LL_CMD_WRITE       = 3'd1;
localparam LL_CMD_READ        = 3'd2;
localparam LL_CMD_START_COND  = 3'd3;
localparam LL_CMD_STOP_COND   = 3'd4;

reg   [7:0]   state           = IDLE;         // State variable for main Finite State Machine

reg   [2:0]   drvr_cmd        = LL_CMD_NONE;  // Command to low level I2C driver module

reg   [7:0]   tx_byte         = 8'd0;   // Byte to transmit over I2C

reg   [4:0]   rx_byte_idx     = 5'd0;   // Index of byte being received (transaction RX's 6 bytes)

reg           ACK             = 1'b0;   // Indication to low level I2C driver whether 
                                        // the current byte received by the master must be
                                        // followed by an ACK or NACK.  I2C requires that
                                        // the final byte of a master receive transaction
                                        // is NACK'd by the master, though not all slaves
                                        // pay attention to this.
                                        
reg  [15:0]   controller_data;     // Final 2 bytes of the 6 bytes of data retrieved from 
                                   // the controller during each transaction.  Upper 4 bytes 
                                   // are ignored since the NES mini controller reports
                                   // no useful data in them.  See additional comments
                                   // in this file for more info.
   
wire          drvr_busy;           // Low level driver busy indication 
wire  [7:0]   drvr_read_byte;      // Low level driver byte received from slave
wire          drvr_done;           // Low level driver command done
wire          drvr_data_valid;     // Low level driver read byte is valid

wire          drvr_scl;            // Low level driver output to SCL pin
wire          drvr_sda;            // Low level driver output to SDA pin


//
// I2C pins connected to external world are calculated below.  These are 
// driven by drvr_scl and drvr_sda regs above which come from the low level
// driver.  The driver sets them to 1 or 0 indicating high, which is 
// actually implemented as open drain, or low, which is driven low.  
//
// Below those 1/0 values are translated to actually either driving 
// the ouput low, or letting it float high (highZ) which implements 
// the standard I2C open drain physical interface.  
//
// This code essentially infers tristate output drivers.  This works
// correctly as tested with Altera Quartus Prime tools and a MAX10 
// family FPGA.  Using the RTL Viewer in that toolset makes it 
// easy to verify how the synthesis tools interpreted this.
//
assign io_scl = drvr_scl ? 1'bZ : 1'b0;
assign io_sda = drvr_sda ? 1'bZ : 1'b0;

//
// Instantiate lower level driver module that performs any of the 
// following actions:
//     
// 1.  generate a start condition
// 2.  generate astart condition
// 3.  transmit a byte to slave (can be address or data)
// 4.  read a byte of data from slave
//
// Standard I2C is 100 KHz (10 usec per bit).   Therefore I2C 
// SCL clock line is low for 5 usec then high for 5 usec for each bit.  
//
// The parameter TICKS_PER_I2C_CLK_PERIOD is used to regulate SCL bit
// timing and must be set properly for your input clock frequency.
//
// For my case, the input clock is 40 MHz, so the correct value is:
//
//    10usec * 40MHz = 400 counts
//
gI2C_low_level_tx_rx
#(
   .TICKS_PER_I2C_CLK_PERIOD(TICKS_PER_I2C_CLK)
)
gI2C_low_level_tx_rx_inst0
(
   // INPUTS
   
   .clk        (clk_40),
   .rst        (rst),
   .command    (drvr_cmd),
   .tx_byte    (tx_byte),
   .ACK        (ACK),
   
   // OUTPUTS
   
   .read_byte  (drvr_read_byte),
   .busy       (drvr_busy),
   .data_valid (drvr_data_valid),
   .done       (drvr_done),

    // I2C line I/O
    
   .i_sda      (io_sda),
   .o_sda      (drvr_sda),
   .o_scl      (drvr_scl)
);


//
// Main state machine that sequences through an entire transaction
// with the slave (controller).  
//
// This state machine defines the sequence of operations for the
// full transaction, while the lower-level state machine driver module 
// implements the bit by bit physical level details of each operation.
//
// A full transaction sequence consists of:
//          
//    1.  start condition
//    2.  TX slave address (write mode)
//    3.  TX byte 0x00
//    4.  stop condition
//    5.  start condition
//    6.  TX slave address (read mode)
//    7.  RX 6 bytes from slave
//    8.  stop condition 
//
// I considered using a Repeated Start Condition to replace
// steps 4 and 5 above, but I wasn't sure if the slave accepted 
// a Repeated Start Condition, so I used the simpler approach 
// of the 8 steps above.
//
always @(posedge clk_40) begin

   if (rst) begin

      state <= IDLE;
   
   end else begin

      case (state)
      
      //
      // IDLE, waiting for a request to retrieve data from the controller
      //
      IDLE: begin
      
         drvr_cmd   <= LL_CMD_NONE;         // Clear cmd trigger sent to driver.
         data_valid <= 0;                   // Indicate to user that no new data available.
         busy       <= 0;                   // Indicate not busy while in IDLE state

         if (request_data) begin            // Wait for user to initiate transaction.
            busy     <= 1'b1;               // Indicate busy since triggered
            drvr_cmd <= LL_CMD_START_COND;  // Send appropriate trigger to driver.
            state    <= TX_START_COND_1;    // Move on to next state
         end
         
      end
      
      //
      // START condition
      //
      TX_START_COND_1: begin
      
         drvr_cmd <= LL_CMD_NONE;            // Clear cmd trigger to driver
         
         if (drvr_done) begin                // Wait for driver to complete action
            drvr_cmd <= LL_CMD_WRITE;        // Send new trigger to driver
            tx_byte  <= {SLAVE_ADDR, 1'b0};  // Provide driver with byte to transmit - slave address, R/*W=0
            state    <= TX_ADDR_WR;          // Move on to next state
         end
         
      end
      
      //
      // Transmit SLAVE ADDRESS (write mode)
      //
      TX_ADDR_WR: begin
      
         drvr_cmd <= LL_CMD_NONE;        // Clear cmd trigger to driver

         if (drvr_done) begin            // Wait for driver to complete action
            drvr_cmd <= LL_CMD_WRITE;    // Send new trigger to driver
            tx_byte  <= 8'h00;           // Provide driver with byte to transmit - 0x00
            state   <= TX_BYTE;          // Move on to next state
         end
         
      end
      
      //
      // Transmit a byte, master to slave
      //
      TX_BYTE: begin
      
         drvr_cmd <= LL_CMD_NONE;           // Clear cmd trigger to driver
         
         if (drvr_done) begin               // Wait for driver to complete action
            drvr_cmd <= LL_CMD_STOP_COND;   // Send new trigger to driver
            state   <= TX_STOP_COND_1;      // Move on to next state
         end
         
      end
      
      //
      // Transmit STOP condition
      //
      TX_STOP_COND_1: begin
      
         drvr_cmd <= LL_CMD_NONE;           // Clear cmd trigger to driver
         
         if (drvr_done) begin               // Wait for driver to complete action
            drvr_cmd <= LL_CMD_START_COND;  // Send new trigger to driver
            state    <= TX_START_COND_2;    // Move on to next state
         end
         
      end

      //
      // Transmit 2nd START condition
      //
      TX_START_COND_2: begin
      
         drvr_cmd <= LL_CMD_NONE;            // Clear cmd trigger to driver
         
         if (drvr_done) begin                // Wait for driver to complete action
            drvr_cmd <= LL_CMD_WRITE;        // Send new trigger to driver
            tx_byte  <= {SLAVE_ADDR, 1'b1};  // Provide driver with byte to transmit - slave address, R/*W=1
            state    <= TX_ADDR_RD;          // Move on to next state
         end
         
      end

      //
      // Transmit SLAVE ADDRESS (read mode)
      //
      TX_ADDR_RD: begin
      
         drvr_cmd <= LL_CMD_NONE;           // Clear cmd trigger to driver
         
         if (drvr_done) begin               // Wait for driver to complete action
            drvr_cmd    <= LL_CMD_READ;     // Send new trigger to driver
            ACK         <= 1'b1;            // Received bytes must be ACK'd by master
            rx_byte_idx <= 0;               // Init index for byte rx'd from slave
            state       <= RX_BYTE;         // Move on to next state
         end
         
      end
      
      //
      // Read 6 bytes, slave to master
      //
      RX_BYTE: begin
      
         if (drvr_data_valid) begin         // Driver has received a full byte from slave,
                                            // so store it, if relevant.  Bytes 0-3 are not.
            if (rx_byte_idx==4) begin
            
               controller_data[15:8] <= drvr_read_byte;    // Save byte 4
               ACK<=1'b0;                                  // NACK the after byte 5
               
            end else if (rx_byte_idx==5) begin
            
               controller_data[ 7:0] <= drvr_read_byte;    // Save byte 5
               
            end
            
            rx_byte_idx <= rx_byte_idx + 1;

         end

         if (drvr_done) begin                   
         
            if (rx_byte_idx==6) begin
            
               drvr_cmd <= LL_CMD_STOP_COND;   // Driver has completed reception
               state    <= TX_STOP_COND_2;     // of final byte, so move on.
               
            end else begin
                                               // Driver has completed reception of a
               drvr_cmd <= LL_CMD_READ;        // byte, but more bytes to receive, so
                                               // trigger another byte to read.
            end                                
                                               
         end else begin                        
                                               // Driver is not done, so clear trigger
            drvr_cmd <= LL_CMD_NONE;           // condition and wait.
                                               
         end                                   

      end

      //
      // Transmit 2nd STOP condition
      //
      TX_STOP_COND_2: begin
      
         drvr_cmd <= LL_CMD_NONE;    // Clear cmd trigger to driver

         if (drvr_done) begin        // Wait for driver to complete action then
            state <= DONE;           // move on to next state
         end
         
      end

      //
      // Done, so cleanup.
      //
      DONE: begin
      
         drvr_cmd   <= LL_CMD_NONE;  // Clear cmd trigger to driver

         data_valid <= 1;            // Indicate full transaction complete to higher level
         state      <= IDLE;         // Return to idle state.
         
      end
      
      //
      // Default case.  Only should get here if something went wrong.
      //
      default: begin
      
         drvr_cmd <= LL_CMD_NONE;    // Clear cmd trigger to driver
         state    <= IDLE;           // Return to idle state.
         
      end
   
      endcase

   end  // end of else-reset
   
end  // end of always



//
// Decode I2C messages received from controller and report 
// the state of each button state both as individual flags 
// and as a bit-packed word.
//
// Testing showed the folloWing data on the 6 bytes received from the
// controller:
//
//  none		[0]=xA1  [1]=x21  [2]=x10  [3]=x0  [4]=xFF  [5]=xFF   
//  up		[0]=xA1  [1]=x21  [2]=x10  [3]=x0  [4]=xFF  [5]=xFE --> bit  0
//  down		[0]=xA1  [1]=x21  [2]=x10  [3]=x0  [4]=xBF  [5]=xFF --> bit 14
//  left		[0]=xA1  [1]=x21  [2]=x10  [3]=x0  [4]=xFF  [5]=xFD --> bit  1
//  right	[0]=xA1  [1]=x21  [2]=x10  [3]=x0  [4]=x7F  [5]=xFF --> bit 15
//  B			[0]=xA1  [1]=x21  [2]=x10  [3]=x0  [4]=xFF  [5]=xBF --> bit  6
//  A			[0]=xA1  [1]=x21  [2]=x10  [3]=x0  [4]=xFF  [5]=xEF --> bit  4
//  Select	[0]=xA1  [1]=x21  [2]=x10  [3]=x0  [4]=xEF  [5]=xFF --> bit 12
//  Start	[0]=xA1  [1]=x21  [2]=x10  [3]=x0  [4]=xFB  [5]=xFF --> bit 10
//
always @(posedge clk_40) begin

   if (data_valid) begin
   
      btn_none   <= &controller_data[15:0] ? 1'b1 : 1'b0;  // Reduction '&' to test for all bits high,
                                                           // i.e. no buttons pressed.
                                                           
      btn_up     <= ~controller_data[ 0] ? 1'b1 : 1'b0;  // 
      btn_down   <= ~controller_data[14] ? 1'b1 : 1'b0;  // 
      btn_left   <= ~controller_data[ 1] ? 1'b1 : 1'b0;  //   Test relevant bits for low since
      btn_right  <= ~controller_data[15] ? 1'b1 : 1'b0;  //   the NES mini controller clears
      btn_select <= ~controller_data[12] ? 1'b1 : 1'b0;  //   bits when pressed. 
      btn_start  <= ~controller_data[10] ? 1'b1 : 1'b0;  // 
      btn_B      <= ~controller_data[ 6] ? 1'b1 : 1'b0;  //
      btn_A      <= ~controller_data[ 4] ? 1'b1 : 1'b0;  // 

      buttons <= { btn_none,  btn_up,     btn_down,  btn_left, 
                   btn_right, btn_select, btn_start, btn_B, 
                   btn_A };
   end
   
end

endmodule


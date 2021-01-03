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
`timescale 1 ns/10 ps

module gI2C_mini_nes_read_tb ( );

   parameter clk_Tb2 = 10;  // clk half period

   reg   clk;
   reg   rst;
   
   reg   request_data;
   wire  data_valid;
   wire  busy;
   
   wire [8:0] buttons;

   wire  scl;
   wire  sda;

   //
   // Instantiate DUT
   //   
   gI2C_mini_nes_read
   #( .SLAVE_ADDR         ( 7'h52 ),
      .TICKS_PER_I2C_CLK  ( 10 )
   )
   DUT
   (
      .clk_40        ( clk ),
      .rst           ( rst ),
      
      .request_data  ( request_data ),
      .data_valid    ( data_valid ),
      .busy          ( busy ),
      
      .btn_none      ( buttons[8] ),
      .btn_up        ( buttons[7] ),
      .btn_down      ( buttons[6] ) ,
      .btn_left      ( buttons[5] ),
      .btn_right     ( buttons[4] ) ,
      .btn_B         ( buttons[3] ),
      .btn_A         ( buttons[2] ),
      .btn_select    ( buttons[1] ),
      .btn_start     ( buttons[0] ),
      
      .io_sda        ( sda ),
      .io_scl        ( scl )
   );

   
   //
   // Setup, then request on full data transaction.
   //
   initial begin
   
      clk          = 0;
      rst          = 1;
      request_data = 0;
      
      #40 rst = 0;
      
      #40 request_data = 1;
      #40 request_data = 0;
      
      #200;
      
   end
   
   //
   // Generate main clock
   //
   always begin
      #(clk_Tb2) clk  = ~clk;
   end
   

endmodule

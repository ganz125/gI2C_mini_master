# gI2C_mini_master
 FPGA implementation of I2C comm with a Nintendo Classic Mini retro controller in Verilog.

This FPGA project communicates with a Nintendo Classic Mini Controller by implementing an I2C master device in Verilog.

https://www.youtube.com/watch?v=zHz0d-TQWGw

The controller I used this project looks like the original NES controller (up/down/left/right, select, start, A, and B only) but has a Wii-style connector. This controller is fundamentally different than the similar-looking USB controllers. NES Classic Mini retro controllers with the Wii connector communicate over an I2C interface, which is necessary for this project.

<p align="center">
   Here’s a pic of the controller I purchased for this project:
   <br> <br>
   <img src="images/gI2C old school controller.jpg" height="300" align="center">
</p>

<p align="center">
   Example of the I2C traffic:
   <br> <br>
   <img src="images/gI2C logic analyzer - A button.png" height="300" align="center">
</p>

I developed this on a Terasic DE10-Lite board using the Quartus Prime 20.1, Light Edition toolchain. Nothing about the Verilog code is very specific to this board or its Altera MAX10 FPGA, so it should be easy to adapt to other boards. The source code is thoroughly commented and is the best place to understand the assumptions and limitations. Additional Caveats are also in a separate section below.

### Caveats:

- does not check for ACK/NACKs from the slave device – if you want error checking you will have to add it!!
- does not support clock stretching
- I2C depends on open-drain connections for the SCL and SDA lines. To achieve this, the Verilog code infers tristate drivers on the toolchain and board mentioned above. If there are issues with other tools or FPGAs, verifying the driver for these lines may be necessary
- if this code is used in a scenario where clock domain crossing is done, additional care naturally will be needed to manage that
- the limitations above were not an issue in this NES controller application. See the source code for more details.

Check out the comments in the source code for lots more detail.

Also more info at my website: http://ganslermike.com/?page_id=1615

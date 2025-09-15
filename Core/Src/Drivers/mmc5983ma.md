

About MMC5983MA:
    - max output data rate is 1Khz
    - 3 axis magnetometer
    - Using SPI, I2C is too slow (maybe not in this case idk, dependent on IMU but shouldn't be using I2C anyways)
    - Data Ready Interrupt
    - Can be directly connected to microprocessor no need for A/D converter or timing resources

Specifications:
    - Refer to pg 2

SPI R/W Protocol:
    - Refer to pg 5
    - CS goes low before we transmit and high after we are done transmitting
    - SCK, serial port clock stops when CS is high (no transmission)
    - SDI and SDO are both data I/O, they are driven at falling edge and should be captured at rising edge of SCK
    - bit 0 is the RW bit, 0 = W, 1 = R
    - bit 1, don't care
    - bit 2-7, address of the indexed reg
    - bit 8-15, for DI that is the data written into the device with MSB first, for DO that is the data read from device with MSB first

SPI 3-Wire Mode:
    - Refer to pg 8
    - can't be full duplex so probably wont be using this
    - 3-wires SPI read mode, by write “1” to Internal Control Register 3 (0CH), bit 6 before reading command.

Absolute Max Ratings:
    - Refer to pg 9

Connection Pin Description:
    - Refer to pg 11

Hardware Design Considerations (PCB Design):
    - Refer to pg 12

Registers:
    - Register Map:
        - Refer to pg 13

    - Register Details:
        - Refer to pg 13-16

    - Data read:
        - It's 16/18 bit res, bit 0 and 1 are 0
        - But instead of using 3 reg, we use two and XYZ holds the lowest two bits
        - So we read x0 x1 and xyzOut2 so x0[17:10] + x1[9:2] + xyzOut2[1:0]

    - Tout:
        - Temperature sensor, if we want to be fancy we can use this to compensate magnetometer error

    - Internal Control 0
        - TM_M, when set to 1 will initiate measurement and bit will be automatically set to 0 at the end of each measurement
        - INT_meas_done_en, if written 1, then it will enable int for completed measurements
        - Set, to clear any magnetic interference
        - Reset, reset magnetometer
        - Auto Reset/Set, automatically do reset and set operations

    - Internal Control 1
        - SW_RST, cause the sensor to reset, similar to power-up, clear all reg and re-read OTP as startup routine
        - BW0 & BW1 to set output resolution, make it same as IMU
        - X-inhibit and Y/Z-inhibit are not used as we need 3DOF

    - Internal Control 2
        - CM_FREQ[2:0], determines how often the chip will take measurements in continuous mode, if you want 200hz then BW = 01
        - Cmm_en, write 1 to enable continuous mode
        - Prd_set will determine how often the chip will do set operation
        - En_prd_set, writing 1 will enable feature of periodic set, needs to work with Auto_SR_en and Cmm_en both set to 1

    - Initialization:
        1. Reset the magnetometer so we will do Reset operation (SW_RST)
        2. Set the config registers
                - INT_meas_done_en
                - Auto_SR_en
                - BW[1:0] = 01, can be changed later
                - CM_Freq[2:0] = 110 (200hz)
                - Cmm_en
                - Prd_set[2:0] = 001 (time of measurement = 25ms)

    - Read Magnetometer
        1. Perform Set Operation
        2. Set TM_M to 1
        3. Interrupt will tell me if data is ready
        4. Check Meas_M_Done to see if it is 1
        5. Write 1 to Meas_M_Done to clear the interrupt
        6. Perform Measurement
        7. Perform Reset Operation
        8. Set TM_M to 1
        9. Interrupt will tell me if data is ready
        10. Check Meas_M_Done to see if it is 1
        11. Write 1 to Meas_M_Done to clear the interrupt
        12. Perform Measurement
        13. Subtract the two measurements (Out1 - Out2) and divide by 2

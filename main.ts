/**
 * Blocks for driving the I2C 16-Servo Driver Board (PCA9685) with configurable pulse widths
 * This extension provides an additional block that allows the minimum and maximum pulse width of the servo signal to be set 
 * The original extension had the minimum pulse width at 550us and the maximum pulse width at 2700uS. These settings cause some servo 
 * motors to growl and over heat when positioned at 0 or 180 degrees. This situation will cause servo motors to fail.  
 * The extra block will allow each of the 16 servo outputs to be individually configured to one of the following six pulse ranges: 
 * 1mS - 2mS (so called industry default standard), 0.9mS - 2.1mS, 0.8mS - 2.2mS, 0.7mS - 2.3mS, 0.6mS - 2.4mS and 0.5mS - 2.5mS. 
 * The PWM frequency is set to 50Hz making each bit of the PCA9685 4096 count equal to 4.88uS
 * 
 * If your multi servo motor projects suffer from growling servo motors hard up against there internal stops and overheating
 * to the point of failure. Growling and squealing servo motors can also cause all sorts of power supply issues ranging from overheating power regulators,
 * brownouts and unexplained resets. If have any of these problems then this extension may be the answer to your problems.
 */

namespace limits {

// PCA9685 address definitions. 
    const CHIP_ADDRESS: number = 0x6A;              // Default Chip address
    const REG_MODE1: number = 0x00;                 // Mode 1 register address 
    const REG_MODE2: number = 0x01;                 // Mode 2 register address 
    const REG_SUB_ADR1: number = 0x02;              // Sub address register 1 address
    const REG_SUB_ADR2: number = 0x03;              // Sub address register 2 address
    const REG_SUB_ADR3: number = 0x04;              // Sub address register 3 address
    const REG_ALL_CALL: number = 0x05;              // All call address register
    const REG_SERVO1_BASE: number = 0x06;           // Servo 1 base address 
    const REG_SERVO_DISTANCE: number = 4;           // Four registers per servo 
    const REG_ALL_LED_ON_L: number = 0xFA;          // All LED on low register address
    const REG_ALL_LED_ON_H: number = 0xFB;          // All LED on high register address
    const REG_ALL_LED_OFF_L: number = 0xFC;         // All LED off low register address
    const REG_ALL_LED_OFF_H: number = 0xFD;         // All LED off high register address 
    const REG_PRE_SCALE: number = 0xFE;             // Pre-scaler register address

    const PWM_FREQUENCY: number = 0x79;             // Pre-scaler value for 50Hz

    let PCA9685_init: boolean = false;              // Flag to allow us to initialise without explicitly calling the initialisation function 

    // List of possible 16 servo motors 
    export enum Servos {
        Servo1 = 1,
        Servo2 = 2,
        Servo3 = 3,
        Servo4 = 4,
        Servo5 = 5,
        Servo6 = 6,
        Servo7 = 7,
        Servo8 = 8,
        Servo9 = 9,
        Servo10 = 10,
        Servo11 = 11,
        Servo12 = 12,
        Servo13 = 13,
        Servo14 = 14,
        Servo15 = 15,
        Servo16 = 16,
    }

    // 
	export enum BoardAddresses{
		Board1 = 0x6A,
	}

    // List of possible output pulse ranges
    export enum PulseRange {
        R500_2500uS = 1,
        R600_2400uS = 2,
        R700_2300uS = 3,
        R800_2200uS = 4,
        R900_2100uS = 5,
        R1000_2000uS = 6,
    }

    // Time             0.5  0.6  0.7  0.8  0.9  1.0 mS
    const loPulseLim = [102, 123, 143, 164, 184, 204];  // Lower pulse limit width in multiples of 4.88uS

    // Time             2.5  2.4  2.3  2.2  2.1  2.0 mS 
    const hiPulseLim = [512, 500, 471, 451, 430, 409];  // Higher pulse limit width in multiples of 4.88uS

    // Time        2.0  1.8  1.6  1.4  1.2  1.0 mS 
    const range = [410, 377, 328, 287, 246, 205];  // Pulse width range in multiples of 4.88uS

    // Servo number   1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16
    let servoRange = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]; // Individual servo pulse range, default = R500 - 2500uS 

    // Function to read i2c register - for testing purposes
    function readReg(addr: number, reg: number): number {       // Read 8 bit little-endian unsigned integer
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8LE);
        return pins.i2cReadNumber(addr, NumberFormat.UInt8LE);
    }

    // Function to map o value from one range into another range 
    function map(value: number, fromLow: number, fromHigh: number, toLow: number, toHigh: number): number {
        return ((value - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow) + toLow;
    }

	/*
	* This initialisation function sets up the PCA9865 servo driver chip. 
    * The PCA9685 comes out of reset in low power mode with the internal oscillator off with no output signals, this allows writes to the pre-scaler register.
    * The pre-scaler register is set to 50Hz producing a refresh rate or frame period of 20mS which inturn makes each bit of the 4096 count equal to 4.88uS.
    * Sets the 16 LED ON registers to 0x00 which starts the high output pulse start at the beginning of each 20mS frame period.
    * Sets the 16 LED OFF registers to 0x133 (4.88uS x 1500) which ends the high output pulse 1.5mS into the frame period. This places all servo motors at 90 degrees or centre travel.
    * It is these LED OFF registers that will be modified to set the pulse high end time to vary the pulse width and the position of the attached servo motor. 
    * Sets the mode1 register to 0x01 to disable restart, use internal clock, disable register auto increment, select normal (run) mode, disable sub addresses and allow LED all call addresses.
    * Finally the initialised flag will be set true.
	* This function should not be called directly by a user, the first servo write will call it.
    * This function initialises all 16 LED ON and LED OFF registers by using a single block write to the 'all LED' addresses.
	*/
	function init(): void {
        let buf = pins.createBuffer(2)                      // Create a buffer for i2c bus data
        buf[0] = REG_PRE_SCALE;                             // Point at pre-scaler register
        buf[1] = PWM_FREQUENCY;                             // Set PWM frequency to 50Hz or repetition rate of 20mS
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);      // Write to PCA9685 
        buf[0] = REG_ALL_LED_ON_L;                          // Point at ALL LED ON low byte register 
        buf[1] = 0x00;                                      // Start high pulse at 0 (0-0x199) 
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);      // Write to PCA9685
        buf[0] = REG_ALL_LED_ON_H;                          //  
        buf[1] = 0x00;                                      // Start each frame with pulse high
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);      // Write to PCA9685
        buf[0] = REG_ALL_LED_OFF_L;                         //
        buf[1] = 0x33;                                      // End high pulse at mid range 1.5mS = 1500/4.88uS = 307 (0x133)
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);      // Write to PCA9685
        buf[0] = REG_ALL_LED_OFF_H;                         //
        buf[1] = 0x01;                                      // End high pulse at mid range 1.5mS = 1500/4.88uS = 307 (0x133)
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);      // Write to PCA9685
        buf[0] = REG_MODE1;                                 //
        buf[1] = 0x01;                                      // Normal mode, start oscillator and allow LED all call registers
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false)       // Write to PCA9685
        basic.pause(10);                                    // Let oscillator start and settle 
        PCA9685_init = true;                                // The PCA9685 is now initialised, no need to do it again
    }
	
    /**
     * Sets the requested servo to the reguested angle.
	 * If the PCA9685 has not yet been initialised calls the initialisation routine
	 *
     * @param Servo Which servo to set
	 * @param degrees the angle to set the servo to
     */
    //% blockId=I2C_servo_write
    //% block="set%Servo|to%degrees"
	//% degrees.min=0 degrees.max=180
	
    export function servoWrite(Servo: Servos, degrees: number): void {
        if (PCA9685_init == false) {                                        // PCA9685 initialised?
            init();                                                         // No, then initialise it 
        }
        let range: number = servoRange[Servo - 1];                          // Get configured pulse range for specified servo
        let lolim: number = loPulseLim[range - 1];                          // Get lower pulse limit for the pulse range
        let hilim: number = hiPulseLim[range - 1];                          // Get upper pulse limit for the pulse range 
        let pulse: number = map(degrees, 0, 180, lolim, hilim);             // Map degrees to pulse range
        let final: number = pulse + lolim;                                  // Pulse range starts at lolim actual pulse starts at zero 

        let buf = pins.createBuffer(2);                                     // Create a buffer for i2c bus data 
        buf[0] = REG_SERVO1_BASE + (REG_SERVO_DISTANCE * Servo - 1) + 2;    // Calculate address of LED OFF low byte register
        buf[1] = final % 256;                                               // Calculate low byte value 
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);                      // Write low byte to PCA9685 
        buf[0] = REG_SERVO1_BASE + (REG_SERVO_DISTANCE * Servo - 1) + 3;    // Calculate address of LED OFF high byte register
        buf[1] = Math.floor(final / 256);                                   // Calculate high byte value
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);                      // Write high byte to PCA9685
    }

    /**
     * Sets the specified servo to the specified pulse range.
	 * On startup all 16 servos are set to the default pulse range of 0.5mS to 2.5mS
	 * This block is used to set the pulse range to a specific range, other than the default
     * 
     * @param Servo Which servo to alter the pulse range.
	 * @param Range The new pulse range for the servo.
     */
    //% blockId=set_pulse_range
    //% block="set%Servo|pulse range%PulseRange"
	export  function setRange (Servo: Servos, Range: PulseRange): void {
        servoRange[Servo - 1] = Range;                  // Store new pulse range in servoRange array 
    }
}
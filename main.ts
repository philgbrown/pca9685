/**
 * Blocks for driving the I2C 16-Servo Driver Board (PCA9685) with configurable pulse widths
 * This extension provides an additional block that allows the minimum and maximum pulse width of the servo signal to be set 
 * The original extension had the minimum pulse width at 550us and the maximum pulse width at 2700uS. These settings cause some servo 
 * motors to growl and over heat when positioned at 0 or 180 degrees. This situation will cause servo motors to fail.  
 * The extra block will allow each of the 16 servo outputs to be individually configured to one of the following six pulse ranges: 
 * 1mS - 2mS (so called industry default standard), 0.9mS - 2.1mS, 0.8mS - 2.2mS, 0.7mS - 2.3mS, 0.6mS - 2.4mS and 0.5mS - 2.5mS. 
 * The PWM frequency is set to 50Hz making each bit of the PCA9685 4096 count equal to 4.88uS
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
    const REG_SERVO1_REG_BASE: number = 0x06;       // Servo 1 base address 

    const REG_ALL_LED_ON_L: number = 0xFA;          // All LED on low register address
    const REG_ALL_LED_ON_H: number = 0xFB;          // All LED on high register address
    const REG_ALL_LED_OFF_L: number = 0xFC;         // All LED off low register address
    const REG_ALL_LED_OFF_H: number = 0xFD;         // All LED off high register address 
    const REG_PRE_SCALE: number = 0xFE;             // Pre-scaler register address

    const PWM_FREQUENCY: number = 30;             // Pre-scaler value for 50Hz

    
// If you wanted to write some code that stepped through the servos then this is the BASe and size to do that 	
//	let Servo1RegBase = 0x08 
//   let ServoRegDistance = 4
	//To get the PWM pulses to the correct size and zero offset these are the default numbers. 
 //   let ServoMultiplier = 226
  //  let ServoZeroOffset = 0x66

    let PCA9685_init: boolean = false;              // Flag to allow us to initialise without explicitly calling the initialisation function 

    //nice big list of servos for the block to use. These represent register offsets in the PCA9685
    export enum Servos {
        Servo1 = 0x08,
        Servo2 = 0x0C,
        Servo3 = 0x10,
        Servo4 = 0x14,
        Servo5 = 0x18,
        Servo6 = 0x1C,
        Servo7 = 0x20,
        Servo8 = 0x24,
        Servo9 = 0x28,
        Servo10 = 0x2C,
        Servo11 = 0x30,
        Servo12 = 0x34,
        Servo13 = 0x38,
        Servo14 = 0x3C,
        Servo15 = 0x40,
        Servo16 = 0x44,
    }

	export enum BoardAddresses{
		Board1 = 0x6A,
	}

    export enum PulseRange {
        R500_2500uS = 1,
        R600_2400uS = 2,
        R700_2300uS = 3,
        R800_2200uS = 4,
        R900_2100uS = 5,
        R1000_2000uS = 6,
    }

    //Trim the servo pulses. These are here for advanced users, and not exposed to blocks.
    //It appears that servos I've tested are actually expecting 0.5 - 2.5mS pulses, 
    //not the widely reported 1-2mS 
    //that equates to multiplier of 226, and offset of 0x66
    // a better trim function that does the maths for the end user could be exposed, the basics are here 
	// for reference

    /*
    export function TrimServoMultiplier(Value: number) {
        if (Value < 113) {
            ServoMultiplier = 113
        }
        else {
            if (Value > 226) {
                ServoMultiplier = 226
            }
            else {
                ServoMultiplier = Value
            }
        }
    }
    export function TrimServoZeroOffset(Value: number) {
        if (Value < 0x66) {
            ServoZeroOffset = 0x66
        }
        else {
            if (Value > 0xCC) {
                ServoZeroOffset = 0xCC
            }
            else {
                ServoZeroOffset = Value
            }
        }
    }
    */

    function readReg(addr: number, reg: number): number {       // Read 8 bit big-endian unsigned integer
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8LE);
        return pins.i2cReadNumber(addr, NumberFormat.UInt8LE);
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
        let data = readReg(CHIP_ADDRESS, REG_PRE_SCALE);
        basic.showNumber(data);
        buf[0] = REG_ALL_LED_ON_L;                          // Point at ALL LED ON low byte register 
        buf[1] = 0x00;                                      // Start high pulse at 0 (0-0x199) 
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);      // Write to PCA9685
        buf[0] = REG_ALL_LED_ON_H;                          //  
        buf[1] = 0x00;                                      // Start each frame with pulse high
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);      // Write to PCA9685
        buf[0] = REG_ALL_LED_OFF_L;                         //
        buf[1] = 0x11;                                      // End high pulse at mid range 1.5mS = 1500/4.88uS = 307 (0x133)
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
        //if (PCA9685_init == false) {                        // PCA9685 initialised?
            init();                                         // No, then initialise it 
        //}

        /*
        let buf = pins.createBuffer(2)
        let HighByte = false
        let deg100 = degrees * 100
        let PWMVal100 = deg100 * ServoMultiplier
        let PWMVal = PWMVal100 / 10000
        PWMVal = Math.floor(PWMVal)
        PWMVal = PWMVal + ServoZeroOffset
        if (PWMVal > 0xFF) {
            HighByte = true
        }
        buf[0] = Servo
        buf[1] = PWMVal
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false)
        if (HighByte) {
            buf[0] = Servo + 1
            buf[1] = 0x01
        }
        else {
            buf[0] = Servo + 1
            buf[1] = 0x00
        }
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false)
        */
    }
}
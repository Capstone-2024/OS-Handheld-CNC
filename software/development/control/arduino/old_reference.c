// FINAL CODE //
#define LED_BASE		    0xFF200000
#define HEX0_HEX3_BASE      0xFF200020
#define HEX4_HEX5_BASE      0xFF200030
#define I2C0_BASE		    0xFFC04000
#define TIMER_BASE          0xFF202000
#define UART_BASE           0xFF201000
#define MOTOR0_STEP_BASE    0xFF709000 // assuming motor is connected to the first GPIO pin
#define MOTOR1_STEP_BASE    0xFF709005
#define MOTOR2_STEP_BASE    0xFF70900A 
#define GPIO_BASE           0xFF200060
#define SW_BASE             0xFF200040

void init_I2C( void );
unsigned char read_over_I2C( unsigned char address );
void write_over_I2C( unsigned char address, unsigned char value );
void read_over_I2C_multiple( unsigned char address, unsigned char* values, unsigned char length );
void init_GS( void );
void init_encoder( unsigned char address );

// -------------------------------- //
// STRUCTURES and GLOBAL VARIABLES
// -------------------------------- //

// I2C Struct
typedef struct _I2Cn
{
		int control;			//0x00
		int target; 			//0x04
		int slave;				//0x08
		int pad0; 				//skip
		int data_cmd;			//0x10
		int std_scl_hcnt;		//0x14
		int std_scl_lcnt;		//0x18
		int fast_scl_hcnt;		//0x1C
		int fast_scl_lcnt;		//0x20
		int pad1;				//skip
		int pad2;				//skip
		int intr_status;		//0x2C
		int intr_mask;			//0x30		
		int raw_intr_status;	//0x34
		int rx_fifo_thr;		//0x38
		int tx_fifo_thr;		//0x3C
		int cmb_intr;			//0x40
		int rx_under_intr;		//0x44
		int rx_over_intr;		//0x48
		int tx_over_intr;		//0x4C
		int intr_read;			//0x50
		int tx_abort_intr;		//0x54
		int rx_done_intr;		//0x58
		int activity_intr;		//0x5C
		int stop_dtct_intr;		//0x60
		int start_dtct_intr;	//0x64
		int gen_call_intr;		//0x68
		int enable;				//0x6C
		int status;				//0x70
		int tx_fifo_lvl;		//0x74
		int rx_fifo_lvl;		//0x78
		int sda_hold;			//0x7C
		int tx_abort_src;		//0x80
		int gen_slave_nack;		//0x84
		int dma_control;		//0x88
		int dma_tx_lvl;			//0x8C
		int rx_data_lvl;		//0x90
		int sda_setup;			//0x94
		int ack_gen_call;		//0x98
		int enable_status;		//0x9C
		int ss_fs_supp;			//0xA0
} I2Cn;

// Timer Struct
typedef struct _interval_timer
{
  int status;
  int control;
  int low_period;
  int high_period;
  int low_counter;
  int high_counter;
} interval_timer;

// Make I2C pointer global for easy access
volatile I2Cn* const I2C_ptr = ( I2Cn* )I2C0_BASE;

// JTAG UART
typedef struct _jtag_uart
{
  int data;
  int control;
} jtag_uart;

// -------------------------------- //
// Global Variables
// -------------------------------- //

volatile I2Cn* const I2C_ptr = ( I2Cn* )I2C0_BASE; // I2C pointer
volatile int * const gpio_ptr = (int *)GPIO_BASE; //points to address of GPIO port
volatile int* const LED_ptr = ( int* )LED_BASE; // pointer to LEDs
volatile int* const LED_ptrH = ( int* )LED_HEX; // pointer to HEX Display
volatile jtag_uart* const uart_ptr = ( jtag_uart* )UART_BASE; // pointer for UART

volatile int* const step0_mode_0_ptr = ( int* )(MOTOR0_STEP_BASE + 1);
volatile int* const step0_mode_1_ptr = ( int* )(MOTOR0_STEP_BASE + 2);
volatile int* const step0_mode_2_ptr = ( int* )(MOTOR0_STEP_BASE + 3);
volatile int* const step1_mode_0_ptr = ( int* )(MOTOR0_STEP_BASE + 1);
volatile int* const step1_mode_1_ptr = ( int* )(MOTOR0_STEP_BASE + 2);
volatile int* const step1_mode_2_ptr = ( int* )(MOTOR0_STEP_BASE + 3);
volatile int* const step2_mode_0_ptr = ( int* )(MOTOR0_STEP_BASE + 1);
volatile int* const step2_mode_1_ptr = ( int* )(MOTOR0_STEP_BASE + 2);
volatile int* const step2_mode_2_ptr = ( int* )(MOTOR0_STEP_BASE + 3);

// -------------------------------- //
// MAIN PROGRAM
// -------------------------------- //
int main( void )
{   
    // -------------------------------- //
    // Data Variables
    // -------------------------------- //

    // Accelerometer data
    int x_data;
	int y_data;
	int z_data; 

    // Orientation Limits
    int x_max_acce=10;
	int y_max_acce=10;
	int z_max_acce=10;

    // Timer
	int interval = 10000000; // 1 second interval
	int current_count = 0; // Timer current count
	int last_count = interval;
	float total_count = 0; // Time for each stepper motor move command

    // Encoder and Motor Variables
	int feedrate = 6000; //mm/min or 100 mm/s, assumed but would usually be fed from main computer
	float encoder_angle[3];
	int desired_steps[3];
	float last_angle[3]; 
	float angle_error[3];

    // Motor Configuration - enabling microstepping
	*(int *)step0_mode_2_ptr = 1; // Enabling only mode2 indicates 1/16 microstepping, page 13
    *(int *)step1_mode_2_ptr = 1;
    *(int *)step2_mode_2_ptr = 1;

	// -------------------------------- //
    //         I2C Initializations      //
    // -------------------------------- //
    // For encoder:
    // Low Phase 0.5 us, = (0.5 + (2.5-0.75)/2)*100 = 138
    // High Phase 0.25 us, = (0.25 + (2.5-0.75)/2)*100 = 112
    init_I2C(0x18); // LID3H

    // three addresses using I2C multiplexer
    init_I2C(0x70); // First motor 
    init_I2C(0x71); // Second motor
    init_I2C(0x72); // Third motor

    // Initialize AS5600 Encoders
	init_encoder(0x70); // First motor 
    init_encoder(0x71); // Second motor
    init_encoder(0x72); // Third motor

    float encoder_offset[3];

    int i; 
    for (i = 0; i < 3; i++) {
        encoder_offset[i] = encoderAngle(); // offset using initial angle reading
    }

	int read_uart[3];

    while (1){
        // Read Buttons
        // Initializing the 3 x-y-z plane limit switches 
        int x_switch = ((*(int *)BUTTONS) & 0b1); // x axis limit switch 
        int y_switch = ((*(int *)BUTTONS) & 0b10); // y axis limit switch 
        int z_switch = ((*(int *)BUTTONS) & 0b100); // z axis limit switch
        int all_sw = x_switch && y_switch && z_switch;

        // Read Switchs
        // Initiating switches to define if the machine is on the workpiece
        int b1 = (*(int *)SW_BASE & 0b10); // first button 
        int b2 = (*(int *)SW_BASE) & 0b100; // second button 
        int b3 = (*(int *)SW_BASE) & 0b1000; // third button 
        int all_buttons = b1 && b2 && b3;


        int error = 0; // error

        if (all_buttons) { // only if we are on the workpiece we can continue

            // -------------------------------- //
            //   Accelerometer Data Processing  //
            // -------------------------------- //

            // X Data
            if ( read_over_I2C(0x27) & 0x02 ) {
                x_data= read_over_I2C( 0x28)+read_over_I2C( 0x29)*256; // Reading linear x acceleration
                // lowest 8 bits 
            }

            // Y Data
            if ( read_over_I2C(0x27) & 0x02 ) { 
                y_data= read_over_I2C( 0x2a)+read_over_I2C( 0x2b)*256;// Reading linear y acceleration
                // lowest 8 bits 
            }

            // Z Data
            if ( read_over_I2C(0x27) & 0x04 ) {
                z_data = read_over_I2C( 0x2c )+read_over_I2C( 0x2d )*256;// Reading linear z acceleration
                // lowest 8 bits 
            }

            // Same as demo code --> determine if the board is levelled within a desirable range
            int x_level = (x_data != 0) && x_data < 65000;
            int y_level = y_data > y_max_o;
            int z_level = z_data < z_max_o;

            // If acceleration are below bounds: 
            if (x_data < x_max_acce && y_data < y_max_acce && z_data < z_max_acce) { 
                
                // -------------------------------- //
                //           Timer and UART         //
                // -------------------------------- //

                timer_1->low_counter = 1; // write anything to get current count
                current_count = timer_1->low_counter + (timer_1->high_counter << 16); //combine 2 16-bit word

                // JTAG UART Receive Data
                read_uart = uart_ptr->data;

                if ( read_uart & ( 1 << 15 )) {
                    read_uart &= 0xFF;
                }
                
                // -------------------------------- //
                //  Limit Switches and Motor Moves  //
                // -------------------------------- //

                // Calculate Error between Desired and Measured for all motors
                int i; 
                float angle_moved[3];
                float desired_angle[3];

                // Calculate Time
                // Speed = 6000 mm/min = 100mm/s, with lead screw of 8mm lead, this translates to 100/8 = 12.5 rotations/s
                // which is 4500 deg/s
                int max_speed = 4500; 
	 	        int move_speed[3] = {angle_moved[0]/time_passed, angle_moved[1]/time_passed, angle_moved[2]/time_passed}; // find current speed

                for (i = 0; i < 3; i++) {
                    encoder_angle[i] = encoderAngle(i) - encoder_offset;
                    angle_moved[i] = encoder_angle[i] - last_angle[i];
                    desired_angle[i] = desired_steps[i]*0.1126; // 200 steps per revolution, 1.8 deg per full step, or 0.1125 deg in 1/16 mode
                    angle_error[i] = desired_angle[i] - angle_moved[i]; // Calculate Error
                }
                
                if (x_level || y_level || z_level) // check if all axis are levelled or within tolerable range
				{
					error  = 1; // error if not levelled
				}
				else if (all_sw == 1)
				{
					error = 3;
				}

		        // X-DIR
		        // If limit switch is pressed, then the direction of that axis will stop 
                if (x_switch != 1 && move_speed[0] <= max_speed) { // If speed too fast then don't move
                    last_angle[0] = motorMove(0, angle_error[0]);
                } else { // If limit switch is pressed, then the direction of that axis will stop 
                    error = 4;
                }
                
		        // Y-DIR
                if (y_switch != 1 && move_speed[1] <= max_speed) { // If speed too fast then don't move 
                    last_angle[1] = motorMove(1, angle_error[1]);
                } else { // If limit switch is pressed, then the direction of that axis will stop 
                    error = 5;
                }
                
		        // Z-DIR
                if (z_switch != 1  && move_speed[2] <= max_speed) { // If speed too fast then don't move
                    last_angle[2] = motorMove(2, angle_error[2]);
                } else { 
                    error = 6;
                }

                // Timer: check if 0.1s has passed
                if (current_count <= last_count)   
                {
                    time_passed += 1000000/10000000; // Increment time moved

                    if (time_passed) >= 1 {
                        last_count = interval; // Reset interval counter
                    }
                    last_count -= 1000000;
                }

                // Send Data to Main Computer through UART
                int j; 
                for (j = 0; j < 3; j++) {
                    uart_ptr->data = last_angle[j];
		        }
            }
        
	    } else { // throw error if we have error
            DisplayHex(error, 1); // Error 9
        }
    }

        
}


// -------------------------------- //
//            MISC Methods          //
// -------------------------------- //

void condition(int value) {
    DisplayHex(value, 1); //
    // 0. Normal
    // 1. Acceleration exceeded
    // 2. Other
    // 3. Other
    // 4. X max reached
    // 5. Y max reached
    // 6. Z max reached
    // 7. Other
    // 8. Other
    // 9. Not on workpiece
}

void DisplayHex(int value, int hex)
{
	char *p; // Pointer to HEX
    
    int hex_code[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F}; // 7-segment display hex codes for digits 0-9
    
    if (hex < 4)
	{
		p = (char *)HEX0_HEX3_BASE;
		p += hex;
	}
	else if (hex >= 4)
	{
		p = (char *)HEX4_HEX5_BASE;
		p += (hex - 4);
	}

    *p = hex_code(value + 1); // Write to HEX
}

// -------------------------------- //
//           Motor Methods          //
// -------------------------------- //
float motorMove(int motor_num, float angle_error) {

    // Pointers for Motors
    volatile int* const motor_step_ptr = ( int* )MOTOR0_STEP_BASE;
    volatile int* const step_dir = ( int* )(MOTOR0_STEP_BASE + 4*(motor_num + 1));
    
    if (angle_error != 0) {
        if (angle_error > 0) {
            int i;
            int steps = angle_error/0.1126;
            for ( i = 0;i <= steps; i++) {
                *motor_step_ptr = 1; // continue to move if we have not reached desired position
            }
        } else {
            int i=0;
            int steps = -angle_error/0.1126;
            *step_dir_ptr = 1; // switch direction
            for (;i <= steps; i++) {
                *motor_step_ptr = 1; // move a step
            }
            *step_dir_ptr = 1; // switch back
        }
	}

    return (encoder_angle + angle_error); // return the 
}

// This function outputs angle data read by the encoder through I2C
float encoderAngle(int motor_num) {
	float encoder_data; // encoder data
    int status_reg; 
    int angle_reg; 

    if (motor_num == 2) {
        status_reg = 0x36;
        angle_reg = 0x44;
    } else if (motor_num == 1) {
        status_reg = 0x36;
        angle_reg = 0x44;
    } else {
        status_reg = 0x36;
        angle_reg = 0x44;
    }

	// Read Encoder
	if (read_over_I2C( status_reg ) > 0x80) // change
	{
		encoder_data = read_over_I2C( angle_reg ); // Read the 11th register, which is the ANGLE register, page 18
	}

	return encoder_data;
}


// -------------------------------- //
//            I2C Methods           //
// -------------------------------- //

void init_I2C(unsigned char address)
{
	I2C_ptr->enable = 2; // abort any transmissions and disable device
	while ( I2C_ptr->enable_status & 0x1 ) // wait
		;
	I2C_ptr->control = 0b01100101; // assuming high speed with 7 bits 
	I2C_ptr->target = target;

	// clock periods
	I2C_ptr->fast_scl_hcnt = 90; // high = 0.6 + 0.6/2 = 0.9 * 100 = 90 
	I2C_ptr->fast_scl_lcnt = 160; // low = 1.3 + 0.6/2 = 1.6 * 100 = 160 

	// turn on IC20
	I2C_ptr->enable = 1;
	// wait until this is done
	while (( I2C_ptr->enable_status & 0x1 ) == 0 )
		;	
}

unsigned char read_over_I2C( unsigned char address )
{
	I2C_ptr->data_cmd = address + 0x400; //restart
	I2C_ptr->data_cmd = 0x100; // switches to read mode
	while ( I2C_ptr->rx_fifo_lvl == 0 ) // wait for data
		;	
	return I2C_ptr->data_cmd;
}

void read_over_I2C_multiple( unsigned char address, unsigned char* values, unsigned char length )
{
	int i; // counter var
	I2C_ptr->data_cmd = address + 0x400; // restart
	I2C_ptr->data_cmd = 0x100; //  switches to read mode		

	for ( i = 0; i < length; i++ ) // read all data
	{
		// hold until data received
		while( I2C_ptr->rx_fifo_lvl == 0 )// if its non zero, we read 
			;
		// read into array
		values[i] = I2C_ptr->data_cmd;
	}
}

void write_over_I2C( unsigned char address, unsigned char value )
{
	
	I2C_ptr->data_cmd = address + 0x400; // restart before sending data
	I2C_ptr->data_cmd = value;// transmite data to queue
}

void write_over_I2C_multiple( unsigned char address, unsigned char* values, unsigned char length ) {
	int i;
	//  restart before sending data
	I2C_ptr->data_cmd = address + 0x400;
	for (i = 0; i < length; i++ )
		I2C_ptr->data_cmd = values[i]; // write each byte to transmit queue to send to device
}

void init_GS( void )
{
	// LIS3DH Configs
	write_over_I2C( 0x23, 0x08 );
	write_over_I2C( 0x20, 0x67 );
}

// DATA for Encoder
// Low Phase 0.5 us
// High Phase 0.25 us
void init_encoder(unsigned char address) {

    write_over_I2C( address, 0x00 ); // Normal power mode, page 19
	// Ensure Magnet is detected
	// Read status register
    while (read_over_I2C(address + 0x0B) == 0)
        ;

	//No other configuration needed, all are default
}
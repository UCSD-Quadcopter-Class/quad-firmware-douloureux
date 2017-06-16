#include <Arduino.h>

// Pin definitions for Gimbals (Analog inputs).  It is not clear why the A1, A2,... etc. don't work, but they don't.
#define PIN_YAW			0 //A0
#define PIN_THROTTLE 		1// A1 
#define PIN_ROLL		2// A2 
#define	PIN_PITCH		3//A3  

// Pin definitions for potentiometers (Analog inputs)
#define PIN_POT1		7//A7
#define PIN_POT2		6//A6

// Pin definitions for digital pins (buttons and indicator LEDs)
#define PIN_BTN1		16		// PG0 (schematic) G0 (red board)
#define PIN_BTN2		17		// PG1 (schematic) G1 (red board)
#define PIN_LED_BLUE		22		// PD6 (schematic) D4 (red board)
#define PIN_LED_GRN		23		// PD5 (schematic) D5 (red board)
#define PIN_LED_RED		24		// PD4 (schematic) D6 (red board)		

// The RF CHannel we will be using
#define CHANEL			13


//MAX && MINS of Potentiometers
#define POT_MIN  		118.0
#define POT_MAX  		815.0

#define POT_MINEW1  		0.0
#define POT_MAXEW1 		1.0
#define POT_MINEW2 		0.0
#define POT_MAXEW2 		3.0


#define THROTTLE_MIN 		116.0
#define THROTTLE_MAX 		810.0
#define PITCH_MIN 		144.0
#define PITCH_MAX 		815.0
#define YAW_MIN 		123.0
#define YAW_MAX 		815.0
#define ROLL_MIN 		131.0
#define ROLL_MAX 		815.0

#define THROTTLE_CONSTRAIN 	250.0
#define PITCH_CONSTRAIN 	20.0
#define YAW_CONSTRAIN 		20.0
#define ROLL_CONSTRAIN 		20.0

#define YAW_MIDPOINT		483.0
#define ROLL_MIDPOINT		504.0
#define PITCH_MIDPOINT		521.0



// PID constants
#define YAW_P 0
#define YAW_I 0
#define YAW_D 0

#define PITCH_P 4.87
#define PITCH_I 0
#define PITCH_D 0.43

#define ROLL_P 0
#define ROLL_I 0
#define ROLL_D 0

// Motor pin number
#define PB5 8
#define PE5 5
#define PE3 3
#define PE4 4

//LED ring PIN
#define LEDPIN 19

//LED patterns
#define NUM_OF_LED_PATTERNS 5






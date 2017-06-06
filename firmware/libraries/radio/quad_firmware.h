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

//MAX && MINS of Potentiometers
#define POT_MIN  118
#define POT_MAX  815
#define POT_MINEW1  0.0
#define POT_MAXEW1 200.0
#define POT_MINEW2 0.0
#define POT_MAXEW2 10.0


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








#include <avr/pgmspace.h>
#include <util/delay.h>

// Hardware params
// Each time you change these, Cone_Info and Thread_Info tables below must be re-calculated
#define DELAY                80       // Enabling stepper motors is followed by a short delay, maybe to let the drivers take time to do their job. 
#define ENC_LINE_PER_REV     3600      // Encoder lines per 1 spindle turn (encoder counts 1800 multiple 2:1 belt drive makes 3600 results in 7200 ticks per rev)
#define MOTOR_Z_STEP_PER_REV 200      // Z motor steps (leadscrew) - almost always 200, microsteps configured below
#define SCREW_Z              500      // Z (leadscrew) pitch, in hundreds of a mm (was: 96 pitch is 5mm, but there is a 1:5 gear in front, minus 4 correction empirically)
#define McSTEP_Z             4        // Z driver microsteps (400 steps = 2, 800 steps = 4, etc)
#define Z_INVERT                      // Comment this line if Z motor is connected directly to the lead screw, uncomment if via belt
// #define Z_ENA_INVERT                  // Comment/uncomment this line if Z motor is getting disabled when it should be enabled
#define MOTOR_X_STEP_PER_REV 200      // X motor steps (cross-slide) - almost always 200, microsteps configured below
#define SCREW_X              400       // X (cross-slide) pitch, in hundreds of a mm
#define REBOUND_X            0      // X backlash in microsteps
#define REBOUND_Z            0      // Z backlash in microsteps
#define McSTEP_X             4        // X driver microsteps (400 steps = 2, 800 steps = 4, etc)
//
#define THRD_ACCEL           25       // K. division from which we will accelerate on Threads, Accel+Ks should be < 255
#define FEED_ACCEL           3        // Rigidity of acceleration at feeds, more value - shorter acceleration.
//
#define MIN_FEED             1        // Min feed in 0.01mm. 2 = 0.02mm
#define MAX_FEED             40       // Max feed in 0.01mm. 20 = 0.20mm
#define MIN_aFEED            20       // Min feed in mm/min. 20 = 20mm/min
#define MAX_aFEED            400      // Max feed in mm/min. 400 = 400mm/min

// Rapid feed           
#define MAX_RAPID_MOTION     25                       // Smaller - faster           //16000000/32/((25+1)*2)/800*60=721rpm
#define MIN_RAPID_MOTION    (MAX_RAPID_MOTION + 150)  // Bigger - slower, max 255 //16000000/32/((150+25+1)*2)/800*60=107rpm
#define REPEAt              (McSTEP_Z * 1)            // Repeats for full speed within one step
                                                      // Acceleration duration = 150/2*REPEAT(4)/Microstep(4) = 75 full steps for acceleration
// Manual pulse generator (100 lines)
#define HC_SCALE_1           1        // 1st position, scale = 100/tick = 1mm/rev
#define HC_SCALE_10          10        // 2nd position, scale = 1000/tick = 10mm/revolution
#define HC_START_SPEED_1     250      // MPG start, 250000/(250+1)/800*60/2 = 37rpm
#define HC_MAX_SPEED_1       150      // maximum MPG speed, 250000/(150+1)/800*60/2 = 62rpm
#define HC_START_SPEED_10    150      // MPG start, 250000/(150+1)/800*60/2 = 62rpm
#define HC_MAX_SPEED_10      48       // maximum MPG speed, 250000/(23+1)/800*60/2 = 391rpm
#define HC_X_DIR             0        // 1-CW, 0-CCW
#define HC_Z_DIR             0        // 1-CW, 0-CCW


//////////////////////////////////////////////////////////////////////////////////////////////////
#define a  (uint32_t)(ENC_LINE_PER_REV / ((float)MOTOR_Z_STEP_PER_REV * McSTEP_Z * MIN_FEED / SCREW_Z) /2 +0.5)
static_assert(a <= 255, "Invalid value MIN_FEED Axis Z");
#define b  (uint32_t)(ENC_LINE_PER_REV / ((float)MOTOR_Z_STEP_PER_REV * McSTEP_Z * MAX_FEED / SCREW_Z) /2 +0.5)
static_assert(b > 1, "Invalid value MAX_FEED Axis Z");
#define c  250000 / ((uint32_t)MIN_aFEED * MOTOR_Z_STEP_PER_REV * McSTEP_Z / ((uint32_t)60 * SCREW_Z / 100) * 2) -1
static_assert(c <= 65535, "Invalid value MIN_aFEED Axis Z");
#define d  250000 / ((uint32_t)MAX_aFEED * MOTOR_Z_STEP_PER_REV * McSTEP_Z / ((uint32_t)60 * SCREW_Z / 100) * 2) -1
static_assert(d > 1, "Invalid value MAX_aFEED Axis Z");

#define e  (uint32_t)(ENC_LINE_PER_REV / ((float)MOTOR_X_STEP_PER_REV * McSTEP_X * MIN_FEED / SCREW_X) /2 +0.5)
static_assert(e <= 255, "Invalid value MIN_FEED Axis X");
#define f  (uint32_t)(ENC_LINE_PER_REV / ((float)MOTOR_X_STEP_PER_REV * McSTEP_X * MAX_FEED / SCREW_X) /2 +0.5)
static_assert(f > 1, "Invalid value MAX_FEED Axis X");
#define g  250000 / ((uint32_t)MIN_aFEED * MOTOR_X_STEP_PER_REV * McSTEP_X / ((uint32_t)60 * SCREW_X / 100) * 2) -1
static_assert(g <= 65535, "Invalid value MIN_aFEED  Axis X");
#define h  250000 / ((uint32_t)MAX_aFEED * MOTOR_X_STEP_PER_REV * McSTEP_X / ((uint32_t)60 * SCREW_X / 100) * 2) -1
static_assert(h > 1, "Invalid value MAX_aFEED  Axis X");
//////////////////////////////////////////////////////////


// ***** MY CONSTANT *****
#define CW               0
#define CCW              1
#ifdef Z_INVERT
  #define ZCW              1
  #define ZCCW             0
#else
  #define ZCW              0
  #define ZCCW             1
#endif
#define ON               1
#define OFF              0


// ***** LCD *****
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);
char LCD_Row_1[17];
char LCD_Row_2[17];

#define Beeper_Init()          DDRH = B01100010;\
                               PORTH = B10011101    // LCD-H5,H6 Buzzer-PH1_Pin16
#define Beeper_On()            PORTH &= ~(1<<1)     // Pin16 0
#define Beeper_Off()           PORTH |= (1<<1)      // Pin16 1



// ***** Stepper Motor *****
#define Motor_Init()           DDRL = B11111011;\
                               PORTL = B0000100

#define Motor_Z_SetPulse()     PORTL &= ~(1<<0)     // Pin49 0
#define Motor_Z_RemovePulse()  PORTL |= (1<<0)      // Pin49 1
#define Motor_Z_InvertPulse()  PORTL ^= (1<<0)      // Pin49
#define Read_Z_State           (PINL & (1<<0))

#define Motor_X_SetPulse()     PORTL &= ~(1<<1)     // Pin48 0
#define Motor_X_RemovePulse()  PORTL |= (1<<1)      // Pin48 1
#define Motor_X_InvertPulse()  PORTL ^= (1<<1)      // Pin48
#define Read_X_State           (PINL & (1<<1))

#ifdef Z_INVERT
  #define Motor_Z_CCW()           PORTL &= ~(1<<6)    // Pin43 0
  #define Motor_Z_CW()          PORTL |= (1<<6)     // Pin43 1
#else
  #define Motor_Z_CW()           PORTL &= ~(1<<6)    // Pin43 0
  #define Motor_Z_CCW()          PORTL |= (1<<6)     // Pin43 1
#endif

#define Motor_X_CW()           PORTL &= ~(1<<5)    // Pin44 0
#define Motor_X_CCW()          PORTL |= (1<<5)     // Pin44 1

#ifdef Z_ENA_INVERT
  #define Motor_Z_Enable()   do {PORTL &= ~(1<<4); _delay_ms(DELAY);} while(0)  // Pin45 0
  #define Motor_Z_Disable()      PORTL |= (1<<4)                              // Pin45 1
  #define Read_Z_Ena_State       !(PINL & (1<<4))
#else
  #define Motor_Z_Enable()   do {PORTL |= (1<<4); _delay_ms(DELAY);} while(0)   // Pin45 1
  #define Motor_Z_Disable()      PORTL &= ~(1<<4)                             // Pin45 0
  #define Read_Z_Ena_State       (PINL & (1<<4))
#endif

#define Motor_X_Enable()   do {PORTL |= (1<<3); _delay_ms(DELAY);} while(0)   // Pin46 1
#define Motor_X_Disable()      PORTL &= ~(1<<3)                             // Pin46 0
#define Read_X_Ena_State       (PINL & (1<<3))


// ***** Tacho *****
#define TachoSetPulse()        PORTL |= (1<<7)      // Pin42 1
#define TachoRemovePulse()     PORTL &= ~(1<<7)     // Pin42 0


// ***** Encoder *****
#define ENC_TICK              (ENC_LINE_PER_REV * 2)    // Working number of impulses
#define Encoder_Init()         DDRD = B00000000;\
                               PORTD = B11111111        // Pull up PIN_21, 20, 19, 18
#define Enc_Read              (PIND & (1<<1))
#define Enc_Ch_A              (PIND & (1<<0))
#define Enc_Ch_B              (PIND & (1<<1))
 


// ***** Hand_Coder *****            // Z/X: Input-E4,E5, Pull-E4,E5, X1/X10: Input-J0,J1, Pull-J0,J1..
#define Hand_Init()            DDRE = B00000000;\
                               PORTE = B11111111;\
                               DDRJ = B00000000;\
                               PORTJ = B11111111
                                     
#define Hand_Ch_A             (PIND & (1<<2))
#define Hand_Ch_B             (PIND & (1<<3))

#define Hand_Axis_Read        (PINE & B00110000)       // E4,E5
byte Hand_Axis_Old = 0;

#define Hand_Scale_Read        (PINJ & B00000011)      // J0,J1
byte Hand_Scale_Old = 0;


//***** Limit Buttons & LEDs *****
#define Limit_Init()           DDRA = B10101010;\
                               PORTA = B01010101    // IN-A0,A2,A4,A6, OUT-A1,A3,A5,A7, Pull up/down

#define Limit_Buttons_Read    (PINA & B01010101)    // PA0 Pin22, PA2 Pin24, PA4 Pin26, PA6 Pin28.
byte Limit_Button_Old = 0;

#define Limit_Rear_LED_On()    PORTA &= ~(1<<1)     // PA1, Pin23 0
#define Limit_Rear_LED_Off()   PORTA |= (1<<1)      // PA1, Pin23 1
#define Limit_Front_LED_On()   PORTA &= ~(1<<3)     // PA3, Pin25 0
#define Limit_Front_LED_Off()  PORTA |= (1<<3)      // PA3, Pin25 1
#define Limit_Right_LED_On()   PORTA &= ~(1<<5)     // PA5, Pin27 0
#define Limit_Right_LED_Off()  PORTA |= (1<<5)      // PA5, Pin27 1
#define Limit_Left_LED_On()    PORTA &= ~(1<<7)     // PA7, Pin29 0
#define Limit_Left_LED_Off()   PORTA |= (1<<7)      // PA7, Pin29 1

#define Limit_Pos_Max          1073741824
#define Limit_Pos_Min         -1073741824


//////////////////
#define Menu_Buttons_Init()    DDRF = B00000000;\
                               PORTF = B11111111;     

#define Buttons_Read           (PINF & B00001111)    // Pin_A0 PF0, Pin_A1 PF1, Pin_A2 PF2, Pin_A3 PF3, Pin_A4 PF4.
byte Button_Old = 0;

#define Button_Sel_Read        (PINF & B00010000)    // Pin_A4 PF4
byte Button_Sel_Old = 0;
bool key_sel_flag = false;

//////////////////
#define Joy_Init()             DDRK = B00000000;\
                               PORTK = B11111111;    // Pull up/down  PIN_A8, A9, A10, A11, A12 // Submode Sw: A13, A14, A15

#define Joy_Read              (PINK & B00001111)     // PK0 PK1 PK2 PK3
#define Button_Rapid          (PINK & B00010000)     // PK4
byte Joy_Old = 0;

////////////////////
#define Submode_Read          (PINK & B11100000)     // PK5 PK6 PK7
byte Submode_Old = 0;


// ***** Mode *****
#define Mode_Switch_Init()     DDRC = B00000000;\
                               PORTC = B11111111;        // PORT_A pull up, MANDATORY! external pull-up to +5 through 1K resistors
#define Mode_Read             (PINC & B11111111)
byte Mode_Old = 0;


enum Mode
{
  Mode_Thread = 1,
  Mode_Feed,
  Mode_aFeed,
  Mode_Cone_L,
  Mode_Cone_R,
  Mode_Reserve,
  Mode_Sphere,
  Mode_Divider
};

enum Sub_Mode_Thread
{
  Sub_Mode_Thread_Int = 1,
  Sub_Mode_Thread_Man,
  Sub_Mode_Thread_Ext,
};

enum Sub_Mode_Feed
{
  Sub_Mode_Feed_Int = 1,
  Sub_Mode_Feed_Man,
  Sub_Mode_Feed_Ext,
};

enum Sub_Mode_aFeed
{
  Sub_Mode_aFeed_Int = 1,
  Sub_Mode_aFeed_Man,
  Sub_Mode_aFeed_Ext,
};

enum Sub_Mode_Cone
{
  Sub_Mode_Cone_Int = 1,
  Sub_Mode_Cone_Man,
  Sub_Mode_Cone_Ext,
};

enum Sub_Mode_Sphere
{
  Sub_Mode_Sphere_Int = 1,
  Sub_Mode_Sphere_Man,
  Sub_Mode_Sphere_Ext,
};


//***** Fast travel *****
#define Timer2_Init()          TCCR2A = (1<<WGM21);\
                               TCCR2B = (1<<CS20)|(1<<CS21); // 16MHz/32 = 500kHz

//***** MPG moving *****
#define Timer3_Init()          TCCR3A = 0;\
                               TCCR3B = (1<<WGM32)|(1<<CS30)|(1<<CS31); // 16MHz/32 = 250kHz

// ***** aFeed *****
#define Timer4_Init()          TCCR4A = 0;\
                               TCCR4B = (1<<WGM42)|(1<<CS40)|(1<<CS41); // 16MHz/32 = 250kHz

// ***** Feed *****
#define Timer5_Init()          TCCR5A = 0;\
                               TCCR5B = (1<<WGM52) | (1<<CS52) | (1<<CS51) | (1<<CS50)


// ***** Cone *****
struct cone_info_type
{
  byte Cs_Div;
  int  Cm_Div;
  char Cone_Print[6];
};


const cone_info_type Cone_Info[] = {
  {  1, 6000, "45dg"},
  { 61, 4784, " KM0"}, // 1:19.212
  { 64, 1504, " KM1"}, // 1:20.047
  { 64,  640, " KM2"}, // 1:20.02
  { 63, 7504, " KM3"}, // 1:19.922
  { 61, 6128, " KM4"}, // 1:19.254
  { 60, 8064, " KM5"}, // 1:19.002
  { 61, 3760, " KM6"}, // 1:19.18
  { 12, 8000, " 1:4"}, // 1:4
  { 16,    0, " 1:5"}, // 1:5
  { 22, 4000, " 1:7"}, // 1:7
  { 32,    0, "1:10"}, // 1:10
  { 51, 2000, "1:16"}, // 1:16
  { 64,    0, "1:20"}, // 1:20
  { 76, 8000, "1:24"}, // 1:24
  { 95, 10000, "1:30"}, // 1:30
  {160,    0, "1:50"}, // 1:50
  { 29, 2571, "7:64"}, // 7:64
  { 11, 3846, " 8dg"},
  {  9,  741, "10dg"},
  {  5, 9713, "15dg"},
  {  2, 7713, "30dg"},
};


#define TOTAL_CONE (sizeof(Cone_Info) / sizeof(Cone_Info[0]))


// ***** Threads *****
struct thread_info_type
{
  byte Ks_Div_Z;
  int  Km_Div_Z;
  byte Ks_Div_X;
  int  Km_Div_X;
  char Thread_Print[7];
  float Step;
  byte Pass;
  char Limit_Print[8];
};

const thread_info_type Thread_Info[] = {
  {720,    0, 576,    0, "0.25mm", 0.250, 4, " 750rpm"},
  {600,    0, 480,    0, "0.30mm", 0.300, 4, " 750rpm"},
  {514, 2857, 411, 4286, "0.35mm", 0.350, 4, " 750rpm"},
  {450,    0, 360,    0, "0.40mm", 0.400, 4, " 750rpm"},
  {360,    0, 288,    0, "0.50mm", 0.500, 4, " 750rpm"},
  {300,    0, 240,    0, "0.60mm", 0.600, 4, " 750rpm"},
  {257, 1429, 205, 7143, "0.70mm", 0.700, 4, " 750rpm"},
  {240,    0, 192,    0, "0.75mm", 0.750, 5, " 750rpm"},
  {225,    0, 180,    0, "0.80mm", 0.800, 5, " 700rpm"},
  {180,    0, 144,    0, "1.00mm", 1.000, 6, " 560rpm"},
  {144,    0, 115, 2000, "1.25mm", 1.250, 7, " 460rpm"},
  {120,    0,  96,    0, "1.50mm", 1.500, 7, " 380rpm"},
  {102, 8571,  82, 2857, "1.75mm", 1.750, 8, " 320rpm"},
  { 90,    0,  72,    0, "2.00mm", 2.000, 9, " 280rpm"},
  { 72,    0,  57, 6000, "2.50mm", 2.500, 11, " 220rpm"},
  { 60,    0,  48,    0, "3.00mm", 3.000, 15, " 190rpm"},
  { 45,    0,  36,    0, "4.00mm", 4.000, 22, " 140rpm"},

  {566, 9291, 453, 5433, "80tpi ", 0.318, 4, " 750rpm"},
  {510, 2362, 408, 1890, "72tpi ", 0.353, 4, " 750rpm"},
  {453, 5433, 362, 8346, "64tpi ", 0.397, 4, " 750rpm"},
  {425, 1969, 340, 1575, "60tpi ", 0.423, 4, " 750rpm"},
  {396, 8504, 317, 4803, "56tpi ", 0.454, 4, " 750rpm"},
  {340, 1575, 272, 1260, "48tpi ", 0.529, 4, " 750rpm"},
  {311, 8110, 249, 4488, "44tpi ", 0.577, 4, " 750rpm"},
  {283, 4646, 226, 7717, "40tpi ", 0.635, 4, " 750rpm"},
  {255, 1181, 204,  945, "36tpi ", 0.706, 5, " 750rpm"},
  {226, 7717, 181, 4173, "32tpi ", 0.794, 5, " 710rpm"},
  {198, 4252, 158, 7402, "28tpi ", 0.907, 5, " 650rpm"},
  {191, 3386, 153,  709, "27tpi ", 0.941, 5, " 600rpm"},
  {184, 2520, 147, 4016, "26tpi ", 0.977, 6, " 570rpm"},
  {170,  787, 136,  630, "24tpi ", 1.058, 6, " 500rpm"},
  {155, 9055, 124, 7244, "22tpi ", 1.155, 6, " 450rpm"},
  {141, 7323, 113, 3858, "20tpi ", 1.270, 7, " 440rpm"},
  {134, 6457, 107, 7165, "19tpi ", 1.337, 7, " 420rpm"},
  {127, 5591, 102,  472, "18tpi ", 1.411, 7, " 380rpm"},
  {113, 3858,  90, 7087, "16tpi ", 1.587, 8, " 350rpm"},
  { 99, 2126,  79, 3701, "14tpi ", 1.814, 9, " 320rpm"},
  { 85,  394,  68,  315, "12tpi ", 2.117, 10, " 270rpm"},
  { 70, 8661,  56, 6929, "10tpi ", 2.540, 11, " 220rpm"},
  { 63, 7795,  51,  236, " 9tpi ", 2.822, 14, " 190rpm"},
  { 56, 6929,  45, 3543, " 8tpi ", 3.175, 16, " 170rpm"},
  { 49, 6063,  39, 6850, " 7tpi ", 3.629, 19, " 150rpm"},
  { 42, 5197,  34,  157, " 6tpi ", 4.233, 24, " 140rpm"},
};
#define TOTAL_THREADS (sizeof(Thread_Info) / sizeof(Thread_Info[0]))
#define PASS_FINISH   3 // THRD_PS_FN ???

//#define Thrd_Accel_Err Thread_Info[0].Ks_Div_Z                 // acceleration set incorrectly
//static_assert(Thrd_Accel_Err + THRD_ACCEL <= 255, "Invalid value THRD_ACCEL");


// ***** Interrupts *****
#define INT0_Init()               EICRA |= (1<<ISC00)
#define INT2_Init()               EICRA |= (1<<ISC20)

#define Enable_INT0()             EIMSK |= (1<<INT0)
#define Disable_INT0()            EIMSK &= ~(1<<INT0)

#define Ena_INT_Hcoder()      do {EIFR = (1<<INTF2); EIMSK |= (1<<INT2);} while(0)
#define Disa_INT_Hcoder()         EIMSK &= ~(1<<INT2)

#define Enable_INT_OCR2A()    do {TCNT2 = 0; TIFR2 = (1<<OCF2A); TIMSK2 = (1<<OCIE2A);} while(0)
#define Disable_INT_OCR2A()       TIMSK2 &= ~(1<<OCIE2A)

#define Enable_INT_OCR2B()    do {TCNT2 = 0; TIFR2 = (1<<OCF2B); TIMSK2 = (1<<OCIE2B);} while(0)
#define Disable_INT_OCR2B()       TIMSK2 &= ~(1<<OCIE2B)

#define Enable_INT_OCR3A()    do {TCNT3 = 0; TIFR3 = (1<<OCF3A); TIMSK3 = (1<<OCIE3A);} while(0)
#define Disable_INT_OCR3A()       TIMSK3 &= ~(1<<OCIE3A)
// timer on output compare register, decoding hand-encoder
#define Enable_INT_OCR3B()    do {TCNT3 = 0; TIFR3 = (1<<OCF3B); TIMSK3 = (1<<OCIE3B);} while(0)
#define Disable_INT_OCR3B()       TIMSK3 &= ~(1<<OCIE3B)
// timer on output compare register, decoding ?
#define Enable_INT_OCR4A()    do {TCNT4 = 0; TIFR4 = (1<<OCF4A); TIMSK4 = (1<<OCIE4A);} while(0)
#define Disable_INT_OCR4A()       TIMSK4 &= ~(1<<OCIE4A)

#define Enable_INT_OCR4B()    do {TCNT4 = 0; TIFR4 = (1<<OCF4B); TIMSK4 = (1<<OCIE4B);} while(0)
#define Disable_INT_OCR4B()       TIMSK4 &= ~(1<<OCIE4B)
// setzt den Takt für den asynchronen Vorschub?
#define Enable_INT_OCR5A()    do {TCNT5 = 0; TIFR5 = (1<<OCF5A); TIMSK5 = (1<<OCIE5A);} while(0)
#define Disable_INT_OCR5A()       TIMSK5 &= ~(1<<OCIE5A)

#define Enable_INT_OCR5B()    do {TCNT5 = 0; TIFR5 = (1<<OCF5B); TIMSK5 = (1<<OCIE5B);} while(0)
#define Disable_INT_OCR5B()       TIMSK5 &= ~(1<<OCIE5B)

////////////////////////////////////////////////////////////
#define Ena_INT_Thrd()        do {Disable_INT_OCR2A();\
                                  Disable_INT_OCR2B();\
                                  Disable_INT_OCR4A();\
                                  Disable_INT_OCR4B();\
                                  Disable_INT_OCR5A();\
                                  Disable_INT_OCR5B();\
                                  Enable_INT0();} while(0)

#define Ena_INT_Z_Feed()      do {Disable_INT0();\
                                  Disable_INT_OCR2A();\
                                  Disable_INT_OCR2B();\
                                  Disable_INT_OCR4A();\
                                  Disable_INT_OCR4B();\
                                  Disable_INT_OCR5B();\
                                  Enable_INT_OCR5A();} while(0)

#define Ena_INT_Z_aFeed()     do {Disable_INT0();\
                                  Disable_INT_OCR2A();\
                                  Disable_INT_OCR2B();\
                                  Disable_INT_OCR4B();\
                                  Disable_INT_OCR5A();\
                                  Disable_INT_OCR5B();\
                                  Enable_INT_OCR4A();} while(0)
                                     
#define Ena_INT_X_Feed()      do {Disable_INT0();\
                                  Disable_INT_OCR2A();\
                                  Disable_INT_OCR2B();\
                                  Disable_INT_OCR4A();\
                                  Disable_INT_OCR4B();\
                                  Disable_INT_OCR5A();\
                                  Enable_INT_OCR5B();} while(0)

#define Ena_INT_X_aFeed()     do {Disable_INT0();\
                                  Disable_INT_OCR2A();\
                                  Disable_INT_OCR2B();\
                                  Disable_INT_OCR4A();\
                                  Disable_INT_OCR5A();\
                                  Disable_INT_OCR5B();\
                                  Enable_INT_OCR4B();} while(0)
                                     
#define Ena_INT_Z_Rapid()     do {Disable_INT0();\
                                  Disable_INT_OCR2B();\
                                  Disable_INT_OCR4A();\
                                  Disable_INT_OCR4B();\
                                  Disable_INT_OCR5A();\
                                  Disable_INT_OCR5B();\
                                  Enable_INT_OCR2A();} while(0)
                                     
#define Ena_INT_X_Rapid()     do {Disable_INT0();\
                                  Disable_INT_OCR2A();\
                                  Disable_INT_OCR4A();\
                                  Disable_INT_OCR4B();\
                                  Disable_INT_OCR5A();\
                                  Disable_INT_OCR5B();\
                                  Enable_INT_OCR2B();} while(0)
                                                                       

// ***** My Flags *****
typedef struct
{ 
   uint8_t bit0 : 1;
   uint8_t bit1 : 1;
   uint8_t bit2 : 1;
   uint8_t bit3 : 1;
   uint8_t bit4 : 1;
   uint8_t bit5 : 1;
   uint8_t bit6 : 1;
   uint8_t bit7 : 1;
}FLAG;
#define Spindle_Dir        ((volatile FLAG*)_SFR_MEM_ADDR(GPIOR0))->bit0    // CW-0, CCW-1
#define Motor_Z_Dir        ((volatile FLAG*)_SFR_MEM_ADDR(GPIOR0))->bit1    // CW-0, CCW-1
#define Joy_Z_flag         ((volatile FLAG*)_SFR_MEM_ADDR(GPIOR0))->bit2    // On-1, Off-0
#define Step_Z_flag        ((volatile FLAG*)_SFR_MEM_ADDR(GPIOR0))->bit3    // On-1, Off-0
#define Motor_X_Dir        ((volatile FLAG*)_SFR_MEM_ADDR(GPIOR0))->bit4    // CW-0, CCW-1
#define Joy_X_flag         ((volatile FLAG*)_SFR_MEM_ADDR(GPIOR0))->bit5    // On-1, Off-0
#define Step_X_flag        ((volatile FLAG*)_SFR_MEM_ADDR(GPIOR0))->bit6    // On-1, Off-0
#define Cone_flag          ((volatile FLAG*)_SFR_MEM_ADDR(GPIOR0))->bit7    // On-1, Off-0

bool spindle_flag = OFF;
bool feed_Z_flag = OFF;
bool feed_X_flag = OFF;

bool rapid_step_Z_flag = OFF;
bool rapid_step_X_flag = OFF;
bool rapid_Z_flag = OFF;
bool rapid_X_flag = OFF;

bool limit_Left_flag = OFF;
bool limit_Right_flag = OFF;
bool limit_Front_flag = OFF;
bool limit_Rear_flag = OFF;

bool limit_button_flag = OFF;
bool button_flag = OFF;

bool a_flag = false;
bool b_flag = false;
bool c_flag = false;
bool d_flag = false;
bool cycle_flag = false;

bool err_1_flag = false;
bool err_2_flag = false;

bool hand_X = OFF;
bool hand_Z = OFF;
bool flag_hand_X = OFF;
bool flag_hand_Z = OFF;
bool X_flag = OFF;                    // temporary
bool Z_flag = OFF;                    // temporary
bool flag_Scale_x1 = OFF;             // only possible for debugging
bool flag_Scale_x10 = OFF;            // only possible for debugging
bool control_flag = OFF;
bool flag_j = OFF;

// ***** MY VARIABLES *****
int Tacho_Count = 0;
int Tacho_Count_Old =0;
int Spindle_Count = 0;

int Enc_Pos = 0;
volatile long Hand_Count = 0;
long Hand_Count_Old = 0;
long Hand_Count_New = 0;
long Hand_Z_Pos = 0;
long Hand_X_Pos = 0;

byte Scale = HC_SCALE_1;

byte Ks_Count = 0;
int Km_Count = 0;
byte Ks_Divisor = 0;
byte tmp_Ks_Divisor = THRD_ACCEL;
int Km_Divisor = 0;
uint16_t Feed_Divisor = 0;
uint16_t aFeed_Divisor = 0;

byte Cs_Count = 0;
int Cm_Count = 0;
byte Cs_Divisor = 0;
int Cm_Divisor = 0;

byte tmp_Accel = THRD_ACCEL;
byte Repeat_Count = 0;

int Brake_Compens = 0;

byte Mode = Mode_Feed;
byte Sub_Mode_Thread = Sub_Mode_Thread_Man;
byte Sub_Mode_Feed = Sub_Mode_Feed_Man;
byte Sub_Mode_aFeed = Sub_Mode_aFeed_Man;
byte Sub_Mode_Cone = Sub_Mode_Cone_Man;
byte Sub_Mode_Sphere = Sub_Mode_Sphere_Man;
// The n-th configuration set of the ThreadInfo Array, not a physical step
byte Thread_Step = 11;
byte Cone_Step = 0;

long Motor_Z_Pos = 0;
long Motor_X_Pos = 0;

long Limit_Pos_Left = Limit_Pos_Max;
long Limit_Pos_Right = Limit_Pos_Min;
long Limit_Pos_Front = Limit_Pos_Max;
long Limit_Pos_Rear = Limit_Pos_Min;
volatile long Limit_Pos = 0;
volatile long Limit_Pos_HC = 0;

uint16_t Feed_mm = 0;
uint16_t aFeed_mm = 0;

uint16_t Start_Speed = ENC_LINE_PER_REV / ((uint32_t)MOTOR_Z_STEP_PER_REV * McSTEP_Z * MIN_FEED / SCREW_Z) /FEED_ACCEL;
uint16_t max_OCR5A = ENC_LINE_PER_REV / ((uint32_t)MOTOR_Z_STEP_PER_REV * McSTEP_Z * MIN_FEED / SCREW_Z) /FEED_ACCEL;
uint16_t max_OCR4A = (250000 / ((uint32_t)MIN_aFEED * MOTOR_Z_STEP_PER_REV * McSTEP_Z / ((uint32_t)60 * SCREW_Z / 100) * 2) - 1) /FEED_ACCEL;

byte Total_Tooth = 1;
byte Current_Tooth = 1;

byte Pass_Total = 1;
byte Pass_Nr = 1;
long Null_X_Pos = 0;
long Null_Z_Pos = 0;
int Ap = 0;
// last 16 different values of feed potentionmeter are stored in array 
int ADC_Feed = 0;
long Sum_ADC = 0;
int ADC_Array[16];
byte x = 0;

long Control_Count = 0;


// Sphere
const int Cutter_Width_array[] = {100, 150, 200, 250, 300};
#define TOTAL_CUTTER_WIDTH (sizeof(Cutter_Width_array) / sizeof(Cutter_Width_array[0]))
byte Cutter_Step = 2;
int Cutter_Width = Cutter_Width_array[Cutter_Step];

const int Cutting_Width_array[] = {10, 25, 50, 100};
#define TOTAL_CUTTING_STEP (sizeof(Cutting_Width_array) / sizeof(Cutting_Width_array[0]))
byte Cutting_Step = 1;
int Cutting_Width = Cutting_Width_array[Cutting_Step];

long Sph_R_mm = 1000;
long Sph_R = 0;
long R_Quad = Sph_R_mm * Sph_R_mm;
long Bar_R_mm = 0;
long Bar_R = 0;

#define KEYB_TIMER_FLAG       (TIFR1 & (1<<OCF1A))
#define CLEAR_KEYB_TIMER   do {TCNT1 = 0; (TIFR1 |= (1<<OCF1A));} while(0)

uint16_t max_OCR3A = HC_START_SPEED_1;
uint16_t min_OCR3A = HC_MAX_SPEED_1;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("   ELS v.7e2    ");
  _delay_ms(1000);
  
  DDRG = B11111111;
  
  TIMSK0 = 0;
  // ***** Timer0 ***** // ***** Tachometer *****
//  TCCR0A = (1<<COM0B0)|(1<<WGM01); // Toggle OC0B on Compare Match // CTC Mode2
//  TCCR0B = (1<<CS00);     // No Prescaler
//  OCR0A = 89; // 1800/10(output pulses)/2-1 = 89
//  TIMSK0 = (1<<OCIE0B);
    
  Encoder_Init();
  Hand_Init();
  Motor_Init();
  
  INT0_Init();
  INT2_Init();

  Timer2_Init();
  OCR2A = MIN_RAPID_MOTION;

  Timer3_Init();
  OCR3A = max_OCR3A;

  Timer4_Init();
  OCR4A = max_OCR4A;
  
  Timer5_Init();
  OCR5A = max_OCR5A;
  
  Ena_INT_Z_Feed();
  
  Limit_Init();
  Limit_Left_LED_Off();
  Limit_Right_LED_Off();
  Limit_Front_LED_Off();
  Limit_Rear_LED_Off();
  Menu_Buttons_Init();
  Joy_Init();
  Mode_Switch_Init();

  Beeper_Init();
  Beeper_Off();
  
  Spindle_Dir = CW;
  Motor_Z_Dir = ZCW;
  Joy_Z_flag = OFF;
  Step_Z_flag = OFF;
  Motor_X_Dir = CW;
  Joy_X_flag = OFF;
  Step_X_flag = OFF;

  TCCR1A = 0;
  TCCR1B = 0
         |(0<<ICNC1)|(0<<ICES1)
         |(0<<WGM13)|(1<<WGM12)
         |(1<<CS12)|(0<<CS11)|(1<<CS10);
  OCR1A = 625;

  Motor_Z_RemovePulse();
  Motor_X_RemovePulse();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  Spindle();
  // position of feed potentiometer
  Read_ADC_Feed();
  if (KEYB_TIMER_FLAG != 0) Menu();
  
  if (Mode == Mode_Divider) Print(); // bye for the test !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//  Print();                         // just for test !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// *****  Thread (and synchronous feed I guess) ***** ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(INT0_vect)
{
   TachoRemovePulse();

   if (!Enc_Ch_A)
   {
      if (!Enc_Ch_B)
      {
         Spindle_Dir = CW;
         if (++Enc_Pos == ENC_TICK)
         {                                           
            Enc_Pos = 0;
            TachoSetPulse();
            if (Joy_Z_flag == ON) {Step_Z_flag = ON;}
            else if (Joy_X_flag == ON) {Step_X_flag = ON;}
         }
      }
      else
      {
        Spindle_Dir = CCW;
        if (--Enc_Pos < 0)
        { 
           Enc_Pos = ENC_TICK - 1;
           TachoSetPulse();
           if (Joy_Z_flag == ON) {Step_Z_flag = ON;}
           else if (Joy_X_flag == ON) {Step_X_flag = ON;}
        }
      }
   }
   
   else
   {
      if (!Enc_Ch_B) 
      {
         Spindle_Dir = CCW;
         if (--Enc_Pos < 0)
         { 
            Enc_Pos = ENC_TICK - 1;
            TachoSetPulse();
            if (Joy_Z_flag == ON) {Step_Z_flag = ON;}
            else if (Joy_X_flag == ON) {Step_X_flag = ON;}
         }
      }
      else 
      {
         Spindle_Dir = CW;
         if (++Enc_Pos == ENC_TICK)
         {                                           
            Enc_Pos = 0;
            TachoSetPulse();
            if (Joy_Z_flag == ON) {Step_Z_flag = ON;}
            else if (Joy_X_flag == ON) {Step_X_flag = ON;}
         }
      }
   }  
   
   if (Step_Z_flag == ON)
   {   Motor_Z_RemovePulse();
      if ( (Motor_Z_Dir == ZCW && Motor_Z_Pos > Limit_Pos) || (Motor_Z_Dir == ZCCW && Motor_Z_Pos < Limit_Pos) || (!Joy_Z_flag) )
      {
         if (tmp_Ks_Divisor < tmp_Accel)
         {
            Ks_Count++;
            if (Ks_Count > tmp_Ks_Divisor)
            {
               Motor_Z_SetPulse();
               if (Motor_Z_Dir == ZCW) {Motor_Z_Pos ++;}
               else {Motor_Z_Pos --;}
               Ks_Count = 0;
               if (++Repeat_Count == REPEAt)
               {
                  Repeat_Count = 0;
                  tmp_Ks_Divisor ++;
               }
            }
         }  
         else {Step_Z_flag = OFF;}
      }

      else
      {
         Ks_Count++;
         if (Ks_Count > tmp_Ks_Divisor)
         {
            Motor_Z_SetPulse();
            if (Motor_Z_Dir == ZCW) {Motor_Z_Pos ++;}
            else {Motor_Z_Pos --;}
         
            if (tmp_Ks_Divisor > Ks_Divisor)
            {
               Ks_Count = 0;
               if (++Repeat_Count == REPEAt)
               {
                  Repeat_Count = 0;
                  tmp_Ks_Divisor --;
               }
            }
            else
            {  
               Km_Count = Km_Count + Km_Divisor;
               if (Km_Count > Km_Divisor)
               {
                  Km_Count = Km_Count - 10000;
                  Ks_Count = 0;
               }
               else {Ks_Count = 1;}
            }
         }
      }
   }
   
   if (Step_X_flag == ON)
   {  Motor_X_RemovePulse();
      if ( (Motor_X_Dir == CW && Motor_X_Pos > Limit_Pos) || (Motor_X_Dir == CCW && Motor_X_Pos < Limit_Pos) || (!Joy_X_flag) )
      {
         if (tmp_Ks_Divisor < tmp_Accel)
         {
            Ks_Count++;
            if (Ks_Count > tmp_Ks_Divisor)
            {
               Motor_X_SetPulse();
               if (Motor_X_Dir == CW) {Motor_X_Pos ++;}
               else {Motor_X_Pos --;}
               Ks_Count = 0;
               if (++Repeat_Count == REPEAt)
               {
                  Repeat_Count = 0;
                  tmp_Ks_Divisor ++;
               }
            }
         }  
         else {Step_X_flag = OFF;}
      }

      else
      {
         Ks_Count++;
         if (Ks_Count > tmp_Ks_Divisor)
         {
            Motor_X_SetPulse();
            if (Motor_X_Dir == CW) {Motor_X_Pos ++;}
            else {Motor_X_Pos --;}
         
            if (tmp_Ks_Divisor > Ks_Divisor)
            {
               Ks_Count = 0;
               if (++Repeat_Count == REPEAt)
               {
                  Repeat_Count = 0;
                  tmp_Ks_Divisor --;
               }
            }
            else
            {  
               Km_Count = Km_Count + Km_Divisor;
               if (Km_Count > Km_Divisor)
               {
                  Km_Count = Km_Count - 10000;
                  Ks_Count = 0;
               }
               else {Ks_Count = 1;}
            }
         }
      }
   }
}


ISR(INT1_vect)
{
   //
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// *****  Tacho ***** ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR (TIMER0_COMPB_vect)                                 // Tachometer
{
   //   
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// *****  Feed & Cone ***** //////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR (TIMER5_COMPA_vect)
{
   if (Joy_Z_flag == ON) {Motor_X_RemovePulse();}
   TachoRemovePulse();
   Tacho_Count = Tacho_Count + (OCR5A+1);
   if (Tacho_Count > ENC_LINE_PER_REV)
   {
      TachoSetPulse();
      Tacho_Count = Tacho_Count - ENC_LINE_PER_REV;
   }
   
   if ( (Motor_Z_Dir == ZCW && Motor_Z_Pos > Limit_Pos) || (Motor_Z_Dir == ZCCW && Motor_Z_Pos < Limit_Pos) || (!feed_Z_flag) )
   {
      if (OCR5A < max_OCR5A)
      {
         Motor_Z_InvertPulse();
         if (!Read_Z_State)
         {
            OCR5A++;
            if (Motor_Z_Dir == ZCW) {Motor_Z_Pos ++;}
            else {Motor_Z_Pos --;}
         }
      }
      else
      {             
         Step_Z_flag = OFF;
         Step_X_flag = OFF;
      }
   }
   
   else
   {
      Step_Z_flag = ON;
      Motor_Z_InvertPulse();
      if (!Read_Z_State)
      {
         if (Motor_Z_Dir == ZCW) {Motor_Z_Pos ++;}
         else {Motor_Z_Pos --;}
      
         if (OCR5A > Feed_Divisor) {OCR5A--;}
         else if (OCR5A < Feed_Divisor) {OCR5A ++;}
      }
   }
   
   if (Step_X_flag == ON)
   {
      if (++Cs_Count > Cs_Divisor)
      {
         Motor_X_SetPulse();

         if (Motor_X_Dir == CW) {Motor_X_Pos ++;}
         else {Motor_X_Pos --;}

         Cm_Count = Cm_Count + Cm_Divisor;
         if (Cm_Count > Cm_Divisor)
         {
            Cm_Count = Cm_Count - 10000;
            Cs_Count = 0;
         }
         else {Cs_Count = 1;}
      }
   }
}

////////////////////////////////////////////////////////////
ISR (TIMER5_COMPB_vect)
{
   TachoRemovePulse();   
   Tacho_Count = Tacho_Count + (OCR5A+1);
   if (Tacho_Count > ENC_LINE_PER_REV)
   {
      TachoSetPulse();
      Tacho_Count = Tacho_Count - ENC_LINE_PER_REV;
   }
  
   if ( (Motor_X_Dir == CW && Motor_X_Pos > Limit_Pos) || (Motor_X_Dir == CCW && Motor_X_Pos < Limit_Pos) || (!feed_X_flag) )
   {
      if (OCR5A < max_OCR5A)
      {
         Motor_X_InvertPulse();
         if (!Read_X_State)
         {
            OCR5A++;
            if (Motor_X_Dir == CW) {Motor_X_Pos ++;}
            else {Motor_X_Pos --;}
         }
      }
      else {Step_X_flag = OFF;}
   }
   
   else 
   {
      Step_X_flag = ON;
      Motor_X_InvertPulse();
      {
         if (!Read_X_State)
         {
            if (Motor_X_Dir == CW) {Motor_X_Pos ++;}
            else {Motor_X_Pos --;}
      
            if (OCR5A > Feed_Divisor) {OCR5A--;}
            else if (OCR5A < Feed_Divisor) {OCR5A ++;}
         }
      }
   }

   /////////////////////////////////////////////////////////
   if (Mode == Mode_Sphere)                               // Sphere mode
   {
      
   }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ***** Rapid Feed & Rapid Cone ***** ///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR (TIMER2_COMPA_vect)
{
   Motor_X_RemovePulse();
   if ( (Motor_Z_Dir == ZCW && Motor_Z_Pos > Limit_Pos) || (Motor_Z_Dir == ZCCW && Motor_Z_Pos < Limit_Pos) || (!rapid_Z_flag) )
   {
      if (OCR2A < MIN_RAPID_MOTION)
      {
         Motor_Z_InvertPulse();
         if (!Read_Z_State)
         {
            if (Motor_Z_Dir == ZCW) { Motor_Z_Pos ++; }
            else { Motor_Z_Pos --; }

            if (++Repeat_Count == REPEAt)
            {
               Repeat_Count = 0;
               OCR2A ++;
            }
         }
      }
      else
      {
         rapid_step_Z_flag = OFF;
         Step_X_flag = OFF;
      }
   }
  
   else
   {
      rapid_step_Z_flag = ON;
      Motor_Z_InvertPulse();
      if (!Read_Z_State)
      {
         if (Motor_Z_Dir == ZCW) { Motor_Z_Pos ++; }
         else { Motor_Z_Pos --; }

         if (OCR2A > MAX_RAPID_MOTION)
         {
            if (++Repeat_Count == REPEAt)
            {
               Repeat_Count = 0;
               OCR2A --;
            }
         }
      }
   }
   
   ///////////////////////////////////////////////////////
   if (Step_X_flag == ON)
   {
      if (++Cs_Count > Cs_Divisor)
      {
         Motor_X_SetPulse();
         if (Motor_X_Dir == CW) { Motor_X_Pos ++; }
         else { Motor_X_Pos --; }
         
         Cm_Count = Cm_Count + Cm_Divisor;
         if (Cm_Count > Cm_Divisor)
         {
            Cm_Count = Cm_Count - 10000;
            Cs_Count = 0;
         }
         else {Cs_Count = 1;}
      }
   }
}

//////////////////////////////////////////////////////////
ISR (TIMER2_COMPB_vect)
{
   if ( (Motor_X_Dir == CW && Motor_X_Pos > Limit_Pos) || (Motor_X_Dir == CCW && Motor_X_Pos < Limit_Pos) || (!rapid_X_flag) )
   {
      if (OCR2A < MIN_RAPID_MOTION)
      {
         Motor_X_InvertPulse();
         if (!Read_X_State)
         {
            if (Motor_X_Dir == CW) { Motor_X_Pos ++; }
            else { Motor_X_Pos --; }

            if (++Repeat_Count == REPEAt)
            {
               Repeat_Count = 0;
               OCR2A ++;
            }
         }
      }
      else  {rapid_step_X_flag = OFF;}
   }
  
   else
   {
      rapid_step_X_flag = ON;
      Motor_X_InvertPulse();
      if (!Read_X_State)
      {
         if (Motor_X_Dir == CW) { Motor_X_Pos ++; }
         else { Motor_X_Pos --; }

         if (OCR2A > MAX_RAPID_MOTION)
         {
            if (++Repeat_Count == REPEAt)
            {
               Repeat_Count = 0;
               OCR2A --;
            }
         }
      }
   }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ***** Asynchron Feed ***** ////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR (TIMER4_COMPA_vect)
{
   if ( (Motor_Z_Dir == ZCW && Motor_Z_Pos > Limit_Pos) || (Motor_Z_Dir == ZCCW && Motor_Z_Pos < Limit_Pos) || (!feed_Z_flag) )
   {
      if (OCR4A < max_OCR4A)
      {
         Motor_Z_InvertPulse();
         if (!Read_Z_State)
         {
            OCR4A ++;
            if (Motor_Z_Dir == ZCW) {Motor_Z_Pos ++;}
            else                   {Motor_Z_Pos --;}
         }
      }
      else
      {             
         Step_Z_flag = OFF;
         Step_X_flag = OFF;
      }
   }
   
   else
   {
      Step_Z_flag = ON;
      Motor_Z_InvertPulse();
      if (!Read_Z_State)
      {
         if (Motor_Z_Dir == ZCW) {Motor_Z_Pos ++;}
         else                   {Motor_Z_Pos --;}

         if      (OCR4A > aFeed_Divisor) {OCR4A --;}
         else if (OCR4A < aFeed_Divisor) {OCR4A ++;}
      }
   }
}

//////////////////////////////////////////////////////////
ISR (TIMER4_COMPB_vect)
{
   if ( (Motor_X_Dir == CW && Motor_X_Pos > Limit_Pos) || (Motor_X_Dir == CCW && Motor_X_Pos < Limit_Pos) || (!feed_X_flag) )
   {
      if (OCR4A < max_OCR4A)
      {
         Motor_X_InvertPulse();
         if (!Read_X_State)
         {
            OCR4A ++;
            if (Motor_X_Dir == CW) {Motor_X_Pos ++;}
            else                   {Motor_X_Pos --;}
         }
      }
      else
      {             
         Step_Z_flag = OFF;
         Step_X_flag = OFF;
      }
   }
   
   else 
   {
      Step_X_flag = ON;
      Motor_X_InvertPulse();
      if (!Read_X_State)
      {
         if (Motor_X_Dir == CW) {Motor_X_Pos ++;}
         else                   {Motor_X_Pos --;}

         if      (OCR4A > aFeed_Divisor) {OCR4A --;}
         else if (OCR4A < aFeed_Divisor) {OCR4A ++;}
      }
   }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ***** HandCoder ***** /////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(INT2_vect)
{
   if (!Hand_Ch_A)
   {
      if (!Hand_Ch_B) {Hand_Count --;}
   }
   
   else
   {
      if (!Hand_Ch_B) {Hand_Count ++;}
   }

}

/////////////////////////////////////////////
ISR (TIMER3_COMPA_vect)
{   
   if (Motor_Z_Dir == ZCW)
   {
      if (Motor_Z_Pos < Null_Z_Pos + Hand_Z_Pos)
      {
         Motor_Z_InvertPulse();
         if (!Read_Z_State) 
         {
            Motor_Z_Pos ++;
            if ((Motor_Z_Pos > Limit_Pos_HC) || (hand_Z == OFF))
            {
               if (OCR3A < max_OCR3A) OCR3A ++;
            }
            else if (Motor_Z_Pos < Limit_Pos_HC)
            {
               if (OCR3A > min_OCR3A) OCR3A --;
            }
         }
      }
      else if (Motor_Z_Pos == Hand_Z_Pos)
      {
      //
      }
   }

   else if (Motor_Z_Dir == ZCCW)
   {
      if (Motor_Z_Pos > Null_Z_Pos + Hand_Z_Pos)
      {
         Motor_Z_InvertPulse();
         if (!Read_Z_State) 
         {
            Motor_Z_Pos --;
            if (Motor_Z_Pos < Limit_Pos_HC  || hand_Z == OFF)
            {
               if (OCR3A < max_OCR3A) OCR3A ++;
            }
            else if (Motor_Z_Pos > Limit_Pos_HC)
            {
               if (OCR3A > min_OCR3A) OCR3A --;
            }
         }
      }
      else if (Motor_Z_Pos == Hand_Z_Pos)
      {
      //
      }
   }
}

//////////////////////////////////////////////////////////
ISR (TIMER3_COMPB_vect)
{   
   if (Motor_X_Dir == CW)
   {
      if (Motor_X_Pos < Null_X_Pos + Hand_X_Pos)
      {
         Motor_X_InvertPulse();
         if (!Read_X_State) 
         {
            Motor_X_Pos ++;
            if ((Motor_X_Pos > Limit_Pos_HC) || (hand_X == OFF))
            {
               if (OCR3A < max_OCR3A) OCR3A ++;
            }
            else if (Motor_X_Pos < Limit_Pos_HC)
            {
               if (OCR3A > min_OCR3A) OCR3A --;
            }
         }
      }
      else if (Motor_X_Pos == Hand_X_Pos)
      {             
      //
      }
   }

   else if (Motor_X_Dir == CCW)
   {
      if (Motor_X_Pos > Null_X_Pos + Hand_X_Pos)
      {
         Motor_X_InvertPulse();
         if (!Read_X_State) 
         {
            Motor_X_Pos --;
            if ((Motor_X_Pos < Limit_Pos_HC) || (hand_X == OFF))
            {
               if (OCR3A < max_OCR3A) OCR3A ++;
            }
            else if (Motor_X_Pos > Limit_Pos_HC)
            {
               if (OCR3A > min_OCR3A) OCR3A --;
            }
         }
      }
      else if (Motor_X_Pos == Hand_X_Pos)
      {             
      //
      }
   }
}


// ***** End ***** ///////////////////////////////////////////////////
void Read_ADC_Feed()
{
   /////////// Feed Variable ///////////
   // 
   if (Mode == Mode_Feed || Mode == Mode_Cone_L || Mode == Mode_Cone_R || Mode == Mode_aFeed || Mode == Mode_Sphere)
   {
      int New_ADC_Feed = analogRead(A7);
      // only read new value if it differs significantly from last cycle 
      if (New_ADC_Feed > ADC_Feed +4 || New_ADC_Feed < ADC_Feed -4)
      {
         // first 16 cycles ADC_Feed value increases unintendedly? At the end ADC_Feed is the arithmetical average of the last 16 changed values
         if (++x > 15) {x = 0;}
         Sum_ADC = Sum_ADC - ADC_Array[x];
         ADC_Array[x] = New_ADC_Feed;
         Sum_ADC = Sum_ADC + New_ADC_Feed;
         ADC_Feed = Sum_ADC /16;
      }
   }
// != Mode_aFeed
   if (Mode == Mode_Feed || Mode == Mode_Cone_L || Mode == Mode_Cone_R || Mode == Mode_Sphere)
   {
      uint16_t Feed_mm_New = MAX_FEED - long(MAX_FEED - MIN_FEED + 1) * ADC_Feed / 1024;
      if (Feed_mm_New != Feed_mm)
      {
         Feed_mm = Feed_mm_New;
         Print();
         Beep();

         switch (Mode)
         {
            case Mode_Feed:    ////////////////////////////////
            if (Joy_Z_flag == ON && Button_Rapid != 0 && Step_Z_flag == ON)
            {
               b_flag = false;
               if (Motor_Z_Dir == ZCW) {Feed_Left(a_flag, b_flag);}
               else                   {Feed_Right(a_flag, b_flag);}
            }
            else if (Joy_X_flag == ON && Button_Rapid != 0 && Step_X_flag == ON)
            {
               b_flag = false;
               if (Motor_X_Dir == CW) {Feed_Front(a_flag, b_flag);}
               else                   {Feed_Rear(a_flag, b_flag);} 
            }
            break;

            case Mode_Cone_L:    ////////////////////////////////
            case Mode_Cone_R:    ////////////////////////////////
            if (Joy_Z_flag == ON && Button_Rapid != 0  && Step_Z_flag == ON)
            {
               b_flag = false;
               if (Motor_Z_Dir == ZCW) {Cone_Left(a_flag, b_flag);}
               else                   {Cone_Right(a_flag, b_flag);}
            }
            else if (Joy_X_flag == ON && Button_Rapid != 0 && Step_X_flag == ON)
            {
               b_flag = false;
               if (Motor_X_Dir == CW) {Feed_Front(a_flag, b_flag);}
               else                   {Feed_Rear(a_flag, b_flag);}
            }
            break;
         }
      }
   }

   else if (Mode == Mode_aFeed)
   {
      uint16_t aFeed_mm_New = MAX_aFEED/10 - long(MAX_aFEED/10 - MIN_aFEED/10 + 1) * ADC_Feed / 1024;
      aFeed_mm_New = (aFeed_mm_New * 10);
      if (aFeed_mm_New != aFeed_mm)
      {
         aFeed_mm = aFeed_mm_New;
         Print();
         Beep();

         switch (Mode)
         {
            case Mode_aFeed:     //////////////////////////////////
            if (Joy_Z_flag == ON && Button_Rapid != 0  && Step_Z_flag == ON)
            {
               b_flag = false;
               if (Motor_Z_Dir == ZCW) {aFeed_Left(a_flag, b_flag);}
               else                   {aFeed_Right(a_flag, b_flag);}
            }
            else if (Joy_X_flag == ON && Button_Rapid != 0  && Step_X_flag == ON)
            {
               b_flag = false;
               if (Motor_X_Dir == CW) {aFeed_Front(a_flag, b_flag);}
               else                   {aFeed_Rear(a_flag, b_flag);}
            }
            break;
         }
      } 
   } 
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Mode "Asynchronous Feed" ********** //////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void aFeed_Left(bool & a_flag, bool & b_flag)
{
   if (b_flag == true) return;
   a_flag = false;
   b_flag = true;

   Joy_X_flag = OFF;
   feed_X_flag = OFF;
   Start_Speed = (250000 / ((uint32_t)MIN_aFEED * MOTOR_Z_STEP_PER_REV * McSTEP_Z / ((uint32_t)60 * SCREW_Z / 100) * 2) - 1) /FEED_ACCEL;
   if (Motor_Z_Pos < Limit_Pos_Left - Start_Speed * 2)
   {
      aFeed_Divisor = 250000 / ((uint32_t)aFeed_mm * MOTOR_Z_STEP_PER_REV * McSTEP_Z / ((uint32_t)60 * SCREW_Z / 100) * 2) - 1;
      if (aFeed_Divisor < Start_Speed)
      {
         max_OCR4A = Start_Speed;
         if (Step_Z_flag == OFF)
         {
            OCR4A = Start_Speed;
         }
      }
      else
      {
         max_OCR4A = aFeed_Divisor;
         OCR4A = aFeed_Divisor;
      }
   }
   else
   {
      aFeed_Divisor = Start_Speed;
      max_OCR4A = Start_Speed;
      OCR4A = Start_Speed;
   }
   
   Brake_Compens = (max_OCR4A - aFeed_Divisor) +1;
   Limit_Pos = Limit_Pos_Left - Brake_Compens;

   Motor_Z_Dir = ZCW;
   Motor_Z_CW();
   if (Read_Z_Ena_State == false) Motor_Z_Enable();
   
   feed_Z_flag = ON;
   Joy_Z_flag = ON;
   Ena_INT_Z_aFeed();
}

/////////////////////////////////////////////
void aFeed_Right(bool & a_flag, bool & b_flag)
{
   if (b_flag == true) return;
   a_flag = false;
   b_flag = true;

   Joy_X_flag = OFF;
   feed_X_flag = OFF;
   Start_Speed = (250000 / ((uint32_t)MIN_aFEED * MOTOR_Z_STEP_PER_REV * McSTEP_Z / ((uint32_t)60 * SCREW_Z / 100) * 2) - 1) /FEED_ACCEL;
   if (Motor_Z_Pos > Limit_Pos_Right + Start_Speed * 2)
   {
      aFeed_Divisor = 250000 / ((uint32_t)aFeed_mm * MOTOR_Z_STEP_PER_REV * McSTEP_Z / ((uint32_t)60 * SCREW_Z / 100) * 2) - 1;
      if (aFeed_Divisor < Start_Speed)
      {
         max_OCR4A = Start_Speed;
         if (Step_Z_flag == OFF)
         {
            OCR4A = Start_Speed;
         }
      }
      else
      {
         max_OCR4A = aFeed_Divisor;
         OCR4A = aFeed_Divisor;
      }
   }
   else
   {
      aFeed_Divisor = Start_Speed;
      max_OCR4A = Start_Speed;
      OCR4A = Start_Speed;
   }
   
   Brake_Compens = (max_OCR4A - aFeed_Divisor) +1;
   Limit_Pos = Limit_Pos_Right + Brake_Compens;

   Motor_Z_Dir = ZCCW;
   Motor_Z_CCW();
   if (Read_Z_Ena_State == false) Motor_Z_Enable();
   
   feed_Z_flag = ON;
   Joy_Z_flag = ON;
   Ena_INT_Z_aFeed();
}

void aFeed_Front(bool & a_flag, bool & b_flag)
{
   if (b_flag == true) return;
   a_flag = false;
   b_flag = true;

   Joy_Z_flag = OFF;
   feed_Z_flag = OFF;
   Start_Speed = (250000 / ((uint32_t)MIN_aFEED * MOTOR_X_STEP_PER_REV * McSTEP_X / ((uint32_t)60 * SCREW_X / 100) * 2) - 1) /FEED_ACCEL;
   if (Motor_X_Pos < Limit_Pos_Front - Start_Speed * 2)
   {
      aFeed_Divisor = 250000 / ((uint32_t)aFeed_mm * MOTOR_X_STEP_PER_REV * McSTEP_X / ((uint32_t)60 * SCREW_X / 100) * 2) - 1;
      if (aFeed_Divisor < Start_Speed)
      {
         max_OCR4A = Start_Speed;
         if (Step_X_flag == OFF)
         {
            OCR4A = Start_Speed;
         }
      }
      else
      {
         max_OCR4A = aFeed_Divisor;
         OCR4A = aFeed_Divisor;
      }
   }
   else
   {
      aFeed_Divisor = Start_Speed;
      max_OCR4A = Start_Speed;
      OCR4A = Start_Speed;
   }
   
   Brake_Compens = (max_OCR4A - aFeed_Divisor) +1;
   Limit_Pos = Limit_Pos_Front - Brake_Compens;

   Motor_X_Dir = CW;
   Motor_X_CW();
   if (Read_X_Ena_State == false) Motor_X_Enable();
   
   feed_X_flag = ON;
   Joy_X_flag = ON;
   Ena_INT_X_aFeed();
}

void aFeed_Rear(bool & a_flag, bool & b_flag)
{
   if (b_flag == true) return;
   a_flag = false;
   b_flag = true;

   Joy_Z_flag = OFF;
   feed_Z_flag = OFF;
   Start_Speed = (250000 / ((uint32_t)MIN_aFEED * MOTOR_X_STEP_PER_REV * McSTEP_X / ((uint32_t)60 * SCREW_X / 100) * 2) - 1) /FEED_ACCEL;
   if (Motor_X_Pos > Limit_Pos_Rear + Start_Speed * 2)
   {
      aFeed_Divisor = 250000 / ((uint32_t)aFeed_mm * MOTOR_X_STEP_PER_REV * McSTEP_X / ((uint32_t)60 * SCREW_X / 100) * 2) - 1;
      if (aFeed_Divisor < Start_Speed)
      {
         max_OCR4A = Start_Speed;
         if (Step_X_flag == OFF)
         {
            OCR4A = Start_Speed;
         }
      }
      else
      {
         max_OCR4A = aFeed_Divisor;
         OCR4A = aFeed_Divisor;
      }
   }
   else
   {
      aFeed_Divisor = Start_Speed;
      max_OCR4A = Start_Speed;
      OCR4A = Start_Speed;
   }

   Brake_Compens = (max_OCR4A - aFeed_Divisor) +1;
   Limit_Pos = Limit_Pos_Rear + Brake_Compens;

   Motor_X_Dir = CCW;
   Motor_X_CCW();
   if (Read_X_Ena_State == false) Motor_X_Enable();
   
   feed_X_flag = ON;
   Joy_X_flag = ON;
   Ena_INT_X_aFeed();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
void aFeed_Ext_Left()
{
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void aFeed_Int_Left()
{
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void aFeed_Ext_Right()
{
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void aFeed_Int_Right()
{
  
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Beeper ********** //////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Beep()
{
   Beeper_On();
   _delay_ms(25);
   Beeper_Off();
}

void BeepBeep()
{
   Beeper_On();
   _delay_ms(25);
   Beeper_Off();   
   _delay_ms(200);
   Beeper_On();
   _delay_ms(25);
   Beeper_Off();
}

void Beep_Error()
{
   Beeper_On();
   _delay_ms(250);
   Beeper_Off();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Cone Mode ********** //////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Cone_Left(bool & a_flag, bool & b_flag)
{  
   if (b_flag == true) return;
   a_flag = false;
   b_flag = true;
   
   Cs_Divisor = Cone_Info[Cone_Step].Cs_Div;
   Cm_Divisor = Cone_Info[Cone_Step].Cm_Div;

   Start_Speed = ENC_LINE_PER_REV / ((uint32_t)MOTOR_Z_STEP_PER_REV * McSTEP_Z * MIN_FEED / SCREW_Z) / FEED_ACCEL /2;
   if (Motor_Z_Pos < Limit_Pos_Left - Start_Speed * 2)
   {
      Feed_Divisor = ENC_LINE_PER_REV / ((uint32_t)MOTOR_Z_STEP_PER_REV * McSTEP_Z * Feed_mm / SCREW_Z) /2;
      if (Feed_Divisor < Start_Speed)
      {
         max_OCR5A = Start_Speed;
         if (Step_Z_flag == OFF)
         {
            OCR5A = Start_Speed;
         }
      }
      else
      {
         max_OCR5A = Feed_Divisor;
         OCR5A = Feed_Divisor;
      }
   }
   else
   {
      Feed_Divisor = Start_Speed;
      max_OCR5A = Start_Speed;
      OCR5A = Start_Speed;
   }
   
   Brake_Compens = max_OCR5A - Feed_Divisor + 1;
   Limit_Pos = Limit_Pos_Left - Brake_Compens;

   Motor_Z_Dir = ZCW;
   Motor_Z_CW();
   if (Read_Z_Ena_State == false) Motor_Z_Enable();

   if      (Mode == Mode_Cone_L) {Motor_X_Dir = CW; Motor_X_CW();}
   else if (Mode == Mode_Cone_R) {Motor_X_Dir = CCW; Motor_X_CCW();}
   if (Read_X_Ena_State == false) Motor_X_Enable();
   
   feed_Z_flag = ON;
   Step_X_flag = ON;
   Joy_Z_flag = ON;
   Ena_INT_Z_Feed();
}

void Cone_Right(bool & a_flag, bool & b_flag)
{
   if (b_flag == true) return;
   a_flag = false;
   b_flag = true; 
   
   Cs_Divisor = Cone_Info[Cone_Step].Cs_Div;
   Cm_Divisor = Cone_Info[Cone_Step].Cm_Div;

   Start_Speed = ENC_LINE_PER_REV / ((uint32_t)MOTOR_Z_STEP_PER_REV * McSTEP_Z * MIN_FEED / SCREW_Z) / FEED_ACCEL /2;
   if (Motor_Z_Pos > Limit_Pos_Right + Start_Speed * 2)
   {
      Feed_Divisor = ENC_LINE_PER_REV / ((uint32_t)MOTOR_Z_STEP_PER_REV * McSTEP_Z * Feed_mm / SCREW_Z) /2;
      if (Feed_Divisor < Start_Speed)
      {
         max_OCR5A = Start_Speed;
         if (Step_Z_flag == OFF)
         {
            OCR5A = Start_Speed;
         }
      }
      else
      {
         max_OCR5A = Feed_Divisor;
         OCR5A = Feed_Divisor;
      }
   }
   else
   {
      Feed_Divisor = Start_Speed;
      max_OCR5A = Start_Speed;
      OCR5A = Start_Speed;
   }
   
   Brake_Compens = max_OCR5A - Feed_Divisor + 1;
   Limit_Pos = Limit_Pos_Right + Brake_Compens;

   Motor_Z_Dir = ZCCW;
   Motor_Z_CCW();
   if (Read_Z_Ena_State == false) Motor_Z_Enable();

   if      (Mode == Mode_Cone_L) {Motor_X_Dir = CCW; Motor_X_CCW();}
   else if (Mode == Mode_Cone_R) {Motor_X_Dir = CW; Motor_X_CW();}
   if (Read_X_Ena_State == false) Motor_X_Enable();
   
   feed_Z_flag = ON;
   Step_X_flag = ON;
   Joy_Z_flag = ON;
   Ena_INT_Z_Feed();
}

void Cone_Front(bool & a_flag, bool & b_flag)
{
   if (b_flag == true) return;
   a_flag = false;
   b_flag = true;
}

void Cone_Rear(bool & a_flag, bool & b_flag)
{
   if (b_flag == true) return;
   a_flag = false;
   b_flag = true; 
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** "Cone! Fast Feed" mode ********** ///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Rapid_Cone_Left(bool & a_flag, bool & b_flag)
{
   if (a_flag == true) return;
   b_flag = false;
   a_flag = true;
   
   Brake_Compens = (MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt + 1;
   Limit_Pos = Limit_Pos_Left - Brake_Compens;
   
   Cs_Count = 0;
   Cm_Count = 0;            

   Motor_Z_Dir = ZCW;
   Motor_Z_CW();
   if (Read_Z_Ena_State == false) Motor_Z_Enable();

   if      (Mode == Mode_Cone_L) {Motor_X_Dir = CW; Motor_X_CW();}
   else if (Mode == Mode_Cone_R) {Motor_X_Dir = CCW; Motor_X_CCW();}
   if (Read_X_Ena_State == false) Motor_X_Enable();
   
   Joy_Z_flag = ON;
   rapid_Z_flag = ON;
   Step_X_flag = ON;
   Ena_INT_Z_Rapid();
}

void Rapid_Cone_Right(bool & a_flag, bool & b_flag)
{
   if (a_flag == true) return;
   b_flag = false;
   a_flag = true;
   
   Brake_Compens = (MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt + 1;
   Limit_Pos = Limit_Pos_Right + Brake_Compens;
   
   Cs_Count = 0;
   Cm_Count = 0;            

   Motor_Z_Dir = ZCCW;
   Motor_Z_CCW();
   if (Read_Z_Ena_State == false) Motor_Z_Enable();

   if      (Mode == Mode_Cone_L) {Motor_X_Dir = CCW; Motor_X_CCW();}
   else if (Mode == Mode_Cone_R) {Motor_X_Dir = CW; Motor_X_CW();}
   if (Read_X_Ena_State == false) Motor_X_Enable();
   
   Joy_Z_flag = ON;
   rapid_Z_flag = ON;
   Step_X_flag = ON;
   Ena_INT_Z_Rapid();  
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Feed mode ********** //////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Feed_Left(bool & a_flag, bool & b_flag)
{
   if (b_flag == true) return;
   a_flag = false;
   b_flag = true;

   Joy_X_flag = OFF;
   feed_X_flag = OFF;
   Start_Speed = ENC_LINE_PER_REV / ((float)MOTOR_Z_STEP_PER_REV * McSTEP_Z * MIN_FEED / SCREW_Z) / FEED_ACCEL /2 +0.5;
   if (Motor_Z_Pos < Limit_Pos_Left - Start_Speed * 2)
   {
      Feed_Divisor = ENC_LINE_PER_REV / ((float)MOTOR_Z_STEP_PER_REV * McSTEP_Z * Feed_mm / SCREW_Z) /2 +0.5;
      if (Feed_Divisor < Start_Speed)
      {
         max_OCR5A = Start_Speed;
         if (Step_Z_flag == OFF)
         {
            OCR5A = Start_Speed;
         }
      }
      else
      {
         max_OCR5A = Feed_Divisor;
         OCR5A = Feed_Divisor;
      }
   }
   else
   {
      Feed_Divisor = Start_Speed;
      max_OCR5A = Start_Speed;
      OCR5A = Start_Speed;
   }
   
   Brake_Compens = max_OCR5A - Feed_Divisor + 1;
   Limit_Pos = Limit_Pos_Left - Brake_Compens;

   Motor_Z_Dir = ZCW;
   Motor_Z_CW();
   if (Read_Z_Ena_State == false) Motor_Z_Enable();
   
   feed_Z_flag = ON;
   Joy_Z_flag = ON;
   Ena_INT_Z_Feed();
}

void Feed_Right(bool & a_flag, bool & b_flag)
{
   if (b_flag == true) return;
   a_flag = false;
   b_flag = true;

   Joy_X_flag = OFF;
   feed_X_flag = OFF;
   Start_Speed = ENC_LINE_PER_REV / ((float)MOTOR_Z_STEP_PER_REV * McSTEP_Z * MIN_FEED / SCREW_Z) / FEED_ACCEL /2 +0.5;
   if (Motor_Z_Pos > Limit_Pos_Right + Start_Speed * 2)
   {
      Feed_Divisor = ENC_LINE_PER_REV / ((float)MOTOR_Z_STEP_PER_REV * McSTEP_Z * Feed_mm / SCREW_Z) /2 +0.5;
      if (Feed_Divisor < Start_Speed)
      {
         max_OCR5A = Start_Speed;
         if (Step_Z_flag == OFF)
         {
            OCR5A = Start_Speed;
         }
      }
      else
      {
         max_OCR5A = Feed_Divisor;
         OCR5A = Feed_Divisor;
      }
   }
   else
   {
      Feed_Divisor = Start_Speed;
      max_OCR5A = Start_Speed;
      OCR5A = Start_Speed;
   }
   
   Brake_Compens = (max_OCR5A - Feed_Divisor) + 1;
   Limit_Pos = Limit_Pos_Right + Brake_Compens;

   Motor_Z_Dir = ZCCW;
   Motor_Z_CCW();
   if (Read_Z_Ena_State == false) Motor_Z_Enable();
   
   feed_Z_flag = ON;
   Joy_Z_flag = ON;
   Ena_INT_Z_Feed();
}

void Feed_Front(bool & a_flag, bool & b_flag)
{  
   if (b_flag == true) return;
   a_flag = false;
   b_flag = true;

   Joy_Z_flag = OFF;
   feed_Z_flag = OFF;
   Start_Speed = ENC_LINE_PER_REV / ((float)MOTOR_X_STEP_PER_REV * McSTEP_X * MIN_FEED / SCREW_X) / FEED_ACCEL /2 +0.5;
   if (Motor_X_Pos < Limit_Pos_Front - Start_Speed * 2)
   {
      Feed_Divisor = ENC_LINE_PER_REV / ((float)MOTOR_X_STEP_PER_REV * McSTEP_X * Feed_mm / SCREW_X) /2 +0.5;
      if (Feed_Divisor < Start_Speed)
      {
         max_OCR5A = Start_Speed;
         if (Step_X_flag == OFF)
         {
            OCR5A = Start_Speed;
         }
      }
      else
      {
         max_OCR5A = Feed_Divisor;
         OCR5A = Feed_Divisor;
      }
   }
   else
   {
      Feed_Divisor = Start_Speed;
      max_OCR5A = Start_Speed;
      OCR5A = Start_Speed;
   }
   
   Brake_Compens = max_OCR5A - Feed_Divisor + 1;
   Limit_Pos = Limit_Pos_Front - Brake_Compens;

   Motor_X_Dir = CW;
   Motor_X_CW();
   if (Read_X_Ena_State == false) Motor_X_Enable();
   
   feed_X_flag = ON;
   Joy_X_flag = ON;
   Ena_INT_X_Feed();
   
}

void Feed_Rear(bool & a_flag, bool & b_flag)
{
   if (b_flag == true) return;
   a_flag = false;
   b_flag = true;

   Joy_Z_flag = OFF;
   feed_Z_flag = OFF;
   Start_Speed = ENC_LINE_PER_REV / ((float)MOTOR_X_STEP_PER_REV * McSTEP_X * MIN_FEED / SCREW_X) / FEED_ACCEL /2 +0.5;
   if (Motor_X_Pos > Limit_Pos_Rear + Start_Speed * 2)
   {
      Feed_Divisor = ENC_LINE_PER_REV / ((float)MOTOR_X_STEP_PER_REV * McSTEP_X * Feed_mm / SCREW_X) /2 +0.5;
      if (Feed_Divisor < Start_Speed)
      {
         max_OCR5A = Start_Speed;
         if (Step_X_flag == OFF)
         {
            OCR5A = Start_Speed;
         }
      }
      else
      {
         max_OCR5A = Feed_Divisor;
         OCR5A = Feed_Divisor;
      }
   }
   else
   {
      Feed_Divisor = Start_Speed;
      max_OCR5A = Start_Speed;
      OCR5A = Start_Speed;
   }

   Brake_Compens = max_OCR5A - Feed_Divisor + 1;
   Limit_Pos = Limit_Pos_Rear + Brake_Compens;

   Motor_X_Dir = CCW;
   Motor_X_CCW();
   if (Read_X_Ena_State == false) Motor_X_Enable();
   
   feed_X_flag = ON;
   Joy_X_flag = ON;
   Ena_INT_X_Feed();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Mode "Quick Feed" ********** //////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Rapid_Feed_Left(bool & a_flag, bool & b_flag)
{
   if (a_flag == true) return;
   b_flag = false;
   a_flag = true;

   Joy_X_flag = OFF;
   feed_X_flag = OFF;
   rapid_X_flag = OFF;
   Brake_Compens = (MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt + 1;
   Limit_Pos = Limit_Pos_Left - Brake_Compens;

   Motor_Z_Dir = ZCW;
   Motor_Z_CW();
   if (Read_Z_Ena_State == false) Motor_Z_Enable();
   
   Joy_Z_flag = ON;             
   rapid_Z_flag = ON;
   Ena_INT_Z_Rapid();
}

void Rapid_Feed_Right(bool & a_flag, bool & b_flag)
{
   if (a_flag == true) return;
   b_flag = false;
   a_flag = true;

   Joy_X_flag = OFF;
   feed_X_flag = OFF;
   rapid_X_flag = OFF;
   Brake_Compens = (MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt + 1;
   Limit_Pos = Limit_Pos_Right + Brake_Compens;

   Motor_Z_Dir = ZCCW;
   Motor_Z_CCW();
   if (Read_Z_Ena_State == false) Motor_Z_Enable();
   
   Joy_Z_flag = ON;
   rapid_Z_flag = ON;
   Ena_INT_Z_Rapid();
}

void Rapid_Feed_Front(bool & a_flag, bool & b_flag)
{
   if (a_flag == true) return;
   b_flag = false;
   a_flag = true;

   Joy_Z_flag = OFF;
   feed_Z_flag = OFF;
   rapid_Z_flag = OFF;
   Brake_Compens = (MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt + 1;
   Limit_Pos = Limit_Pos_Front - Brake_Compens;

   Motor_X_Dir = CW;
   Motor_X_CW();
   if (Read_X_Ena_State == false) Motor_X_Enable();
   
   Joy_X_flag = ON;
   rapid_X_flag = ON;
   Ena_INT_X_Rapid();
}

void Rapid_Feed_Rear(bool & a_flag, bool & b_flag)
{
   if (a_flag == true) return;
   b_flag = false;
   a_flag = true;

   Joy_Z_flag = OFF;
   feed_Z_flag = OFF;
   rapid_Z_flag = OFF;
   Brake_Compens = (MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt + 1;
   Limit_Pos = Limit_Pos_Rear + Brake_Compens;

   Motor_X_Dir = CCW;
   Motor_X_CCW();
   if (Read_X_Ena_State == false) Motor_X_Enable();
   
   Joy_X_flag = ON;
   rapid_X_flag = ON;
   Ena_INT_X_Rapid();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Mode "Cycle Feed" ********** /////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Feed_Ext_Left()
{
   if ((Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Rear && Step_Z_flag == OFF) ||
       (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Null_X_Pos && Step_Z_flag == OFF))
   {  
      if (cycle_flag == false && Pass_Nr <= Pass_Total)
      {
         cycle_flag = true;
         a_flag = false;
         b_flag = false;

         long Infeed_Value = (int)((float)MOTOR_X_STEP_PER_REV * Ap / SCREW_X + 0.5) * McSTEP_X;
         if (Infeed_Value == 0)
         {
            if (Pass_Nr == 1) Limit_Pos_Front = (Null_X_Pos + 1);
            else              Limit_Pos_Front = (Limit_Pos_Rear + REBOUND_X);
         }
         else
         {
            if (Pass_Nr == 1) Limit_Pos_Front = (Null_X_Pos + Infeed_Value);
            else              Limit_Pos_Front = (Limit_Pos_Rear + REBOUND_X + Infeed_Value);
         }
         Limit_Front_LED_On();
         BeepBeep();
         Feed_Front(a_flag, b_flag);
      }
      
      else if (cycle_flag == false && Pass_Nr > Pass_Total)
      {
         a_flag = false;
         b_flag = false;

         Limit_Pos_Front = Limit_Pos_Rear + REBOUND_X ;
         Feed_Front(a_flag, b_flag);

         Limit_Front_LED_Off();
         Limit_Rear_LED_Off();
         Limit_Pos_Front = Limit_Pos_Max;
         Limit_Pos_Rear = Limit_Pos_Min;
         Pass_Total = 1;
         Pass_Nr = 1;
         Print();
      }
   }

   else if (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Front && Step_X_flag == OFF)
   {
      if (Pass_Nr <= Pass_Total)
      {
         a_flag = false;
         b_flag = false;

         Feed_Left(a_flag, b_flag);
      }
   }

   else if (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Front && Step_Z_flag == OFF)
   {
      cycle_flag = false;
      a_flag = false;
      b_flag = false;
      
      Limit_Pos_Rear = (Limit_Pos_Front - REBOUND_X);
      Limit_Rear_LED_On();
      Feed_Rear(a_flag, b_flag);
   }

   else if (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Rear && Step_X_flag == OFF)
   {
      a_flag = false;
      b_flag = false;

      Pass_Nr++;
      Print();
      Rapid_Feed_Right(a_flag, b_flag);
   }
}

////////////////////////////////////////////////////////////////////////////////////////////////
void Feed_Ext_Right()
{
   if ((Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Rear && Step_Z_flag == OFF) ||
       (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Null_X_Pos && Step_Z_flag == OFF))
   {  
      if (cycle_flag == false && Pass_Nr <= Pass_Total)
      {
         cycle_flag = true;
         a_flag = false;
         b_flag = false;
         
         long Infeed_Value = (int)((float)MOTOR_X_STEP_PER_REV * Ap / SCREW_X + 0.5) * McSTEP_X;
         if (Infeed_Value == 0)
         {
            if (Pass_Nr == 1) Limit_Pos_Front = (Null_X_Pos + 1);
            else              Limit_Pos_Front = (Limit_Pos_Rear + REBOUND_X);
         }
         else
         {
            if (Pass_Nr == 1) Limit_Pos_Front = (Null_X_Pos + Infeed_Value);
            else              Limit_Pos_Front = (Limit_Pos_Rear + REBOUND_X + Infeed_Value);
         }
         Limit_Front_LED_On();
         BeepBeep();
         Feed_Front(a_flag, b_flag);
      }

      else if (cycle_flag == false && Pass_Nr > Pass_Total)
      {
         a_flag = false;
         b_flag = false;
         
         Limit_Pos_Front = Limit_Pos_Rear + REBOUND_X;
         Feed_Front(a_flag, b_flag);

         Limit_Front_LED_Off();
         Limit_Rear_LED_Off();
         Limit_Pos_Front = Limit_Pos_Max;
         Limit_Pos_Rear = Limit_Pos_Min;
         Pass_Total = 1;
         Pass_Nr = 1;
         Print();
      }
   }

   else if (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Front && Step_X_flag == OFF)  
   {
      if (Pass_Nr <= Pass_Total)
      {
         a_flag = false;
         b_flag = false;

         Feed_Right(a_flag, b_flag);
      }
   }

   else if (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Front && Step_Z_flag == OFF)
   {
      cycle_flag = false;
      a_flag = false;
      b_flag = false;
      
      Limit_Pos_Rear = (Limit_Pos_Front - REBOUND_X);
      Limit_Rear_LED_On();
      Feed_Rear(a_flag, b_flag);
   }

   else if (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Rear && Step_X_flag == OFF)
   {
      a_flag = false;
      b_flag = false;

      Pass_Nr++;
      Print();
      Rapid_Feed_Left(a_flag, b_flag);
   }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void Feed_Int_Left()
{
   if ((Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Front && Step_Z_flag == OFF) ||
       (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Null_X_Pos && Step_Z_flag == OFF))
   {  
      if (cycle_flag == false && Pass_Nr <= Pass_Total)
      {
         cycle_flag = true;
         a_flag = false;
         b_flag = false;

         long Infeed_Value = (int)((float)MOTOR_X_STEP_PER_REV * Ap / SCREW_X + 0.5) * McSTEP_X;
         if (Infeed_Value == 0)
         {
            if (Pass_Nr == 1) Limit_Pos_Rear = (Null_X_Pos - 1);
            else              Limit_Pos_Rear = (Limit_Pos_Front - REBOUND_X);
         }
         else
         {
            if (Pass_Nr == 1) Limit_Pos_Rear = (Null_X_Pos - Infeed_Value);
            else              Limit_Pos_Rear = (Limit_Pos_Front - REBOUND_X - Infeed_Value);
         }
         Limit_Rear_LED_On();
         BeepBeep();
         Feed_Rear(a_flag, b_flag);
      }
      
      else if (cycle_flag == false && Pass_Nr > Pass_Total)
      {
         a_flag = false;
         b_flag = false;
         
         Limit_Pos_Rear = Limit_Pos_Front - REBOUND_X;
         Feed_Rear(a_flag, b_flag);

         Limit_Front_LED_Off();
         Limit_Rear_LED_Off();
         Limit_Pos_Front = Limit_Pos_Max;
         Limit_Pos_Rear = Limit_Pos_Min;
         Pass_Total = 1;
         Pass_Nr = 1;
         Print();
      }
   }

   else if (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Rear && Step_X_flag == OFF)
   {
      if (Pass_Nr <= Pass_Total)
      {
         a_flag = false;
         b_flag = false;

         Feed_Left(a_flag, b_flag);
      }
   }

   else if (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Rear && Step_Z_flag == OFF)
   {
      cycle_flag = false;
      a_flag = false;
      b_flag = false;
      
      Limit_Pos_Front = (Limit_Pos_Rear + REBOUND_X);
      Limit_Front_LED_On();
      Feed_Front(a_flag, b_flag);
   }

   else if (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Front && Step_X_flag == OFF)
   {
      a_flag = false;
      b_flag = false;

      Pass_Nr++;
      Print();
      Rapid_Feed_Right(a_flag, b_flag);
   }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void Feed_Int_Right()
{
   if ((Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Front && Step_Z_flag == OFF) ||
       (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Null_X_Pos && Step_Z_flag == OFF))
   {  
      if (cycle_flag == false && Pass_Nr <= Pass_Total)
      {
         cycle_flag = true;
         a_flag = false;
         b_flag = false;
         
         long Infeed_Value = (int)((float)MOTOR_X_STEP_PER_REV * Ap / SCREW_X + 0.5) * McSTEP_X;
         if (Infeed_Value == 0)
         {
            if (Pass_Nr == 1) Limit_Pos_Rear = (Null_X_Pos - 1);
            else              Limit_Pos_Rear = (Limit_Pos_Front - REBOUND_X);
         }
         else
         {
            if (Pass_Nr == 1) Limit_Pos_Rear = (Null_X_Pos - Infeed_Value);
            else              Limit_Pos_Rear = (Limit_Pos_Front - REBOUND_X - Infeed_Value);
         }
         Limit_Rear_LED_On();
         BeepBeep();
         Feed_Rear(a_flag, b_flag);
      }

      else if (cycle_flag == false && Pass_Nr > Pass_Total)
      {
         a_flag = false;
         b_flag = false;
         
         Limit_Pos_Rear = Limit_Pos_Front - REBOUND_X;
         Feed_Rear(a_flag, b_flag);

         Limit_Front_LED_Off();
         Limit_Rear_LED_Off();
         Limit_Pos_Front = Limit_Pos_Max;
         Limit_Pos_Rear = Limit_Pos_Min;
         Pass_Total = 1;
         Pass_Nr = 1;
         Print();
      }
   }

   else if (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Rear && Step_X_flag == OFF)
   {
      if (Pass_Nr <= Pass_Total)
      {
         a_flag = false;
         b_flag = false;

         Feed_Right(a_flag, b_flag);
      }
   }

   else if (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Rear && Step_Z_flag == OFF)
   {
      cycle_flag = false;
      a_flag = false;
      b_flag = false;
      
      Limit_Pos_Front = (Limit_Pos_Rear + REBOUND_X);
      Limit_Front_LED_On();
      Feed_Front(a_flag, b_flag);
   }

   else if (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Front && Step_X_flag == OFF)
   {
      a_flag = false;
      b_flag = false;

      Pass_Nr++;
      Print();
      Rapid_Feed_Left(a_flag, b_flag);
   }
}

///////////////////////////////////////////////////////////////////////////////////////////////
void Feed_Ext_Front()
{
   if ((Motor_X_Pos == Limit_Pos_Rear && Motor_Z_Pos == Limit_Pos_Right && Step_X_flag == OFF) ||
       (Motor_X_Pos == Limit_Pos_Rear && Motor_Z_Pos == Null_Z_Pos && Step_X_flag == OFF))
   {  
      if (cycle_flag == false && Pass_Nr <= Pass_Total)
      {
         cycle_flag = true;
         a_flag = false;
         b_flag = false;

         long Infeed_Value = (int)((float)MOTOR_Z_STEP_PER_REV * Ap / SCREW_Z + 0.5) * McSTEP_Z;
         if (Infeed_Value == 0)
         {
            if (Pass_Nr == 1) Limit_Pos_Left = (Null_Z_Pos + 1);
            else              Limit_Pos_Left = (Limit_Pos_Right + REBOUND_Z);
         }
         else
         {
            if (Pass_Nr == 1) Limit_Pos_Left = (Null_Z_Pos + Infeed_Value);
            else              Limit_Pos_Left = (Limit_Pos_Right + REBOUND_Z + Infeed_Value);
         }
         Limit_Left_LED_On();
         BeepBeep();
         Feed_Left(a_flag, b_flag);
      }
      
      else if (cycle_flag == false && Pass_Nr > Pass_Total)
      {
         a_flag = false;
         b_flag = false;
         
         Limit_Pos_Left = Limit_Pos_Right + REBOUND_Z;
         Feed_Left(a_flag, b_flag);

         Limit_Left_LED_Off();
         Limit_Right_LED_Off();
         Limit_Pos_Left = Limit_Pos_Max;
         Limit_Pos_Right = Limit_Pos_Min;
         Pass_Total = 1;
         Pass_Nr = 1;
         Print();
      }
   }

   else if (Motor_X_Pos == Limit_Pos_Rear && Motor_Z_Pos == Limit_Pos_Left && Step_Z_flag == OFF)
   {
      if (Pass_Nr <= Pass_Total)
      {
         a_flag = false;
         b_flag = false;

         Feed_Front(a_flag, b_flag);
      }
   }

   else if (Motor_X_Pos == Limit_Pos_Front && Motor_Z_Pos == Limit_Pos_Left && Step_X_flag == OFF)
   {
      cycle_flag = false;
      a_flag = false;
      b_flag = false;
      
      Limit_Pos_Right = (Limit_Pos_Left - REBOUND_Z);
      Limit_Right_LED_On();
      Feed_Right(a_flag, b_flag);
   }

   else if (Motor_X_Pos == Limit_Pos_Front && Motor_Z_Pos == Limit_Pos_Right && Step_Z_flag == OFF)
   {
      a_flag = false;
      b_flag = false;

      Pass_Nr++;
      Print();
      Rapid_Feed_Rear(a_flag, b_flag);
   }
}

//////////////////////////////////////////////////////////////////////////////
void Feed_Ext_Rear()
{
   if ((Motor_X_Pos == Limit_Pos_Front && Motor_Z_Pos == Limit_Pos_Right && Step_X_flag == OFF) ||
       (Motor_X_Pos == Limit_Pos_Front && Motor_Z_Pos == Null_Z_Pos && Step_X_flag == OFF))
   {  
      if (cycle_flag == false && Pass_Nr <= Pass_Total)
      {
         cycle_flag = true;
         a_flag = false;
         b_flag = false;

         long Infeed_Value = (int)((float)MOTOR_Z_STEP_PER_REV * Ap / SCREW_Z + 0.5) * McSTEP_Z;
         if (Infeed_Value == 0)
         {
            if (Pass_Nr == 1) Limit_Pos_Left = (Null_Z_Pos + 1);
            else              Limit_Pos_Left = (Limit_Pos_Right + REBOUND_Z);
         }
         else
         {
            if (Pass_Nr == 1) Limit_Pos_Left = (Null_Z_Pos + Infeed_Value);
            else              Limit_Pos_Left = (Limit_Pos_Right + REBOUND_Z + Infeed_Value);
         }
         Limit_Left_LED_On();
         BeepBeep();
         Feed_Left(a_flag, b_flag);
      }
      
      else if (cycle_flag == false && Pass_Nr > Pass_Total)
      {
         a_flag = false;
         b_flag = false;
         
         Limit_Pos_Left = Limit_Pos_Right + REBOUND_Z;
         Feed_Left(a_flag, b_flag);

         Limit_Left_LED_Off();
         Limit_Right_LED_Off();
         Limit_Pos_Left = Limit_Pos_Max;
         Limit_Pos_Right = Limit_Pos_Min;
         Pass_Total = 1;
         Pass_Nr = 1;
         Print();
      }
   }

   else if (Motor_X_Pos == Limit_Pos_Front && Motor_Z_Pos == Limit_Pos_Left && Step_Z_flag == OFF)
   {
      if (Pass_Nr <= Pass_Total)
      {
         a_flag = false;
         b_flag = false;

         Feed_Rear(a_flag, b_flag);
      }
   }

   else if (Motor_X_Pos == Limit_Pos_Rear && Motor_Z_Pos == Limit_Pos_Left && Step_X_flag == OFF)
   {
      cycle_flag = false;
      a_flag = false;
      b_flag = false;
      
      Limit_Pos_Right = (Limit_Pos_Left - REBOUND_Z);
      Limit_Right_LED_On();
      Feed_Right(a_flag, b_flag);
   }

   else if (Motor_X_Pos == Limit_Pos_Rear && Motor_Z_Pos == Limit_Pos_Right && Step_Z_flag == OFF)
   {
      a_flag = false;
      b_flag = false;

      Pass_Nr++;
      Print();
      Rapid_Feed_Front(a_flag, b_flag);
   }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void Feed_Int_Front()
{
//
}

////////////////////////////////////////////////////////////////////////////////////////////////
void Feed_Int_Rear() 
{
//
}
void H_Coder()
{
   ///////////////
   // Scale
   ///////////////
   if (Motor_Z_Pos == Null_Z_Pos + Hand_Z_Pos && Motor_X_Pos == Null_X_Pos + Hand_X_Pos)
   {
      if (Scale == HC_SCALE_10 && flag_Scale_x1 == ON)
      { 
         Hand_Count = 0;
         Hand_Count_New = 0;
         Hand_Count_Old = 0;
         Hand_Z_Pos = 0;
         Hand_X_Pos = 0;

         Null_Z_Pos = Motor_Z_Pos;
         Null_X_Pos = Motor_X_Pos;

         max_OCR3A = HC_START_SPEED_1;
         min_OCR3A = HC_MAX_SPEED_1;
         
         Scale = HC_SCALE_1;
      }
      else if (Scale == HC_SCALE_1 && flag_Scale_x10 == ON)
      {
         Hand_Count = 0;
         Hand_Count_New = 0;
         Hand_Count_Old = 0;
         Hand_Z_Pos = 0;
         Hand_X_Pos = 0;

         Null_Z_Pos = Motor_Z_Pos;
         Null_X_Pos = Motor_X_Pos;

         max_OCR3A = HC_START_SPEED_10;
         min_OCR3A = HC_MAX_SPEED_10;
         
         Scale = HC_SCALE_10;
      }
   }
   
   /////////////////
   // active axis
   /////////////////
   /////////////////
   if (hand_Z == ON)
   {
      Disa_INT_Hcoder();
      //Hand_Count_New = Hand_Count;
      if (HC_Z_DIR == 0) {Hand_Count_New = Hand_Count;}
      else               {Hand_Count_New = Hand_Count - Hand_Count *2;}
      Ena_INT_Hcoder();
      
      if (Hand_Count_New != Hand_Count_Old)
      {
         Hand_Count_Old = Hand_Count_New;

         Hand_Z_Pos = (Hand_Count_New * Scale * MOTOR_Z_STEP_PER_REV * McSTEP_Z / SCREW_Z +McSTEP_Z /2)  & ~(McSTEP_Z - 1);
         Brake_Compens = max_OCR3A - min_OCR3A +1;
         Disable_INT_OCR3B();
         Enable_INT_OCR3A();
      } 


      if (Motor_Z_Pos < Null_Z_Pos + Hand_Z_Pos)
      {
         Motor_Z_Dir = ZCW;
         Motor_Z_CW();
         Limit_Pos_HC = Null_Z_Pos + Hand_Z_Pos - Brake_Compens;
         Limit_Pos = Limit_Pos_Left - Brake_Compens;
      }
      else if (Motor_Z_Pos > Null_Z_Pos + Hand_Z_Pos)
      {
         Motor_Z_Dir = ZCCW;
         Motor_Z_CCW();
         Limit_Pos_HC = Null_Z_Pos + Hand_Z_Pos + Brake_Compens;
         Limit_Pos = Limit_Pos_Right + Brake_Compens;
      }
   }

   //////////////////////////
   else if (hand_X == ON)
   {
      Disa_INT_Hcoder();
      if (HC_X_DIR == 0) {Hand_Count_New = Hand_Count;}
      else               {Hand_Count_New = Hand_Count - Hand_Count *2;}
      Ena_INT_Hcoder();
      
      if (Hand_Count_New != Hand_Count_Old)
      {
         Hand_Count_Old = Hand_Count_New;

         Hand_X_Pos = (Hand_Count_New * Scale * MOTOR_X_STEP_PER_REV * McSTEP_X / SCREW_X +McSTEP_X /2)  & ~(McSTEP_X - 1);
         Brake_Compens = max_OCR3A - min_OCR3A +1;
         Disable_INT_OCR3A();
         Enable_INT_OCR3B();
      } 


      if (Motor_X_Pos < Null_X_Pos + Hand_X_Pos)
      {
         Motor_X_Dir = CW;
         Motor_X_CW();
         Limit_Pos_HC = Null_X_Pos + Hand_X_Pos - Brake_Compens;
      }
      else if (Motor_X_Pos > Null_X_Pos + Hand_X_Pos)
      {
         Motor_X_Dir = CCW;
         Motor_X_CCW();
         Limit_Pos_HC = Null_X_Pos + Hand_X_Pos + Brake_Compens;
      }
   }
      
   /////////////////////////////////////////////////
   else if (hand_Z == OFF && hand_X == OFF)
   {
     //
   }
}
void Menu()
{
   /////// Joystick ///////////////////////////////////////////////
   byte Joy_New = Joy_Read;
   {
      if      (Joy_New == B00001110) Joy_LeftPressed();
      else if (Joy_New == B00001101) Joy_RightPressed();
      else if (Joy_New == B00001011) Joy_UpPressed();
      else if (Joy_New == B00000111) Joy_DownPressed();
      else if (Joy_New == B00001111) Joy_NoPressed();
   }

      
   /////// Mode Switch ////////////////////////////////
   if (!Joy_Z_flag && !Joy_X_flag)
   {
      byte Mode_New = Mode_Read;
      if (Mode_New != Mode_Old)
      {
         if      (Mode_New == B01111111) {Switch_Thread();}
         else if (Mode_New == B10111111) {Switch_Feed();}
         else if (Mode_New == B11011111) {Switch_aFeed();}
         else if (Mode_New == B11101111) {Switch_Cone_L();}
         else if (Mode_New == B11110111) {Switch_Cone_R();}
         else if (Mode_New == B11111011) {Switch_Reserve();}
         else if (Mode_New == B11111101) {Switch_Sphere();}
         else if (Mode_New == B11111110) {Switch_Divider();}
         Mode_Old = Mode_New;
      }
   }


   /////// Sub Mode Switch //////////////////////////////
   if (!Joy_Z_flag && !Joy_X_flag)
   {
      byte Submode_New = Submode_Read;
      if (Submode_New != Submode_Old)
      {
         if      (Submode_New == B11000000) Switch_Int();
         else if (Submode_New == B10100000) Switch_Man();
         else if (Submode_New == B01100000) Switch_Ext();
         Submode_Old = Submode_New;
      }
   }

    
   /////////// Menu Buttons //////////////////////////////////////
   byte Button_Sel_New = Button_Sel_Read;
   if (Button_Sel_New == Button_Sel_Old)
   {
      if (!Button_Sel_Read) {Key_Select_Pressed();}
      else                  {key_sel_flag = false; Print();}
   }
   Button_Sel_Old = Button_Sel_New;

   byte Button_New = Buttons_Read;
   if (Button_New == Button_Old)
   {
      if      (Button_New == B00000111) Key_Down_Pressed();
      else if (Button_New == B00001011) Key_Up_Pressed();
      else if (Button_New == B00001101) Key_Right_Pressed();
      else if (Button_New == B00001110) Key_Left_Pressed();
      else     button_flag = false;
   }
   Button_Old = Button_New;


   /////// Limit Buttons ///////////////////////////////////////
   byte Limit_Button_New = Limit_Buttons_Read;
   if (Limit_Button_New == Limit_Button_Old)
   {
      if      (Limit_Button_New == B00010101) Limit_Left_Pressed();
      else if (Limit_Button_New == B01000101) Limit_Right_Pressed();
      else if (Limit_Button_New == B01010001) Limit_Front_Pressed();
      else if (Limit_Button_New == B01010100) Limit_Rear_Pressed();
      else     limit_button_flag = false;
   }
   Limit_Button_Old = Limit_Button_New;


   /////// Axis switch for GRI ////////////////////////////
   byte Hand_Axis_New = Hand_Axis_Read;
   if (Hand_Axis_New != Hand_Axis_Old)
   {
      if      (Hand_Axis_New == B00100000) {Switch_Hand_Axis_Z();}
      else if (Hand_Axis_New == B00010000) {Switch_Hand_Axis_X();}
      else if (Hand_Axis_New == B00110000) {Switch_Hand_Axis_No();}
      Hand_Axis_Old = Hand_Axis_New;
   }


   /////// GRI Scale Switch //////////////////////
   byte Hand_Scale_New = Hand_Scale_Read;
   if (Hand_Scale_New != Hand_Scale_Old)
   {
      if      (Hand_Scale_New == B00000001) {Switch_Scale_x1();}
      else if (Hand_Scale_New == B00000010) {Switch_Scale_x10();}
      Hand_Scale_Old = Hand_Scale_New;
   }
   CLEAR_KEYB_TIMER;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Joystick Handling ********** ///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Handler Joystick Left **********
void Joy_LeftPressed()
{  
   flag_j = ON;
   Disa_INT_Hcoder();
   Disable_INT_OCR3A();
   Disable_INT_OCR3B();
   hand_Z = OFF;
   hand_X = OFF;
   Hand_Count = 0;
   Hand_Count_New = 0;
   Hand_Count_Old = 0;
   Hand_Z_Pos = 0;
   Hand_X_Pos = 0;

   ///////////////////////
   if (Mode == Mode_Thread)
   {
      if (Sub_Mode_Thread == Sub_Mode_Thread_Man && err_1_flag == false && err_2_flag == false)
      {
         if (Spindle_Dir == CW) {Thread_Left(c_flag, d_flag);}    
         else
         {
            if (!Button_Rapid) {Limit_Pos = Limit_Pos_Min;}
            else               {Limit_Pos = Limit_Pos_Right + Brake_Compens;}
            Thread_Right(c_flag, d_flag);
         }
      }
      else if (Sub_Mode_Thread == Sub_Mode_Thread_Ext)
      {
         if (Spindle_Dir == CW) {Thread_Ext_Left();}    
         else                   {Thread_Ext_Right();}
      }
      else if (Sub_Mode_Thread == Sub_Mode_Thread_Int)
      {
         if (Spindle_Dir == CW) {Thread_Int_Left();}    
         else                   {Thread_Int_Right();}
      }
   }

   //////////////////////////
   else if (Mode == Mode_Feed)
   {
      if (Sub_Mode_Feed == Sub_Mode_Feed_Man && err_1_flag == false && err_2_flag == false)
      {
         if (!Button_Rapid)
         {  
            if (Motor_Z_Pos < (Limit_Pos_Left - ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
            {
               feed_Z_flag = OFF;
               if (!Step_Z_flag) {Rapid_Feed_Left(a_flag, b_flag);}
            }
         }
         else
         {
            if (OCR5A == max_OCR5A)
            {
               rapid_Z_flag = OFF;
               if (!rapid_step_Z_flag)
               {
                  if (Read_Z_Ena_State == false) Motor_Z_Enable();
                  feed_Z_flag = ON;
                  Feed_Left(a_flag, b_flag);
               }
            }
         }
      }
      else if (Sub_Mode_Feed == Sub_Mode_Feed_Ext)
      {
         Feed_Ext_Left();
      }
      else if (Sub_Mode_Feed == Sub_Mode_Feed_Int)
      {
         Feed_Int_Left();
      }  
   }

   ///////////////////////////
   else if (Mode == Mode_aFeed)
   {
      if (Sub_Mode_aFeed == Sub_Mode_aFeed_Man && err_1_flag == false && err_2_flag == false)
      {
         if (!Button_Rapid)
         {  
            if (Motor_Z_Pos < (Limit_Pos_Left - ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
            {
               feed_Z_flag = OFF;
               if (!Step_Z_flag) {Rapid_Feed_Left(a_flag, b_flag);}
            }
         }
         else
         {
            if (OCR4A == max_OCR4A)
            { 
               rapid_Z_flag = OFF;
               if (!rapid_step_Z_flag)
               {
                  if (Read_Z_Ena_State == false) Motor_Z_Enable();
                  feed_Z_flag = ON;
                  aFeed_Left(a_flag, b_flag);
               }
            }
         }
      }
      else if (Sub_Mode_aFeed == Sub_Mode_aFeed_Ext)
      {
         aFeed_Ext_Left();
      }
      else if (Sub_Mode_Feed == Sub_Mode_aFeed_Int)
      {
         aFeed_Int_Left();
      }  
   }

   ///////////////////////////////////////////////////////////////////////////////////////////////////
   else if ((Mode == Mode_Cone_L || Mode == Mode_Cone_R) && err_1_flag == false && err_2_flag == false)
   {
      if (!Button_Rapid)
      {  
         if (Motor_Z_Pos < (Limit_Pos_Left - ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
         {
            feed_Z_flag = OFF;
            if (!Step_Z_flag) {Rapid_Cone_Left(a_flag, b_flag);}
         }
      }
      else
      {
         if (OCR5A == max_OCR5A)
         {
            rapid_Z_flag = OFF;
            if (!rapid_step_Z_flag)
            {
               Cone_Left(a_flag, b_flag);
            }
         }
      }
   }

   //////////////////////////////////////////////////////////////////////////
   else if (Mode == Mode_Sphere && err_1_flag == false && err_2_flag == false)
   {
      Sphere_Ext(a_flag, b_flag);
   }
   
}  


// ********** Handler Joystick Right **********
void Joy_RightPressed()
{
   flag_j = ON;
   Disa_INT_Hcoder();
   Disable_INT_OCR3A();
   Disable_INT_OCR3B();
   hand_Z = OFF;
   hand_X = OFF;
   Hand_Count = 0;
   Hand_Count_New = 0;
   Hand_Count_Old = 0;
   Hand_Z_Pos = 0;
   Hand_X_Pos = 0;

   ////////////////////////
   if (Mode == Mode_Thread)
   {
      if (Sub_Mode_Thread == Sub_Mode_Thread_Man && err_1_flag == false && err_2_flag == false)
      {
         if (Spindle_Dir == CW)
         {
            if (!Button_Rapid) {Limit_Pos = Limit_Pos_Min;}
            else               {Limit_Pos = Limit_Pos_Right + Brake_Compens;}
            Thread_Right(c_flag, d_flag);
         }
         else {Thread_Left(c_flag, d_flag);}
      }
      else if (Sub_Mode_Thread == Sub_Mode_Thread_Ext)
      {
         if (Spindle_Dir == CW) {Thread_Ext_Right();}
         else                   {Thread_Ext_Left();}
      }
      else if (Sub_Mode_Thread == Sub_Mode_Thread_Int)
      {
         if (Spindle_Dir == CW) {Thread_Int_Right();}
         else                   {Thread_Int_Left();}
      }
   }  

   ///////////////////////////
   else if (Mode == Mode_Feed)
   {
      if (Sub_Mode_Feed == Sub_Mode_Feed_Man && err_1_flag == false && err_2_flag == false)
      {
         if (!Button_Rapid)
         {  
            if (Motor_Z_Pos > (Limit_Pos_Right + ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
            {
               feed_Z_flag = OFF;
               if (!Step_Z_flag) {Rapid_Feed_Right(a_flag, b_flag);}
            }
         }
         else
         {
            if (OCR5A == max_OCR5A)
            {
               rapid_Z_flag = OFF;
               if (!rapid_step_Z_flag)
               {
                  if (Read_Z_Ena_State == false) Motor_Z_Enable();
                  feed_Z_flag = ON;
                  Feed_Right(a_flag, b_flag);
               }
            }
         }
      }
      else if (Sub_Mode_Feed == Sub_Mode_Feed_Ext)
      {
         Feed_Ext_Right();
      }
      else if (Sub_Mode_Feed == Sub_Mode_Feed_Int)
      {
         Feed_Int_Right();
      }  
   }

   ////////////////////////////
   else if (Mode == Mode_aFeed)
   {
      if (Sub_Mode_aFeed == Sub_Mode_aFeed_Man && err_1_flag == false && err_2_flag == false)
      {
         if (!Button_Rapid)
         {  
            if (Motor_Z_Pos > (Limit_Pos_Right + ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
            {
               feed_Z_flag = OFF;
               if (!Step_Z_flag) {Rapid_Feed_Right(a_flag, b_flag);}
            }
         }
         else
         {
            if (OCR4A == max_OCR4A)
            {
               rapid_Z_flag = OFF;
               if (!rapid_step_Z_flag)
               {
                  if (Read_Z_Ena_State == false) Motor_Z_Enable();
                  feed_Z_flag = ON;
                  aFeed_Right(a_flag, b_flag);
               }
            }
         }
      }
      else if (Sub_Mode_aFeed == Sub_Mode_aFeed_Ext)
      {
         aFeed_Ext_Right();
      }
      else if (Sub_Mode_aFeed == Sub_Mode_aFeed_Int)
      {
         aFeed_Int_Right();
      }  
   }

   ////////////////////////////////////////////////////////////////////////////////////////////////////
   else if ((Mode == Mode_Cone_L || Mode == Mode_Cone_R) && err_1_flag == false && err_2_flag == false)
   {
      if (!Button_Rapid)
      {
         if (Motor_Z_Pos > (Limit_Pos_Right + ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
         {
            feed_Z_flag = OFF;
            if (!Step_Z_flag) {Rapid_Cone_Right(a_flag, b_flag);}
         }
      }
      else
      {
         if (OCR5A == max_OCR5A)
         {
            rapid_Z_flag = OFF;
            if (!rapid_step_Z_flag)
            {
               Cone_Right(a_flag, b_flag);
            }
         }
      }
   }
}


// ********** Handler Joystick Up **********
void Joy_UpPressed()
{
   flag_j = ON;
   Disa_INT_Hcoder();
   Disable_INT_OCR3A();
   Disable_INT_OCR3B();
   hand_X = OFF;
   hand_Z = OFF;
   Hand_Count = 0;
   Hand_Count_New = 0;
   Hand_Count_Old = 0;
   Hand_Z_Pos = 0;
   Hand_X_Pos = 0;

   ///////////////////////
   if (Mode == Mode_Thread)
   {  
      if (Sub_Mode_Thread == Sub_Mode_Thread_Man && err_1_flag == false && err_2_flag == false)
      {
         if (Spindle_Dir == CW) {Thread_Front(c_flag, d_flag);}
         else                   {Thread_Rear(c_flag, d_flag);}
      }
   }

   //////////////////////////
   else if (Mode == Mode_Feed)
   {
      if (Sub_Mode_Feed == Sub_Mode_Feed_Man && err_1_flag == false && err_2_flag == false)
      {
         if (!Button_Rapid)
         {
            if (Motor_X_Pos < (Limit_Pos_Front - ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
            {  
               feed_X_flag = OFF;
               if (!Step_X_flag) {Rapid_Feed_Front(a_flag, b_flag);}
            }
         }
         else
         {
            if (OCR5A == max_OCR5A)
            {
               rapid_X_flag = OFF;
               if (!rapid_step_X_flag)
               {
                  if (Read_X_Ena_State == false) Motor_X_Enable();
                  feed_X_flag = ON;
                  Feed_Front(a_flag, b_flag);
               }
            }
         }
      }
      else if (Sub_Mode_Feed == Sub_Mode_Feed_Ext)
      {
         Feed_Ext_Front();
      }
      else if (Sub_Mode_Feed == Sub_Mode_Feed_Int)
      {
         Feed_Int_Front();
      } 
   }

   //////////////////////////
   else if (Mode == Mode_aFeed)
   {
      if (Sub_Mode_aFeed == Sub_Mode_aFeed_Man && err_1_flag == false && err_2_flag == false)
      {
         if (!Button_Rapid)
         {
            if (Motor_X_Pos < (Limit_Pos_Front - ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
            {  
               feed_X_flag = OFF;
               if (!Step_X_flag) {Rapid_Feed_Front(a_flag, b_flag);}
            }
         }
         else
         {
            if (OCR4A == max_OCR4A)
            {
               rapid_X_flag = OFF;
               if (!rapid_step_X_flag)
               {
                  if (Read_X_Ena_State == false) Motor_X_Enable();
                  feed_X_flag = ON;
                  aFeed_Front(a_flag, b_flag);
               }
            }
         }
      }
      else if (Sub_Mode_aFeed == Sub_Mode_aFeed_Ext)
      {
         //
      }
      else if (Sub_Mode_aFeed == Sub_Mode_aFeed_Int)
      {
         //
      } 
   }

   //////////////////////////////////////////////////////////////////////////
   else if (Mode == Mode_Cone_L && err_1_flag == false && err_2_flag == false)
   {
      if (!Button_Rapid)
      {
         if (Motor_X_Pos < (Limit_Pos_Front - ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
         {  
            feed_X_flag = OFF;
            if (!Step_X_flag) {Rapid_Feed_Front(a_flag, b_flag);}
         }
      }
      else
      {
         if (OCR5A == max_OCR5A)
         {
            rapid_X_flag = OFF;
            if (!rapid_step_X_flag)
            {
               if (Read_X_Ena_State == false) Motor_X_Enable();
               feed_X_flag = ON;
               Feed_Front(a_flag, b_flag);
            }
         }
      }
   }

   //////////////////////////////////////////////////////////////////////////
   else if (Mode == Mode_Cone_R && err_1_flag == false && err_2_flag == false)
   {
      if (!Button_Rapid)
      {
         if (Motor_X_Pos < (Limit_Pos_Front - ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
         {  
            feed_X_flag = OFF;
            if (!Step_X_flag) {Rapid_Feed_Front(a_flag, b_flag);}
         }
      }
      else
      {
         if (OCR5A == max_OCR5A)
         {
            rapid_X_flag = OFF;
            if (!rapid_step_X_flag)
            {
               if (Read_X_Ena_State == false) Motor_X_Enable();
               feed_X_flag = ON;
               Feed_Front(a_flag, b_flag);
            }
         }
      }
   }
}


// ********** Handler Joystick Down **********
void Joy_DownPressed()
{
   flag_j = ON;
   Disa_INT_Hcoder();
   Disable_INT_OCR3A();
   Disable_INT_OCR3B();
   hand_X = OFF;
   hand_Z = OFF;
   Hand_Count = 0;
   Hand_Count_New = 0;
   Hand_Count_Old = 0;
   Hand_Z_Pos = 0;
   Hand_X_Pos = 0;

   ///////////////////////
   if (Mode == Mode_Thread)
   {
      if (Sub_Mode_Thread == Sub_Mode_Thread_Man && err_1_flag == false && err_2_flag == false)
      {
         if (Spindle_Dir == CW) {Thread_Rear(c_flag, d_flag);}
         else                   {Thread_Front(c_flag, d_flag);}
      }
   }

   ///////////////////////////
   else if (Mode == Mode_Feed)
   {
      if (Sub_Mode_Feed == Sub_Mode_Feed_Man && err_1_flag == false && err_2_flag == false)
      {
         if (!Button_Rapid)
         {
            if (Motor_X_Pos > (Limit_Pos_Rear + ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
            {  
               feed_X_flag = OFF;
               if (!Step_X_flag) {Rapid_Feed_Rear(a_flag, b_flag);}
            }
         }
         else
         {
            if (OCR5A == max_OCR5A)
            {
               rapid_X_flag = OFF;
               if (!rapid_step_X_flag)
               {
                  if (Read_X_Ena_State == false) Motor_X_Enable();
                  feed_X_flag = ON;
                  Feed_Rear(a_flag, b_flag);
               }
            }
         }
      }
      else if (Sub_Mode_Feed == Sub_Mode_Feed_Ext)
      {
         Feed_Ext_Rear();
      }
      else if (Sub_Mode_Feed == Sub_Mode_Feed_Int)
      {
         Feed_Int_Rear();
      }
   }


   ///////////////////////////
   else if (Mode == Mode_aFeed)
   {
      if (Sub_Mode_aFeed == Sub_Mode_aFeed_Man && err_1_flag == false && err_2_flag == false)
      {
         if (!Button_Rapid)
         {
            if (Motor_X_Pos > (Limit_Pos_Rear + ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
            {  
               feed_X_flag = OFF;
               if (!Step_X_flag) {Rapid_Feed_Rear(a_flag, b_flag);}
            }
         }
         else
         {
            if (OCR4A == max_OCR4A)
            {
               rapid_X_flag = OFF;
               if (!rapid_step_X_flag)
               {
                  if (Read_X_Ena_State == false) Motor_X_Enable();
                  feed_X_flag = ON;
                  aFeed_Rear(a_flag, b_flag);
               }
            }
         }
      }
      else if (Sub_Mode_aFeed == Sub_Mode_aFeed_Ext)
      {
         //
      }
      else if (Sub_Mode_aFeed == Sub_Mode_aFeed_Int)
      {
         //
      }
   }

   //////////////////////////////////////////////////////////////////////////
   else if (Mode == Mode_Cone_L && err_1_flag == false && err_2_flag == false)
   {
      if (!Button_Rapid)
      {
         if (Motor_X_Pos > (Limit_Pos_Rear + ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
         {  
            feed_X_flag = OFF;
            if (!Step_X_flag) {Rapid_Feed_Rear(a_flag, b_flag);}
         }
      }
      else
      {
         if (OCR5A == max_OCR5A)
         {
            rapid_X_flag = OFF;
            if (!rapid_step_X_flag)
            {
               if (Read_X_Ena_State == false) Motor_X_Enable();
               feed_X_flag = ON;
               Feed_Rear(a_flag, b_flag);
            }
         }
      }
   }

   //////////////////////////////////////////////////////////////////////////
   else if (Mode == Mode_Cone_R && err_1_flag == false && err_2_flag == false)
   {
      if (!Button_Rapid)
      {
         if (Motor_X_Pos > (Limit_Pos_Rear + ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
         {  
            feed_X_flag = OFF;
            if (!Step_X_flag) {Rapid_Feed_Rear(a_flag, b_flag);}
         }
      }
      else
      {
         if (OCR5A == max_OCR5A)
         {
            rapid_X_flag = OFF;
            if (!rapid_step_X_flag)
            {
               if (Read_X_Ena_State == false) Motor_X_Enable();
               feed_X_flag = ON;
               Feed_Rear(a_flag, b_flag);
            }
         }
      }
   }
}


// ********** Handler Joystick in Neutral **********
void Joy_NoPressed()
{
   if (flag_j == ON)
   {
      if (!Step_Z_flag && !rapid_step_Z_flag && !Step_X_flag && !rapid_step_X_flag)
      {
         flag_j = OFF;
         
         Motor_Z_Pos = ((Motor_Z_Pos + McSTEP_Z / 2) & ~(McSTEP_Z - 1));
         Motor_X_Pos = ((Motor_X_Pos + McSTEP_X / 2) & ~(McSTEP_X - 1));
         Null_Z_Pos = Motor_Z_Pos;
         Null_X_Pos = Motor_X_Pos;

         Motor_Z_RemovePulse();
         Motor_X_RemovePulse();
         Ks_Count = 0;
         Km_Count = 0;
         Cs_Count = 0;
         Cm_Count = 0;
         Repeat_Count = 0;
         a_flag = false;
         c_flag = false;
         d_flag = false;
         cycle_flag = false;
         Pass_Nr = 1;
         OCR5A = max_OCR5A;
         OCR4A = max_OCR4A;
         OCR2A = MIN_RAPID_MOTION;
         if (!flag_hand_Z) {Motor_Z_Disable();}
         if (!flag_hand_X) {Motor_X_Disable();}
      }
   }
   
   Joy_Z_flag = OFF;
   Joy_X_flag = OFF;
   feed_Z_flag = OFF;
   feed_X_flag = OFF;
   rapid_Z_flag = OFF;
   rapid_X_flag = OFF;
   b_flag = false;

   
   if (!Step_Z_flag && !rapid_step_Z_flag)
   {
      if (hand_Z == OFF)
      {         
         if (flag_hand_Z == ON)
         {
            hand_Z = ON;
         }
      }
      H_Coder();
   }

   if (!Step_X_flag && !rapid_step_X_flag)
   {
      if (hand_X == OFF)
      {
         if (flag_hand_X == ON)
         {
            hand_X = ON;
         }
      }
      H_Coder();
   }


   if (Mode == Mode_Thread)
   {
      //
      if (Sub_Mode_Thread != Sub_Mode_Thread_Man)
      {
         //
      }
   }
   
   if (Mode == Mode_Feed || Mode == Mode_aFeed)
   {
      //
      if (Sub_Mode_Feed != Sub_Mode_Feed_Man)
      {
         //
      }
   }

   if (Mode == Mode_Cone_L || Mode == Mode_Cone_R)
   {
      if (!Step_Z_flag && !rapid_step_Z_flag)
      {
         rapid_step_X_flag = OFF;
         Step_X_flag = OFF;
      }
   }

   

   if ((Mode == Mode_Thread && Sub_Mode_Thread == Sub_Mode_Thread_Man)||
       (Mode == Mode_Feed && Sub_Mode_Feed == Sub_Mode_Feed_Man)||
       (Mode == Mode_aFeed && Sub_Mode_aFeed == Sub_Mode_aFeed_Man) ||
       (Mode == Mode_Cone_L && Sub_Mode_Cone == Sub_Mode_Cone_Man) ||
       (Mode == Mode_Cone_R && Sub_Mode_Cone == Sub_Mode_Cone_Man) ||
       (Mode == Mode_Sphere && Sub_Mode_Sphere == Sub_Mode_Sphere_Man) ||
       (Mode == Mode_Divider))
    {
    //
    }
    else
    {
       Disa_INT_Hcoder();
    }  
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Handling the Mode Switch ********** //////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Switch_Thread()
{
   Hand_Count = 0;
   Hand_Count_New = 0;
   Hand_Count_Old = 0;
   Hand_Z_Pos = 0;
   Hand_X_Pos = 0;
   Null_Z_Pos = Motor_Z_Pos;
   Null_X_Pos = Motor_X_Pos;
   
   Ena_INT_Thrd();
   Mode = Mode_Thread;
   Step_Z_flag = OFF;
   Step_X_flag = OFF;
   rapid_step_Z_flag = OFF;
   rapid_step_X_flag = OFF;
   Ks_Count = 0;
   Km_Count = 0;
   Repeat_Count = 0;
   Print();
}

void Switch_Feed()
{
   Hand_Count = 0;
   Hand_Count_New = 0;
   Hand_Count_Old = 0;
   Hand_Z_Pos = 0;
   Hand_X_Pos = 0;
   Null_Z_Pos = Motor_Z_Pos;
   Null_X_Pos = Motor_X_Pos;
   
   Ena_INT_Z_Feed();
   Mode = Mode_Feed;
   Step_Z_flag = OFF;
   Step_X_flag = OFF;
   rapid_step_Z_flag = OFF;
   rapid_step_X_flag = OFF;
   Ks_Count = 0;
   Km_Count = 0;
   Repeat_Count = 0;
   Pass_Total = 1;
   Print();
}

void Switch_aFeed()
{
   Hand_Count = 0;
   Hand_Count_New = 0;
   Hand_Count_Old = 0;
   Hand_Z_Pos = 0;
   Hand_X_Pos = 0;
   Null_Z_Pos = Motor_Z_Pos;
   Null_X_Pos = Motor_X_Pos;
   
   Mode = Mode_aFeed;
   Step_Z_flag = OFF;
   Step_X_flag = OFF;
   rapid_step_Z_flag = OFF;
   rapid_step_X_flag = OFF;
   Pass_Total = 1;
   Print();
}

void Switch_Cone_L()
{
   Hand_Count = 0;
   Hand_Count_New = 0;
   Hand_Count_Old = 0;
   Hand_Z_Pos = 0;
   Hand_X_Pos = 0;
   Null_Z_Pos = Motor_Z_Pos;
   Null_X_Pos = Motor_X_Pos;
   
   Ena_INT_Z_Feed();
   Mode = Mode_Cone_L;
   Step_Z_flag = OFF;
   Step_X_flag = OFF;
   rapid_step_Z_flag = OFF;
   rapid_step_X_flag = OFF;
   Ks_Count = 0;
   Km_Count = 0;
   Repeat_Count = 0;
   Pass_Total = 1;
   Print();
}

void Switch_Cone_R()
{
   Hand_Count = 0;
   Hand_Count_New = 0;
   Hand_Count_Old = 0;
   Hand_Z_Pos = 0;
   Hand_X_Pos = 0;
   Null_Z_Pos = Motor_Z_Pos;
   Null_X_Pos = Motor_X_Pos;
   
   Ena_INT_Z_Feed();
   Mode = Mode_Cone_R;
   Step_Z_flag = OFF;
   Step_X_flag = OFF;
   rapid_step_Z_flag = OFF;
   rapid_step_X_flag = OFF;
   Ks_Count = 0;
   Km_Count = 0;
   Repeat_Count = 0;
   Pass_Total = 1;
   Print();
}

void Switch_Reserve()
{
   Hand_Count = 0;
   Hand_Count_New = 0;
   Hand_Count_Old = 0;
   Hand_Z_Pos = 0;
   Hand_X_Pos = 0;
   Null_Z_Pos = Motor_Z_Pos;
   Null_X_Pos = Motor_X_Pos;
   
   Mode = Mode_Reserve;
   Print();
}

void Switch_Sphere()
{
   Hand_Count = 0;
   Hand_Count_New = 0;
   Hand_Count_Old = 0;
   Hand_Z_Pos = 0;
   Hand_X_Pos = 0;
   Null_Z_Pos = Motor_Z_Pos;
   Null_X_Pos = Motor_X_Pos;
   
   //   Ena_INT_Z_Feed(); //
   Mode = Mode_Sphere;
   Step_Z_flag = OFF;
   Step_X_flag = OFF;
   rapid_step_Z_flag = OFF;
   rapid_step_X_flag = OFF;
   Ks_Count = 0;
   Km_Count = 0;
   Repeat_Count = 0;
   Pass_Total = 1;
   Print();
}

void Switch_Divider()
{
   Hand_Count = 0;
   Hand_Count_New = 0;
   Hand_Count_Old = 0;
   Hand_Z_Pos = 0;
   Hand_X_Pos = 0;
   Null_Z_Pos = Motor_Z_Pos;
   Null_X_Pos = Motor_X_Pos;
   
   Ena_INT_Thrd();
   Mode = Mode_Divider;
   Step_Z_flag = OFF;
   Step_X_flag = OFF;
   rapid_step_Z_flag = OFF;
   rapid_step_X_flag = OFF;
   Ks_Count = 0;
   Km_Count = 0;
   Repeat_Count = 0;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Submode Switch Handling ********** ///////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Switch_Int()
{
   switch (Mode) 
   {
      case Mode_Thread:
      case Mode_Feed:
      case Mode_aFeed:
      case Mode_Cone_L:
      case Mode_Cone_R:
      if ((limit_Left_flag == ON && limit_Right_flag == ON) || (limit_Front_flag == ON && limit_Rear_flag == ON))
      {
         Sub_Mode_Thread = Sub_Mode_Thread_Int;
         Sub_Mode_Feed = Sub_Mode_Feed_Int;
         Sub_Mode_aFeed = Sub_Mode_aFeed_Int;
         Sub_Mode_Cone = Sub_Mode_Cone_Int;
         err_1_flag = false;
         Step_Z_flag = OFF;
         Step_X_flag = OFF;
         rapid_step_Z_flag = OFF;
         rapid_step_X_flag = OFF;
         Ks_Count = 0;
         Km_Count = 0;
         Repeat_Count = 0;

         if (Motor_Z_Pos == Limit_Pos_Right || Motor_Z_Pos == Limit_Pos_Left || Motor_X_Pos == Limit_Pos_Rear || Motor_X_Pos == Limit_Pos_Front)
         {
            err_2_flag = false;
         }
         else
         {
            Sub_Mode_Thread = Sub_Mode_Thread_Man;
            Sub_Mode_Feed = Sub_Mode_Feed_Man;
            Sub_Mode_aFeed = Sub_Mode_aFeed_Man;
            Sub_Mode_Cone = Sub_Mode_Cone_Man;
            err_2_flag = true;
            Beep_Error();
         }
      }
      else
      {
         Sub_Mode_Thread = Sub_Mode_Thread_Man;
         Sub_Mode_Feed = Sub_Mode_Feed_Man;
         Sub_Mode_aFeed = Sub_Mode_aFeed_Man;
         Sub_Mode_Cone = Sub_Mode_Cone_Man;
         err_1_flag = true;
         Beep_Error();
      }
      Print();
      break;

      case Mode_Sphere:  /////////////////////////////////////////////////////////////////////////////////////
      if (limit_Right_flag == ON && limit_Rear_flag == ON)
      {
         Sub_Mode_Sphere = Sub_Mode_Sphere_Int;
         err_1_flag = false;
         Step_Z_flag = OFF;
         Step_X_flag = OFF;
         Ks_Count = 0;
         Km_Count = 0;
         Repeat_Count = 0;
   
         if (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Rear)
         {
            err_2_flag = false;
         }
         else
         {
            Sub_Mode_Sphere = Sub_Mode_Sphere_Man;
            err_2_flag = true;
            Beep_Error();
         }
      }
      else
      {
         Sub_Mode_Sphere = Sub_Mode_Sphere_Man;
         err_1_flag = true;
         Beep_Error();
      }
      Print();
      break;
   }
}
   
void Switch_Man()
{  
      switch (Mode) 
      {
         case Mode_Thread:
         case Mode_Feed:
         case Mode_aFeed:
         case Mode_Cone_L:
         case Mode_Cone_R:
         case Mode_Sphere:
         Sub_Mode_Thread = Sub_Mode_Thread_Man;
         Sub_Mode_Feed = Sub_Mode_Feed_Man;
         Sub_Mode_aFeed = Sub_Mode_aFeed_Man;
         Sub_Mode_Cone = Sub_Mode_Cone_Man;
         Sub_Mode_Sphere = Sub_Mode_Sphere_Man;
         err_1_flag = false;
         err_2_flag = false;
         Step_Z_flag = OFF;
         Step_X_flag = OFF;
         rapid_step_Z_flag = OFF;
         rapid_step_X_flag = OFF;
         Ks_Count = 0;
         Km_Count = 0;
         Repeat_Count = 0;
         Print();
         break;
      }
}

void Switch_Ext()
{
   switch (Mode) //////////////////////////////////////////////////////////////////////////////////////////////
   {
      case Mode_Thread:
      case Mode_Feed:
      case Mode_aFeed:
      case Mode_Cone_L:
      case Mode_Cone_R:
      if ((limit_Left_flag == ON && limit_Right_flag == ON) || (limit_Front_flag == ON && limit_Rear_flag == ON))
      {
         Sub_Mode_Thread = Sub_Mode_Thread_Ext;
         Sub_Mode_Feed = Sub_Mode_Feed_Ext;
         Sub_Mode_aFeed = Sub_Mode_aFeed_Ext;
         Sub_Mode_Cone = Sub_Mode_Cone_Ext;
         err_1_flag = false;
         Step_Z_flag = OFF;
         Step_X_flag = OFF;
         rapid_step_Z_flag = OFF;
         rapid_step_X_flag = OFF;
         Ks_Count = 0;
         Km_Count = 0;
         Repeat_Count = 0;

         if (Motor_Z_Pos == Limit_Pos_Right || Motor_Z_Pos == Limit_Pos_Left || Motor_X_Pos == Limit_Pos_Rear || Motor_X_Pos == Limit_Pos_Front)
         {
            err_2_flag = false;
         }
         else
         {
            Sub_Mode_Thread = Sub_Mode_Thread_Man;
            Sub_Mode_Feed = Sub_Mode_Feed_Man;
            Sub_Mode_aFeed = Sub_Mode_aFeed_Man;
            Sub_Mode_Cone = Sub_Mode_Cone_Man;
            err_2_flag = true;
            Beep_Error();
         }
      }
      else
      {
         Sub_Mode_Thread = Sub_Mode_Thread_Man;
         Sub_Mode_Feed = Sub_Mode_Feed_Man;
         Sub_Mode_aFeed = Sub_Mode_aFeed_Man;
         Sub_Mode_Cone = Sub_Mode_Cone_Man;
         err_1_flag = true;
         Beep_Error();
      }
      Print();
      break;
      
      case Mode_Sphere:  /////////////////////////////////////////////////////////////////////////////////////
      if (limit_Right_flag == ON && limit_Rear_flag == ON)
      {
         Sub_Mode_Sphere = Sub_Mode_Sphere_Ext;
         err_1_flag = false;
         Step_Z_flag = OFF;
         Step_X_flag = OFF;
         Ks_Count = 0;
         Km_Count = 0;
         Repeat_Count = 0;
   
         if (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Rear)
         {
            err_2_flag = false;
         }
         else
         {
            Sub_Mode_Sphere = Sub_Mode_Sphere_Man;
            err_2_flag = true;
            Beep_Error();
         }
      }
      else
      {
         Sub_Mode_Sphere = Sub_Mode_Sphere_Man;
         err_1_flag = true;
         Beep_Error();
      }
      Print();
      break;
   }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Handling Menu Buttons ********** ///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Select button click handler **********
void Key_Select_Pressed()
{
   if (!key_sel_flag)
   {  
      switch (Mode) 
      {
         case Mode_Feed:
         if (!Joy_Z_flag && !Joy_X_flag)
         {
            //
         }
         break;
        
         case Mode_Cone_L:
         if (!Joy_Z_flag && !Joy_X_flag)
         {
            //
         }
         break;
     
         case Mode_Cone_R:
         if (!Joy_Z_flag && !Joy_X_flag)
         {
            //
         }
         break;
    
         case Mode_Thread:
         if (!Joy_Z_flag && !Joy_X_flag)
         {
            //
         }
         break;         

         case Mode_Sphere:
         if (!Joy_Z_flag && !Joy_X_flag)
         {
            Beep();
         }
         break;
         
         case Mode_Divider:
         if (!Joy_Z_flag && !Joy_X_flag)
         {
            Enc_Pos = 0;
            Beep();
         }
         break;
      }
      key_sel_flag = true;
      Print();
   }
}

// ********** Up button click handler **********
void Key_Up_Pressed()
{
  if (!button_flag)
  {
     switch (Mode)
     {
        case Mode_Feed:
        if (!Joy_Z_flag && !Joy_X_flag)
        {
           if (Ap < 100)      {Ap = Ap + 10;  Beep();}
           else if (Ap < 200) {Ap = Ap + 20;  Beep();}
           else if (Ap < 500) {Ap = Ap + 50;  Beep();}
           else if (Ap < 900) {Ap = Ap + 100; Beep();}
        }
        Print();
        break;
      
        case Mode_Thread:
        if (Thread_Step < TOTAL_THREADS - 1)
        {
           if (!Joy_Z_flag && !Joy_X_flag)
           {
              Thread_Step++;
              Ks_Count = 0;
              Km_Count = 0;
              Repeat_Count = 0;
              Step_Z_flag = OFF;
              Step_X_flag = OFF;
              rapid_step_Z_flag = OFF;
              rapid_step_X_flag = OFF;
              Beep();
           }
        }
        Print();
        break;        

        case Mode_Sphere:
        if (!Joy_Z_flag && !Joy_X_flag)
        {
           if (!key_sel_flag)
           {
              if      (Sph_R_mm < 1250) {Sph_R_mm = Sph_R_mm + 25;  Beep();}
              else if (Sph_R_mm < 2500) {Sph_R_mm = Sph_R_mm + 50;  Beep();}
              else if (Sph_R_mm < 4750) {Sph_R_mm = Sph_R_mm + 250; Beep();}

              R_Quad = Sph_R_mm * Sph_R_mm;
              Sph_R = (MOTOR_X_STEP_PER_REV * McSTEP_X * Sph_R_mm / SCREW_X);
           }
           else
           {
              if (Cutter_Step < TOTAL_CUTTER_WIDTH-1)
              {
                 Cutter_Step++;
                 Cutter_Width = Cutter_Width_array[Cutter_Step];
                 Beep();
              }
           }
        }
        Print();
        break;
        
        case Mode_Divider:
        if (Total_Tooth < 255)
        {
           Total_Tooth++;
           Current_Tooth = 1;
           Beep();
        }
        Print();
        break;
     }
     button_flag = true;
  }
}

// ********** Down button click handler **********
void Key_Down_Pressed()
{
   if (!button_flag)
   {
      switch (Mode)
      {
         case Mode_Feed:
         if (!Joy_Z_flag && !Joy_X_flag)
         {
            if (Ap > 500)      {Ap = Ap - 100; Beep();}
            else if (Ap > 200) {Ap = Ap - 50;  Beep();}
            else if (Ap > 100) {Ap = Ap - 20;  Beep();}
            else if (Ap > 0)   {Ap = Ap - 10;  Beep();}
         }
         Print();
         break;
         
         case Mode_Thread:
         if (Thread_Step > 0)
         {
            if (!Joy_Z_flag && !Joy_X_flag)
            {
               Thread_Step--;
               Ks_Count = 0;
               Km_Count = 0;
               Repeat_Count = 0;
               Step_Z_flag = OFF;
               Step_X_flag = OFF;
               rapid_step_Z_flag = OFF;
               rapid_step_X_flag = OFF;
               Beep();
            }
         }
         Print();
         break;        

         case Mode_Sphere:
         if (!Joy_Z_flag && !Joy_X_flag)
         {
            if (!key_sel_flag)
            {
               if      (Sph_R_mm > 2500) {Sph_R_mm = Sph_R_mm - 250; Beep();}
               else if (Sph_R_mm > 1250) {Sph_R_mm = Sph_R_mm - 50;  Beep();}
               else if (Sph_R_mm > 50)   {Sph_R_mm = Sph_R_mm - 25;  Beep();}
               if (Sph_R_mm < Bar_R_mm) Bar_R_mm = Sph_R_mm;

               R_Quad = Sph_R_mm * Sph_R_mm;
               Sph_R = (MOTOR_X_STEP_PER_REV * McSTEP_X * Sph_R_mm / SCREW_X);   // sphere radius in steps
            }
            else
            {
               if (Cutter_Step > 0)
               {
                  Cutter_Step--;
                  Cutter_Width = Cutter_Width_array[Cutter_Step];
                  Beep();
               }
            }

         }
         Print();
         break;
        
         case Mode_Divider:
         if (Total_Tooth > 1)
         {
            Total_Tooth--;
            Current_Tooth = 1;
            Beep();
         }
         Print();
         break;
      }
      button_flag = true;
   }
}

// ********** Left button click handler **********
void Key_Left_Pressed()
{
   if (!button_flag)
   {
      switch (Mode)
      {
         case Mode_Feed:
         if (Pass_Total > 1)
         {
            if (!Joy_Z_flag && !Joy_X_flag)
            {
               Pass_Total--;
               Beep();
            }
         }
         Print();
         break;

        
         case Mode_Cone_L:
         case Mode_Cone_R:
         if (Cone_Step > 0)
         {
            if (!Joy_Z_flag && !Joy_X_flag)
            {
               Cone_Step--;
               Ks_Count = 0;
               Km_Count = 0;
               Repeat_Count = 0;
               Step_Z_flag = OFF;
               Step_X_flag = OFF;
               rapid_step_Z_flag = OFF;
               rapid_step_X_flag = OFF;
               Beep();
            }
         }
         Print();
         break;

         case Mode_Sphere:
         if (!Joy_Z_flag && !Joy_X_flag)
         {
            if (!key_sel_flag)
            {
               if (Bar_R_mm > 0)
               {
                  Bar_R_mm = Bar_R_mm - 25; 
                  Bar_R = (MOTOR_X_STEP_PER_REV * McSTEP_X * Bar_R_mm / SCREW_X);
                  Beep();
               }
            }
            else
            {
               if (Cutting_Step > 0)
               {
                  Cutting_Step--;
                  Cutting_Width = Cutting_Width_array[Cutting_Step];
                  Beep();
               }
            }
         }
         Print();
         break;
     
         case Mode_Divider:
         if (Current_Tooth > 1)
         {
            Current_Tooth--;
            Beep();
         }
         else if (Current_Tooth == 1)
         {
            Current_Tooth = Total_Tooth;
            Beep();
         }
         Print();
         break;  
      }
      button_flag = true;
   }
}

// ********** Right button click handler **********
void Key_Right_Pressed()
{
   if (!button_flag)
   {
      switch (Mode)
      {
         case Mode_Feed:
         if (Pass_Total < 9)
         {
            if (!Joy_Z_flag && !Joy_X_flag)
            {
               Pass_Total++;
               Beep();
            }
         }
         Print();
         break;
        
         case Mode_Cone_L:
         case Mode_Cone_R:
         if (Cone_Step < TOTAL_CONE - 1)
         {
            if (!Joy_Z_flag && !Joy_X_flag)
            {
               Cone_Step++;
               Ks_Count = 0;
               Km_Count = 0;
               Repeat_Count = 0;
               Step_Z_flag = OFF;
               Step_X_flag = OFF;
               rapid_step_Z_flag = OFF;
               rapid_step_X_flag = OFF;
               Beep();
            }
         }
         Print();
         break;

         case Mode_Sphere:
         if (!Joy_Z_flag && !Joy_X_flag)
         {
            if (!key_sel_flag)
            {
               if (Bar_R_mm < Sph_R_mm)
               {
                  Bar_R_mm = Bar_R_mm + 25;
                  Bar_R = (MOTOR_X_STEP_PER_REV * McSTEP_X * Bar_R_mm / SCREW_X);   // undercut radius in steps
                  Beep();
               }
            }
            else
            {
               if (Cutting_Step < TOTAL_CUTTING_STEP-1)
               {
                  Cutting_Step++;
                  Cutting_Width = Cutting_Width_array[Cutting_Step];
                  Beep();
               }
            }
         }
         Print();
         break;
     
         case Mode_Divider:
         if (Current_Tooth < Total_Tooth)
         {
            Current_Tooth++;
            Beep();
         }
         else if (Current_Tooth == Total_Tooth)
         {
            Current_Tooth = 1;
            Beep();
         }
         Print();
         break;
      }  
      button_flag = true;
   }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Processing limit buttons ********** /////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Limit_Left button click handler **********
void Limit_Left_Pressed()
{
   if (!limit_button_flag)
   {
      limit_button_flag = true;
      switch (Mode)
      {
         case Mode_Thread:
         case Mode_Feed:
         case Mode_aFeed:
         case Mode_Cone_L:
         case Mode_Cone_R:
         if (!Joy_Z_flag && Submode_Read == B10100000)
         { 
            if (limit_Left_flag == OFF)
            {
               if (Motor_Z_Pos > (Limit_Pos_Right + ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
               {
                  limit_Left_flag = ON;
                  Limit_Pos_Left = ((Motor_Z_Pos + McSTEP_Z / 2) & ~(McSTEP_Z - 1));
                  Limit_Left_LED_On();
                  Beep();
               }
            }
            else
            {
               limit_Left_flag = OFF;
               Limit_Pos_Left = Limit_Pos_Max;
               Limit_Left_LED_Off();
               Beep();
            }
         }
      }
   }
}

// ********** Limit_Right button click handler **********
void Limit_Right_Pressed()
{
   if (!limit_button_flag)
   {
      limit_button_flag = true;
      switch (Mode)
      {
         case Mode_Thread:
         case Mode_Feed:
         case Mode_aFeed:
         case Mode_Cone_L:
         case Mode_Cone_R:
         case Mode_Sphere:
         if (!Joy_Z_flag && Submode_Read == B10100000)
         { 
            if (limit_Right_flag == OFF)
            {
               if (Motor_Z_Pos < (Limit_Pos_Left - ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
               {
                  limit_Right_flag = ON;
                  Limit_Pos_Right = ((Motor_Z_Pos + McSTEP_Z / 2) & ~(McSTEP_Z - 1));
                  Limit_Right_LED_On();
                  Beep();
               }
            }
            else
            {
               limit_Right_flag = OFF;
               Limit_Pos_Right = Limit_Pos_Min;
               Limit_Right_LED_Off();
               Beep();
            }
         }
      }
   }
}

// ********** Limit_Front button click handler **********
void Limit_Front_Pressed()
{
   if (!limit_button_flag)
   {
      limit_button_flag = true;
      switch (Mode)
      { 
         case Mode_Thread:
         case Mode_Feed:
         case Mode_aFeed:
         if (!Joy_X_flag && Submode_Read == B10100000)
         { 
            if (limit_Front_flag == OFF)
            {
               if (Motor_X_Pos > (Limit_Pos_Rear + ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
               {
                  limit_Front_flag = ON;
                  Limit_Pos_Front = ((Motor_X_Pos + McSTEP_X / 2) & ~(McSTEP_X - 1));
                  Limit_Front_LED_On();
                  Beep();
               }
            }
            else
            {
               limit_Front_flag = OFF;
               Limit_Pos_Front = Limit_Pos_Max;
               Limit_Front_LED_Off();
               Beep();
            }
         }
      }
   }
}

// ********** Limit_Rear button click handler **********
void Limit_Rear_Pressed()
{
   if (!limit_button_flag)
   {
      limit_button_flag = true;
      switch (Mode)
      { 
         case Mode_Thread:
         case Mode_Feed:
         case Mode_aFeed:
         case Mode_Sphere:
         if (!Joy_X_flag && Submode_Read == B10100000)
         { 
            if (limit_Rear_flag == OFF)
            {
               if (Motor_X_Pos < (Limit_Pos_Front - ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) * 2))
               {
                  limit_Rear_flag = ON;
                  Limit_Pos_Rear = ((Motor_X_Pos + McSTEP_X / 2) & ~(McSTEP_X - 1));
                  Limit_Rear_LED_On();
                  Beep();
               }
            }
            else
            {
               limit_Rear_flag = OFF;
               Limit_Pos_Rear = Limit_Pos_Min;
               Limit_Rear_LED_Off();
               Beep();
            }
         }
      }
   }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** RGI axis switch processing ********** /////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Switch_Hand_Axis_Z()
{
   Motor_X_Disable();
   Motor_Z_Enable();
   
   flag_hand_X = OFF;
   hand_X = OFF;
   flag_hand_Z = ON;

   Hand_Count = 0;
   Hand_Count_New = 0;
   Hand_Count_Old = 0;
   Hand_Z_Pos = 0;
   Null_Z_Pos = Motor_Z_Pos;
   
   Disable_INT_OCR3B();
   Enable_INT_OCR3A();
   Ena_INT_Hcoder();
}

void Switch_Hand_Axis_X()
{
   Motor_Z_Disable();
   Motor_X_Enable();
   
   flag_hand_Z = OFF;
   hand_Z = OFF;
   flag_hand_X = ON;

   Hand_Count = 0;
   Hand_Count_New = 0;
   Hand_Count_Old = 0;
   Hand_X_Pos = 0;
   Null_X_Pos = Motor_X_Pos;
   
   Disable_INT_OCR3A();
   Enable_INT_OCR3B();
   Ena_INT_Hcoder();
}

void Switch_Hand_Axis_No()
{
   Motor_X_Disable();
   Motor_Z_Disable();
   
   flag_hand_Z = OFF;
   hand_Z = OFF;
   flag_hand_X = OFF;
   hand_X = OFF;

   Hand_Count = 0;
   Hand_Count_New = 0;
   Hand_Count_Old = 0;
   Hand_Z_Pos = 0;
   Hand_X_Pos = 0;

   Null_Z_Pos = Motor_Z_Pos;
   Null_X_Pos = Motor_X_Pos;

   Disa_INT_Hcoder();
   Disable_INT_OCR3A();
   Disable_INT_OCR3B();

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** RGI Scale Processing ********** /////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Switch_Scale_x1()
{
   flag_Scale_x10 = OFF;
   flag_Scale_x1 = ON;
   hand_Z = OFF;
   hand_X = OFF;
}

void Switch_Scale_x10()
{
   flag_Scale_x1 = OFF;
   flag_Scale_x10 = ON;
   hand_Z = OFF;
   hand_X = OFF;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ***** Print ***** /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Print()
{
   if (Mode == Mode_Thread)  //////////////////////////////////////////////////////////
   {
      snprintf(LCD_Row_1, 17, "Thrd      %s", Thread_Info[Thread_Step].Thread_Print);
      
      if      (Sub_Mode_Thread == Sub_Mode_Thread_Int) snprintf(LCD_Row_2, 17, "Int  Max:%s", Thread_Info[Thread_Step].Limit_Print);
      else if (Sub_Mode_Thread == Sub_Mode_Thread_Man) snprintf(LCD_Row_2, 17, "Man  Max:%s", Thread_Info[Thread_Step].Limit_Print);
      else if (Sub_Mode_Thread == Sub_Mode_Thread_Ext) snprintf(LCD_Row_2, 17, "Ext  Max:%s", Thread_Info[Thread_Step].Limit_Print);
   } 

  
   else if (Mode == Mode_Feed)  //////////////////////////////////////////////////////////
   {
      snprintf(LCD_Row_1, 17, "Feed mm/rev %1d.%02dmm", Feed_mm/100, Feed_mm%100);

      if      (Sub_Mode_Feed == Sub_Mode_Feed_Int) snprintf(LCD_Row_2, 17, "Int  Pq:%1d Ap:%1d.%02d", Pass_Total-Pass_Nr+1, Ap/100, Ap%100);
      else if (Sub_Mode_Feed == Sub_Mode_Feed_Man) snprintf(LCD_Row_2, 17, "Man  Pq:%1d Ap:%1d.%02d", Pass_Total, Ap/100, Ap%100);
      else if (Sub_Mode_Feed == Sub_Mode_Feed_Ext) snprintf(LCD_Row_2, 17, "Ext  Pq:%1d Ap:%1d.%02d", Pass_Total-Pass_Nr+1, Ap/100, Ap%100);
   }


   else if (Mode == Mode_aFeed)  //////////////////////////////////////////////////////////
   {
      snprintf(LCD_Row_1, 17, "Feed mm/min  %3d", aFeed_mm);
      
      if      (Sub_Mode_aFeed == Sub_Mode_aFeed_Int) snprintf(LCD_Row_2, 17, "Int  Pq:%1d Ap:%1d.%02d", Pass_Total, Ap/100, Ap%100);
      else if (Sub_Mode_aFeed == Sub_Mode_aFeed_Man) snprintf(LCD_Row_2, 17, "Man  Pq:%1d Ap:%1d.%02d", Pass_Total, Ap/100, Ap%100);
      else if (Sub_Mode_aFeed == Sub_Mode_aFeed_Ext) snprintf(LCD_Row_2, 17, "Ext  Pq:%1d Ap:%1d.%02d", Pass_Total, Ap/100, Ap%100);
   }
 
   
   else if (Mode == Mode_Cone_L)  //////////////////////////////////////////////////////////////
   {
      snprintf(LCD_Row_1, 17, "Cone < %s %1d.%02dmm", Cone_Info[Cone_Step].Cone_Print, Feed_mm/100, Feed_mm%100);

      if      (Sub_Mode_Cone == Sub_Mode_Cone_Int) snprintf(LCD_Row_2, 17, "Int  Pq:%1d Ap:%1d.%02d", Pass_Total, Ap/100, Ap%100);
      else if (Sub_Mode_Cone == Sub_Mode_Cone_Man) snprintf(LCD_Row_2, 17, "Man  Pq:%1d Ap:%1d.%02d", Pass_Total, Ap/100, Ap%100);
      else if (Sub_Mode_Cone == Sub_Mode_Cone_Ext) snprintf(LCD_Row_2, 17, "Ext  Pq:%1d Ap:%1d.%02d", Pass_Total, Ap/100, Ap%100);
   }

   
   else if (Mode == Mode_Cone_R)  //////////////////////////////////////////////////////////////
   {
      snprintf(LCD_Row_1, 17, "Cone > %s %1d.%02dmm", Cone_Info[Cone_Step].Cone_Print, Feed_mm/100, Feed_mm%100);
      
      if      (Sub_Mode_Cone == Sub_Mode_Cone_Int) snprintf(LCD_Row_2, 17, "Int  Pq:%1d Ap:%1d.%02d", Pass_Total, Ap/100, Ap%100);
      else if (Sub_Mode_Cone == Sub_Mode_Cone_Man) snprintf(LCD_Row_2, 17, "Man  Pq:%1d Ap:%1d.%02d", Pass_Total, Ap/100, Ap%100);
      else if (Sub_Mode_Cone == Sub_Mode_Cone_Ext) snprintf(LCD_Row_2, 17, "Ext  Pq:%1d Ap:%1d.%02d", Pass_Total, Ap/100, Ap%100);
   }


   else if (Mode == Mode_Reserve)  //////////////////////////////////////////////////////////////
   {
      snprintf(LCD_Row_1, 17, "Reserve         ");
      snprintf(LCD_Row_2, 17, "Reserve         ");
   }
   

   else if (Mode == Mode_Sphere)  //////////////////////////////////////////////////////////////
   {
      if (!key_sel_flag)
      {
         snprintf(LCD_Row_1, 17, "Sphr %2ld.%01ldmm %1d.%02dmm", Sph_R_mm * 2 / 100, Sph_R_mm * 2 / 10 %10, Feed_mm/100, Feed_mm%100);

         if      (Sub_Mode_Sphere == Sub_Mode_Sphere_Int) snprintf(LCD_Row_2, 17, "Mode not Exist  ");
         else if (Sub_Mode_Sphere == Sub_Mode_Sphere_Man) snprintf(LCD_Row_2, 17, "     BarDia %2ld.%01ld", Bar_R_mm*2/100, Bar_R_mm*2%100);
         else if (Sub_Mode_Sphere == Sub_Mode_Sphere_Ext) snprintf(LCD_Row_2, 17, "Ext  BarDia %2ld.%01ld", Bar_R_mm*2/100, Bar_R_mm*2%100);
      }
      
      else
      {
         snprintf(LCD_Row_1, 17, "Cut.Width %1d.%02dmm", Cutter_Width/100, Cutter_Width%100);
         snprintf(LCD_Row_2, 17, "Cut.StepZ %1d.%02dmm", Cutting_Width/100, Cutting_Width%100);
      }
   }

    
   else if (Mode == Mode_Divider)  /////////////////////////////////////////////////////////////
   { 
      long Spindle_Angle = Enc_Pos * 36000 / ENC_TICK;
      long Required_Angle = 36000 * (Current_Tooth - 1) / Total_Tooth;
      snprintf(LCD_Row_1, 17, "Req:%3ld.%02ld z:%3d", Required_Angle/100, Required_Angle%100, Total_Tooth);
      snprintf(LCD_Row_2, 17, "Rea:%3ld.%02ld a:%3d", Spindle_Angle/100, Spindle_Angle%100, Current_Tooth);
   }

   // Print error
   if      (err_1_flag == true) snprintf(LCD_Row_2, 17, "Limits not Set  ");
   else if (err_2_flag == true) snprintf(LCD_Row_2, 17, "Move to Init Pos");   
      
   lcd.setCursor(0, 0);
   lcd.print(LCD_Row_1);
   lcd.print("   ");

   lcd.setCursor(0, 1);
   lcd.print(LCD_Row_2);
   lcd.print("   ");
}
void Sphere_Ext(bool & a_flag, bool & b_flag)
{
   int Pass_Total = Sph_R_mm * 2 / Cutting_Width;

   if (Motor_X_Pos == Limit_Pos_Rear && Motor_Z_Pos == Limit_Pos_Right && Step_X_flag == OFF)
   {  
      if (cycle_flag == false && Pass_Nr <= Pass_Total +1)
      {
         cycle_flag = true;
         a_flag = false;
         b_flag = false;

         long Infeed_Value;
         {
            if (Pass_Nr <= Pass_Total/2) Infeed_Value = (long)((float)MOTOR_Z_STEP_PER_REV * (Cutting_Width * Pass_Nr) / SCREW_Z + 0.5) *McSTEP_Z;
            else                         Infeed_Value = (long)((float)MOTOR_Z_STEP_PER_REV * ((Cutting_Width * (Pass_Nr-1) + Cutter_Width)) / SCREW_Z +0.5) *McSTEP_Z;
         }
         
         Limit_Pos_Left = (Null_Z_Pos + Infeed_Value);
         Limit_Left_LED_On();
         BeepBeep();
         Feed_Left(a_flag, b_flag);
      }
   }

   else if (Motor_X_Pos == Limit_Pos_Rear && Motor_Z_Pos == Limit_Pos_Left && Step_Z_flag == OFF)
   {
      if (Pass_Nr <= Pass_Total +1)
      {
         a_flag = false;
         b_flag = false;

         long Infeed_Value;
         if (Pass_Nr > Pass_Total / 2)
         {
            long A = Cutting_Width * (Pass_Nr - (Pass_Total/2+1));
            float B = sqrt(R_Quad - A*A);
            float E = (Sph_R_mm - B) * MOTOR_X_STEP_PER_REV / SCREW_X;
            Infeed_Value = ((long)E * McSTEP_X);
            if (Infeed_Value > Sph_R - Bar_R)
            {
               Infeed_Value = Sph_R - Bar_R;
            }
         }
         
         else
         {
            long A = Sph_R_mm - Cutting_Width * Pass_Nr;
            float B = sqrt(R_Quad - A*A);
            float E = (Sph_R_mm - B) * MOTOR_X_STEP_PER_REV / SCREW_X;
            Infeed_Value = ((long)E * McSTEP_X);
         }
         
         Limit_Pos_Front = (Null_X_Pos + Infeed_Value);
         Limit_Pos_Right = Limit_Pos_Left;
         Limit_Front_LED_On();
         
         Feed_Front(a_flag, b_flag);
      }
   }

   else if (Motor_X_Pos == Limit_Pos_Front && Motor_Z_Pos == Limit_Pos_Left && Step_Z_flag == OFF)
   {
      cycle_flag = false;
      a_flag = false;
      b_flag = false;
      
      Limit_Pos_Rear = (Null_X_Pos - REBOUND_X);
      Pass_Nr++;
      if (Motor_X_Pos >= (Limit_Pos_Rear + ((MIN_RAPID_MOTION - MAX_RAPID_MOTION) * REPEAt) *2)) {Rapid_Feed_Rear(a_flag, b_flag);}
      else                                                                                       {Feed_Rear(a_flag, b_flag);}
   }

   if (Pass_Nr > Pass_Total +1)
   {
      Limit_Pos_Left = Limit_Pos_Max;
      Limit_Left_LED_Off();
      Limit_Pos_Right = Limit_Pos_Min;
      Limit_Right_LED_Off();
      Limit_Pos_Front = Limit_Pos_Max;
      Limit_Front_LED_Off();
      Limit_Pos_Rear = Limit_Pos_Min;
      Limit_Rear_LED_Off();
   }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Spindle started ********** ////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Spindle()
{
   if (++Spindle_Count > 750) // ~~1sec.
   {
      Spindle_Count = 0;
      if (Tacho_Count == Tacho_Count_Old) {spindle_flag = OFF;}
      else                                {spindle_flag = ON;}
      Tacho_Count_Old = Tacho_Count;
   } 
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** Thread mode ********** //////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Thread_Left(bool & c_flag, bool & d_flag)
{
   if (c_flag == true) return;
   d_flag = false;
   c_flag = true;

   Joy_X_flag = OFF;
   if (Motor_Z_Pos < (Limit_Pos_Left - (THRD_ACCEL * REPEAt )* 2))
   {
      Ks_Divisor = Thread_Info[Thread_Step].Ks_Div_Z;
      if (tmp_Ks_Divisor != Ks_Divisor)
      {
         tmp_Accel = THRD_ACCEL + Ks_Divisor;
         tmp_Ks_Divisor = THRD_ACCEL + Ks_Divisor;
      }
      Brake_Compens = THRD_ACCEL * REPEAt + 1;
   }
   else
   {
      Ks_Divisor = THRD_ACCEL + Thread_Info[0].Ks_Div_Z;
      tmp_Accel = Ks_Divisor;
      tmp_Ks_Divisor = Ks_Divisor;
      Brake_Compens = tmp_Accel - Ks_Divisor + 1;
   }
   
   Km_Divisor = Thread_Info[Thread_Step].Km_Div_Z;
   Ks_Count = 0;
   Km_Count = 0;
   Limit_Pos = Limit_Pos_Left - Brake_Compens;

   Motor_Z_Dir = ZCW;
   Motor_Z_CW();
   if (Read_Z_Ena_State == false) Motor_Z_Enable();
   Joy_Z_flag = ON;
}
  

void Thread_Right(bool & c_flag, bool & d_flag)
{
   if (d_flag == true) return;
   c_flag = false;
   d_flag = true;

   Joy_X_flag = OFF;
   if (Motor_Z_Pos > Limit_Pos_Right + THRD_ACCEL * REPEAt * 2 || Motor_Z_Pos <= Limit_Pos_Right)
   {
      Ks_Divisor = Thread_Info[Thread_Step].Ks_Div_Z;
      if (tmp_Ks_Divisor != Ks_Divisor)
      {
         tmp_Accel = THRD_ACCEL + Ks_Divisor;
         tmp_Ks_Divisor = THRD_ACCEL + Ks_Divisor;
      }  
      Brake_Compens = THRD_ACCEL * REPEAt + 1;
   }
   else
   {
      Ks_Divisor = THRD_ACCEL + Thread_Info[0].Ks_Div_Z;
      tmp_Accel = Ks_Divisor;
      tmp_Ks_Divisor = Ks_Divisor;
      Brake_Compens = tmp_Accel - Ks_Divisor + 1;
   }

   Km_Divisor = Thread_Info[Thread_Step].Km_Div_Z;
   Ks_Count = 0;
   Km_Count = 0;
   Limit_Pos = Limit_Pos_Right + Brake_Compens;

   Motor_Z_Dir = ZCCW;
   Motor_Z_CCW();
   if (Read_Z_Ena_State == false) Motor_Z_Enable();
   Joy_Z_flag = ON;
}


void Thread_Front(bool & c_flag, bool & d_flag)
{
   if (c_flag == true) return;
   d_flag = false;
   c_flag = true;

   Joy_Z_flag = OFF;
   if (Motor_X_Pos < (Limit_Pos_Front - (THRD_ACCEL * REPEAt) * 2))
   {
    
      if (Sub_Mode_Thread == Sub_Mode_Thread_Man)
      {
         Ks_Divisor = Thread_Info[Thread_Step].Ks_Div_X;
         Km_Divisor = Thread_Info[Thread_Step].Km_Div_X;
      }
      else
      {
         Ks_Divisor = Thread_Info[Thread_Step].Ks_Div_Z;
         Km_Divisor = Thread_Info[Thread_Step].Km_Div_Z;
         Ks_Divisor = (Ks_Divisor + (float)(Km_Divisor + 5000) /10000) * ((float)McSTEP_Z / McSTEP_X);
         Km_Divisor = 0;
      }

      if (tmp_Ks_Divisor != Ks_Divisor)
      {
         tmp_Accel = THRD_ACCEL + Ks_Divisor;
         tmp_Ks_Divisor = THRD_ACCEL + Ks_Divisor;
      }   
      Brake_Compens = THRD_ACCEL * REPEAt + 1;
   }
   else
   {
      Ks_Divisor = THRD_ACCEL + Thread_Info[0].Ks_Div_X;
      Km_Divisor = 0;
      tmp_Accel = Ks_Divisor;
      tmp_Ks_Divisor = Ks_Divisor;
      Brake_Compens = tmp_Accel - Ks_Divisor + 1;
   }
  
   Ks_Count = 0;
   Km_Count = 0;
   Limit_Pos = Limit_Pos_Front - Brake_Compens;

   Motor_X_Dir = CW;
   Motor_X_CW();
   if (Read_X_Ena_State == false) Motor_X_Enable();
   Joy_X_flag = ON;
}


void Thread_Rear(bool & c_flag, bool & d_flag)
{
   if (d_flag == true) return;
   c_flag = false;
   d_flag = true;

   Joy_Z_flag = OFF;
   if (Motor_X_Pos > (Limit_Pos_Rear + (THRD_ACCEL * REPEAt) * 2))
   {
      if (Sub_Mode_Thread == Sub_Mode_Thread_Man)
      {
         Ks_Divisor = Thread_Info[Thread_Step].Ks_Div_X;
         Km_Divisor = Thread_Info[Thread_Step].Km_Div_X;
      }
      else
      {
         Ks_Divisor = Thread_Info[Thread_Step].Ks_Div_Z;
         Km_Divisor = Thread_Info[Thread_Step].Km_Div_Z;
         Ks_Divisor = (Ks_Divisor + (float)(Km_Divisor + 5000) /10000) * ((float)McSTEP_Z / McSTEP_X);
         Km_Divisor = 0;
      }
      
      if (tmp_Ks_Divisor != Ks_Divisor)
      {
         tmp_Accel = THRD_ACCEL + Ks_Divisor;
         tmp_Ks_Divisor = THRD_ACCEL + Ks_Divisor;
      }     
      Brake_Compens = THRD_ACCEL * REPEAt + 1;
   }
   else
   {
      Ks_Divisor = THRD_ACCEL + Thread_Info[0].Ks_Div_X;
      Km_Divisor = 0;
      tmp_Accel = Ks_Divisor;
      tmp_Ks_Divisor = Ks_Divisor;
      Brake_Compens = tmp_Accel - Ks_Divisor + 1;
   }

   Ks_Count = 0;
   Km_Count = 0;
   Limit_Pos = Limit_Pos_Rear + Brake_Compens;

   Motor_X_Dir = CCW;
   Motor_X_CCW();
   if (Read_X_Ena_State == false) Motor_X_Enable();
   Joy_X_flag = ON;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ********** "Cycle Thread" mode ********** /////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Thread_Ext_Left()
{
   if ((Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Rear) || (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Null_X_Pos))
   {  
      Pass_Total = Thread_Info[Thread_Step].Pass;
      if (cycle_flag == false && Pass_Nr <= Pass_Total)
      {
         cycle_flag = true;
         c_flag = false;
         d_flag = false;
         
         float Step_mm = Thread_Info[Thread_Step].Step;
         float Pass_Depth;
         if(Pass_Nr == 1) {Pass_Depth = ((Step_mm * 0.866) - (Step_mm * 0.866 / 6) - (Step_mm * 0.866 / 8)) / sqrt(Pass_Total-1) * sqrt(0.3);}
         else             {Pass_Depth = ((Step_mm * 0.866) - (Step_mm * 0.866 / 6) - (Step_mm * 0.866 / 8)) / sqrt(Pass_Total-1) * sqrt(Pass_Nr-1);}
         long Infeed_Value = long(Pass_Depth / ((float)SCREW_X/100 / MOTOR_X_STEP_PER_REV) + 0.5) * McSTEP_X + McSTEP_X;

         Limit_Pos_Front = (Null_X_Pos + Infeed_Value);
         Limit_Front_LED_On();
         Pass_Nr++;
         BeepBeep();
         Thread_Front(c_flag, d_flag);
      }

      else if ((cycle_flag == false) && (Pass_Nr > Pass_Total && Pass_Nr <= Pass_Total + PASS_FINISH))
      {
         cycle_flag = true;
         c_flag = false;
         d_flag = false;

         Pass_Nr++;
         BeepBeep();
         Thread_Front(c_flag, d_flag);
      }
      
      else if (cycle_flag == false && Pass_Nr > Pass_Total + PASS_FINISH)
      {
         c_flag = false;
         d_flag = false;
         
         Limit_Pos_Front = Null_X_Pos;
         Thread_Front(c_flag, d_flag);
         
         Limit_Front_LED_Off();
         Limit_Rear_LED_Off();
         Limit_Pos_Front = Limit_Pos_Max;
         Limit_Pos_Rear = Limit_Pos_Min;         
      }
   }

   else if (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Front)
   {
      c_flag = false;
      d_flag = false;

      Thread_Left(c_flag, d_flag);
   }

   else if (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Front)
   {
      cycle_flag = false;
      c_flag = false;
      d_flag = false;
      
      Limit_Pos_Rear = (Null_X_Pos - REBOUND_X);
      Limit_Rear_LED_On();
      
      Thread_Rear(c_flag, d_flag);
   }

   else if (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Rear)
   {
      c_flag = false;
      d_flag = false;

      Thread_Right(c_flag, d_flag);
   }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Thread_Ext_Right()  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
{
   if ((Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Rear) || (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Null_X_Pos))
   {  
      Pass_Total = Thread_Info[Thread_Step].Pass;
      if (cycle_flag == false && Pass_Nr <= Pass_Total)
      {
         cycle_flag = true;
         c_flag = false;
         d_flag = false;
         
         float Step_mm = Thread_Info[Thread_Step].Step;
         float Pass_Depth = 0;
         if(Pass_Nr == 1) {Pass_Depth = ((Step_mm * 0.866) - (Step_mm * 0.866 / 6) - (Step_mm * 0.866 / 8)) / sqrt(Pass_Total-1) * sqrt(0.3);}
         else             {Pass_Depth = ((Step_mm * 0.866) - (Step_mm * 0.866 / 6) - (Step_mm * 0.866 / 8)) / sqrt(Pass_Total-1) * sqrt(Pass_Nr-1);}
         long Infeed_Value = long(Pass_Depth / ((float)SCREW_X/100 / MOTOR_X_STEP_PER_REV) + 0.5) * McSTEP_X + McSTEP_X;
         Limit_Pos_Front = (Null_X_Pos + Infeed_Value);
         Limit_Front_LED_On();
         Pass_Nr++;

         BeepBeep();
         Thread_Front(c_flag, d_flag);
      }

      else if ((cycle_flag == false) && (Pass_Nr > Pass_Total && Pass_Nr <= Pass_Total + PASS_FINISH))
      {
         cycle_flag = true;
         c_flag = false;
         d_flag = false;

         Pass_Nr++;
         BeepBeep();
         Thread_Front(c_flag, d_flag);
      }
      
      else if (cycle_flag == false && Pass_Nr > Pass_Total + PASS_FINISH)
      {
         c_flag = false;
         d_flag = false;
         
         Limit_Pos_Front = Null_X_Pos;
         Thread_Front(c_flag, d_flag);

         Limit_Front_LED_Off();
         Limit_Rear_LED_Off();
         Limit_Pos_Front = Limit_Pos_Max;
         Limit_Pos_Rear = Limit_Pos_Min;
      }
   }

   else if (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Front)
   {
      c_flag = false;
      d_flag = false;

      Thread_Right(c_flag, d_flag);
   }

   else if (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Front)
   {
      cycle_flag = false;
      c_flag = false;
      d_flag = false;
      
      Limit_Pos_Rear = (Null_X_Pos - REBOUND_X);
      Limit_Rear_LED_On();

      Thread_Rear(c_flag, d_flag);
   }

   else if (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Rear)
   {
      c_flag = false;
      d_flag = false;

      Thread_Left(c_flag, d_flag);
   }   
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Thread_Int_Left()  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
{
   if ((Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Front) || (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Null_X_Pos))
   {  
      Pass_Total = Thread_Info[Thread_Step].Pass;
      if (cycle_flag == false && Pass_Nr <= Pass_Total)
      {
         cycle_flag = true;
         c_flag = false;
         d_flag = false;
         
         float Step_mm = Thread_Info[Thread_Step].Step;
         float Pass_Depth = 0;
         if(Pass_Nr == 1) {Pass_Depth = ((Step_mm * 0.866) - (Step_mm * 0.866 / 4) - (Step_mm * 0.866 / 18)) / sqrt(Pass_Total-1) * sqrt(0.3);}
         else             {Pass_Depth = ((Step_mm * 0.866) - (Step_mm * 0.866 / 4) - (Step_mm * 0.866 / 18)) / sqrt(Pass_Total-1) * sqrt(Pass_Nr-1);}
         long Infeed_Value = long(Pass_Depth / ((float)SCREW_X/100 / MOTOR_X_STEP_PER_REV) + 0.5) * McSTEP_X + McSTEP_X;
         Limit_Pos_Rear = (Null_X_Pos - Infeed_Value);
         Limit_Rear_LED_On();
         Pass_Nr++;

         BeepBeep();
         Thread_Rear(c_flag, d_flag);
      }

      else if ((cycle_flag == false) && (Pass_Nr > Pass_Total && Pass_Nr <= Pass_Total + PASS_FINISH))
      {
         cycle_flag = true;
         c_flag = false;
         d_flag = false;

         Pass_Nr++;
         BeepBeep();
         Thread_Rear(c_flag, d_flag);
      }
      
      else if (cycle_flag == false && Pass_Nr > Pass_Total + PASS_FINISH)
      {
         c_flag = false;
         d_flag = false;
         
         Limit_Pos_Rear = Null_X_Pos;
         Thread_Rear(c_flag, d_flag);

         Limit_Front_LED_Off();
         Limit_Rear_LED_Off();
         Limit_Pos_Front = Limit_Pos_Max;
         Limit_Pos_Rear = Limit_Pos_Min;
      }
   }

   else if (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Rear)
   {
      c_flag = false;
      d_flag = false;

      Thread_Left(c_flag, d_flag);
   }

   else if (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Rear)
   {
      cycle_flag = false;
      c_flag = false;
      d_flag = false;
      
      Limit_Pos_Front = (Null_X_Pos + REBOUND_X);
      Limit_Front_LED_On();

      Thread_Front(c_flag, d_flag);
   }

   else if (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Front)
   {
      c_flag = false;
      d_flag = false;

      Thread_Right(c_flag, d_flag);
   } 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Thread_Int_Right()  //////////////////////////////////////////////////////////////////////////////////////////////////////////
{
   if ((Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Front) || (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Null_X_Pos))
   {  
      Pass_Total = Thread_Info[Thread_Step].Pass ;
      if (cycle_flag == false && Pass_Nr <= Pass_Total)
      {
         cycle_flag = true;
         c_flag = false;
         d_flag = false;
         
         float Step_mm = Thread_Info[Thread_Step].Step;
         float Pass_Depth = 0;
         if(Pass_Nr == 1) {Pass_Depth = ((Step_mm * 0.866) - (Step_mm * 0.866 / 4) - (Step_mm * 0.866 / 18)) / sqrt(Pass_Total-1) * sqrt(0.3);}
         else             {Pass_Depth = ((Step_mm * 0.866) - (Step_mm * 0.866 / 4) - (Step_mm * 0.866 / 18)) / sqrt(Pass_Total-1) * sqrt(Pass_Nr-1);}
         long Infeed_Value = long(Pass_Depth / ((float)SCREW_X/100 / MOTOR_X_STEP_PER_REV) + 0.5) * McSTEP_X + McSTEP_X;
         Limit_Pos_Rear = (Null_X_Pos - Infeed_Value);
         Limit_Rear_LED_On();
         Pass_Nr++;

         BeepBeep();
         Thread_Rear(c_flag, d_flag);
      }

      else if ((cycle_flag == false) && (Pass_Nr > Pass_Total && Pass_Nr <= Pass_Total + PASS_FINISH))
      {
         cycle_flag = true;
         c_flag = false;
         d_flag = false;

         Pass_Nr++;
         BeepBeep();
         Thread_Rear(c_flag, d_flag);
      }
      
      else if (cycle_flag == false && Pass_Nr > Pass_Total + PASS_FINISH)
      {
         c_flag = false;
         d_flag = false;
         
         Limit_Pos_Rear = Null_X_Pos;
         Thread_Rear(c_flag, d_flag);

         Limit_Front_LED_Off();
         Limit_Rear_LED_Off();
         Limit_Pos_Front = Limit_Pos_Max;
         Limit_Pos_Rear = Limit_Pos_Min;
      }
   }

   else if (Motor_Z_Pos == Limit_Pos_Left && Motor_X_Pos == Limit_Pos_Rear)
   {
      c_flag = false;
      d_flag = false;

      Thread_Right(c_flag, d_flag);
   }

   else if (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Rear)
   {
      cycle_flag = false;
      c_flag = false;
      d_flag = false;
      
      Limit_Pos_Front = (Null_X_Pos + REBOUND_X);
      Limit_Front_LED_On();

      Thread_Front(c_flag, d_flag);
   }

   else if (Motor_Z_Pos == Limit_Pos_Right && Motor_X_Pos == Limit_Pos_Front)
   {
      c_flag = false;
      d_flag = false;

      Thread_Left(c_flag, d_flag);
   } 
}

#define MAX_SPEED  40 // In timer value
#define MIN_SPEED  1000

#define EN_PIN 6    // Nano v3:   16 Mega:    38  //enable (CFG6)
#define DIR_PIN 8   //            19          55  //direction
#define STEP_PIN 9  //            18          54  //step
#define CS_PIN 10   //            17          40  //chip select

#define STEP_PORT   PORTF
#define STEP_BIT     0
#define STALL_VALUE 0 // [-64..63]
const long BAUD = 115200;

int speed = 10;
bool SHOW_SPI = false;
bool running = false;
float Rsense = 0.11;
float hold_x = 0.5;
boolean toggle1 = 0;
uint8_t stall_value = 9;
volatile uint32_t step_counter = 0;
const uint32_t steps_per_mm = 80 * 16; // @ 256 microsteps
const uint16_t fsteps_per_rotation = 255;
const uint32_t MHz = 16000000>>8; // Scaled by 256
const uint16_t acceleration = 2000;
unsigned long intevale_frequence = 10000;
bool vsense;
unsigned long microstep_ = 0;

#include <TMC2130Stepper.h>
#include <TMC2130Stepper_REGDEFS.h>
TMC2130Stepper myStepper = TMC2130Stepper(CS_PIN);

uint16_t rms_current(uint8_t CS, float Rsense = 0.11) {
  return (float)(CS+1)/32.0 * (vsense?0.180:0.325)/(Rsense+0.02) / 1.41421 * 1000;
}

void setup() {
  //init serial port
  {
    Serial.begin(115200); //init serial port and set baudrate
    while(!Serial); //wait for serial port to connect (needed for Leonardo only)
    Serial.println("\nStart...");
    pinMode(EN_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH); //deactivate myStepper (LOW active)
    digitalWrite(DIR_PIN, LOW); //LOW or HIGH
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(CS_PIN, HIGH);
    SPI.begin();
    pinMode(MISO, INPUT_PULLUP);
  }

  //set TMC2130 config
  {
    myStepper.push();
    myStepper.toff(3);
    myStepper.tbl(1);
    myStepper.hysteresis_start(4);
    myStepper.hysteresis_end(-2);
    myStepper.rms_current(500); // mA
    myStepper.microsteps(0);
    myStepper.diag1_stall(1);
    myStepper.diag1_active_high(1);
    myStepper.coolstep_min_speed(0xFFFFF); // 20bit max
    myStepper.THIGH(0);
    myStepper.semin(5);
    myStepper.semax(2);
    myStepper.sedn(0b01);
    myStepper.sg_stall_value(STALL_VALUE);
    myStepper.sfilt(1);
    myStepper.sgt(16);
  }

  // Set stepper interrupt
  {
    cli();//stop interrupts
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    OCR1A = 256;// = (16*10^6) / (1*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS11 bits for 8 prescaler
    TCCR1B |= (1 << CS11);// | (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei();//allow interrupts
  }

  //TMC2130 outputs on (LOW active)
  digitalWrite(EN_PIN, LOW);

  vsense = myStepper.vsense();
}

ISR(TIMER1_COMPA_vect){
	STEP_PORT |= 1 << STEP_BIT;
	STEP_PORT &= ~(1 << STEP_BIT);
	step_counter++;
}
void serialTuple(String cmd, int arg) {
	Serial.print("Received command: ");
	Serial.print(cmd);
	Serial.print("(");
	Serial.print(arg);
	Serial.println(")");
}

static uint32_t last_time=0;
unsigned long last_time2=0;
unsigned long temps=0;
int8_t read_byte = 0;
bool run_ = false;
bool only_run = false;
void loop()
{
  uint32_t ms = micros();
  while(Serial.available() > 0) {
    String cmd = Serial.readStringUntil(' ');
		String strArg = Serial.readStringUntil('\n');

    int arg = strArg.toInt();

    if (cmd == "run") {
			serialTuple("run", arg);
      run_ = arg? true: false;        
    }
    else if (cmd == "setCurrent") {
			serialTuple("setCurrent", arg);
			myStepper.setCurrent(arg, Rsense, hold_x);
		}
    else if (cmd == "microstep") {
      serialTuple("microstep", arg);
      myStepper.microsteps(arg);      
      microstep_ = arg;
    }
    else if (cmd == "only_run") {
			serialTuple("only run", arg);
      only_run = arg;
		}
    else if (cmd == "goto") {
			serialTuple("goto", arg);
      moveAngle(arg);
    }else if (cmd == "frequency") {
			serialTuple("frequency", arg);
      intevale_frequence = arg;      
		}else if (cmd == "shaft") {
			serialTuple("shaft", arg);
			myStepper.shaft(arg);
		}
    else if (cmd == "sgt") {
			serialTuple("sgt", arg);
			myStepper.sgt(arg);
		}    
    else {
			Serial.println("Invalid command!");
		}
  }
  if((ms-last_time) > 1000 && run_ && only_run) //run every 0.1s
  {
    last_time = ms;
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW); 
    uint32_t drv_status = myStepper.DRV_STATUS();
    Serial.print("measure,");
    Serial.print((drv_status & SG_RESULT_bm)>>SG_RESULT_bp , DEC);
    Serial.print(",");
    Serial.println(rms_current((drv_status & CS_ACTUAL_bm)>>CS_ACTUAL_bp), DEC);
    last_time2 = intevale_frequence-(micros()-last_time2);
    delayMicroseconds(last_time2>=0?last_time2:0);
    last_time2 = micros();
  }  
}

unsigned long time = 0;
unsigned long time_total = 0;
unsigned long current_time_ms = 0;
unsigned long current_time_ = 0;
double time_move = 0;
/*
* angle in °
*/
void moveAngle(double angle){
  if(run_ && !only_run){
    current_time_ms = 0;
    time_total = 0;
    if(angle<0){
      myStepper.shaft(1);
      angle = angle*(-1);
    }else{ myStepper.shaft(0);}
    microstep_ =  microstep_==0?1:microstep_;
    Serial.print("microstep : ");  
    Serial.println(microstep_);
    time_move = microstep_*(6600.0*200.0)*(angle/360)*(intevale_frequence/(10000.0))*1.5163;//*0.0713782; (21.5*200/255)*(angle*intevale_frequence/(5000));//*0.0713782; //microseconds // 21500 milliseconds, for full step with amplitude 255 --> 0.4268s/tour --->  1°-- 1185ms 
    Serial.print("estimated time (s): ");  
    Serial.println(time_move);  
    Serial.println(time_move/(1000000)); 
    while(time_total<time_move){
      current_time_ms = micros(); 
            
      digitalWrite(STEP_PIN, HIGH);
      digitalWrite(STEP_PIN, LOW);
      
      time_total += intevale_frequence;
      current_time_ms = intevale_frequence-(micros() - current_time_ms);
      delayMicroseconds(current_time_ms); // 
      
    }
  }  
}
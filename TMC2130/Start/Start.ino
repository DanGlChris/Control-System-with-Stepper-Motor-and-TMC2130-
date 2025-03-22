
/**
 * Author danglchris (daglox kankwanda)
 * Initializes the library and runs the stepper
 * motor in alternating directions.
 */
#include "./TMCStepper.h"

#define EN_PIN           26 // Enable
#define DIR_PIN          31 // Direction
#define STEP_PIN         27 // Step
#define CS_PIN           32 // Chip select
#define SW_MOSI          10 // Software Master Out Slave In (MOSI)
#define SW_MISO          11 // Software Master In Slave Out (MISO)
#define SW_SCK           9 // Software Slave Clock (SCK)

int8_t link = 1;
#define R_SENSE 0.11f
TMC2130Stepper driver(CS_PIN, SW_MOSI, SW_MISO, SW_SCK, link);

void setup() {
  
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware

  // TMC2130 config
  //driver.begin();                 //  SPI2: Init CS pins and possible SW SPI pins
  //driver.steadyCycle_init();       //  Initialization SteadyCycle Configuration
  
  //driver.toff(5);                 // Enables driver in software
  //driver.rms_current(600);        // Set motor RMS current
  //driver.microsteps(16);          // Set microsteps to 1/16th

  //driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  //driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  //driver.pwm_autoscale(true);     // Needed for stealthChop
}

//bool shaft = false;

void loop() {
  // Run 5000 steps and switch direction in software
  for (uint16_t i = 5000; i>0; i--) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(160);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(160);
  }
  //shaft = !shaft;
 // driver.shaft(shaft);
}

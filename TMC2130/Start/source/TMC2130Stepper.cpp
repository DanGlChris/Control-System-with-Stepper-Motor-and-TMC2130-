#include "../TMCStepper.h"
#include "./TMC_MACROS.h"
#include "./SPI2.h"



int8_t TMC2130Stepper::chain_length = 0;
uint32_t TMC2130Stepper::spi_speed = 16000000/8; //SPI2X, SPR1, SPR2 1 0 1 --> Fcl/8
//SPSR~&(1<<SPI2X) SPCR~&((1<<SPR1)|(0<<SPR1))

TMC2130Stepper::TMC2130Stepper(uint16_t pinCS, float RS, int8_t link = -1) :
  TMCStepper(RS),
  _pinCS(pinCS),
  link_index(link)
  {
    defaults();

    if (link > chain_length)
      chain_length = link;
  }

TMC2130Stepper::TMC2130Stepper(uint16_t pinCS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK, int8_t link = -1) :
  TMCStepper(default_RS),
  _pinCS(pinCS),
  link_index(link)
  {
    SPI2.init(pinMOSI, pinMISO, pinSCK, pinCS);
    defaults();

    if (link > chain_length)
      chain_length = link;
  }

TMC2130Stepper::TMC2130Stepper(uint16_t pinCS, float RS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK, int8_t link = -1) :
  TMCStepper(RS),
  _pinCS(pinCS),
  link_index(link)
  {
    SPI2.init(pinMOSI, pinMISO, pinSCK, pinCS);
    defaults();

    if (link > chain_length)
      chain_length = link;
  }

void TMC2130Stepper::defaults() {
  //MSLUT0_register.sr = ???;
  //MSLUT1_register.sr = ???;
  //MSLUT2_register.sr = ???;
  //MSLUT3_register.sr = ???;
  //MSLUT4_register.sr = ???;
  //MSLUT5_register.sr = ???;
  //MSLUT6_register.sr = ???;
  //MSLUT7_register.sr = ???;
  //MSLUTSTART_register.start_sin90 = 247;
  PWMCONF_register.sr = 0x00050480;
}

void TMC2130Stepper::setSPISpeed(uint32_t speed) {
  spi_speed = speed;
}

void TMC2130Stepper::switchCSpin(bool state) {
  digitalWrite(_pinCS, state); //SPI2 has Arduino.h
}

void TMC2130Stepper::beginTransaction() {
  SPI2.beginTransaction(SPI2Settings(spi_speed, MSBFIRST, SPI_MODE3));
}
void TMC2130Stepper::endTransaction() {
  SPI2.endTransaction();
}

uint8_t TMC2130Stepper::transfer(const uint8_t data) {
  uint8_t out = 0;
  out = SPI2.transfer(data);
  return out;
}

void TMC2130Stepper::transferEmptyBytes(const uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    transfer(0x00);
  }
}

uint32_t TMC2130Stepper::read(uint8_t addressByte) {
  uint32_t out = 0UL;
  int8_t i = 1;

  beginTransaction();
  switchCSpin(LOW);
  transfer(addressByte);
  // Clear SPI2
  transferEmptyBytes(4);

  // Shift the written data to the correct driver in chain
  // Default link_index = -1 and no shifting happens
  while(i < link_index) {
    transferEmptyBytes(5);
    i++;
  }

  switchCSpin(HIGH);
  switchCSpin(LOW);

  // Shift data from target link into the last one...
  while(i < chain_length) {
    transferEmptyBytes(5);
    i++;
  }

  // ...and once more to MCU
  status_response = transfer(addressByte); // Send the address byte again
  SPI_STATUS_register.sr = status_response;
  out  = transfer(0x00);
  out <<= 8;
  out |= transfer(0x00);
  out <<= 8;
  out |= transfer(0x00);
  out <<= 8;
  out |= transfer(0x00);

  endTransaction();
  switchCSpin(HIGH);
  return out;
}

void TMC2130Stepper::write(uint8_t addressByte, uint32_t config) {
  addressByte |= TMC_WRITE;
  int8_t i = 1;

  beginTransaction();
  switchCSpin(LOW);
  status_response = transfer(addressByte);
  transfer(config>>24);
  transfer(config>>16);
  transfer(config>>8);
  transfer(config);

  // Shift the written data to the correct driver in chain
  // Default link_index = -1 and no shifting happens
  while(i < link_index) {
    transferEmptyBytes(5);
    i++;
  }

  endTransaction();
  switchCSpin(HIGH);
}

void TMC2130Stepper::begin() {  
  //clear(); // To prevent reinitialization

  SPI2.begin();

  GCONF(GCONF_register.sr);
  CHOPCONF(CHOPCONF_register.sr);
  COOLCONF(COOLCONF_register.sr);
  PWMCONF(PWMCONF_register.sr);
  IHOLD_IRUN(IHOLD_IRUN_register.sr);

  toff(8); //off_time(8);
  tbl(1); //blank_time(24);
}

void TMC2130Stepper::end() {
  SPI2.end();
}

/**
 * @brief SteadyCycle Initialization  
 * 
 */
void TMC2130Stepper::steadyCycle_init(){  
  CHOPCONF(0x000100C3); // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle)
  IHOLD_IRUN(0x00061F0A); // IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
  TPOWERDOWN(0x0000000A); // TPOWERDOWN=10: Delay before power down in stand still
  CHOPCONF(0x000100C7); // EN_PWM_MODE=1 enables StealthChop (with default PWMCONF)
  TPWMTHRS(0x000001F4); // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
  PWMCONF(0x000401C8); // PWMCONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1
  
  toff(8); //off_time(8);
  tbl(1); //blank_time(24);
}


/**
 *  Helper functions
 */

bool TMC2130Stepper::isEnabled() { return !drv_enn_cfg6() && toff(); }

/*
*/
void TMC2130Stepper::push() {
  GCONF(GCONF_register.sr);
  IHOLD_IRUN(IHOLD_IRUN_register.sr);
  TPOWERDOWN(TPOWERDOWN_register.sr);
  TPWMTHRS(TPWMTHRS_register.sr);
  TCOOLTHRS(TCOOLTHRS_register.sr);
  THIGH(THIGH_register.sr);
  XDIRECT(XDIRECT_register.sr);
  VDCMIN(VDCMIN_register.sr);
  CHOPCONF(CHOPCONF_register.sr);
  COOLCONF(COOLCONF_register.sr);
  DCCTRL(DCCTRL_register.sr);
  PWMCONF(PWMCONF_register.sr);
  ENCM_CTRL(ENCM_CTRL_register.sr);
}

///////////////////////////////////////////////////////////////////////////////////////
// R: SPI_STATUS
uint8_t TMC2130Stepper::SPI_STATUS() { return SPI_STATUS_register.sr;}
bool TMC2130Stepper::reset_flag(){return SPI_STATUS_register.reset_flag;}
bool TMC2130Stepper::driver_error(){return SPI_STATUS_register.driver_error;}
bool TMC2130Stepper::sg2(){return SPI_STATUS_register.sg2;}
bool TMC2130Stepper::standstill(){return SPI_STATUS_register.standstill;}

///////////////////////////////////////////////////////////////////////////////////////
// R: IOIN
uint32_t  TMC2130Stepper::IOIN()    { return read(IOIN_register.address); }

bool TMC2130Stepper::step()         { IOIN_t r{0}; r.sr = IOIN(); return r.step; }
bool TMC2130Stepper::dir()          { IOIN_t r{0}; r.sr = IOIN(); return r.dir; }
bool TMC2130Stepper::dcen_cfg4()    { IOIN_t r{0}; r.sr = IOIN(); return r.dcen_cfg4; }
bool TMC2130Stepper::dcin_cfg5()    { IOIN_t r{0}; r.sr = IOIN(); return r.dcin_cfg5; }
bool TMC2130Stepper::drv_enn_cfg6() { IOIN_t r{0}; r.sr = IOIN(); return r.drv_enn_cfg6; }
bool TMC2130Stepper::dco()          { IOIN_t r{0}; r.sr = IOIN(); return r.dco; }
uint8_t TMC2130Stepper::version()   { IOIN_t r{0}; r.sr = IOIN(); return r.version; }

///////////////////////////////////////////////////////////////////////////////////////
// RW: GCONF
uint32_t TMC2130Stepper::GCONF() { return read(GCONF_register.address); }
void TMC2130Stepper::GCONF(uint32_t input) {
  GCONF_register.sr = input;
  write(GCONF_register.address, GCONF_register.sr);
}
////////////////////////////////////////////////////////////////////////////
// R: DRV_STATUS
uint32_t TMC2130Stepper::DRV_STATUS() { return read(DRV_STATUS_register.address); }

// W: TCOOLTHRS
uint32_t TMC2130Stepper::TCOOLTHRS() { return TCOOLTHRS_register.sr; }
void TMC2130Stepper::TCOOLTHRS(uint32_t input) {
  TCOOLTHRS_register.sr = input;
  write(TCOOLTHRS_register.address, TCOOLTHRS_register.sr);
}
///////////////////////////////////////////////////////////////////////////////////////
// W: THIGH
uint32_t TMC2130Stepper::THIGH() { return THIGH_register.sr; }
void TMC2130Stepper::THIGH(uint32_t input) {
  THIGH_register.sr = input;
  write(THIGH_register.address, THIGH_register.sr);
}
///////////////////////////////////////////////////////////////////////////////////////
// RW: XDIRECT
uint32_t TMC2130Stepper::XDIRECT() {
  return read(XDIRECT_register.address);
}
void TMC2130Stepper::XDIRECT(uint32_t input) {
  XDIRECT_register.sr = input;
  write(XDIRECT_register.address, XDIRECT_register.sr);
}

///////////////////////////////////////////////////////////////////////////////////////
// RW: CHOPCONF
uint32_t TMC2130Stepper::CHOPCONF() { return read(CHOPCONF_register.address); }
void TMC2130Stepper::CHOPCONF(uint32_t input) {
  CHOPCONF_register.sr = input;
  write(CHOPCONF_register.address, CHOPCONF_register.sr);
}
void TMC2130Stepper::toff(uint8_t input){CHOPCONF_register.toff = input; CHOPCONF(CHOPCONF_register.sr);} // Write Data toff
void TMC2130Stepper::tbl(uint8_t input){CHOPCONF_register.tbl = input; CHOPCONF(CHOPCONF_register.sr);}


uint8_t TMC2130Stepper::sedn() {}

///////////////////////////////////////////////////////////////////////////////////////
// RW: PWMCONF
uint32_t TMC2130Stepper::PWMCONF() { return read(PWMCONF_register.address); }
void TMC2130Stepper::PWMCONF(uint32_t input) {
  PWMCONF_register.sr = input;
  write(PWMCONF_register.address, PWMCONF_register.sr);
}


void TMC2130Stepper::coil_A(int16_t B)  { XDIRECT_register.coil_A = B; write(XDIRECT_register.address, XDIRECT_register.sr); }
void TMC2130Stepper::coil_B(int16_t B)  { XDIRECT_register.coil_B = B; write(XDIRECT_register.address, XDIRECT_register.sr); }
int16_t TMC2130Stepper::coil_A()        { XDIRECT_t r{0}; r.sr = XDIRECT(); return r.coil_A; }
int16_t TMC2130Stepper::coil_B()        { XDIRECT_t r{0}; r.sr = XDIRECT(); return r.coil_B; }
///////////////////////////////////////////////////////////////////////////////////////
// W: VDCMIN
uint32_t TMC2130Stepper::VDCMIN() { return VDCMIN_register.sr; }
void TMC2130Stepper::VDCMIN(uint32_t input) {
  VDCMIN_register.sr = input;
  write(VDCMIN_register.address, VDCMIN_register.sr);
}
///////////////////////////////////////////////////////////////////////////////////////
// W: DCCTRL

uint32_t TMC2130Stepper::DCCTRL() {
	return read(DCCTRL_register.address);
}
void TMC2130Stepper::DCCTRL(uint32_t input) {
	DCCTRL_register.sr = input;
	write(DCCTRL_register.address, DCCTRL_register.sr);
}

// DCCTRL.DC_TIME
void TMC2130Stepper::dc_time(uint16_t input) {
	DCCTRL_register.dc_time = input;
	write(DCCTRL_register.address, DCCTRL_register.sr);
}

//DCCTRL.DC_SG
void TMC2130Stepper::dc_sg(uint8_t input) {
	DCCTRL_register.dc_sg = input;
	write(DCCTRL_register.address, DCCTRL_register.sr);
}

uint16_t TMC2130Stepper::dc_time() {
	DCCTRL_t r{0};
  r.sr = DCCTRL();
	return r.dc_time;
}
uint8_t TMC2130Stepper::dc_sg() {
	DCCTRL_t r{0};
  r.sr = DCCTRL();
	return r.dc_sg;
}
///////////////////////////////////////////////////////////////////////////////////////
// R: PWM_SCALE
uint8_t TMC2130Stepper::PWM_SCALE() { return read(PWM_SCALE_register.address); }
///////////////////////////////////////////////////////////////////////////////////////
// W: ENCM_CTRL
uint8_t TMC2130Stepper::ENCM_CTRL() { return ENCM_CTRL_register.sr; }
void TMC2130Stepper::ENCM_CTRL(uint8_t input) {
  ENCM_CTRL_register.sr = input;
  write(ENCM_CTRL_register.address, ENCM_CTRL_register.sr);
}
void TMC2130Stepper::inv(bool B)      { ENCM_CTRL_register.inv = B;       write(ENCM_CTRL_register.address, ENCM_CTRL_register.sr); }
void TMC2130Stepper::maxspeed(bool B) { ENCM_CTRL_register.maxspeed  = B; write(ENCM_CTRL_register.address, ENCM_CTRL_register.sr); }
bool TMC2130Stepper::inv()            { return ENCM_CTRL_register.inv; }
bool TMC2130Stepper::maxspeed()       { return ENCM_CTRL_register.maxspeed; }
///////////////////////////////////////////////////////////////////////////////////////
// R: LOST_STEPS
uint32_t TMC2130Stepper::LOST_STEPS() { return read(LOST_STEPS_register.address); }

void TMC2130Stepper::sg_current_decrease(uint8_t value) {
  switch(value) {
    case 32: sedn(0b00); break;
    case  8: sedn(0b01); break;
    case  2: sedn(0b10); break;
    case  1: sedn(0b11); break;
  }
}
uint8_t TMC2130Stepper::sg_current_decrease() {
  switch(sedn()) {
    case 0b00: return 32;
    case 0b01: return  8;
    case 0b10: return  2;
    case 0b11: return  1;
  }
  return 0;
}

void TMC2130Stepper::clear(){
  CLEAR_REGISTER(IHOLD_IRUN)= {};	// 32b IHOLD_IRUN_t IHOLD_IRUN_register = {}
  CLEAR_REGISTER(TPOWERDOWN)= {};		// 8b
  CLEAR_REGISTER(TPWMTHRS)= {};			// 32b
  CLEAR_REGISTER(TSTEP)= {};			// 32b
  CLEAR_REGISTER(MSCNT)= {};			// 32b
  CLEAR_REGISTER(MSCURACT)= {};			// 32b
  CLEAR_REGISTER(DRV_STATUS)= {};
  CLEAR_REGISTER(GCONF) = {};		// 32b
  CLEAR_REGISTER(TCOOLTHRS) = {};	// 32b
  CLEAR_REGISTER(THIGH)= {};		// 32b
  CLEAR_REGISTER(XDIRECT)= {};		// 32b
  CLEAR_REGISTER(VDCMIN)= {};		// 32b
  CLEAR_REGISTER(CHOPCONF)= {};	// 32b
  CLEAR_REGISTER(COOLCONF)= {};	// 32b
  CLEAR_REGISTER(DCCTRL)= {};		// 32b
  CLEAR_REGISTER(PWMCONF)= {};		// 32b
  CLEAR_REGISTER(ENCM_CTRL)= {};	//  8b
  CLEAR_REGISTER(DRV_STATUS)= {};	//  32b
  CLEAR_REGISTER(IOIN)= {};		//  32b
  CLEAR_REGISTER(PWM_SCALE)= {};	//  8b
  CLEAR_REGISTER(LOST_STEPS)= {};	//  32b
  CLEAR_REGISTER(SPI_STATUS)= {};	// 8b
}

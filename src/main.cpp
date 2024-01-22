#include "pico/stdlib.h"
#include "stdio.h"
#include "config.h"
#include "hardware/i2c.h"
#include "MS5611.h"
#include "SPL06.h"
#include "BMP280.h"
#include "ms4525.h"
#include "sdp3x.h"
#include "vario.h"
#include "voltage.h"
#include "gps.h"
#include "tusb.h"
#include "crsf_in.h"
#include "crsf_out.h"
#include "param.h"
#include "hardware/pio.h"
#include "sbus_out_pwm.h"
#include "sbus_in.h"
#include "hardware/watchdog.h"
#include "tools.h"
#include "sport.h"
#include "jeti.h"
#include "exbus.h"
#include "hott.h"
#include "mpx.h"
#include "ibus.h"
#include "sbus2_tlm.h"
#include "fbus.h"
#include "srxl2.h"
//#include "param.h"

#include "ws2812.h"
#include "rpm.h"
#include "EMFButton.h"
#include "ads1115.h"
#include "mpu.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "ds18b20.h"
#include "hardware/timer.h"

#include "hardware/rtc.h"

#include "sd_card.h"
#include "ff.h"

// to do : add rpm, temp telemetry fields to jeti protocol
//         support ex bus jeti protocol on top of ex jeti protocol (not sure it makes lot of sense because bandwitdth is limited)
//         add switching 8 gpio from one channel
//         try to detect MS5611 and other I2C testing the different I2C addresses
//         if ds18b20 would be supported, then change the code in order to avoid long waiting time that should block other tasks.
//         stop core1 when there is no I2C activity while saving the config (to avoid I2C conflict)
//         add airspeed field and compensated Vspeed to all protocols (currently it is only in sport)
//         add spektrum protocol (read the bus already in set up, change baudrate, fill all fields in different frames)
//         look to use pitch and roll to stabilize 2 servos for a gimball

// Look at file in folder "doc" for more details
//
// So pio 0 sm0 is used for CRSF Tx  or for Sport TX or JETI TX or HOTT TX or MPX TX
//        0   1                         for Sport Rx            or HOTT RX or MPX RX
//        0   2            sbus out             

//        1   0 is used for gps Tx  (was used for one pwm)        
//        1   1 is used for gps Rx  (was used for one pwm)
//        1   3 is used for RGB led
// So UART0 is used for Secondary crsf of Sbus in ( was GPS before)
//    UART1 is used for primary crsf in of SBUS IN (was only Sbus in before)
 
// Pin that can be used are:
// C1 = 0/15  ... C16 = 0/15
// GPS_TX = 0/29
// GPS_RX = 0/29
// PRI = 5 ,9, 21 ,25  (UART1)
// SEC = 1, 13 , 17 ,29 (UART0) 
// SBUS_OUT = 0/29
// TLM = 0/29
// VOLT1= 26/29 ... VOLT4 = 26/29
// SDA = 2, 6, 10, 14, 18, 22, 26  (I2C1)
// SCL = 3, 7, 11, 15, 19, 23, 27  (I2C1)
// RPM = 0/29 
// LED = 16



VOLTAGE voltage ;    // class to handle voltages

MS5611 baro1( (uint8_t) 0x77  );    // class to handle MS5611; adress = 0x77 or 0x76
SPL06 baro2( (uint8_t) 0x76  );    // class to handle SPL06; adress = 0x77 or 0x76
BMP280 baro3( (uint8_t) 0x76) ;    // class to handle BMP280; adress = 0x77 or 0x76

ADS1115 adc1( I2C_ADS_Add1 , 0) ;     // class to handle first ads1115 (adr pin connected to grnd)
ADS1115 adc2( I2C_ADS_Add2 , 1) ;     // class to handle second ads1115 (adr pin connected to vdd)

MS4525 ms4525 ( (uint8_t) MS4525_ADDRESS ) ; // 0x28 is the default I2C adress of a 4525DO sensor)
SDP3X sdp3x( (uint8_t) SDPXX_ADDRESS) ;      // 0X21 is the default I2C address of asdp31,... sensor (diffrent for sdp8xx)

//XGZP6897D
#include "XGZP6897D.h"
XGZP6897D xgzp6897d (8192);


VARIO vario1;

//queue_t gpsQueue ; // queue is used to transfer the data from the uart0 used by GPS
GPS gps;

// objet to manage the mpu6050
MPU mpu(1);

field fields[NUMBER_MAX_IDX];  // list of all telemetry fields and parameters that can be measured (not only used by Sport)

// remapping from sbus value to pwm value
uint16_t fromSbusMin = FROM_SBUS_MIN;
uint16_t toPwmMin = TO_PWM_MIN; 
uint16_t fromSbusMax = FROM_SBUS_MAX;
uint16_t toPwmMax = TO_PWM_MAX; 

// CRSF is managed with 2 pio and not with the internal uart1 in order to freely select the pins

EMFButton btn (3, 0); // button object will be associated to the boot button of rp2040; requires a special function to get the state (see tool.cpp)
                       // parameters are not used with RP2040 boot button 
extern uint32_t lastRcChannels;
extern CONFIG config;
bool configIsValid = true;
bool configIsValidPrev = true;
bool blinking = true ;
uint8_t ledState = STATE_NO_SIGNAL;
uint8_t prevLedState = STATE_NO_SIGNAL;

uint32_t lastBlinkMillis;

queue_t qSensorData;       // send one sensor data to core0; when type=0XFF, it means a command then data= the command (e.g.0XFFFFFFFF = save config)
queue_t qSendCmdToCore1;
volatile bool core1SetupDone = false;
void core1_main(); // prototype of core 1 main function

uint8_t forcedFields = 0; // use to debug a protocol; force the values when = 'P' (positive) or 'N' (negative)

int32_t cameraPitch;
int32_t cameraRoll;

void setupI2c(){
    if ( config.pinScl == 255 || config.pinSda == 255) return; // skip if pins are not defined
    // send 10 SCL clock to force sensor to release sda
    /*
    gpio_init(config.pinSda);
    gpio_init(config.pinScl);
    gpio_set_dir(config.pinSda, GPIO_IN);
    gpio_set_dir(config.pinScl, GPIO_OUT);
    gpio_pull_up(config.pinSda);
    gpio_pull_up(config.pinScl);
    sleep_us(10);
    while ( gpio_get(config.pinSda) == 0) {;
        
        for (uint8_t i=0; i<9; i++){
            gpio_put(config.pinScl, 1);
            sleep_us(10);
            gpio_put(config.pinScl, 0);
            sleep_us(10);
            printf("trying to unlock I2C\n");
        }
    }    
    gpio_put(config.pinScl, 1);
    gpio_set_dir(config.pinSda, GPIO_OUT);
    gpio_put(config.pinScl, 0);
    sleep_us(10);
    gpio_put(config.pinSda, 0);
    sleep_us(10);
    gpio_put(config.pinScl, 1);
    sleep_us(10);
    gpio_put(config.pinSda, 1);
    sleep_us(10);
    gpio_set_dir(config.pinSda, GPIO_IN);
    if ( gpio_get(config.pinSda) == 0) printf("I2C still locked\n");    
    */
    // initialize I2C     
    i2c_init( i2c1, 400 * 1000);
    gpio_set_function(config.pinSda, GPIO_FUNC_I2C);
    gpio_set_function(config.pinScl, GPIO_FUNC_I2C);
    gpio_pull_up(config.pinSda);
    gpio_pull_up(config.pinScl); 
}

void setupSensors(){     // this runs on core1!!!!!!!!
      
      //sleep_ms(3000);
      //printf("start core1 setup\n");
      //startTimerUs(0); //xxxxx to debug only 
      voltage.begin();      
      //printf("voltage done\n");
      setupI2c();      // setup I2C
      //printf("i2C done\n");
      baro1.begin();  // check MS5611; when ok, baro1.baroInstalled  = true
      //printf("baro1 done\n");
      if (! baro1.baroInstalled) {
        baro2.begin();  // check SPL06;  when ok, baro2.baroInstalled  = true
        //printf("baro2 done\n");
      }
      if (! ( baro1.baroInstalled or baro2.baroInstalled)) {
        baro3.begin(); // check BMP280;  when ok, baro3.baroInstalled  = true
        //printf("baro3 done\n");
      }
      adc1.begin() ; 
      //printf("adc1 done\n");
      adc2.begin() ;
      //printf("adc2 done\n");
      mpu.begin(); 
      //printf("mpu done\n");
    //blinkRgb(0,10,0,500,1000000); blink red, green, blue at 500 msec for 1000 0000 X
      gps.setupGps();  //use a Pio
      //printf("gps done\n");
      ms4525.begin();
      if (! ms4525.airspeedInstalled) {
        sdp3x.begin();
      }
      if (! sdp3x.airspeedInstalled) {
        xgzp6897d.begin();
      }
      #ifdef USEDS18B20
      ds18b20Setup(); 
      #endif
      setupRpm(); // this function perform the setup of pio Rpm
      //printf("rpm done\n");
      
      core1SetupDone = true;
      //printf("end core1 setup\n") ;    
      //getTimerUs(0);   // xxxxx
}

void getSensors(void){      // this runs on core1 !!!!!!!!!!!!
  voltage.getVoltages();
  if ( baro1.baroInstalled){
    if ( baro1.getAltitude() == 0) { // if an altitude is calculated
      vario1.calculateAltVspeed(baro1.altitudeCm , baro1.altIntervalMicros ); // Then calculate Vspeed ...   
    }
  } else if ( baro2.baroInstalled){
    if ( baro2.getAltitude() == 0) { // if an altitude is calculated
      vario1.calculateAltVspeed(baro2.altitudeCm , baro2.altIntervalMicros); // Then calculate Vspeed ... 
    }
  } else if (baro3.baroInstalled) {
    if ( baro3.getAltitude() == 0) { // if an altitude is calculated
      vario1.calculateAltVspeed(baro3.altitudeCm , baro3.altIntervalMicros); // Then calculate Vspeed ... 
    }
  }
  adc1.readSensor(); 
  adc2.readSensor();
  mpu.getAccZWorld();  
  gps.readGps();
  if (ms4525.airspeedInstalled){
    ms4525.getDifPressure();
    calculateAirspeed( );
    vario1.calculateVspeedDte();
  }
  if (sdp3x.airspeedInstalled){
    sdp3x.getDifPressure();
    calculateAirspeed( );
    vario1.calculateVspeedDte();
  } 
  if (xgzp6897d.airspeedInstalled){
    xgzp6897d.getDifPressure();
    calculateAirspeed( );
    vario1.calculateVspeedDte();
  } 
  readRpm();
  #ifdef USE_DS18B20
  ds18b20Read(); 
  #endif
}

void mergeSeveralSensors(void){
}

void setColorState(){    // set the colors based on the RF link
    lastBlinkMillis = millisRp(); // reset the timestamp for blinking
    switch (ledState) {
        case STATE_OK:
            setRgbColorOn(0, 10, 0); //green
            break;
        case STATE_PARTLY_OK:
            setRgbColorOn(10, 5, 0); //yellow
            break;
        case STATE_FAILSAFE:
            setRgbColorOn(0, 0, 10); //blue
            break;
        default:
            setRgbColorOn(10, 0, 0); //red
            break;     
    }
}

enum bootButtonStates_t {NOT_ARMED, ARMED, SAVED};
bootButtonStates_t bootButtonState =  NOT_ARMED;

void handleBootButton(){ 
    // check boot button; after double click, change LED to fix blue and next HOLD within 5 sec save the current channnels as Failsafe values
    static uint32_t bootArmedMillis = 0;
    btn.tick();
    if ( ( btn.hasClicks() == 2) && ( (millisRp() - lastRcChannels ) < 100 ) ) { // double click + a recent frame exist
        bootArmedMillis = millisRp();
        bootButtonState =  ARMED;
        setRgbColorOn(0, 0, 10); //blue
        //printf("armed\n");
    } else if (bootButtonState == ARMED) {
        if (btn.hasOnlyHeld() && ( (millisRp() - lastRcChannels ) < 100 )) {     // saved when long hold + recent frame exist
            bootButtonState =  SAVED;
            setRgbColorOn(5, 5, 5);
            bootArmedMillis = millisRp();
            cpyChannelsAndSaveConfig();   // copy the channels values and save them into the config.
            //printf("saving failsafe\n");
        } else if ( (millisRp() - bootArmedMillis) > 5000) {
            bootButtonState =  NOT_ARMED;  // reset if no hold withing the 5 sec
            setColorState();               // restore colors based on RF link
            //printf("loosing armed\n");
        }
    } else if ((bootButtonState == SAVED) && ((millisRp() - bootArmedMillis) > 2000) ){
        bootButtonState =  NOT_ARMED;  // reset after 2 sec
        setColorState();               // restore colors based on RF link 
        //printf("done\n");
    }
}

volatile uint8_t testU8 = 0;
void test_callback(uint alarmNum){
    testU8++;
}

char sdFileName[15];

void setupSD(){
  FRESULT fr;
  FATFS fs;
  FIL fil;
  int ret;
  int fileNum;
  char line[100];
  char filename[] = "flightlog.txt";
  setRgbColorOn(10, 5, 0); //Set yellow
  //Check if log.conf exists
  //Last date, number > into variable

  //Activate flag when GPS lat and long is not N/A
  //When activate, if last date != current date, set number to 1, else increase number
  //Set file name to GPS Date_No.csv
  //Input variable headers
  //Start loop when flag is active

  if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
        setRgbColorOn(10, 5, 0);
        while (true){
          printf("Setup SD failed \n");
          sleep_ms(1000);
        };
  }

  /*
  printf("\n TRYING TO MOUNT 1\n");
  sleep_ms(1000);
  printf("\n TRYING TO MOUNT 2\n");
  sleep_ms(1000);
  printf("\n TRYING TO MOUNT 3\n");
  sleep_ms(1000);
  printf("\n TRYING TO MOUNT 4\n");
  sleep_ms(1000);
  */

  fr = f_mount(&fs, "0:", 1);

  //printf("\n MOUNTED \n");

  if (fr == FR_OK){
    /*
    printf("SD Card Detected1\n");
    sleep_ms(1000);
    printf("SD Card Detected2\n");
    sleep_ms(1000);
    printf("SD Card Detected3\n");
    sleep_ms(1000);
    printf("SD Card Detected4\n");
    sleep_ms(1000);
    printf("SD Card Detected5\n");
    sleep_ms(1000);
    */
    setRgbColorOn(10, 5, 0);
  }
  
  fr = f_open(&fil, filename, FA_READ);

  f_gets(line, sizeof line, &fil);
  //printf("LINE %s\n", line);
  //printf("Length %d\n", strlen(line));
  
  if (int(strlen(line)) == 12){
    //printf("CHAR 7 %c", line[7]);
    fileNum = int(line[7]-'0');
  }
  else if (int(strlen(line)) == 13) {
    fileNum = int(line[7]-'0') * 10 + int(line[8]-'0');
  }
  else if (int(strlen(line)) == 14) {
    fileNum = int(line[7]-'0') * 100 + int(line[8]-'0') * 10 * int(line[9]-'0');
  }
  else {
    fr = f_close(&fil);
    fr = f_open(&fil, filename, FA_WRITE | FA_OPEN_APPEND);
    sprintf(sdFileName, "logfile1.csv");
    f_printf(&fil, "logfile1.csv");
    //fr = f_close(&fil);
    
    fileNum = 0;
  }

  fileNum++;

  //printf("NUMBER %d\n",fileNum);

  sprintf(sdFileName, "logfile%d.csv", fileNum);
  
  fr = f_close(&fil);  
  

  //Write new filename
  fr = f_open(&fil, filename, FA_WRITE);
  f_printf(&fil, sdFileName);
  fr = f_close(&fil);
  

  //printf("Filename NEW %s\n",sdFileName);

  //Print header for new filename
  fr = f_open(&fil, sdFileName, FA_WRITE | FA_OPEN_APPEND);
  f_printf(&fil, "LATITUDE,LONGITUDE,GROUNDSPEED,HEADING,ALTITUDE,GPS_DATE,GPS_TIME,GPS_HOME_BEARING,GPS_HOME_DISTANCE,GPS_CUMUL_DIST,VSPEED,RELATIVEALT,PITCH,ROLL,YAW,VTAS,CURR,COMPENSATED_VSPEED,AIRSPEED\n");
  fr = f_close(&fil);
  /*
  if (fr != FR_OK) {
      printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
      while (true);
  }

  // Open file for writing ()
  fr = f_open(&fil, filename, FA_WRITE | FA_OPEN_APPEND);
  if (fr != FR_OK) {
      printf("ERROR: Could not open file (%d)\r\n", fr);
      while (true);
  }

  // Write something to file
  ret = f_printf(&fil, "This is another test\r\n");
  if (ret < 0) {
      printf("ERROR: Could not write to file (%d)\r\n", ret);
      f_close(&fil);
      while (true);
  }
  ret = f_printf(&fil, "of writing to an SD card.\r\n");
  if (ret < 0) {
      printf("ERROR: Could not write to file (%d)\r\n", ret);
      f_close(&fil);
      while (true);
  }

  // Close file
  fr = f_close(&fil);
  if (fr != FR_OK) {
      printf("ERROR: Could not close file (%d)\r\n", fr);
      while (true);
  }

  // Open file for reading
  fr = f_open(&fil, filename, FA_READ);
  if (fr != FR_OK) {
      printf("ERROR: Could not open file (%d)\r\n", fr);
      while (true);
  }

  // Print every line in file over serial
  printf("Reading from file '%s':\r\n", filename);
  printf("---\r\n");
  while (f_gets(buf, sizeof(buf), &fil)) {
      printf(buf);
  }
  printf("\r\n---\r\n");

  // Close file
  fr = f_close(&fil);
  if (fr != FR_OK) {
      printf("ERROR: Could not close file (%d)\r\n", fr);
      while (true);
  }

  // Unmount drive
  f_unmount("0:");

  */
}

void setup() {
  stdio_init_all();
  setupSD();
  bool clockChanged; 
  clockChanged = set_sys_clock_khz(133000, false);
  setupLed();
  setRgbColorOn(10,0,10); // start with 2 color
  
  rtc_init();

  datetime_t defaultTime = {
        .year = 2023,
        .month = 1,
        .day = 1,
        //.dotw = 0,  // 0 is Sunday, so 5 is Friday
        .hour = 0,
        .min = 0,
        .sec = 0
    };
    
    rtc_set_datetime(&defaultTime);

  #ifdef DEBUG
  
  uint16_t counter = 10;                      // after an upload, watchdog_cause_reboot is true.
  //if ( watchdog_caused_reboot() ) counter = 0; // avoid the UDC wait time when reboot is caused by the watchdog   
  while ( (!tud_cdc_connected()) && (counter--)) { 
  //while ( (!tud_cdc_connected()) ) { 
    sleep_ms(100);
    toggleRgb();
    }
  sleep_ms(2000);  // in debug mode, wait a little to let USB on PC be able to display all messages
  uint8_t a1[2] = {1, 2};
  int debug = 2;
  debugAX("aa", a1 , 2);
  //int debug = 2;
  //dp("test\n");
  // test
  //int32_t testValue = -10;
  //printf("rounding -10 = %d\n" , ( int_round(testValue , 100) ) +500);
  //testValue = -60;
  //printf("rounding -60 = %d\n" , ( int_round(testValue , 100) ) +500);
  //testValue = -200;
  //printf("rounding -200 = %d\n" , ( int_round(testValue , 100) ) +500);
  //testValue = +10;
  //printf("rounding +10 = %d\n" , ( int_round(testValue , 100) ) +500);
  //testValue = +60;
  //printf("rounding +60 = %d\n" , ( int_round(testValue , 100) ) +500);
  //uint8_t readBuffer[2] = {0XFF, 0XFE}; 
  //printf("test %f\n",(float) ((int16_t) (readBuffer[0] << 8 | readBuffer[1] & 0X00FF)));
  //int16_t test = 0X7B03;
  //int32_t testi32 = (int32_t) test;
  //uint8_t testFirst = * (&test);
  //printf("test %X %X %X\n",  test , testi32 , testFirst) ; 
  //if (hardware_alarm_is_claimed(0)) printf("alarm 0 is used\n");
  //if (hardware_alarm_is_claimed(1)) printf("alarm 1 is used\n");
  //if (hardware_alarm_is_claimed(2)) printf("alarm 2 is used\n");
  //if (hardware_alarm_is_claimed(3)) printf("alarm 3 is used\n");
  //hardware_alarm_set_callback(2 , test_callback);
  //hardware_alarm_set_target(2 , time_us_64()+200);
  //hardware_alarm_set_target(2 , time_us_64()+700);
  //sleep_us(100);
  //printf("testU8= %d\n", (int) testU8);
  //sleep_us(200);
  //printf("testU8= %d\n", (int) testU8);
  //sleep_us(700);
  //printf("testU8= %d\n", (int) testU8);
  #endif
  
  if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
    } else {
        printf("Clean boot\n");
        //sleep_ms(1000); // wait that GPS is initialized
    }
  setRgbColorOn(0,0,10);  // switch to blue during the setup of different sensors/pio/uart
  setupConfig(); // retrieve the config parameters (crsf baudrate, voltage scale & offset, type of gps, failsafe settings)  
  if (configIsValid){ // continue with setup only if config is valid
      for (uint8_t i = 0 ;  i< NUMBER_MAX_IDX ; i++){ // initialise the list of fields being used 
        fields[i].value= 0;
        fields[i].available= false;
        fields[i].onceAvailable = false;
      }  
      queue_init(&qSensorData, sizeof(queue_entry_t) , 50) ; // max 50 groups of 5 bytes.  create queue to get data from core1
      queue_init(&qSendCmdToCore1, 1, 10); // queue to send a cmd to core 1 (e.g. to perform a calibration of mp6050)
      multicore_launch_core1(core1_main);// start core1 and so start I2C sensor discovery
      uint32_t setup1StartUs = microsRp();  
      while ( core1SetupDone == false) {
            if ((microsRp() - setup1StartUs) > (2 * 1000000)) {
                sleep_ms(3000);
                printf("Attention: setup on core 1 did not ended within timeout\n");
                break   ;
            }
      }
      uint32_t core1SetupUs = microsRp() - setup1StartUs ; 
      printf("Setup1 takes %d usec\n",(int) core1SetupUs) ;
      if ( config.protocol == 'C'){   //crsf
        setupCrsfIn();  // setup one/two uart and the irq handler (for primary Rx) 
        setupCrsf2In();  // setup one/two uart and the irq handler (for secondary Rx) 
        setupCrsfOut(); //  setup 1 pio/sm (for TX ) and the DMA (for TX)   
      } else if (config.protocol == 'S') { // sport
        setupSbusIn();
        setupSbus2In();
        setupSport();
      } else if (config.protocol == 'J') {   //jeti non exbus
        setupSbusIn();
        setupSbus2In();
        setupJeti();
      } else if (config.protocol == 'H') {   //hott
        setupSbusIn();
        setupSbus2In();
        setupHott();
      } else if (config.protocol == 'M') {   //Mpx
        setupSbusIn();
        setupSbus2In();
        setupMpx();
      } else if (config.protocol == 'I') {  // ibus
        setupSbusIn();
        setupSbus2In();
        setupIbus();
      } else if (config.protocol == '2')  {  // Sbus2 futaba
        setupSbusIn();
        setupSbus2In();
        setupSbus2Tlm();
      } else if (config.protocol == 'F') {   // fBus frsky
        setupFbus();
        setupSbus2In();
        //setupSbus2Tlm();
      } else if (config.protocol == 'L') {   // srxl2 Spektrum
        setupSrxl2();
        //setupSbus2In(); // to do add a second input
        //setupSbus2Tlm();
      } else if (config.protocol == 'E') {   // jeti Exbus
        setupExbus();
        //setupSbus2In(); // to do add a second input
        //setupSbus2Tlm();
      }
      if (config.pinSbusOut != 255) { // configure 1 pio/sm for SBUS out (only if Sbus out is activated in config).
          setupSbusOutPio();
        }
      setupPwm();
      watchdog_enable(3500, 0); // require an update once every 500 msec
  } 
  printConfig(); 
  setRgbColorOn(10,0,0); // set color on red (= no signal)
  /*
  if (clockChanged){
    printf("clock is changed to 133mHz\n");
  } else {
    printf("clock is not changed\n");
  }
  */ 
    //checkLedColors(); // this program does not end and so main loop is not called.
}

void getSensorsFromCore1(){
    queue_entry_t entry;
    //printf("qlevel= %d\n",queue_get_level(&qSensorData));
    
    while( !queue_is_empty(&qSensorData)){
        if ( queue_try_remove(&qSensorData,&entry)){
            if (entry.type >= NUMBER_MAX_IDX) {
                if (entry.type == SAVE_CONFIG_ID && entry.data == 0XFFFFFFFF) {  // this is a command to save the config.
                    watchdog_enable(15000,false);
                    sleep_ms(1000);
                    printf("\nCalibration has been done");
                    printConfigOffsets();
                    printf("\nConfig will be saved\n\n");
                    sleep_ms(1000);
                    saveConfig();
                    printf("Config has been saved\n");  
                    printf("Device will reboot\n\n");
                    watchdog_enable(1500,false);
                    sleep_ms(1000);
                    watchdog_reboot(0, 0, 100); // this force a reboot!!!!!!!!!!
                } else if (entry.type == CAMERA_PITCH_ID) {
                    cameraPitch = entry.data;
                } else if (entry.type == CAMERA_ROLL_ID) {
                    cameraRoll = entry.data;
                } else {
                    printf("error : invalid type of sensor = %d\n", entry.type);
                }    
            } else {
                fields[entry.type].value = entry.data;
                fields[entry.type].available = true ;
                if (fields[entry.type].onceAvailable == false){
                    fields[entry.type].onceAvailable = true ;
                    // update sportMaxBandwidth for some protocols
                    if ( (config.protocol == 'S') || (config.protocol == 'F') )  calculateSportMaxBandwidth(); 
                }    
                //printf("t=%d  %10.0f\n",entry.type ,  (float)entry.data);
            }    
        }
    }
    if ((forcedFields == 1) || (forcedFields == 2)) fillFields(forcedFields); // force dummy vallues for debuging a protocol
}

void sendtoSD(){
  static uint32_t lastSDUs;
  static uint32_t lastSDUs2;
  
  if ( ( millisRp() - lastSDUs) < 50) return; // perform calculation only every 100 msec 
  lastSDUs = millisRp();

  if ( ( millisRp() - lastSDUs2) >= 1000){ // perform calculation only every 1000 msec 
    lastSDUs2 = millisRp();
    /*
    datetime_t curT;

    rtc_get_datetime(&curT);

    printf("Date: %d/%d/%d\n", curT.day, curT.month, curT.year);
    printf("Current RTC: %d:%d:%d\n", curT.hour, curT.min, curT.sec);
    //printf("SD \n");
    printf("RTC Running2: %d\n", rtc_running());

    */
  }

  FRESULT fr;
  FATFS fs;
  FIL fil;
  
  fr = f_mount(&fs, "0:", 1);
  fr = f_open(&fil, sdFileName, FA_WRITE | FA_OPEN_APPEND);

  int logVar[] = {LATITUDE, LONGITUDE, GROUNDSPEED, HEADING, ALTITUDE, GPS_DATE, GPS_TIME, GPS_HOME_BEARING, GPS_HOME_DISTANCE, GPS_CUMUL_DIST, VSPEED, RELATIVEALT, PITCH, ROLL, YAW, ADS_1_1, ADS_1_2, AIRSPEED_COMPENSATED_VSPEED, AIRSPEED};
  int numOfParams = 19;
  //printf("\n");

  
  for (uint8_t i=0; i<numOfParams ;i++){
    if (fields[logVar[i]].onceAvailable){
      switch (logVar[i]) {
              case LATITUDE:
                  //printf("GPS Latitude = %.7f degree\n", ((float) fields[i].value) / 10000000.0);
                  f_printf(&fil, "%.7f", ((float) fields[logVar[i]].value) / 10000000.0);
                  break;
              case LONGITUDE:
                  //printf("GPS Longitude = %.7f degree\n", ((float) fields[i].value) / 10000000.0);
                  f_printf(&fil, "%.7f", ((float) fields[logVar[i]].value) / 10000000.0);
                  break;
              case GROUNDSPEED:
                  //printf("GPS Groundspeed = %d cm/s\n", (int) fields[i].value) ;
                  f_printf(&fil, "%d", (int) fields[logVar[i]].value);
                  break;
              case HEADING:
                  //printf("GPS Heading = %f degree\n", ((float) fields[i].value) / 100.0) ;
                  f_printf(&fil, "%f", ((float) fields[logVar[i]].value) / 100.0);
                  break;
              case ALTITUDE:
                  //printf("GPS Altitude = %d cm\n", (int) fields[i].value) ;
                  f_printf(&fil, "%d", (int) fields[logVar[i]].value);
                  break;
              case NUMSAT:
                  //printf("GPS Num sat. = %d\n", (int) fields[i].value) ;
                  f_printf(&fil, "%d", (int) fields[logVar[i]].value);
                  break;
              case GPS_DATE:
                  //printf("GPS Date D M Y = %d %d %d \n", (uint8_t) (fields[i].value >>8) , (uint8_t) (fields[i].value >> 16) ,
                  //    (uint8_t) (fields[i].value >> 24) ) ;
                  f_printf(&fil, "%d/%d/%d", (uint8_t) (fields[logVar[i]].value >> 8) , (uint8_t) (fields[logVar[i]].value >> 16) ,
                      (uint8_t) (fields[logVar[i]].value >> 24));
                  break;
              case GPS_TIME:
                  //printf("GPS Time H M S = %d %d %d \n", (uint8_t) (fields[i].value >>24) , (uint8_t) (fields[i].value >> 16) ,
                  //    (uint8_t) (fields[i].value >> 8) ) ;
                  f_printf(&fil, "%d:%d:%d", (uint8_t) (fields[logVar[i]].value >>24) + 8 , (uint8_t) (fields[logVar[i]].value >> 16) ,
                      (uint8_t) (fields[logVar[i]].value >> 8));
                  //f_printf(&fil, "%x", fields[logVar[i]].value);
                  break;
              case GPS_PDOP:
                  //printf("GPS Pdop = %d \n", (int) fields[i].value) ;
                  break;
              case GPS_HOME_BEARING:
                  //printf("GPS Home bearing = %d degree\n", (int) fields[i].value) ;
                  f_printf(&fil, "%d", (int) fields[logVar[i]].value * 6);
                  break;
              case GPS_HOME_DISTANCE:
                  //printf("GPS Home distance = %d m\n", (int) fields[i].value) ;
                  f_printf(&fil,"%d", (int) fields[logVar[i]].value * 12.5);
                  break;
              case MVOLT:
                  //printf("Volt 1 = %d mVolt\n", (int) fields[i].value) ;
                  break;
              case CURRENT:
                  //printf("Current (Volt 2) = %d mA\n", (int) fields[i].value) ;
                  break;
              case CAPACITY:
                  //printf("Capacity (using current) = %d mAh\n", (int) fields[i].value) ;
                  break;     
              case VSPEED:
                  //printf("Vspeed = %d cm/s\n", (int) fields[i].value) ;
                  f_printf(&fil, "%d", (int) fields[logVar[i]].value);
                  break;        
              case RELATIVEALT:
                  //printf("Baro Rel altitude = %d cm\n", (int) fields[i].value) ;
                  f_printf(&fil, "%d", (int) fields[logVar[i]].value);
                  break;        
              case PITCH:
                  //printf("Pitch = %d degree\n", (int) fields[i].value) ;
                  f_printf(&fil, "%d", (int) fields[logVar[i]].value);
                  break;        
              case ROLL:
                  //printf("Roll = %d degree\n", (int) fields[i].value) ;
                  f_printf(&fil, "%d", (int) fields[logVar[i]].value);
                  break;        
              case YAW:
                  //printf("Yaw = %d degree\n", (int) fields[i].value) ;
                  f_printf(&fil, "%d", (int) fields[logVar[i]].value);
                  break;        
              case RPM:
                  //printf("RPM = %d Hertz\n", (int) fields[i].value) ;
                  break;
              case ADS_1_1:
                  //printf("Ads 1 1 = %d mVolt\n", (int) fields[i].value) ;
                  f_printf(&fil, "%d", (int) fields[logVar[i]].value);
                  //Battery voltage

                  break;        
              case ADS_1_2:
                  //printf("Ads 1 2 = %d mVolt\n", (int) fields[i].value) ;
                  f_printf(&fil, "%d", (int) fields[logVar[i]].value);
                  //Current

                  break;        
              case ADS_1_3:
                  //printf("Ads 1 3 = %d mVolt\n", (int) fields[i].value) ;
                  break;        
              case ADS_1_4:
                  //printf("Ads 1 4 = %d mVolt\n", (int) fields[i].value) ;
                  break;        
              case ADS_2_1:
                  //printf("Ads 2 1 = %d mVolt\n", (int) fields[i].value) ;
                  break;        
              case ADS_2_2:
                  //printf("Ads 2 2 = %d mVolt\n", (int) fields[i].value) ;
                  break;        
              case ADS_2_3:
                  //printf("Ads 2 3 = %d mVolt\n", (int) fields[i].value) ;
                  break;        
              case ADS_2_4:
                  //printf("Ads 2 4 = %d mVolt\n", (int) fields[i].value) ;
                  break;
              case AIRSPEED:
                  //printf("Airspeed = %d cm/s\n", (int) fields[i].value) ;
                  f_printf(&fil, "%d", (int) fields[logVar[i]].value);
                  break;
              case AIRSPEED_COMPENSATED_VSPEED:
                  //printf("Compensated Vspeed = %d cm/s\n", (int) fields[i].value) ;
                  f_printf(&fil, "%d", (int) fields[logVar[i]].value);
                  break;
              case GPS_CUMUL_DIST :
                  //printf("Gps cumulative distance = %d\n", (int) fields[i].value) ;
                  f_printf(&fil, "%d", (int) fields[logVar[i]].value);
                  break;
                          
          } // end switch
    }
    else {
      f_printf(&fil, "N/A");
    }
    f_printf(&fil, ",");
  }
  f_printf(&fil,"\n");
  //if (config.VspeedCompChannel != 255){
  //    printf("Vspeed compensation = %.2f\n", dteCompensationFactor);
  //}
  //f_printf(&fil,"TEST,TEST,TEST,TEST,TEST,TEST,TEST,TEST,TEST,TEST,TEST,TEST,TEST,TEST\n");
  fr = f_close(&fil);
  f_unmount("0:");
}

void loop() {
  //debugBootButton();
  if (configIsValid){
      getSensorsFromCore1();
      mergeSeveralSensors();
      watchdog_update();
      //sendtoSD();
      if ( config.protocol == 'C'){   //elrs/crsf
        fillCRSFFrame();
        handleCrsfIn();
        handleCrsf2In();
        fillSbusFrame();
      } else if (config.protocol == 'S') {  // sport
        handleSportRxTx();
        handleSbusIn();
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == 'J') {  //jeti
        handleJetiTx();
        handleSbusIn();
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == 'H') {  //Hott
        handleHottRxTx();
        handleSbusIn();
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == 'M') {  // multiplex
        handleMpxRxTx();
        handleSbusIn();
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == 'I') {  // Ibus flysky
        handleIbusRxTx();
        handleSbusIn();           //???????????is this OK
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == '2') { // Sbus2 Futaba
        handleSbusIn();
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == 'F') {  // Fbus frsky
        handleFbusRxTx();
        handleSbus2In();
        fillSbusFrame();
      } else if (config.protocol == 'L') {  // SRXL2 Spektrum
        handleSrxl2RxTx();
        //handleSbus2In();  // to do processa second inpunt
        fillSbusFrame();
      } else if (config.protocol == 'E') {  // Jeti Exbus
        handleExbusRxTx();
        //handleSbus2In();  // to do processa second inpunt
        fillSbusFrame();
      }
      watchdog_update();
      updatePWM();

            //updatePioPwm();
            
  }
  watchdog_update();
  //if (tud_cdc_connected()) {
  handleUSBCmd();  // process the commands received from the USB
  tud_task();      // I suppose that this function has to be called periodicaly
  //}  
  if ( configIsValidPrev != configIsValid) {
    configIsValidPrev = configIsValid;
    if (configIsValid) {
        blinking = true; // setRgbColorOn(0,10,0); // red , green , blue
    } else {
        blinking = false; // setRgbColorOn(10,0,0);
        setRgbOn();  
    }
  }

  handleBootButton(); // check boot button; after double click, change LED to fix blue and next HOLD within 5 sec save the current channnels as Failsafe values
  if (( bootButtonState == ARMED) || ( bootButtonState == SAVED)){
    //setRgbColorOn(0, 0, 10); //blue
  } else if ( ledState != prevLedState){
    //printf(" %d\n ",ledState);
    prevLedState = ledState;
    setColorState();     
  } else if ( blinking && (( millisRp() - lastBlinkMillis) > 300 ) ){
    toggleRgb();
    lastBlinkMillis = millisRp();
  }
  //if (get_bootsel_button()) {
  //  printf("p\n");
  //} 
  //enlapsedTime(0);
}

// initialisation of core 1 that capture the sensor data
void setup1(){
    multicore_lockout_victim_init();
    setupSensors();    
}
// main loop on core 1 in order to read the sensors and send the data to core0
void loop1(){
    uint8_t qCmd;
    getSensors(); // get sensor
    if ( ! queue_is_empty(&qSendCmdToCore1)){
        queue_try_remove(&qSendCmdToCore1, &qCmd);
        if ( qCmd == 0X01) { // 0X01 is the code to request a calibration
            mpu.calibrationExecute();
        }
    }
    sendtoSD();
}

void core1_main(){
    setup1();
    while(1) loop1();
}

int main(){
  setup();
  while(1) loop();
}
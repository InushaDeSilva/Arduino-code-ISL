//#include <LibPrintf.h>
#include <Arduino.h>

/*
 * DUAL TIMER CAMERA SYNCHRONIZATION SYSTEM WITH IMU-DERIVED 60Hz
 * ==============================================================
 * Timer4: FLIR_BOZON_SYNC_PIN at 60Hz (120Hz interrupt rate, toggle every interrupt) - FALLBACK
 * Timer5: CAM_SYNC_PIN asymmetric timing (4ms HIGH / 16ms LOW for camera control)
 * INT5: XSense IMU 50Hz input → Phase Accumulator → 60Hz FLIR sync (PRIMARY)
 * Both systems synchronized to GPS PPS for absolute timing accuracy
 * 
 * IMU Phase Accumulator: 50Hz → 60Hz using phase increment = (60/50) * 2^16 = 78643
 * FLIR Requirements: 59.75Hz to 60.25Hz for optimal imaging performance
 * Timer4 serves as automatic fallback if IMU 50Hz signal is lost
 * 
 * FLIR Timer4: 16MHz/8/120Hz = 48869 preload → 60Hz square wave (FALLBACK)
 * Camera Timer5: ORIGINAL asymmetric timing (preload_hi=4ms/preload_lo=16ms) - DO NOT CHANGE!
 * 
 * Timer Overflow Formula: TCNT = 65536 - (f_clk / (prescaler × f_overflow))
 */

//------- PIN definitions--PIN means the pin of the port------------------------------
#define POWER_LED 25        //POWER LED
#define PPS_LED 26          //PPS LED
#define RTK_LED 27          //RTK LED
#define ERR_LED 28
#define POWER_LED_PIN 3     //PORT A3
#define PPS_LED_PIN 4       //PORT A4
#define RTK_LED_PIN 5       //PORT A5
#define ERR_LED_PIN 6       //PORT A6
#define SIM_PPS_PIN 5       //PORT E5 ???????
#define FLIR_BOZON_SYNC_PIN 6     //PORT J6
#define CAM_SYNC_PIN 5      //PORT E5
#define JETSON_REC 24       //Jetson Force Recovery //SYNCN
#define JETSON_RST 23       //Jetson Reset //SYNCP
#define JETSON_PWR 22       //Jetson PWR   
#define JETSON_REC_PIN 2    //Jetson Force Recovery //SYNCN PORT A2
#define JETSON_RST_PIN 1    //Jetson Reset //SYNCP PORT A1
#define JETSON_PWR_PIN 0    //Jetson PWR PORT A0  
#define LEVEL_CNV_ENABLE A8   //Level converter0 enable 
#define LEVEL_CNV1_ENABLE A9  //Level converter1 enable
#define GPS_PPS_PIN 7         //PORT E7 INT7
#define RTC_PPS_PIN 6         //PORT E6 INT6
#define RTK_FIX_PIN 4         //PORT E4 INT4
#define IMU_50HZ_PIN 5        //PORT E5 INT5 - XSense IMU 50Hz input

#define EXT_CNTRL1_PIN 0      //HUB Control input 1 PORT A0
#define EXT_CNTRL2_PIN 1      //HUB Control input 2 PORT A1
#define EXT_CNTRL3_PIN 2      //HUB Control input 3 PORT A2
#define EXT_CNTRL4_PIN 3      //HUB Control input 4 PORT A3

// TEST PIN - REMOVE AFTER TESTING
#define TEST_FLIR_PIN 5       //PORTF5 (Analog pin A5) - Mirror of FLIR_BOZON_SYNC_PIN for oscilloscope testing

#define PPS_MUX 64            //PK2 LOW
#define GNSS_MUX_EN 65        //PK3 LOW For Enable
#define GNSS_MUX_SEL 66       //PK4 High for Onboard GNSS



//Efficient port set/clear functions ----Does not work for analog pins-----------------
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define TOGGLE(x,y) (x=((~x)&(1<<y))|((x)&(~(1<<y))))
#define READ(x,y,z) (z=(x>>y)&1)  // read to z
#define READ2(x,y) ((x>>y)&1)     // read out

// variable for state machines---------------------------------------------------------
int cam_pulse_count = 0;   // stores the pulse count of camera control timer (2x) 
int cam_pulse_count_pre = 0;   // stores the pulse count of camera control timer (2x) 
int cam_pulse_count_for_GPRMC = 0;   // stores the pulse count of camera control timer -for GPRMC write (2x) 
int gps_pulse_count = 0;
int gps_pulse_count_last = 0;
int pps_width_count = 0;  // used to control the pulse width of pps_sim
bool flag_pps_high = false; // used to keep track of lidar pin state
bool flag_cam_high = false; // used to keep track of camera pin state
bool flag_flir_high = false; // used to keep track of FLIR BOZON sync pin state
bool flag_write_serial = false; // used to control serial write to Jetson
unsigned int preload_hi = 57559; //4ms: 57536 + 23 ticks compensation for interrupt overhead
unsigned int preload_lo = 33559; //16ms: 33536 + 23 ticks compensation for interrupt overhead
unsigned int preload_flir_hi = 49286; //8.33ms: 49286 for 60Hz 50% duty cycle high phase  
unsigned int preload_flir_lo = 49286; //8.33ms: 49286 for 60Hz 50% duty cycle low phase
unsigned int max_val = 65535;
int cam_pps_error = 0;
int cam_pps_correction = 0;
int flir_pulse_count = 0;   // stores the pulse count for FLIR BOZON sync timing

// Timer4 variables for FLIR 60Hz sync
// CORRECTED CALCULATION: Timer clock = 16MHz/8 = 2MHz
// For 60Hz output: need 120Hz interrupt = 2MHz/120Hz = 16,667 ticks
// Preload = 65536 - 16,667 = 48,869 (was incorrectly 52,219 = 75Hz)
unsigned int preload_flir_timer4 = 48877; // 120Hz: 65536-16MHz/8/120Hz = 48869 (for 60Hz toggle) ; 48869 + 8 ticks compensation
int flir_pps_error = 0;
int flir_pps_correction = 0;

// Phase Accumulator variables for 60Hz generation from IMU 50Hz input
// Phase increment = (60Hz / 50Hz) * 2^16 = 1.2 * 65536 = 78643.2 ≈ 78643
long phase_accumulator_60hz = 0;
const long PHASE_INCREMENT_60HZ = 78643;  // For generating 60Hz from 50Hz input
bool flir_60hz_from_imu = false;          // State of 60Hz signal derived from IMU
bool imu_50hz_active = false;             // Flag to indicate IMU 50Hz is being used


//variables for pps syncing
unsigned int gps_pps_period = 0;
long gps_tmr_val_pre = 0;
long gps_tmr_val_crr = 0;
long phase_diff = 0;
bool flag_flock = false;
bool flag_plock_ready = false;
int flock_count = 0;
long phase_diff_corr = 0;
unsigned int preload_flock = 0;
bool flag_write_serial2 = false;

//Vars LVI_handhold MCU time to LiDAR via gprmc ---
char gprmcSTR[8] = "$GPRMC,";
int chckNum = 0;
char chckNumChar[2];
char value_1[100] = "";
char value_2[100] = "";
int ss = 0;
int mm = 0;
int hh = 0;


// variables to hold the parsed NMEA data
bool flag_send_next_nema = false;
const int numChars = 256; // increase as needed
char receivedChars[numChars];
char tempChars[numChars];
char messageFromPC[numChars] = { 0 };
char messageToLivox[numChars] = { 0 };
int integerFromPC = 0;
float floatFromPC = 0.0;

long onTime = 0;
int lastReading = LOW;
int bounceTime = 1000;

bool newData = false;

const byte buff_len = 90;
char CRCbuffer[buff_len];

// setup
void setup() {

  // set input output
  pinMode(LEVEL_CNV_ENABLE, OUTPUT);  //Level converter (TXS0108E) output enable (Low = High Impedence)
  pinMode(LEVEL_CNV1_ENABLE, OUTPUT);
  pinMode(PPS_LED, OUTPUT);  // Green LED
  pinMode(RTK_LED, OUTPUT);  // Red LED
  pinMode(POWER_LED, OUTPUT);  // Yellow LED
  pinMode(ERR_LED, OUTPUT);
  pinMode(JETSON_REC, OUTPUT);   //Jetson Force Recovery
  pinMode(JETSON_RST, OUTPUT);   //Jetson RST
  pinMode(JETSON_PWR, OUTPUT);   //Jetson PWR
  pinMode(PPS_MUX, OUTPUT);
  pinMode(GNSS_MUX_EN, OUTPUT);
  pinMode(GNSS_MUX_SEL, OUTPUT);

  // Set using DDR regiter for non mapped pins
  SET(DDRE, SIM_PPS_PIN);   //PORTE PE5
  SET(DDRJ, FLIR_BOZON_SYNC_PIN); //PORTJ PJ6
  SET(DDRE, CAM_SYNC_PIN); //PORTE PE5
  SET(DDRA, POWER_LED_PIN); //PORTA PA3 - POWER LED
  SET(DDRA, ERR_LED_PIN); //PORTA PA6 - ERR LED for debugging
  SET(DDRF, EXT_CNTRL1_PIN);
  SET(DDRF, EXT_CNTRL2_PIN);
  SET(DDRF, EXT_CNTRL3_PIN);
  SET(DDRF, EXT_CNTRL4_PIN);
  SET(DDRF, TEST_FLIR_PIN); // TEST PIN - REMOVE AFTER TESTING - Analog pin A5 for oscilloscope
  CLR(DDRE, GPS_PPS_PIN); //PORTE PE7
  CLR(DDRE, RTC_PPS_PIN); //PORTE PE6
  CLR(DDRE, IMU_50HZ_PIN); //PORTE PE5 - IMU 50Hz input
  //DDRJ |= B00101000;  // Set using DDR regiter for non mapped pins

  // Startup devices
  digitalWrite(POWER_LED, HIGH);  // Turns on level converter
  SET(PORTA, POWER_LED_PIN); // Turn on POWER LED solid using PORT register (PORTA3)
  CLR(PORTF, EXT_CNTRL1_PIN); // Initialize GPS PPS LED OFF - will toggle when GPS PPS is received
  SET(PORTF, EXT_CNTRL2_PIN); // Initialize EXT_CNTRL2_PIN HIGH - mirrors EXT_CNTRL4_PIN (Backfly 50Hz signal)
  digitalWrite(LEVEL_CNV_ENABLE, HIGH);  // Turns on level converter
  digitalWrite(LEVEL_CNV1_ENABLE, HIGH);
  digitalWrite(PPS_MUX, LOW);
  digitalWrite(GNSS_MUX_EN, LOW);
  digitalWrite(GNSS_MUX_SEL, HIGH);



  //Serial ports
  Serial1.begin(115200); // Jetson Communication
  Serial.begin(115200); // Terminal
  Serial2.begin(38400); // GPS Reciever

  // Configure Timer5 for 100Hz - toggle achieves the cam trigger of 50 Hz
  noInterrupts();           // disable all interrupts

  // Configure Timer4 for FLIR 120Hz - toggle achieves 60Hz FLIR sync
  // Timer4: 16MHz/8/120Hz = 48869 preload value for precise 60Hz square wave
  TCCR4A = 0;               // disable compare capture A
  TCCR4B = 0;               // disable compare capture B
  TCNT4 = preload_flir_timer4; // preload timer 65536-16MHz/8/120Hz = 48869 for 60Hz toggle
  TCCR4B |= (1 << CS41);    // PS 8 prescaler :Timer resolution 1/16e6*PS
  TIMSK4 |= (1 << TOIE4);   // enable timer overflow interrupt

  // Configure Timer5 for 100Hz - toggle achieves the cam trigger of 50 Hz
  // Timer5: Uses asymmetric timing (preload_hi/preload_lo) for camera control
  TCCR5A = 0;               // disable compare capture A
  TCCR5B = 0;               // disable compare capture B
  TCNT5 = 25536; // asymmetric camera timing (4ms/16ms)
  TCCR5B |= (1 << CS51);    // PS 8 prescaler :Timer resolution 1/16e6*PS
  TIMSK5 |= (1 << TOIE5);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts


  //Masked External interrupt (GPS)  PCINT 16
  //PCICR |= B00000100; // Pin change interrupt 1 enabled (PCINT15:8)
  //PCMSK2 |= B00000001; // Specify which pin is enabled (PCINT16 and 17)

  //Enable external interrupts INT7 (GPS PPS) and INT5 (IMU 50Hz)
  EICRB |= B11001100;  // INT7: rising edge, INT6: rising edge, INT5: rising edge  
  EIMSK |= B10100000;  // Enable INT7 (GPS PPS) and INT5 (IMU 50Hz)

  //SET(PORTE,CAM_SYNC_PIN);
  SET(PORTF, EXT_CNTRL4_PIN); //HUB Power LED pin - now driving the Backlfy camera

  // Initialize FLIR BOZON sync pin high and start counting
  SET(PORTJ, FLIR_BOZON_SYNC_PIN); // Start FLIR sync pin HIGH
  SET(PORTF, TEST_FLIR_PIN); // TEST PIN - REMOVE AFTER TESTING - Mirror for oscilloscope
  flag_flir_high = true;
  flir_pulse_count = 0;
}

// Loop functions
void loop() {
  // IMU 50Hz timeout detection and Timer4 fallback logic
  static unsigned long last_imu_check = 0;
  static unsigned long imu_timeout_counter = 0;
  
  if (millis() - last_imu_check > 100) {  // Check every 100ms
    last_imu_check = millis();
    
    if (imu_50hz_active) {
      imu_timeout_counter = 0;  // Reset timeout counter
      imu_50hz_active = false;  // Reset flag for next check
      // IMU is working, optionally disable Timer4
      // TIMSK4 &= ~(1 << TOIE4);  // Uncomment to disable Timer4 when IMU active
    } else {
      imu_timeout_counter++;
      if (imu_timeout_counter > 10) {  // No IMU for 1 second
        // Re-enable Timer4 as fallback
        TIMSK4 |= (1 << TOIE4);  // Enable Timer4 interrupt as backup
        // Serial.println("IMU 50Hz timeout - using Timer4 fallback");
      }
    }
  }

  //digitalWrite(JETSON_PWR,HIGH);
  //digitalWrite(JETSON_REC,HIGH);
  //digitalWrite(JETSON_RST,HIGH);
  if (flag_write_serial == true) {
    Serial1.write(messageToLivox);
    Serial.write(messageToLivox);
    flag_write_serial = false;
  }

  /*    Serial1.write(value_2);
      Serial1.write("%s%02d%02d%02d%s");
      Serial1.write(gprmcSTR);
      Serial1.write(hh);
      Serial1.write(mm);
      Serial1.write(ss);
      Serial1.write(".00,A,2237.496474,N,11356.089515,E,0.0,225.5,230520,2.3,S,A*");
      strcpy(value_1,value_2);
      chckNum =checkNum(value_1);
      Serial1.write(chckNumChar);
      Serial1.write("%02X");
      Serial1.write(chckNum);
      Serial1.write("%s");
      Serial1.write(value_2);
      Serial1.write("%s\n");
      Serial1.write(chckNumChar);
      //Serial.write("%s",value_2);
      //Serial.write("s%",chckNumChar);
      flag_write_serial = false;
    }*/

    //if(flag_write_serial2){
      // this for debugging
      /*
      Serial.println(cam_pps_correction);
      Serial.println(gps_pulse_count_last);
      Serial.println(gps_tmr_val_crr);
      Serial.println(gps_tmr_val_pre);
      Serial.println(phase_diff);
      Serial.println(preload_flock);
      //Serial.println(preload);
      flag_write_serial2 = false;*/
      //}

   //Process GPS
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();
    showParsedData();
    newData = false;
  }

  //while (Serial2.available()){
   //  Serial.print(Serial2.read());
  //}
  //Serial.print("test");




}

// ISR -  PC inturupts - Masked Block 2
// This is only invoked when GPS PPS is there
ISR(INT7_vect) {
  TOGGLE(PORTA, PPS_LED_PIN);
  TOGGLE(PORTF, EXT_CNTRL1_PIN);

  //Reset the timer5 to initial condition 
  SET(PORTA, JETSON_RST_PIN);
  CLR(PORTA, JETSON_REC_PIN);
  CLR(PORTA, JETSON_PWR_PIN);
  SET(PORTF, EXT_CNTRL3_PIN);
  SET(PORTE, CAM_SYNC_PIN);
  SET(PORTF, EXT_CNTRL2_PIN);
  SET(PORTF, EXT_CNTRL4_PIN);
  SET(PORTJ, FLIR_BOZON_SYNC_PIN); // Reset FLIR sync pin HIGH on PPS
  SET(PORTF, TEST_FLIR_PIN); // TEST PIN - REMOVE AFTER TESTING
  flag_pps_high = true;
  flag_flir_high = true; // Reset FLIR sync state
  cam_pulse_count = 0;
  flir_pulse_count = 0; // Reset FLIR pulse counter
  pps_width_count = 0;
  if (cam_pulse_count_for_GPRMC < 100) {
    sendDummyTime();
  }
  cam_pulse_count_for_GPRMC = 0;
  cam_pps_error = max_val - TCNT5;
  flir_pps_error = max_val - TCNT4; // Get Timer4 error for FLIR sync
  TCNT5 = preload_hi; // Use existing preload_hi for Timer5 asymmetric timing
  TCNT4 = preload_flir_timer4; // Reset Timer4 for FLIR sync

  //timer 5 rate adjustment
  Serial.println(cam_pps_error);
  //this values shuodl be minimized to noise level (50) by adjusting rate
  if (cam_pps_error > 50) {
    cam_pps_correction = 20; //this is not implemented
  }
  else if (cam_pps_error < -50) {
    cam_pps_correction = -20;
  }
  else {
    cam_pps_correction = 0;
  }

  // Timer4 FLIR rate adjustment - similar to Timer5
  Serial.print("FLIR PPS Error: ");
  Serial.println(flir_pps_error);
  if (flir_pps_error > 50) {
    flir_pps_correction = 20; // adjust FLIR timer rate
  }
  else if (flir_pps_error < -50) {
    flir_pps_correction = -20;
  }
  else {
    flir_pps_correction = 0;
  }


  //pulse Lidar
  //SET(PORTA,JETSON_RST_PIN);
  //CLR(PORTA,JETSON_REC_PIN);
  //pps_width_count = 0;
  //flag_pps_high = true;
  //TCNT5 = 25536; // camera 50 cycle start 
  //SET(PORTF,EXT_CNTRL2_PIN); // Lidar PPS pin

  //force camera to trigger
  //cam_pulse_count_pre = cam_pulse_count;
  //cam_pulse_count = 0;
  //flag_cam_high = true;
  //SET(PORTE,CAM_SYNC_PIN); // 25Hz pulse width 50%
  //SET(PORTF,EXT_CNTRL3_PIN);
  //TCNT5 = preload_hi;



  //Timer 5 force overflow and adjust rate

  //over flow interupt checks Turn off lidar pulse at 380ms
  //over flow interupt checks toggle the camera sync 25Hz pulse 50% duty
}

ISR(INT5_vect) {
  // DEBUG: Visual indicator of IMU 50Hz activity
  TOGGLE(PORTA, ERR_LED_PIN);
  
  // Set flag to indicate IMU 50Hz is active (for fallback logic)
  imu_50hz_active = true;
  
  // Original camera sync functionality
  pps_width_count++;
  cam_pulse_count++;
  cam_pulse_count_for_GPRMC++;
  //TCNT5 = 25536;

  // Handle 50Hz camera sync (25Hz effective rate)
  flag_cam_high = READ2(PORTE, CAM_SYNC_PIN);
  TOGGLE(PORTE, CAM_SYNC_PIN);
  flag_cam_high = !flag_cam_high;

  if (flag_cam_high) {
    TOGGLE(PORTF, EXT_CNTRL4_PIN);
    TOGGLE(PORTF, EXT_CNTRL2_PIN); // Mirror EXT_CNTRL4_PIN - visual representation of 50Hz Backfly signal
    TCNT5 = preload_hi;
  }
  else {
    //CLR(PORTE,CAM_SYNC_PIN); // 25Hz pulse width 50%
    TCNT5 = preload_lo;
  }

  if (cam_pulse_count >= 100) {
    SET(PORTA, JETSON_RST_PIN);
    CLR(PORTA, JETSON_REC_PIN);
    CLR(PORTA, JETSON_PWR_PIN);
    SET(PORTF, EXT_CNTRL3_PIN);
    SET(PORTE, CAM_SYNC_PIN);
    SET(PORTF, EXT_CNTRL2_PIN);
    SET(PORTF, EXT_CNTRL4_PIN);
    flag_pps_high = true;
    cam_pulse_count = 0;
    pps_width_count = 0;
  }

  if (flag_pps_high && pps_width_count >= 38) {
    CLR(PORTA, JETSON_RST_PIN);
    SET(PORTA, JETSON_REC_PIN);
    SET(PORTA, JETSON_PWR_PIN);
    CLR(PORTF, EXT_CNTRL3_PIN);
    flag_pps_high = false;
    pps_width_count = 0;
  }

  if (cam_pulse_count_for_GPRMC >= 100) { //writes serial every second
    sendDummyTime();
    // Toggle RTK LED every 1 second - FOR TESTING ONLY, REMOVE AFTER TESTING
    TOGGLE(PORTA, RTK_LED_PIN);
  }
  
  // NEW: Phase Accumulator for 60Hz FLIR generation from 50Hz IMU input
  // Every 50Hz pulse, increment phase by (60/50) * 2^16 = 78643
  phase_accumulator_60hz += PHASE_INCREMENT_60HZ;
  
  // Check for phase overflow (indicates 60Hz pulse should occur)
  if (phase_accumulator_60hz >= 65536L) {
    phase_accumulator_60hz -= 65536L;  // Remove one full cycle
    
    // Generate 60Hz pulse for FLIR BOZON
    TOGGLE(PORTJ, FLIR_BOZON_SYNC_PIN);
    TOGGLE(PORTF, TEST_FLIR_PIN); // TEST PIN - REMOVE AFTER TESTING
    flir_60hz_from_imu = !flir_60hz_from_imu;
    
    // Optionally disable Timer4 when using IMU-derived 60Hz
    // Comment out to keep Timer4 as backup
    // TIMSK4 &= ~(1 << TOIE4);  // Disable Timer4 interrupt
  }
}

// Timer4 ISR for FLIR BOZON 60Hz sync
ISR(TIMER4_OVF_vect) {
  flir_pulse_count++; // Increment FLIR pulse counter

  // Handle 60Hz FLIR BOZON sync - simple toggle every interrupt = 120Hz/2 = 60Hz square wave
  TOGGLE(PORTJ, FLIR_BOZON_SYNC_PIN);
  TOGGLE(PORTF, TEST_FLIR_PIN); // TEST PIN - REMOVE AFTER TESTING
  // TOGGLE(PORTA, ERR_LED_PIN);   // DEBUG: Visual indicator of Timer4 activity
  flag_flir_high = !flag_flir_high;

  // Reset timer with preload for 120Hz (60Hz toggle rate)
  TCNT4 = preload_flir_timer4;

  // Reset FLIR counter every 120 interrupts (1 second at 120Hz)
  if (flir_pulse_count >= 120) {
    SET(PORTJ, FLIR_BOZON_SYNC_PIN); // Reset FLIR sync pin HIGH
    SET(PORTF, TEST_FLIR_PIN); // TEST PIN - REMOVE AFTER TESTING
    flag_flir_high = true; // Reset FLIR sync state
    flir_pulse_count = 0; // Reset FLIR counter
  }
}

void sendDummyTime() {
  if (ss < 59) {
    ss++;
  }
  else {
    ss = 0;
    if (mm < 59) {
      mm++;
    }
    else {
      mm = 0;
      if (hh < 23) {
        hh++;
      }
      else {
        hh = 0;
      }
    }
  }
  cam_pulse_count_for_GPRMC = 0;
  //prepare the message
  char time_str[] = "000000";
  byte crc_calc = 0;
  char crc_str[] = "4C";
  memset(messageToLivox, 0, strlen(messageToLivox));
  strcat(messageToLivox, "$GPRMC,");
  sprintf(time_str, "%02d%02d%02d", hh, mm, ss);
  strcat(messageToLivox, time_str);
  strcat(messageToLivox, ".00,A,2237.496474,N,11356.089515,E,0.0,225.5,230520,2.3,S,A*");
  crc_calc = convertToCRC(messageToLivox);
  sprintf(crc_str, "%X", crc_calc);
  strcat(messageToLivox, crc_str);
  strcat(messageToLivox, "\n\r");
  flag_write_serial = true;
}


void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '$';
  char endMarker = '\n';
  char rc;

  while (Serial2.available() > 0 && newData == false) {
    rc = Serial2.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
      recvInProgress = true;
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
  }
}

//============

void parseData() {      // split the data into its parts

  char* strtokIndx; // this is used by strtok() as an index
  byte crc_calc = 0;
  byte crc_recv = 0;
  char crc_str[] = "4C";

  strtokIndx = strtok(tempChars, ",");      // get the first part - the string
  if (strcmp(strtokIndx, "$GNRMC") == 0) {
    strcpy(messageFromPC, strtokIndx);
    strcat(messageFromPC, ",");
    strtokIndx = strtok(NULL, "\n");
    strcat(messageFromPC, strtokIndx); // copy it to messageFromPC
    strcat(messageFromPC, "\n"); //puts newlinein

    //CRC calculation
    if (messageFromPC[strlen(messageFromPC) - 4] == '*') {
      crc_calc = convertToCRC(messageFromPC);
      crc_str[0] = messageFromPC[strlen(messageFromPC) - 3];
      crc_str[1] = messageFromPC[strlen(messageFromPC) - 2];
      //Serial.println(crc_calc);
      //crc_str = strcat(messageFromPC[strlen(messageFromPC)-2],messageFromPC[strlen(messageFromPC)-1]);
      crc_recv = strtol(crc_str, NULL, 16);
      if (crc_recv != crc_calc) {
        memset(messageFromPC, 0, strlen(messageFromPC));
        Serial.write("Checksum error\n");
      }
    }
    else {
      memset(messageFromPC, 0, strlen(messageFromPC));
      Serial.write("No * in NMEA\n");
    }
    //Serial.write(messageFromPC);
  }
  else {
    if (strcmp(strtokIndx, "$GNGGA") == 0) {
      strcpy(messageFromPC, strtokIndx);
      strcat(messageFromPC, ",");
      strtokIndx = strtok(NULL, "\n"); //takes the rest of the message
      strcat(messageFromPC, strtokIndx);
      strcat(messageFromPC, "\n"); //puts newlinein
      Serial1.write(messageFromPC);
      Serial.write(messageFromPC);
      //Serial.println();
      memset(messageFromPC, 0, strlen(messageFromPC));
    }
    else {
      memset(messageFromPC, 0, strlen(messageFromPC));
    }
  }
  /*strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  integerFromPC = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ",");
  floatFromPC = atof(strtokIndx);     // convert this part to a float
*/
}

//============

void showParsedData() {
  if (strlen(messageFromPC) > 0 && flag_send_next_nema) {
    //Serial.print("Message ");
    //Serial2.println(messageFromPC);
    //Serial1.write(messageFromPC);
    //Serial.write(messageFromPC);
    //flag_send_next_nema = false;
    //flag_send_next_nema_delayed = true;
    //modifyNEMAData();
    //strcpy(messageToLidar,messageFromPC);   
  }
}

byte convertToCRC(char* buff) {
  // NMEA CRC: XOR each byte with previous for all chars between '$' and '*'
  char c;
  byte i;
  byte start_with = 0;
  byte end_with = 0;
  byte crc = 0;

  for (i = 0; i < buff_len; i++) {
    c = buff[i];
    if (c == '$') {
      start_with = i;
    }
    if (c == '*') {
      end_with = i;
    }
  }
  if (end_with > start_with) {
    for (i = start_with + 1; i < end_with; i++) { // XOR every character between '$' and '*'
      crc = crc ^ buff[i];  // compute CRC
    }
  }
  else { // else if error, print a msg (to both ports)
    Serial.write("CRC ERROR");
    //SERIALN.println("CRC ERROR");
  }
  return crc;
  //based on code by Elimeléc López - July-19th-2013
}


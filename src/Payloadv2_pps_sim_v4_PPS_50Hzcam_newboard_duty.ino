//#include <LibPrintf.h>
#include <Arduino.h>

//------- PIN definitions--PIN means the pin of the port------------------------------
#define POWER_LED 25        //POWER LED
#define PPS_LED 26          //PPS LED
#define RTK_LED 27          //RTK LED
#define ERR_LED 28
#define POWER_LED_PIN 3     //PORT A3
#define PPS_LED_PIN 4       //PORT A4
#define RTK_LED_PIN 5       //PORT A6
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
unsigned int preload_hi = 57570; //4ms: 57536 + 34 ticks compensation for interrupt overhead
unsigned int preload_lo = 33570; //16ms: 33536 + 34 ticks compensation for interrupt overhead
unsigned int preload_flir_hi = 49286; //8.33ms: 49286 for 60Hz 50% duty cycle high phase  
unsigned int preload_flir_lo = 49286; //8.33ms: 49286 for 60Hz 50% duty cycle low phase
unsigned int max_val = 65535;
int cam_pps_error = 0;
int cam_pps_correction = 0;
int flir_pulse_count = 0;   // stores the pulse count for FLIR BOZON sync timing
bool err_led_state = false; // tracks ERR LED state for 1s blinking


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
  SET(DDRF, EXT_CNTRL1_PIN);
  SET(DDRF, EXT_CNTRL2_PIN);
  SET(DDRF, EXT_CNTRL3_PIN);
  SET(DDRF, EXT_CNTRL4_PIN);
  SET(DDRF, TEST_FLIR_PIN); // TEST PIN - REMOVE AFTER TESTING - Analog pin A5 for oscilloscope
  CLR(DDRE, GPS_PPS_PIN); //PORTE PE7
  CLR(DDRE, RTC_PPS_PIN); //PORTE PE6
  //DDRJ |= B00101000;  // Set using DDR regiter for non mapped pins

  // Startup devices
  digitalWrite(POWER_LED, HIGH);  // Turns on level converter
  digitalWrite(LEVEL_CNV_ENABLE, HIGH);  // Turns on level converter
  digitalWrite(LEVEL_CNV1_ENABLE, HIGH);
  digitalWrite(PPS_MUX, LOW);
  digitalWrite(GNSS_MUX_EN, LOW);
  digitalWrite(GNSS_MUX_SEL, HIGH);



  //Serial ports
  Serial1.begin(115200); // Jetson Communication
  Serial.begin(115200); // Terminal
  Serial2.begin(38400); // GPS Reciever

  // Configure Timer 4 for 100Hz -  toggle achieves the cam trigger of 50 Hz
  noInterrupts();           // disable all interrupts
  //TCCR4A = 0;               // disable compare capture A
  //TCCR4B = 0;               // disable compare capture B
  //TCNT4 = preload_hi; //25536;            // preload timer 65536-16MHz/8/100Hz  - 50Hz overflow
  //TCCR4B |= (1 << CS41);    // PS 8 prescaler :Timer resolution 1/16e6*PS
  //TIMSK4 |= (1 << TOIE4);   // enable timer overflow interrupt

  TCCR5A = 0;               // disable compare capture A
  TCCR5B = 0;               // disable compare capture B
  TCNT5 = 25536; //25536;            // preload timer 65536-16MHz/8/100Hz  - 50Hz overflow
  TCCR5B |= (1 << CS51);    // PS 8 prescaler :Timer resolution 1/16e6*PS
  TIMSK5 |= (1 << TOIE5);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts


  //Masked External interrupt (GPS)  PCINT 16
  //PCICR |= B00000100; // Pin change interrupt 1 enabled (PCINT15:8)
  //PCMSK2 |= B00000001; // Specify which pin is enabled (PCINT16 and 17)

  //Enable eny external inturupts
  EICRB |= B11000000;
  EIMSK |= B10000000;

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
  TCNT5 = preload_hi;

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

ISR(TIMER5_OVF_vect) {
  pps_width_count++;
  cam_pulse_count++;
  cam_pulse_count_for_GPRMC++;
  flir_pulse_count++; // Increment FLIR pulse counter
  //TCNT5 = 25536;

  // Handle 50Hz camera sync (25Hz effective rate)
  flag_cam_high = READ2(PORTE, CAM_SYNC_PIN);
  TOGGLE(PORTE, CAM_SYNC_PIN);
  TOGGLE(PORTF, EXT_CNTRL2_PIN);
  flag_cam_high = !flag_cam_high;

  // Handle 50Hz FLIR BOZON sync - EXACTLY like CAM_SYNC_PIN
  // Simple toggle every interrupt = 100Hz/2 = 50Hz square wave
  TOGGLE(PORTJ, FLIR_BOZON_SYNC_PIN);
  TOGGLE(PORTF, TEST_FLIR_PIN); // TEST PIN - REMOVE AFTER TESTING
  flag_flir_high = !flag_flir_high;

  if (flag_cam_high) {
    TOGGLE(PORTF, EXT_CNTRL4_PIN);
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
    SET(PORTJ, FLIR_BOZON_SYNC_PIN); // Reset FLIR sync pin HIGH
    SET(PORTF, TEST_FLIR_PIN); // TEST PIN - REMOVE AFTER TESTING
    flag_pps_high = true;
    flag_flir_high = true; // Reset FLIR sync state
    cam_pulse_count = 0;
    flir_pulse_count = 0; // Reset FLIR counter
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
    // Toggle ERR LED every 1 second
    err_led_state = !err_led_state;
    digitalWrite(ERR_LED, err_led_state);
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


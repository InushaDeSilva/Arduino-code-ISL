#include <LibPrintf.h>

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
#define FLIR_SYNC_PIN 6     //PORT J6
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

#define EXT_CNTRL1_PIN 0      //HUB Control input 1 PORT A0 - Now used as 25Hz LED sync indicator
#define EXT_CNTRL2_PIN 1      //HUB Control input 2 PORT A1
#define EXT_CNTRL3_PIN 2      //HUB Control input 3 PORT A2
#define EXT_CNTRL4_PIN 3      //HUB Control input 4 PORT A3 - Now used as a SYNC line for FLIR Backfly

#define PPS_MUX 64            //PK2 LOW



//Efficient port set/clear functions ----Does not work for analog pins-----------------
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define TOGGLE(x,y) (x=((~x)&(1<<y))|((x)&(~(1<<y))))
#define READ(x,y,z) (z=(x>>y)&1)  // read to z
#define READ2(x,y) ((x>>y)&1)     // read out

// variable for state machines---------------------------------------------------------
int cam_pulse_count = 0;   // stores the pulse count of camera control timer (2x) 
int cam_pulse_count_for_GPRMC = 0;   // stores the pulse count of camera control timer -for GPRMC write (2x) 
int gps_pulse_count = 0 ;
int gps_pulse_count_last = 0 ;
int pps_width_count = 0;  // used to control the pulse width of pps_sim
bool flag_pps_sim_high = false; // stores the current pps sim signal value
bool flag_cam_sim_high = false; // stores the current cam signal value
bool flag_write_serial = false; // used to control serial write to Jetson
unsigned int preload_hi = 57536; //4ms
unsigned int preload_lo = 33536; //16ms;
unsigned int max_val = 65535;
int cam_pps_preload =0;
int cam_pps_correction =0;


//variables for pps syncing
unsigned int gps_pps_period = 0 ;
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
char gprmcSTR[7]="$GPRMC,";
int chckNum=0;
char chckNumChar[2];
char value_1[100]="";
char value_2[100]="";
int ss=0;
int mm=0;
int hh=0;


 // variables to hold the parsed NMEA data
bool flag_send_next_nema = false;
const int numChars = 256; // increase as needed
char receivedChars[numChars];
char tempChars[numChars]; 
char messageFromPC[numChars] = {0};
char messageToLidar[numChars] = {0};
int integerFromPC = 0;
float floatFromPC = 0.0;

long onTime = 0;
int lastReading = LOW;
int bounceTime = 1000;

boolean newData = false;

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

  // Set using DDR regiter for non mapped pins
  SET(DDRE,SIM_PPS_PIN);   //PORTJ PJ5
  SET(DDRJ,FLIR_SYNC_PIN); //PORTJ PJ4
  SET(DDRE,CAM_SYNC_PIN); //PORTJ PJ3
  SET(DDRF,EXT_CNTRL1_PIN);
  SET(DDRF,EXT_CNTRL2_PIN);
  SET(DDRF,EXT_CNTRL3_PIN);
  SET(DDRF,EXT_CNTRL4_PIN);
  CLR(DDRE,GPS_PPS_PIN); //PORTJ PK0
  CLR(DDRE,RTC_PPS_PIN); //PORTJ PK1
  //DDRJ |= B00101000;  // Set using DDR regiter for non mapped pins

  // Startup devices
  digitalWrite(POWER_LED, HIGH);  // Turns on level converter
  digitalWrite(LEVEL_CNV_ENABLE, HIGH);  // Turns on level converter
  digitalWrite(LEVEL_CNV1_ENABLE,HIGH);
  digitalWrite(PPS_MUX,LOW);
  SET(PORTE,CAM_SYNC_PIN);

  //Serial ports
  Serial1.begin(115200); // Jetson Communication
  Serial.begin(38400); // GPS Reciever
  Serial3.begin(38400); // GPS Reciever

  // Configure Timer 4 for 100Hz -  toggle achieves the cam trigger of 50 Hz
  noInterrupts();           // disable all interrupts
  TCCR4A = 0;               // disable compare capture A
  TCCR4B = 0;               // disable compare capture B
  TCNT4 = preload_hi; //25536;            // preload timer 65536-16MHz/8/100Hz  - 50Hz overflow
  TCCR4B |= (1 << CS41);    // PS 8 prescaler :Timer resolution 1/16e6*PS
  TIMSK4 |= (1 << TOIE4);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts


  //Masked External interrupt (GPS)  PCINT 16
  //PCICR |= B00000100; // Pin change interrupt 1 enabled (PCINT15:8)
  //PCMSK2 |= B00000001; // Specify which pin is enabled (PCINT16 and 17)

  //Enable eny external inturupts
  EICRB |= B11000000;
  EIMSK |= B10000000;
  

  printf("test \n");
}

// Loop functions
void loop() {
  //digitalWrite(JETSON_PWR,HIGH);
  //digitalWrite(JETSON_REC,HIGH);
  //digitalWrite(JETSON_RST,HIGH);
  if (flag_write_serial == true){
      Serial1.write(value_2);
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
    }

    if(flag_write_serial2){
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
    }
 
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

 /*while (Serial3.available()){
    printf("test \n");
    Serial.write(Serial3.read());
 }*/


  
}

// ISR -  PC inturupts - Masked Block 2
// This is only invoked when GPS PPS is there
ISR(INT7_vect) {
  TOGGLE(PORTA,PPS_LED_PIN); 
}

// Inturupt service routine - timer overflow inturupt 100Hz
ISR(TIMER4_OVF_vect)        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  cam_pulse_count++;
  cam_pulse_count_for_GPRMC++;
  pps_width_count++;
  gps_pulse_count++;
  flag_pps_sim_high = READ2(PORTE,CAM_SYNC_PIN);
  //if(flag_pps_sim_high) CLR(PORTE,CAM_SYNC_PIN);
  //else SET(PORTE,CAM_SYNC_PIN);
  TOGGLE(PORTE,CAM_SYNC_PIN); // 25Hz pulse width 50%
  flag_pps_sim_high = !flag_pps_sim_high;
  if (flag_pps_sim_high){
        TCNT4 = preload_hi;}
  else{
        TCNT4 = preload_lo;
  }


  if(cam_pulse_count>=cam_pps_preload+100 && flag_pps_sim_high){ //100 => 1s
    //SET(PORTF,EXT_CNTRL1_PIN);
    //digitalWrite(3,HIGH);
    //SET(PORTE,SIM_PPS_PIN);
    SET(PORTA,JETSON_RST_PIN);
    CLR(PORTA,JETSON_REC_PIN);
    SET(PORTF,EXT_CNTRL1_PIN);
    SET(PORTF,EXT_CNTRL2_PIN);
    SET(PORTF,EXT_CNTRL3_PIN);
    SET(PORTF,EXT_CNTRL4_PIN);
    //digitalWrite(JETSON_RST,HIGH);
    //digitalWrite(JETSON_REC,LOW);
    //digitalWrite(PPS_LED,HIGH); //IMU Sync LED on*/
    cam_pulse_count = 0;
    pps_width_count = 0;
    flag_pps_sim_high = true;
  }

  if(flag_pps_sim_high && pps_width_count>=cam_pps_preload+38){  // 38=>38/2*0.02 =>380ms
    //digitalWrite(3,LOW);
    //CLR(PORTE,SIM_PPS_PIN);
    CLR(PORTA,JETSON_RST_PIN);
    SET(PORTA,JETSON_REC_PIN);
    CLR(PORTF,EXT_CNTRL1_PIN);
    CLR(PORTF,EXT_CNTRL2_PIN);
    CLR(PORTF,EXT_CNTRL3_PIN);
    CLR(PORTF,EXT_CNTRL4_PIN);
    //digitalWrite(JETSON_RST,LOW);
    //digitalWrite(JETSON_REC,HIGH);
    //digitalWrite(PPS_LED,LOW); //IMU Sync LED on*/
    pps_width_count = 0;
    flag_pps_sim_high = false;
  }

  if(cam_pulse_count_for_GPRMC>=100 && flag_pps_sim_high){ //writes serial every second
    if(ss<59){
      ss++;
		}else{
      ss=0;
      if(mm<59){
				 mm++;
			 }else{
         mm=0;
         if(hh<23){
           hh++;
           }else{
             hh=0;
             }
			 }
		}
    cam_pulse_count_for_GPRMC=0; 
    flag_write_serial = true;
    

    //PPS LOCK handling
    //if(plock_ready){     
    //}
  }
  
}

unsigned char result;
int i;
int checkNum(const char *gprmcContext)
{
    if (gprmcContext == NULL) 
    {
        // printf("Input is NULL.\n");
        return -1;
		}

    result = gprmcContext[1];

    for (i = 2; gprmcContext[i] != '*' && gprmcContext[i] != '\0'; i++)
    {
        // printf("Processing character: %c (ASCII: %d)\n", gprmcContext[i], gprmcContext[i]);
        result ^= gprmcContext[i];
    }

    if (gprmcContext[i] != '*') 
    {
        // printf("No '*' found in the string.\n");
        return -1;
    }

    //printf("Final result before returning: %02X\n", result);
    return result;
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

    char * strtokIndx; // this is used by strtok() as an index
    byte crc_calc = 0;
    byte crc_recv = 0;
    char crc_str[] = "4C";
    
    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    if (strcmp(strtokIndx,"$GNRMC")==0){
      strcpy(messageFromPC, strtokIndx);
      strcat(messageFromPC, ",");
      strtokIndx = strtok(NULL, "\n");
      strcat(messageFromPC, strtokIndx); // copy it to messageFromPC
      
      //CRC calculation
      if(messageFromPC[strlen(messageFromPC)-4]=='*'){
        crc_calc= convertToCRC(messageFromPC);
        crc_str[0] = messageFromPC[strlen(messageFromPC)-3];
        crc_str[1] = messageFromPC[strlen(messageFromPC)-2]; 
        //Serial.println(crc_calc);
        //crc_str = strcat(messageFromPC[strlen(messageFromPC)-2],messageFromPC[strlen(messageFromPC)-1]);
        crc_recv = strtol(crc_str, NULL, 16);
        if(crc_recv!=crc_calc){
          memset(messageFromPC, 0,strlen(messageFromPC));
          Serial1.write("Checksum error");
          } 
        }
        else{
          memset(messageFromPC, 0,strlen(messageFromPC));   
          Serial1.write("No * in NMEA");
        }
    }
    else{
      if (strcmp(strtokIndx,"$GNGGA")==0){
        strcpy(messageFromPC, strtokIndx);
        strcat(messageFromPC, ",");
        strtokIndx = strtok(NULL, "\n");
        strcat(messageFromPC, strtokIndx); 
        Serial1.write(messageFromPC);
        memset(messageFromPC, 0,strlen(messageFromPC));
      }
      else {
          memset(messageFromPC, 0,strlen(messageFromPC));
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
  if (strlen(messageFromPC)>0 && flag_send_next_nema){
    //Serial.print("Message ");
    //Serial2.println(messageFromPC);
    Serial1.write(messageFromPC);
    //flag_send_next_nema = false;
    //flag_send_next_nema_delayed = true;
    //modifyNEMAData();
    //strcpy(messageToLidar,messageFromPC);   
  }
}

byte convertToCRC(char *buff) {
  // NMEA CRC: XOR each byte with previous for all chars between '$' and '*'
  char c;
  byte i;
  byte start_with = 0;
  byte end_with = 0;
  byte crc = 0;

  for (i = 0; i < buff_len; i++) {
    c = buff[i];
    if(c == '$'){
      start_with = i;
    }
    if(c == '*'){
      end_with = i;
    }      
  }
  if (end_with > start_with){
    for (i = start_with+1; i < end_with; i++){ // XOR every character between '$' and '*'
      crc = crc ^ buff[i] ;  // compute CRC
    }
  }
  else { // else if error, print a msg (to both ports)
    Serial1.write("CRC ERROR");
    //SERIALN.println("CRC ERROR");
  }
  return crc;
  //based on code by Elimeléc López - July-19th-2013
}


////////////////////////////
//
// V21.1 Skeleton.ino
//
//
// 2022-12-17 Jens Andersson
//
////////////////////////////

//
// Select library
#include <datacommlib.h>

//
// Prototypes
//
// predefined functions
void l1_send(unsigned long l2frame, int framelen);
boolean l1_receive(int timeout);
// your own

//
// Runtime
//

// Runtime variables

// State
int state = NONE;

//////////////////////////////////////////////////////////
//
// Add global constant and variable declarations here
//
Shield sh;  // note! no () since constructor takes no arguments
Transmit tx;
Receive rx;
int select_led() = 0;
//void l1_send(unsigned long frame, int framelen);
#define T_S 100
#define LEN_PREAMBLE 8
#define PREAMBLE_SEQ 0b10101010
#define LEN_SFD 8
#define SFD_SEQ 0b01111110

//////////////////////////////////////////////////////////

//
// Code
//
void setup() {
  sh.begin();

  //////////////////////////////////////////////////////////
  //
  // Add init code here
  //

  state = NONE;
  
  // Set your development node's address here

  //////////////////////////////////////////////////////////
}

void loop() {

  //////////////////////////////////////////////////////////
  //
  // State machine
  // Add code for the different states here
  //

  switch (state) {

    case L1_SEND:
      Serial.println("[State] L1_SEND");
      // +++ add code here and to the predefined function void l1_send(unsigned long l2frame, int framelen) below
      Serial.print("L1_SEND: ");
      l1_send(tx.frame, LEN_FRAME);
      state = HALT;
      // ---
      break;

    case L1_RECEIVE:
      Serial.println("[State] L1_RECEIVE");
      // +++ add code here and to the predefined function boolean l1_receive(int timeout) below
	  
      // ---
      break;

    case L2_DATA_SEND:
      Serial.println("[State] L2_DATA_SEND");
      // +++ add code here
 
      // ---
      break;

    case L2_RETRANSMIT:
      Serial.println("[State] L2_RETRANSMIT");
      // +++ add code here

      // ---
      break;

    case L2_FRAME_REC:
      Serial.println("[State] L2_FRAME_REC");
      // +++ add code here

      // ---
      break;

    case L2_ACK_SEND:
      Serial.println("[State] L2_ACK_SEND");
      // +++ add code here

      // ---
      break;

    case L2_ACK_REC:
      Serial.println("[State] L2_ACK_REC");
      // +++ add code here

      // ---
      break;

    case APP_PRODUCE:
      Serial.println("[State] APP_PRODUCE");
      valdLED = sh.select_led();
      if(valdLED >= 1 && valdLED <= 3){
        Serial.print("Vald LED: " + valdLED);

        state = HALT;
      }
      // +++ add code here
      // ---
      break;

    case APP_ACT:
      Serial.println("[State] APP_ACT");
      // +++ add code here

      // ---
      break;

    case HALT:
      Serial.println("[State] HALT");
      sh.halt();
      break;

    default:
      Serial.println("UNDEFINED STATE");
      break;
  }

  //////////////////////////////////////////////////////////
}

//////////////////////////////////////////////////////////
//
// Add code to the predefined functions
//

void l1_send(unsigned long frame, int framelen) {

  //preamble saken under samma pulse
  for (int i = LEN_PREAMBLE - 1; i >= 0; i--) {
    int bit = (PREAMBLE_SEQ >> i) & 0x1;
    tx.pulse(bit);
    delay(T_S);
  }

  //SFD
  for (int i = LEN_SFD - 1; i >= 0; i--) {
    int bit = (SFD_SEQ >> i) & 0x1;
    tx.pulse(bit);
    delay(T_S);
  }
  
  //L2 frame som skickas
  for (int i = framelen - 1; i >= 0; i--) {
    int bit = (frame >> i) & 0x1;
    tx.pulse(bit);
    delay(T_S);
  }
  
boolean l1_receive(int timeout) {
	
	return true;
}

//////////////////////////////////////////////////////////
//
// Add your functions here
//

/* *************************************************************
   Encoder definitions

   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   ************************************************************ */

#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode

  void initEncoder() {
    // Nothing to do
  }

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table

  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  ISR (PCINT2_vect){
	static uint8_t enc_last=0;

	enc_last <<=2; //shift previous state two places
	enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits

	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }

  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  ISR (PCINT1_vect){
        static uint8_t enc_last=0;

	enc_last <<=2; //shift previous state two places
	enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits

	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }

  void initEncoder() {
    //set as inputs
    DDRD &= ~(1<<LEFT_ENC_PIN_A);
    DDRD &= ~(1<<LEFT_ENC_PIN_B);
    DDRC &= ~(1<<RIGHT_ENC_PIN_A);
    DDRC &= ~(1<<RIGHT_ENC_PIN_B);

    //enable pull up resistors
    PORTD |= (1<<LEFT_ENC_PIN_A);
    PORTD |= (1<<LEFT_ENC_PIN_B);
    PORTC |= (1<<RIGHT_ENC_PIN_A);
    PORTC |= (1<<RIGHT_ENC_PIN_B);

    // tell pin change mask to listen to left encoder pins
    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
    // tell pin change mask to listen to right encoder pins
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);

    // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
    PCICR |= (1 << PCIE1) | (1 << PCIE2);
  }

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }
#elif defined POLOLU_ASTAR_ROBOT_CONTROLLER
  #include <AStar32U4.h>
  #ifdef USE_ENABLE_INTERRUPT
    #include <EnableInterrupt.h>
  #endif

  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;

  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table

#ifndef USE_ENABLE_INTERRUPT
  static void pciSetup(byte pin) {
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR |= 1; // clear any outstanding interrupt
    *digitalPinToPCICR(pin) |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
  }

   /* Interrupt routine for LEFT encoder, taking care of actual counting */
  ISR(PCINT0_vect) {
    static uint8_t enc_last = 0;
    bool b = FastGPIO::Pin<LEFT_ENC_PIN_B>::isInputHigh();
    bool a = FastGPIO::Pin<LEFT_ENC_PIN_XOR>::isInputHigh() ^ b;

    enc_last <<= 2;
    enc_last |= ((uint8_t)a) << 1 | (uint8_t)b;

    left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }

  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  static void rightISR() {
    static uint8_t enc_last = 0;
    bool b = FastGPIO::Pin<RIGHT_ENC_PIN_B>::isInputHigh();
    bool a = FastGPIO::Pin<RIGHT_ENC_PIN_XOR>::isInputHigh() ^ b;

    enc_last <<= 2;
    enc_last |= ((uint8_t)a) << 1 | (uint8_t)b;

    right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }

#else //USE_ENABLE_INTERRUPT

  // Update the encoder when the left A pin changes. We increment or
  // decrement by 2 since only the A channel is connected to an
  // interrupt. (This leaves PID constants unchanged if both A and B
  // interrupts are used.)
  void leftISR() {
    left_enc_pos +=
      (FastGPIO::Pin<LEFT_ENC_PIN_A>::isInputHigh()
       == FastGPIO::Pin<LEFT_ENC_PIN_B>::isInputHigh())
        ? 2
        : -2;
  }
  
  // Update the encoder when the right A pin changes. We increment or
  // decrement by 2 since only the A channel is connected to an
  // interrupt. (This leaves PID constants unchanged if both A and B
  // interrupts are used.)
  void rightISR() {
    right_enc_pos +=
      (FastGPIO::Pin<RIGHT_ENC_PIN_A>::isInputHigh()
       == FastGPIO::Pin<RIGHT_ENC_PIN_B>::isInputHigh())
        ? 2
        : -2;
  }
#endif //USE_ENABLE_INTERRUPT
  
  void initEncoder() {
    #ifndef USE_ENABLE_INTERRUPT
      FastGPIO::Pin<LEFT_ENC_PIN_XOR>::setInputPulledUp();
      FastGPIO::Pin<LEFT_ENC_PIN_B>::setInputPulledUp();
      FastGPIO::Pin<RIGHT_ENC_PIN_XOR>::setInputPulledUp();
      FastGPIO::Pin<RIGHT_ENC_PIN_B>::setInputPulledUp();
      pciSetup(LEFT_ENC_PIN_XOR);
      attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_XOR), rightISR, CHANGE);
    #else
      FastGPIO::Pin<LEFT_ENC_PIN_A>::setInputPulledUp();
      FastGPIO::Pin<LEFT_ENC_PIN_B>::setInputPulledUp();
      FastGPIO::Pin<RIGHT_ENC_PIN_A>::setInputPulledUp();
      FastGPIO::Pin<RIGHT_ENC_PIN_B>::setInputPulledUp();
      // Only enable interrupts on the A encoder pins, to reduce the
      // rate of interrupts.
      enableInterrupt(LEFT_ENC_PIN_A, leftISR, CHANGE);
      enableInterrupt(RIGHT_ENC_PIN_A, rightISR, CHANGE);
    #endif //USE_ENABLE_INTERRUPT
  }

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) {
      return left_enc_pos;
    }
    else if (i == RIGHT) {
      return right_enc_pos;
    }

    return 0;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) {
      left_enc_pos = 0L;
    }
    else if (i == RIGHT) {
      right_enc_pos = 0L;
    }
  }

#elif defined SPARKFUN_REDBOT_ENCODER
#include "RedBot.h"

RedBotEncoder *encoder = new RedBotEncoder(A0, A1);

void initEncoder() {
  // Nothing to do
}

long readEncoder(int i) {
  if (i == LEFT) {
    return encoder->getTicks(WHEEL_LEFT);
  }
  else if (i == RIGHT) {
    return encoder->getTicks(WHEEL_RIGHT);
  }

  return 0;
}

void resetEncoder(int i) {
  if (i == LEFT) {
    encoder->clearEnc(WHEEL_LEFT);
  }
  else if (i == RIGHT) {
    encoder->clearEnc(WHEEL_RIGHT);
  }
}

void setDir(int i, char dir) {
  if (i == LEFT) {
    encoder->setDir(WHEEL_LEFT, dir);
  }
  else if (i == RIGHT) {
    encoder->setDir(WHEEL_RIGHT, dir);
  }
}

#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif


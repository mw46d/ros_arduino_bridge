#ifndef __I2C_H__
#define __I2C_H__

#define USE_I2C

#define I2C_MAX_SENT_BYTES 8
#define I2C_ADDRESS        0x42

#define ANALOG_RATE        20
#define ANALOG_NUM         8
#define ANALOG_INTERVAL    1000 / (ANALOG_RATE * ANALOG_NUM)

volatile unsigned char i2c_buffer[] = {
  // 0x00 - 0x07 Version, read-only
  '2', '0', '1', '5', '1', '2', '1', '1',
  // 0x08 - 0x0f VendorId, read-only
  'm', 'w', '4', '6', 'd', ' ', ' ', ' ',
  // 0x10 - 0x17 DeviceId, read-only
  'P', 'i', 'A', '*', 'H', 'a', 't', ' ',
  // space
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  // 0x20 - 0x2f
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  // 0x30 - 0x3f
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  // 0x40 Command byte, write-only
  0x00,
  // space
  0x00, 0x00, 0x00,
  // 0x44 - 0x47 left encoder, LSB first
  0x00, 0x00, 0x00, 0x00,
  // 0x48 - 0x4b right encoder, LSB first
  0x00, 0x00, 0x00, 0x00,
  // 0x4c - 0x4d left speed, LSB first
  0x00, 0x00,
  // 0x4e - 0x4f right speed, LSB first
  0x00, 0x00,
  // 0x50 - 0x51 Kp, LSB first
  0x00, 0x00,
  // 0x52 - 0x53 Ki, LSB first
  0x00, 0x00,
  // 0x54 - 0x55 Kd, LSB first
  0x00, 0x00,
  // 0x56 - 0x57 Ko, LSB first
  0x00, 0x00,
  // 8 * 2: Analog Read
  0x00, 0x00, // 0x58
  0x00, 0x00, // 0x5a
  0x00, 0x00, // 0x5c
  0x00, 0x00, // 0x5e
  0x00, 0x00, // 0x60
  0x00, 0x00, // 0x62
  0x00, 0x00, // 0x64
  0x00, 0x00, // 0x66
  // space
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

unsigned long next_analog_read_time = ANALOG_INTERVAL;
unsigned char next_analog_read_pin = 0;
unsigned char i2c_requested_address = 0;

static void i2c_update_encoder_buffer(unsigned char cmd) {
  long e = readEncoder((cmd - 0x44) / 4);

  i2c_buffer[cmd    ] =  e        & 0xff;
  i2c_buffer[cmd + 1] = (e >>  8) & 0xff;
  i2c_buffer[cmd + 2] = (e >> 16) & 0xff;
  i2c_buffer[cmd + 3] = (e >> 24) & 0xff;
}

static void i2c_update_analog_read() {
  if (millis() > next_analog_read_time) {
    unsigned int i = analogRead(next_analog_read_pin);
    unsigned char addr = 0x58 + next_analog_read_pin * 2;

    i2c_buffer[addr    ] =  i        & 0xff;
    i2c_buffer[addr + 1] = (i >>  8) & 0xff;

    next_analog_read_time += ANALOG_INTERVAL;
    next_analog_read_pin = (next_analog_read_pin + 1) % ANALOG_NUM;
  }
}

static unsigned int calculate_fletcher16(unsigned char *buff, unsigned char len) {
  unsigned char s1 = 0;
  unsigned char s2 = 0;

  for (unsigned char i = 0; i < len; i++) {
    s1 += buff[i];
    s2 += s1;
  }

  return ((unsigned int)s2) << 8 | ((unsigned int)s1) & 0xff;
}

static void i2c_request_event() {
  unsigned char addr = i2c_requested_address;
  unsigned char len = 1;
  unsigned char buff[16];

  if (addr < 0x40) {
    addr = (addr / 8) * 8;
    len = 8;
  }
  else if (addr < 0x44) {
    // Just one byte useless values
  }
  else if (addr < 0x4c) {
    addr = (addr / 4) * 4;
    len = 4;
  }
  else if (addr < sizeof(i2c_buffer)) {
    addr = (addr / 2) * 2;
    len = 2;
  }

  buff[0] = I2C_ADDRESS;
  buff[1] = addr;
  buff[2] = len + 2;

  for (unsigned char i = 0; i < len; i++) {
    buff[3 + i] = ((unsigned char *)(i2c_buffer + addr))[i];
  }

  unsigned int chk = calculate_fletcher16(buff, len + 3);

  buff[3 + len] = chk & 0xff;
  buff[3 + len + 1] = (chk >> 8) & 0xff;

  Wire.write(len + 2);
  Wire.write((unsigned char *)(buff + 3), len + 2);
}

static void i2c_receive_event(int bytes_received) {
  unsigned char buff[I2C_MAX_SENT_BYTES];
  unsigned char cmd;

  for (int a = 0; a < I2C_MAX_SENT_BYTES; a++) {
    buff[a] = 0;
  }

  buff[0] = I2C_ADDRESS;

  for (int a = 0; a < bytes_received; a++) {
    if (a  + 1 < I2C_MAX_SENT_BYTES) {
      buff[a + 1] = Wire.read();
    }
    else {
      Wire.read();  // if we receive more data then allowed just throw it away
    }
  }

  cmd = buff[1] = buff[1] % sizeof(i2c_buffer);

  if (bytes_received == 1) {
    // Those are actually reads
    i2c_requested_address = cmd;

    if (cmd == 0x44 || cmd == 0x48) {
       i2c_update_encoder_buffer(cmd);
    }

    if (cmd == 0x58 || cmd == 0x5a ||
        cmd == 0x5c || cmd == 0x5e ||
        cmd == 0x60 || cmd == 0x62 ||
        cmd == 0x64 || cmd == 0x66) {
      // The analog reads are done asynchronosly!
    }

    cmd = 0;
  }

  if (bytes_received > 1) {
    // We got a write/process.
    unsigned char len = buff[2];

    if (len < 3) {
      // Too short
      DEBUG_SERIAL_STREAM.println("i2c_receive_event too short");
      return;
    }

    // remove the checkup bytes
    len -= 2;
    unsigned int chk = calculate_fletcher16(buff, len + 3); // +3 -- the header bytes
    unsigned int chk_r = buff[3 + len + 1] << 8 | buff[3 + len];
    if (chk != chk_r) {
      // Checksum error
      DEBUG_SERIAL_STREAM.print("i2c_receive_event checksum error ");
      DEBUG_SERIAL_STREAM.print(chk);
      DEBUG_SERIAL_STREAM.print(" != ");
      DEBUG_SERIAL_STREAM.println(chk_r);
      return;
    }

    if (cmd == 0x40) {
      // We got a command
      i2c_buffer[cmd] = buff[3];
    }
    else if (cmd == 0x4c || cmd == 0x4e ||
        cmd == 0x50 || cmd == 0x52 || cmd == 0x54 || cmd == 0x56) {
      // two bytes expected
      i2c_buffer[cmd] = buff[3];
      i2c_buffer[cmd + 1] = buff[4];
    }
  }
}

/*
static void processScanValue(int index, int pin, int value) {
  unsigned char addr = 0x58 + index * 2;
  unsigned int v = i2c_buffer[addr + 1] << 8 + i2c_buffer[addr];

  v = (v + v + v + value) >> 2;

  i2c_buffer[addr    ] =  v        & 0xff;
  i2c_buffer[addr + 1] = (v >>  8) & 0xff;
}

// Creates an instance of the analog pin scanner.
static AnalogScanner scanner;
*/

static void i2c_get_analog_values() {
  if (millis() > next_analog_read_time) {
    for (int i = 0; i < ANALOG_NUM; i++) {
      unsigned int v = analogRead(i);
      unsigned char addr = 0x58 + i * 2;

      i2c_buffer[addr    ] =  v        & 0xff;
      i2c_buffer[addr + 1] = (v >>  8) & 0xff;
    }

    next_analog_read_time += 50;
  }
}

void initI2c() {
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(i2c_receive_event);
  Wire.onRequest(i2c_request_event);

  // Kp
  i2c_buffer[0x50] = Kp & 0xff;
  i2c_buffer[0x51] = (Kp >> 8) & 0xff;
  // Ki
  i2c_buffer[0x52] = Ki & 0xff;
  i2c_buffer[0x53] = (Ki >> 8) & 0xff;
  // Kd
  i2c_buffer[0x54] = Kd & 0xff;
  i2c_buffer[0x55] = (Kd >> 8) & 0xff;
  // Ko
  i2c_buffer[0x56] = Ko & 0xff;
  i2c_buffer[0x57] = (Ko >> 8) & 0xff;

/*
  int scanOrder[] = { A0, A1, A2, A3, A4, A5, A6, A7 };
  scanner.setScanOrder(8, scanOrder);
  scanner.setCallback(A0, processScanValue);
  scanner.setCallback(A1, processScanValue);
  scanner.setCallback(A2, processScanValue);
  scanner.setCallback(A3, processScanValue);
  scanner.setCallback(A4, processScanValue);
  scanner.setCallback(A5, processScanValue);
  scanner.setCallback(A6, processScanValue);
  scanner.setCallback(A7, processScanValue);
  scanner.beginScanning();
  delay(1);
*/
}

void runI2c() {
  unsigned char cmd = i2c_buffer[0x40];

  i2c_buffer[0x40] = 0;

  // if (cmd > 0) {
  //   DEBUG_SERIAL_STREAM.print("i2c_handle_command("); DEBUG_SERIAL_STREAM.print(cmd);
  // }

  switch(cmd) {
  case ANALOG_READ:
    // not needed, just read the i2c area
    break;
  case READ_ENCODERS:
    // not needed, just read the i2c area
    break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (i2c_buffer[0x4c] == 0 && i2c_buffer[0x4d] == 0 && i2c_buffer[0x4e] == 0 && i2c_buffer[0x4f] == 0) {
      setMotorSpeeds(0, 0);
      moving = 0;
    }
    else {
      moving = 1;
    }
    leftPID.TargetTicksPerFrame = (double)((int)i2c_buffer[0x4d] << 8 | ((int)i2c_buffer[0x4c] & 0xff));
    rightPID.TargetTicksPerFrame = (double)((int)i2c_buffer[0x4f] << 8 | ((int)i2c_buffer[0x4e] & 0xff));
    DEBUG_SERIAL_STREAM.print(") leftTpF= "); DEBUG_SERIAL_STREAM.print(leftPID.TargetTicksPerFrame);
    DEBUG_SERIAL_STREAM.print("  rightTpF= "); DEBUG_SERIAL_STREAM.print(rightPID.TargetTicksPerFrame);
    break;
  case UPDATE_PID:
    Kp = (int)i2c_buffer[0x51] << 8 | ((int)i2c_buffer[0x50] & 0xff);
    Ki = (int)i2c_buffer[0x53] << 8 | ((int)i2c_buffer[0x52] & 0xff);
    Kd = (int)i2c_buffer[0x55] << 8 | ((int)i2c_buffer[0x54] & 0xff);
    Ko = (int)i2c_buffer[0x57] << 8 | ((int)i2c_buffer[0x56] & 0xff);
    DEBUG_SERIAL_STREAM.print(")  Kp= ");
    DEBUG_SERIAL_STREAM.print(Kp);
    DEBUG_SERIAL_STREAM.print("  Kd= ");
    DEBUG_SERIAL_STREAM.print(Kd);
    DEBUG_SERIAL_STREAM.print("  Ki= ");
    DEBUG_SERIAL_STREAM.print(Ki);
    DEBUG_SERIAL_STREAM.print("  Ko= ");
    DEBUG_SERIAL_STREAM.println(Ko);
    break;
  }
  // if (cmd > 0) {
  //   DEBUG_SERIAL_STREAM.println(" ");
  // }

  /*
  i2c_update_analog_read();
  */
  i2c_get_analog_values();
}

#endif

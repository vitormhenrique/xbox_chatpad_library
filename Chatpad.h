/*
 * Chatpad.h - Use an Xbox360 mini-keyboard with your Arduino.
 *
 * Copyright (C) 2011 Cliff L. Biffle, all rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef Chatpad_h
#define Chatpad_h

#include <stdint.h>

// Declaration of Arduino's internal serial port type.
class HardwareSerial;


/*
 * A serial interface to the Xbox 360 chatpad.  Usage:
 *
 *  Chatpad pad;
 *
 *  void my_callback(Chatpad &pad, Chatpad::keycode_t key,
 *      Chatpad::eventtype_t event) {
 *    ...
 *  }
 *
 *  void setup() {
 *    ...
 *    pad.init(Serial3, my_callback);
 *  }
 *
 *  void loop() {
 *    ...
 *    pad.poll();
 *  }
 */
class Chatpad {
public:
  // Provides names for the numeric keycodes.
  enum keycode_t {
    Key1 = 0x17,
    Key2 = 0x16,
    Key3 = 0x15,
    Key4 = 0x14,
    Key5 = 0x13,
    Key6 = 0x12,
    Key7 = 0x11,
    Key8 = 0x67,
    Key9 = 0x66,
    Key0 = 0x65,

    KeyQ = 0x27,
    KeyW = 0x26,
    KeyE = 0x25,
    KeyR = 0x24,
    KeyT = 0x23,
    KeyY = 0x22,
    KeyU = 0x21,
    KeyI = 0x76,
    KeyO = 0x75,
    KeyP = 0x64,

    KeyA = 0x37,
    KeyS = 0x36,
    KeyD = 0x35,
    KeyF = 0x34,
    KeyG = 0x33,
    KeyH = 0x32,
    KeyJ = 0x31,
    KeyK = 0x77,
    KeyL = 0x72,
    KeyComma = 0x62,

    KeyZ = 0x46,
    KeyX = 0x45,
    KeyC = 0x44,
    KeyV = 0x43,
    KeyB = 0x42,
    KeyN = 0x41,
    KeyM = 0x52,
    KeyPeriod = 0x53,
    KeyEnter = 0x63,

    KeyLeft = 0x55,
    KeySpace = 0x54,
    KeyRight = 0x51,
    KeyBackspace = 0x71,

    KeyShift = 0x81,
    KeyGreenSquare = 0x82,
    KeyPeople = 0x83,
    KeyOrangeCircle = 0x84,
  };

  enum eventtype_t {
    Up = 0,
    Down = 1,
  };

  typedef void (*callback_t)(Chatpad &, keycode_t, eventtype_t);

  /*
   * Sets up communications with the chatpad, including initializing the
   * serial port.
   */
  void init(HardwareSerial &, callback_t);

  /*
   * Processes any pending messages from the chatpad.
   */
  void poll();

  /*
   * Checks if Shift is down.  Use this during a callback.
   */
  bool isShiftDown() const;
  /*
   * Checks if Green Square is down.  Use this during a callback.
   */
  bool isGreenSquareDown() const;
  /*
   * Checks if Orange Circle is down.  Use this during a callback.
   */
  bool isOrangeCircleDown() const;
  /*
   * Checks if People is down.  Use this during a callback.
   */
  bool isPeopleDown() const;

  /*
   * Converts a key to ASCII, given the current status of the Shift
   * key.  If the key doesn't map to an ASCII character (example:
   * the left arrow key), returns 0.
   */
  char toAscii(keycode_t);

private:
  HardwareSerial *_serial;
  callback_t _callback;
  uint8_t _buffer[8];

  uint8_t _last_modifiers;
  uint8_t _last_key0;
  uint8_t _last_key1;

  uint32_t _last_ping;

  void dispatch(uint8_t, int is_down);
};

#endif  // Chatpad_h

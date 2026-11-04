/*
 * Ps4Input.h - PS4-esp32 style wrapper around Bluepad32
 *
 * Lets ps4_drive.cpp keep calling PS4.xxx() while Bluepad32 does the Bluetooth.
 *
 * SETUP:
 *   1. Copy this file into include/.
 *   2. Call PS4.update() once at the top of loop() (it runs BP32.update()).
 *   3. Do not use aed3/PS4-esp32 (two Bluetooth stacks cannot coexist).
 *
 * NOTES:
 *   - Pairing: hold Share + PS until the lightbar double-flashes.
 *   - D-pad: Up() / Down() / Left() / Right() read Bluepad32's dpad() bits
 *     (UP 0x01, DOWN 0x02, RIGHT 0x04, LEFT 0x08). If one does not respond,
 *     print ctl->dpad() while pressing it and change the masks below.
 *   - Only one controller is used; a second one that connects is dropped.
 *   - Share / Options come from miscButtons(). The masks below are unverified
 *     on your Bluepad32 version.
 */
#pragma once
#include <Arduino.h>
#include <Bluepad32.h>

#ifndef BP_SHARE_MASK
#define BP_SHARE_MASK 0x02
#endif
#ifndef BP_OPTIONS_MASK
#define BP_OPTIONS_MASK 0x04
#endif

#ifndef BP_DPAD_UP
#define BP_DPAD_UP    0x01
#define BP_DPAD_DOWN  0x02
#define BP_DPAD_RIGHT 0x04
#define BP_DPAD_LEFT  0x08
#endif

class Ps4Shim
{
public:
  // The MAC argument is ignored: Bluepad32 pairs by putting the controller in
  // pairing mode (hold Share + PS until the lightbar double-flashes).
  void begin(const char * = nullptr)
  {
    BP32.setup(&Ps4Shim::onConnected, &Ps4Shim::onDisconnected);
    BP32.enableVirtualDevice(false);
  }

  void update() { BP32.update(); }

  // Forget every stored pairing (use once if a controller will not reconnect)
  void forgetPairings() { BP32.forgetBluetoothKeys(); }

  bool isConnected()
  {
    ControllerPtr c = slot();
    return c != nullptr && c->isConnected();
  }

  // Buttons
  bool Cross()    { ControllerPtr c = live(); return c && c->a(); }
  bool Circle()   { ControllerPtr c = live(); return c && c->b(); }
  bool Square()   { ControllerPtr c = live(); return c && c->x(); }
  bool Triangle() { ControllerPtr c = live(); return c && c->y(); }
  bool L1()       { ControllerPtr c = live(); return c && c->l1(); }
  bool R1()       { ControllerPtr c = live(); return c && c->r1(); }
  bool Share()    { ControllerPtr c = live(); return c && (c->miscButtons() & BP_SHARE_MASK) != 0; }
  bool Options()  { ControllerPtr c = live(); return c && (c->miscButtons() & BP_OPTIONS_MASK) != 0; }

  // D-pad
  bool Up()       { ControllerPtr c = live(); return c && (c->dpad() & BP_DPAD_UP)    != 0; }
  bool Down()     { ControllerPtr c = live(); return c && (c->dpad() & BP_DPAD_DOWN)  != 0; }
  bool Right()    { ControllerPtr c = live(); return c && (c->dpad() & BP_DPAD_RIGHT) != 0; }
  bool Left()     { ControllerPtr c = live(); return c && (c->dpad() & BP_DPAD_LEFT)  != 0; }

  // Debug: raw D-pad bits, and whether the controller has delivered data
  uint8_t dpadRaw() { ControllerPtr c = live(); return c ? c->dpad() : 0; }
  bool    hasData() { return live() != nullptr; }

  // Lightbar. Bluepad32 sends it immediately, so sendToController() does nothing.
  void setLed(uint8_t r, uint8_t g, uint8_t b)
  {
    ControllerPtr c = slot();
    if (c && c->isConnected()) c->setColorLED(r, g, b);
  }
  void sendToController() {}

private:
  static ControllerPtr &slot()
  {
    static ControllerPtr c = nullptr;
    return c;
  }

  // The controller, but only when it is connected and has delivered data
  static ControllerPtr live()
  {
    ControllerPtr c = slot();
    return (c && c->isConnected() && c->hasData()) ? c : nullptr;
  }

  static void onConnected(ControllerPtr ctl)
  {
    if (slot() == nullptr)
    {
      slot() = ctl;
      Serial.println("PS4 controller connected");
    }
    else
    {
      ctl->disconnect();  // already have one
    }
  }

  static void onDisconnected(ControllerPtr ctl)
  {
    if (slot() == ctl)
    {
      slot() = nullptr;
      Serial.println("PS4 controller disconnected");
    }
  }
};

static Ps4Shim PS4;
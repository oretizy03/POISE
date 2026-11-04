/*
 * PS4 D-pad test (Bluepad32, no robot hardware needed)
 *
 * Talks to Bluepad32 directly (no Ps4Input.h), so it shows what the
 * controller really sends. Open serial at 115200, pair the controller
 * (hold Share + PS until the lightbar double-flashes).
 *
 * Prints a line whenever dpad / buttons / misc buttons change, and a status
 * line every 2 s so you can tell "connected, nothing pressed" from "no data".
 *
 * Expected: UP=0x01 DOWN=0x02 RIGHT=0x04 LEFT=0x08.
 * Circle -> b, L1 -> l1. Pressing Share or Options shows the misc value.
 */
#include <Arduino.h>
#include <Bluepad32.h>

static ControllerPtr pad = nullptr;

static void onConnected(ControllerPtr ctl)
{
  if (pad == nullptr)
  {
    pad = ctl;
    Serial.println(">> controller connected");
  }
  else
  {
    ctl->disconnect();
  }
}

static void onDisconnected(ControllerPtr ctl)
{
  if (pad == ctl)
  {
    pad = nullptr;
    Serial.println(">> controller disconnected");
  }
}

static void dpadText(uint8_t d, char *out)
{
  out[0] = 0;
  if (d & 0x01) strcat(out, "UP ");
  if (d & 0x02) strcat(out, "DOWN ");
  if (d & 0x04) strcat(out, "RIGHT ");
  if (d & 0x08) strcat(out, "LEFT ");
  if (!out[0]) strcpy(out, "-");
}

void setup()
{
  Serial.begin(115200);
  delay(300);
  Serial.println("\nPS4 D-pad test");

  BP32.setup(&onConnected, &onDisconnected);
  BP32.enableVirtualDevice(false);
  Serial.println("Waiting for controller. Hold Share + PS to pair.");
}

void loop()
{
  BP32.update();   // must run every loop

  static uint8_t lastDpad = 0xFF, lastMisc = 0xFF;
  static uint16_t lastBtn = 0xFFFF;
  static uint32_t lastStatus = 0;

  if (pad && pad->isConnected() && pad->hasData())
  {
    uint8_t d = pad->dpad();
    uint16_t b = pad->buttons();
    uint8_t m = pad->miscButtons();

    if (d != lastDpad || b != lastBtn || m != lastMisc)
    {
      lastDpad = d;
      lastBtn = b;
      lastMisc = m;

      char txt[32];
      dpadText(d, txt);
      Serial.printf("dpad=0x%02X [%s]  buttons=0x%04X  misc=0x%02X  |  X:%d O:%d []:%d /\\:%d L1:%d R1:%d\n",
                    d, txt, b, m,
                    pad->a(), pad->b(), pad->x(), pad->y(), pad->l1(), pad->r1());
    }
  }

  if (millis() - lastStatus >= 2000)
  {
    lastStatus = millis();
    if (!pad)                     Serial.println("status: no controller");
    else if (!pad->isConnected()) Serial.println("status: controller not connected");
    else if (!pad->hasData())     Serial.println("status: connected but NO DATA yet");
    else                          Serial.println("status: connected, data OK");
  }

  delay(5);
}

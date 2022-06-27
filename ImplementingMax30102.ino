
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

#include <Arduino_GFX_Library.h>

#define TFT_BL 9

#include <arduino-timer.h>
#include <RTClib.h>
//#include <DS3231M.h>

/* More dev device declaration: https://github.com/moononournation/Arduino_GFX/wiki/Dev-Device-Declaration */
#if defined(DISPLAY_DEV_KIT)
Arduino_GFX *gfx = create_default_Arduino_GFX();
#else  /* !defined(DISPLAY_DEV_KIT) */
// Arduino_DataBus *bus = create_default_Arduino_DataBus();
Arduino_DataBus *bus = new Arduino_ESP32SPI(12 /* DC */, 5 /* CS */, 18 /* SCK */, 23 /* MOSI */, -1 /* MISO */);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, 33 /* RST */, 4 /* rotation */, true /* IPS */);
#endif /* !defined(DISPLAY_DEV_KIT) */

const byte RATE_SIZE = 6; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
long irValue;

unsigned long delayStart = 0; // the time the delay started
bool delayRunning = false; // true if still waiting for delay to finish

String dString;
String tString;
/*
  const uint8_t  SPRINTF_BUFFER_SIZE{32}; //Buffer RTC
  volatile int interruptCounter;
  hw_timer_t *timer = NULL;
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;*/

int buttonState = LOW;
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers
int screenState = -1;


MAX30105 particleSensor;
RTC_DS3231 rtc;
//DS3231M_Class DS3231M;
auto timer = timer_create_default();

//Funcion de interrupción
bool refrescar_LCD(void *) {
  refrescar();
  return true; // repeat? true
}

void setup()
{

  Serial.begin(115200);

  pinMode(0, INPUT);
  pinMode(10, INPUT);
  pinMode(13, INPUT);

  /*  Botones
    if(digitalRead(0)==LOW){
    Serial.println("Has pulsado el boton 1");
    }
    if(digitalRead(10)==HIGH){
    Serial.println("Has pulsado el boton 2");
    }
    if(digitalRead(13)==HIGH){
    Serial.println("Has pulsado el boton 3");
    }
    delay(100);*/

  //Wire.begin();
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  //Habilitar sensor de pulso
  particleSensor.setup();                    // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);  // Turn off Green LED


  //Configurar Fecha/hora correcta
  rtc.begin();
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  //Inicializar la pantalla
  gfx->begin();
  gfx->fillScreen(BLACK);

#ifdef DF_GFX_BL
  pinMode(DF_GFX_BL, OUTPUT);
  digitalWrite(DF_GFX_BL, HIGH);
#endif

  gfx->setCursor((gfx->width() / 2) - 4, gfx->height() / 2);
  gfx->setTextColor(WHITE);
  gfx->println("Setup Ok");
  delay(3000); // 3 seconds

  timer.every(1000, refrescar_LCD);
}


void loop()
{
  timer.tick();
  medirRitmo();

}


float refrescar()
{
  gfx->flush();
  gfx->fillScreen(BLACK);
  gfx->setCursor((gfx->width() / 4), (gfx->height() / 4) );
  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->println(getTime(1));
  gfx->setCursor((gfx->width() / 2) - (gfx->width() / 6), (gfx->height() / 6) * 2 );
  gfx->println(getTime(2));
  if (irValue < 50000) {
    gfx->setCursor((gfx->width() / 8), (gfx->height() / 2));
    gfx->println("Verifique Sensor");
  } else {
    gfx->setCursor((gfx->width() / 4), gfx->height() / 2);
    gfx->println("BPM:  " + String(beatAvg));
  }


}

void medirRitmo()
{
  irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    // We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 30)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
      rateSpot %= RATE_SIZE;                    // Wrap variable

      // Take average of readings
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000)
    Serial.print(" No finger?");

  Serial.println();
}
String getTime(int x) {

  static uint8_t secs;
  DateTime now = rtc.now();  // get the current time from device
  if (secs != now.second())            // Output if seconds have changed
  {
    // Use sprintf() to pretty print the date/time with leading zeros
    /*char output_buffer[SPRINTF_BUFFER_SIZE];  ///< Temporary buffer for sprintf()
      sprintf(output_buffer, "%04d-%02d-%02d           %02d:%02d:%02d", now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());*/
    char output_buffer1[16];
    char output_buffer2[16];
    sprintf(output_buffer1, "%04d-%02d-%02d", now.year(), now.month(), now.day());
    sprintf(output_buffer2, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    dString =  output_buffer1;
    tString =  output_buffer2;

    secs = now.second();  // Set the counter variable
  } 
    if (x == 1) {
      return dString;
    } else {
      return tString;
    }
    //return output_buffer;
                           // of if the seconds have changed
  //readCommand();          // See if serial port has incoming data
}  // of method loop()

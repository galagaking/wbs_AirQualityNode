/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example will send Temperature and Air Pressure
 * using frequency and encryption settings matching those of
 * the The Things Network. Application will 'sleep' 7x8 seconds (56 seconds)
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
******************************************************************************
  Marshall Taylor @ SparkFun Electronics

  April 4, 2017

  https://github.com/sparkfun/CCS811_Air_Quality_Breakout
  https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library

  This example configures the nWAKE and nINT pins.
  The interrupt pin is configured to pull low when the data is
  ready to be collected.
  The wake pin is configured to enable the sensor during I2C communications

  Hardware Connections (Breakoutboard to Arduino):
  3.3V to 3.3V pin
  GND to GND pin
  SDA to A4
  SCL to A5
  NOT_INT to D3 (Should be D2 or D3 on 328, only interrupt pins)
  NOT_WAKE to D8 
  Resources:
  Uses Wire.h for i2c operation

  Development environment specifics:
  Arduino IDE 1.8.1

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).

  Please review the LICENSE.md file included with this example. If you have any questions
  or concerns with licensing, please contact techsupport@sparkfun.com.

  Distributed as-is; no warranty is given.

  BME280 forceRead function:
  Moteino Weather Station: Copyright (C) 2016 by Xose Pérez <xose dot perez at gmail dot com>
  https://bitbucket.org/xoseperez/weatherstation_moteino
******************************************************************************
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *****************************************************************************
Payload Function to use in your The Things Network Application:

function Decoder(bytes, port) {
var mbar = 970+((bytes[1] >> 2) & 0x3F);
var temperature = -2400+6.25*(((bytes[1] & 0x03) << 8) | bytes[0]); 
var humidity = 5+3*((bytes[3] >> 3) & 0x1F);
var co2 = 4*(((bytes[3] & 0x07) << 8) | bytes[2]);
var tvoc = 5*bytes[4];
return {
pressure: mbar,
temperature: temperature / 100.0,
humidity:humidity,
co2:co2,
tvoc:tvoc
};
}

 *******************************************************************************
 *code adapted by F. Beks for TTN Workshops in the Eindhoven IoT community     *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SparkFunCCS811.h>
#include <SparkFunBME280.h>
#include "LowPower.h"
#include <Arduino.h>


//#define CCS811_ADDR 0x5B //Default I2C Address
#define CCS811_ADDR 0x5A //Alternate I2C Address
#define PIN_NOT_WAKE 8    // must be 0 to Wake up the sensor
#define PIN_NOT_INT 3     // not used in this example

#define BME280_ADDR 0x76    //Sparkfun
//#define BME280_ADDR 0x77  //Alternate I2C Address (Adafruit board)


CCS811 myCCS811(CCS811_ADDR);
BME280 myBME280;

byte  buffer[]={0x03,0x67,0x00,0xBE,0x00}; //default values

uint16_t baseline; // offset baseline CCS811

// LoRaWAN NwkSKey, network session key, AppSKey, application session key, end-device address
static const PROGMEM u1_t NWKSKEY[16] = { 0xA5, 0x60, 0xFD, 0x60, 0xC1, 0x01, 0xE0, 0x7E, 0xA8, 0x36, 0x45, 0x5E, 0x97, 0xFE, 0x4E, 0x1E };
static const PROGMEM u1_t APPSKEY[16] = { 0xCA, 0x23, 0xE9, 0x67, 0xE5, 0x61, 0xFE, 0x6B, 0xB7, 0xE6, 0x4B, 0x60, 0x2E, 0x11, 0x47, 0xC4 };
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26011CFE; //fill in your devaddr ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

bool next = false;

// Pin mapping is hardware specific.
// Pin mapping LoraDuino
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN,
  .dio = {4, 5, 7},
};

void wakeUp()
{
    
}
//printDriverError decodes the CCS811Core::status type and prints the
//type of error to the serial terminal.
//
//Save the return value of any function of type CCS811Core::status, then pass
//to this function to see what the output was.
void printDriverError( CCS811Core::status errorCode )
{
  Serial.print(F("errorCode: "));
  Serial.println(errorCode);
  /*
  switch ( errorCode )
  {
    case CCS811Core::SENSOR_SUCCESS:
      Serial.print(F("SUCCESS"));
      break;
    case CCS811Core::SENSOR_ID_ERROR:
      Serial.print(F("ID_ERROR"));
      break;
    case CCS811Core::SENSOR_I2C_ERROR:
      Serial.print(F("I2C_ERROR"));
      break;
    case CCS811Core::SENSOR_INTERNAL_ERROR:
      Serial.print(F("INTERNAL_ERROR"));
      break;
    case CCS811Core::SENSOR_GENERIC_ERROR:
      Serial.print(F("GENERIC_ERROR"));
      break;
    default:
      Serial.print(F("Unspecified error."));
  }*/
}

//printSensorError gets, clears, then prints the errors
//saved within the error register.
void printSensorError()
{
  uint8_t error = myCCS811.getErrorRegister();

  if ( error == 0xFF ) //comm error
  {
    Serial.println("Failed to get ERROR_ID register.");
  }
  else
  {
    Serial.print(F("Error: "));
    Serial.println(error,HEX); /*
    if (error & 1 << 5) Serial.print(F("HeaterSupply"));
    if (error & 1 << 4) Serial.print(F("HeaterFault"));
    if (error & 1 << 3) Serial.print(F("MaxResistance"));
    if (error & 1 << 2) Serial.print(F("MeasModeInvalid"));
    if (error & 1 << 1) Serial.print(F("ReadRegInvalid"));
    if (error & 1 << 0) Serial.print(F("MsgInvalid"));
    Serial.println(); */
  }
}
// -----------------------------------------------------------------------------
// BME280
// -----------------------------------------------------------------------------

void bmeForceRead() {

    // We set the sensor in "forced mode" to force a reading.
    // After the reading the sensor will go back to sleep mode.

    uint8_t value = myBME280.readRegister(BME280_CTRL_MEAS_REG);
    value = (value & 0xFC) + 0x01;
    myBME280.writeRegister(BME280_CTRL_MEAS_REG, value);

    // Measurement Time (as per BME280 datasheet section 9.1)
    // T_max(ms) = 1.25
    //  + (2.3 * T_oversampling)
    //  + (2.3 * P_oversampling + 0.575)
    //  + (2.4 * H_oversampling + 0.575)
    //  ~ 9.3ms for current settings
    delay(10);

}
//---------------------------------------------------------------
void do_send(osjob_t* j)
{
  float temperature,humidity,pressure,tVOC,CO2;
  int16_t s_value,t_value,h_value,p_value,c_value,v_value;

  CO2=myCCS811.getCO2();
  tVOC=myCCS811.getTVOC();

  baseline=myCCS811.getBaseline();

  bmeForceRead();
  temperature = myBME280.readTempC();
  humidity =    myBME280.readFloatHumidity();
  pressure =    myBME280.readFloatPressure();

    Serial.print("CO2: ");
    //Returns calculated CO2 reading
    Serial.print(CO2,0);
    Serial.print(" ppm, tVOC:");
    //Returns calculated TVOC reading
    Serial.print(tVOC,0);
    Serial.print(" ppb, Humidity: ");
    Serial.print(humidity, 0);
    Serial.print("%, Pressure: ");
    Serial.print(pressure/100, 0);
    Serial.print(" mbar, Temp: ");
    Serial.print(temperature, 2);
    Serial.print(", Baseline: ");
    //Simply the time since program start
    Serial.print(baseline,HEX);
    Serial.println();
  //This sends the temperature data to the CCS811
  myCCS811.setEnvironmentalData(humidity,temperature);

  // compress data to 5 bytes
  
  temperature = constrain(temperature,-24,40);  //temp in range -24 to 40 (64 steps)
  pressure=constrain(pressure/100,970,1034);    //pressure in range 970 to 1034 (64 steps)
  humidity=constrain(humidity,5,100); // humidity in range 5-100% (96 steps, div 3 = 32 steps)
  tVOC=constrain(tVOC,0,1200); // tVoc in range The Total Volatile Organic Compound (TVOC) output range for
                                            //CCS811 is from 0ppb to 1187ppb. Values outside this range are clipped.
  CO2=constrain(CO2,400,8191); //The equivalent CO2 (eCO2) output range for CCS811 is from 400ppm to 8192ppm.
                                             // Values outside this range are clipped.
  t_value=int16_t((temperature*(100/6.25)+2400/6.25)); //0.0625 degree steps with offset
                                                      // no negative values
  p_value=int16_t((pressure-970)/1); //1 mbar steps, offset 970.
  s_value=(p_value<<10) + t_value;  // putting the bits in the right place
  buffer[0]=s_value&0xFF; //lower byte
  buffer[1]=s_value>>8;   //higher byte
  h_value=int16_t(((humidity-5)/3)); // 3% steps, offset 5 (5 bits)
  c_value=int16_t((CO2/4)); // 4 ppm steps, no offset 0-2047 (11 bits)
  v_value=int16_t((tVOC/5)); // 5 ppm steps, no offset 0-238 (8 bits)
  s_value=c_value+(h_value<<11);
  buffer[2]=s_value&0xFF; //lower byte
  buffer[3]=s_value>>8;   //higher byte
  buffer[4]=v_value;

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, buffer, 5, 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.

}


void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      // Re-init
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
        if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print(F("Data Received: "));
        Serial.println(LMIC.frame[LMIC.dataBeg],HEX);
      }
      // Schedule next transmission
      next = true;
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_SCAN_FOUND:
      Serial.println(F("EV_SCAN_FOUND"));
      break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void setup()
{
//Start the serial
Serial.begin(115200);
Serial.println(F("*******************************"));
Serial.println(F("*                             *"));
Serial.println(F("*  Inside Air Quality Sensor  *"));
Serial.println(F("*     The Things Network      *"));
Serial.println(F("*                             *"));
Serial.println(F("*******************************"));
Serial.println();
Serial.println( "Compiled: " __DATE__ ", " __TIME__);
Serial.println(F("LMIC settings:"));
#ifdef DISABLE_PING
  Serial.println(F("- ping disabled"));
#else
  Serial.println(F("- ping enabled"));
#endif
#ifdef DISABLE_BEACONS
  Serial.println(F("- beacon disabled"));
#else
  Serial.println(F("- beacon enabled"));
#endif
#ifdef DISABLE_JOIN
  Serial.println(F("- join disabled"));
#else
  Serial.println(F("- join enabled"));
#endif
#ifdef LMIC_FAILURE_TO
  Serial.println(F("- LMIC_FAILURE TO enabled"));
#else
  Serial.println(F("- LMIC_FAILURE_TO disabled"));
#endif
#ifdef LMIC_PRINTF_TO
  Serial.println(F("- LMIC_PRINTF TO enabled"));
#else
  Serial.println(F("- LMIC_PRINTF_TO disabled"));
#endif
#ifdef CAYENNE_LPP
  Serial.println(F("Cayenne Format"));
#endif

// Configure wake up pin as input.
// This will consumes few uA of current.
pinMode(PIN_NOT_INT, INPUT_PULLUP);
pinMode(PIN_NOT_WAKE, OUTPUT);
digitalWrite(PIN_NOT_WAKE, 0); //Start awake

CCS811Core::status returnCode;
//This begins the CCS811 sensor and prints error status of .begin()
returnCode = myCCS811.begin();

//Initialize BME280
//For I2C, enable the following and disable the SPI section
myBME280.settings.commInterface = I2C_MODE;
myBME280.settings.I2CAddress = BME280_ADDR;
myBME280.settings.runMode = 1; //Forced Mode
myBME280.settings.tStandby = 0;
myBME280.settings.filter = 4;
myBME280.settings.tempOverSample = 1;
myBME280.settings.pressOverSample = 1;
myBME280.settings.humidOverSample = 1;

//Calling .begin() causes the settings to be loaded
delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
myBME280.begin();

//This sets the mode to 60 second reads, and prints returned error status.
returnCode = myCCS811.setDriveMode(3); //3: 60 seconds

//Configure and enable the interrupt line,
//then print error status
returnCode = myCCS811.disableInterrupts();

//LMIC init
os_init();
// Reset the MAC state. Session and pending data transfers will be discarded.
LMIC_reset();
// Set static session parameters. Instead of dynamically establishing a session
// by joining the network, precomputed session parameters are be provided.
uint8_t appskey[sizeof(APPSKEY)];
uint8_t nwkskey[sizeof(NWKSKEY)];
memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));

if (DEVADDR==0)
  {
  Serial.println(F("Invalid Node Address, check your TTN device ID"));
  while(1);
  }
LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif

// Disable link check validation
LMIC_setLinkCheckMode(0);

// TTN uses SF9 for its RX2 window.
LMIC.dn2Dr = DR_SF9;

// Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
LMIC_setDrTxpow(DR_SF7,14);
do_send(&sendjob);
}

void loop()
{
extern volatile unsigned long timer0_overflow_count;

if (next == false) {
  os_runloop_once();
  }
  else {
  Serial.flush(); // give the serial print chance to complete
  // Allow wake up pin to trigger interrupt on low.
  attachInterrupt(digitalPinToInterrupt(PIN_NOT_INT), wakeUp, LOW);
  myCCS811.enableInterrupts();
  digitalWrite(PIN_NOT_WAKE, 1); //Start asleep
  // Enter power down state with ADC and BOD module disabled.
  // Wake up when INT pin is low.
  Serial.println(F("Sleeping..."));
  delay(20);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
  // LMIC uses micros() to keep track of the duty cycle, so
  // hack timer0_overflow for a rude adjustment:
  cli();
  timer0_overflow_count+= 8 * 64 * clockCyclesPerMicrosecond();
  sei();
  // Disable external pin interrupt on wake up pin.
  detachInterrupt(digitalPinToInterrupt(PIN_NOT_INT)); 
  Serial.println(F("Waking up"));
  //Wake up the CCS811 logic engine
  digitalWrite(PIN_NOT_WAKE, 0);
  //Need to wait at least 50 us
  delay(1);
  //Interrupt signal caught, so cause the CCS811 to run its algorithm
  myCCS811.readAlgorithmResults(); //Calling this function updates the global tVOC and CO2 variables
  next = false;
  // Start job
  do_send(&sendjob);
  //Now put the CCS811's logic engine to sleep
  digitalWrite(PIN_NOT_WAKE, 1);
  //Need to be asleep for at least 20 us
  delay(1);
  }
}

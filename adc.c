// ----------------------------------------------------------------------------- 
// ADC Driver with oversampling, persisted calibration & linearization
//
// (c) 2012, 2014 karl@pitrich.com
// inspired by AVR120, AVR121
// ----------------------------------------------------------------------------- 

#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "adc.h"

// ----------------------------------------------------------------------------- 

#define adcOversampleCount        128
#define adcOversampleShift          4
#define adcCalibrationMarkHi    30000UL  // 1/10mV, upper calibration point
#define adcCalibrationMarkLo     3000UL  // 1/10mV, lower calibration point

#define adcScale                   16UL  // 2^4 fixed point scaling (shift by this value)
#define adcRoundingCorrection   32768UL  // add 0.5, which is: (2 ^ adcScale / 2)

#define adcRangeScaled  ((adcCalibrationMarkHi - adcCalibrationMarkLo) << adcScale)
#define adcOffsetScaled (adcCalibrationMarkLo << adcScale)

#define adcFactor(ch)       (adcRangeScaled / (adcCalibration[ch].hi - adcCalibration[ch].lo))
#define adcCorrection(ch)   (adcOffsetScaled - (adcCalibration[ch].lo * adcFactor(ch)))

// ----------------------------------------------------------------------------- 
// default values (Data Precision 8200 +-10ppm at ATMega Vcc = 5.0V)
// NB: these will vary per ATMega device
//
AdcCalibration_t eeAdcCalibration[adcChannels] EEMEM = {
  {
    4908UL, // 3.000V calibration point
     459UL  // 0.300V calibration point
  },
  {
    4908UL, // 3.000V calibration point
     459UL  // 0.300V calibration point
  }
};

AdcCalibration_t adcCalibration[adcChannels];

// ----------------------------------------------------------------------------- 

uint16_t adcAVcc;
uint8_t adcChannel;
volatile uint32_t adcOversampledValue[adcChannels];

// ----------------------------------------------------------------------------- 
//
ISR(ADC_vect) 
{
  static uint32_t adcOversampleSum[adcChannels];
  static uint8_t adcOversampleCounter[adcChannels];

  adcOversampleSum[adcChannel] += ADC;

  if (++adcOversampleCounter[adcChannel] > adcOversampleCount - 1) {
    adcOversampledValue[adcChannel] = adcOversampleSum[adcChannel] >> adcOversampleShift;
    adcOversampleSum[adcChannel] = 0;
    adcOversampleCounter[adcChannel] = 0;
  }

  // next channel
  adcChannel = (adcChannel + 1) % adcChannels;
  adcSelectChannel(adcChannel); 

  // no dummy read after switch results is the necessary noise for oversampling
}

// ----------------------------------------------------------------------------- 
// if we were to wait long engough after power on
// for the voltage to stabilize we can measure 
// the internal 1.1V reference against Vcc == Vref
// and calculate a compensation factor 'adcAVcc'.
//
void adcCalculateReferenceCompensationFactor()
{
  cli();

  adcEnableExternalVref();
  adcSelectChannel(0xe);
  adcEnable();

  _delay_ms(500);

  // dump intial dummy conversion
  adcStartConversion();
  adcWaitForConversion();
  adcAVcc = ADC; // ADC must be read, says datasheet
  adcAVcc = 0;   // TODO: check if the compiler swallows something here

  int i = 0;
  for (; i < 5; i++) {
    adcStartConversion();
    adcWaitForConversion();
    adcAVcc += ADC;
  }

  sei();

  adcAVcc = 1126400L / (adcAVcc / 5); // mV compared to Vref
}

// ----------------------------------------------------------------------------- 

void adcInit(void)
{
  adcChannel = 0;

  adcLoadCalibrationData();
  adcEnableExternalVref();
  adcSelectChannel(adcChannel);  
  
  // ADCSRA |= (1<<ADPS2) | (1<<ADPS1); // prescale clk/64 = 125kHz@8Mhz
  ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // prescale clk/128 = 62.5kHz@8Mhz

  DIDR0  |= (1<<ADC1D) | (1<<ADC0D); // disable digital inputs on ADC pins

  adcEnable();
  adcEnableInt();
  ADCSRA |= (1<<ADATE); // auto trigger enable
  ADCSRB = 0x00;        // free running mode
  adcStartConversion();
}

// ----------------------------------------------------------------------------- 

uint32_t adcValue(uint8_t channel, AdcValueType_t valuetype)
{
  cli();
  uint32_t v = adcOversampledValue[channel];
  sei();

  if (valuetype == AdcReadLinearized) {
    v = (v * adcFactor(channel) + adcCorrection(channel) + adcRoundingCorrection) >> adcScale;
  }

  return v;
}

// ----------------------------------------------------------------------------- 
// result in m°C: 21500 / 1000 = 21.5°C
//
uint16_t adcInternalTemperature() {
  uint16_t result = 0;

  cli();
  adcEnableInternalVref();
  adcSelectChannel(8); // thermometer
  _delay_ms(5);

  // dummy conversion
  adcStartConversion();
  adcWaitForConversion();
  result = ADC;

  adcStartConversion();
  adcWaitForConversion();
  result = ADC;

  sei();
  return (result - 125) * 1075;
}

// ----------------------------------------------------------------------------- 

void adcLoadCalibrationData(void)
{
  do {} while (!(eeprom_is_ready()));
  eeprom_read_block(&adcCalibration, &eeAdcCalibration, sizeof(adcCalibration));
}

// ----------------------------------------------------------------------------- 

void adcSaveCalibrationData(void)
{
  do {} while (!(eeprom_is_ready()));
  eeprom_write_block(&adcCalibration, &eeAdcCalibration, sizeof(adcCalibration));
}

// -----------------------------------------------------------------------------

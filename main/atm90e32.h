/* Dual use Weather and Splot Phase Energy Meter

Intended to be used with CircuitSetup.us Split Phase Energy Meter and some extra tacked on sensors.

The MIT License (MIT)
Copyright (c) 2019 npotts
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.  
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


Portions Modified from https://github.com/CircuitSetup/Split-Single-Phase-Energy-Meter
*/


//INFO: https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/spi_master.html

/*
    Generally this is connected to VSPI line - that is
    const int CS_pin = 04; 
    D0/16 - CS
    D8/15 - CS  // wemos d1
    D5/14 - CLK
    D6/12 - MISO
    D7/13 - MOSI
*/

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

/* STATUS REGISTERS */
#define ATM90E32_MeterEn 0x00 		// Metering Enable
#define ATM90E32_ChannelMapI 0x01 	// Current Channel Mapping Configuration
#define ATM90E32_ChannelMapU 0x02 	// Voltage Channel Mapping Configuration
#define ATM90E32_SagPeakDetCfg 0x05 	// Sag and Peak Detector Period Configuration
#define ATM90E32_OVth 0x06 			// Over Voltage Threshold
#define ATM90E32_ZXConfig 0x07 		// Zero-Crossing Config
#define ATM90E32_SagTh 0x08 			// Voltage Sag Th
#define ATM90E32_PhaseLossTh 0x09 	// Voltage Phase Losing Th
#define ATM90E32_INWarnTh 0x0A 		// Neutral Current (Calculated) Warning Threshold
#define ATM90E32_OIth 0x0B 			// Over Current Threshold
#define ATM90E32_FreqLoTh 0x0C		// Low Threshold for Frequency Detection
#define ATM90E32_FreqHiTh 0x0D		// High Threshold for Frequency Detection
#define ATM90E32_PMPwrCtrl 0x0E		// Partial Measurement Mode Power Control
#define ATM90E32_IRQ0MergeCfg 0x0F 	// IRQ0 Merge Configuration

/* EMM STATUS REGISTERS */
#define ATM90E32_SoftReset 0x70		// Software Reset
#define ATM90E32_EMMState0 0x71		// EMM State 0
#define ATM90E32_EMMState1 0x72		// EMM State 1
#define ATM90E32_EMMIntState0 0x73   // EMM Interrupt Status 0
#define ATM90E32_EMMIntState1 0x74   // EMM Interrupt Status 1
#define ATM90E32_EMMIntEn0 0x75		// EMM Interrupt Enable 0
#define ATM90E32_EMMIntEn1 0x76		// EMM Interrupt Enable 1
#define ATM90E32_LastSPIData 0x78	// Last Read/Write SPI Value
#define ATM90E32_CRCErrStatus 0x79	// CRC Error Status
#define ATM90E32_CRCDigest 0x7A		// CRC Digest
#define ATM90E32_CfgRegAccEn 0x7F	// Configure Register Access Enable

/* LOW POWER MODE REGISTERS - NOT USED */
#define ATM90E32_DetectCtrl 0x10
#define ATM90E32_DetectTh1 0x11
#define ATM90E32_DetectTh2 0x12
#define ATM90E32_DetectTh3 0x13
#define ATM90E32_PMOffsetA 0x14
#define ATM90E32_PMOffsetB 0x15
#define ATM90E32_PMOffsetC 0x16
#define ATM90E32_PMPGA 0x17
#define ATM90E32_PMIrmsA 0x18
#define ATM90E32_PMIrmsB 0x19
#define ATM90E32_PMIrmsC 0x1A
#define ATM90E32_PMConfig 0x10B
#define ATM90E32_PMAvgSamples 0x1C
#define ATM90E32_PMIrmsLSB 0x1D

/* CONFIGURATION REGISTERS */
#define ATM90E32_PLconstH 0x31 		// High Word of PL_Constant
#define ATM90E32_PLconstL 0x32 		// Low Word of PL_Constant
#define ATM90E32_MMode0 0x33 		// Metering Mode Config
#define ATM90E32_MMode1 0x34 		// PGA Gain Configuration for Current Channels
#define ATM90E32_PStartTh 0x35 		// Startup Power Th (P)
#define ATM90E32_QStartTh 0x36 		// Startup Power Th (Q)
#define ATM90E32_SStartTh 0x37		// Startup Power Th (S)
#define ATM90E32_PPhaseTh 0x38 		// Startup Power Accum Th (P)
#define ATM90E32_QPhaseTh 0x39		// Startup Power Accum Th (Q)
#define ATM90E32_SPhaseTh 0x3A		// Startup Power Accum Th (S)

/* CALIBRATION REGISTERS */
#define ATM90E32_PoffsetA 0x41 		// A Line Power Offset (P)
#define ATM90E32_QoffsetA 0x42 		// A Line Power Offset (Q)
#define ATM90E32_PoffsetB 0x43 		// B Line Power Offset (P)
#define ATM90E32_QoffsetB 0x44 		// B Line Power Offset (Q)
#define ATM90E32_PoffsetC 0x45 		// C Line Power Offset (P)
#define ATM90E32_QoffsetC 0x46 		// C Line Power Offset (Q)
#define ATM90E32_PQGainA 0x47 		// A Line Calibration Gain
#define ATM90E32_PhiA 0x48  		// A Line Calibration Angle
#define ATM90E32_PQGainB 0x49 		// B Line Calibration Gain
#define ATM90E32_PhiB 0x4A  		// B Line Calibration Angle
#define ATM90E32_PQGainC 0x4B 		// C Line Calibration Gain
#define ATM90E32_PhiC 0x4C  		// C Line Calibration Angle

/* FUNDAMENTAL/HARMONIC ENERGY CALIBRATION REGISTERS */
#define ATM90E32_POffsetAF 0x51		// A Fund Power Offset (P)
#define ATM90E32_POffsetBF 0x52		// B Fund Power Offset (P)
#define ATM90E32_POffsetCF 0x53		// C Fund Power Offset (P)
#define ATM90E32_PGainAF	0x54	// A Fund Power Gain (P)
#define ATM90E32_PGainBF	0x55	// B Fund Power Gain (P)
#define ATM90E32_PGainCF	0x56	// C Fund Power Gain (P)

/* MEASUREMENT CALIBRATION REGISTERS */
#define ATM90E32_UgainA 0x61 		// A Voltage RMS Gain
#define ATM90E32_IgainA 0x62 		// A Current RMS Gain
#define ATM90E32_UoffsetA 0x63 		// A Voltage Offset
#define ATM90E32_IoffsetA 0x64 		// A Current Offset
#define ATM90E32_UgainB 0x65 		// B Voltage RMS Gain
#define ATM90E32_IgainB 0x66 		// B Current RMS Gain
#define ATM90E32_UoffsetB 0x67 		// B Voltage Offset
#define ATM90E32_IoffsetB 0x68 		// B Current Offset
#define ATM90E32_UgainC 0x69 		// C Voltage RMS Gain
#define ATM90E32_IgainC 0x6A 		// C Current RMS Gain
#define ATM90E32_UoffsetC 0x6B 		// C Voltage Offset
#define ATM90E32_IoffsetC 0x6C 		// C Current Offset
#define ATM90E32_IoffsetN 0x6E 		// N Current Offset

/* ENERGY REGISTERS */
#define ATM90E32_APenergyT 0x80 		// Total Forward Active	
#define ATM90E32_APenergyA 0x81 		// A Forward Active
#define ATM90E32_APenergyB 0x82 		// B Forward Active
#define ATM90E32_APenergyC 0x83 		// C Forward Active
#define ATM90E32_ANenergyT 0x84 		// Total Reverse Active	
#define ATM90E32_ANenergyA 0x85 		// A Reverse Active
#define ATM90E32_ANenergyB 0x86 		// B Reverse Active
#define ATM90E32_ANenergyC 0x87 		// C Reverse Active
#define ATM90E32_RPenergyT 0x88 		// Total Forward Reactive
#define ATM90E32_RPenergyA 0x89 		// A Forward Reactive
#define ATM90E32_RPenergyB 0x8A 		// B Forward Reactive
#define ATM90E32_RPenergyC 0x8B 		// C Forward Reactive
#define ATM90E32_RNenergyT 0x8C 		// Total Reverse Reactive
#define ATM90E32_RNenergyA 0x8D 		// A Reverse Reactive
#define ATM90E32_RNenergyB 0x8E 		// B Reverse Reactive
#define ATM90E32_RNenergyC 0x8F 		// C Reverse Reactive

#define ATM90E32_SAenergyT 0x90 		// Total Apparent Energy
#define ATM90E32_SenergyA 0x91  		// A Apparent Energy		
#define ATM90E32_SenergyB 0x92 		    // B Apparent Energy
#define ATM90E32_SenergyC 0x93 		    // C Apparent Energy


/* FUNDAMENTAL / HARMONIC ENERGY REGISTERS */
#define ATM90E32_APenergyTF 0xA0 	// Total Forward Fund. Energy
#define ATM90E32_APenergyAF 0xA1 	// A Forward Fund. Energy
#define ATM90E32_APenergyBF 0xA2 	// B Forward Fund. Energy
#define ATM90E32_APenergyCF 0xA3 	// C Forward Fund. Energy
#define ATM90E32_ANenergyTF 0xA4  	// Total Reverse Fund Energy
#define ATM90E32_ANenergyAF 0xA5 	// A Reverse Fund. Energy
#define ATM90E32_ANenergyBF 0xA6 	// B Reverse Fund. Energy 
#define ATM90E32_ANenergyCF 0xA7 	// C Reverse Fund. Energy
#define ATM90E32_APenergyTH 0xA8 	// Total Forward Harm. Energy
#define ATM90E32_APenergyAH 0xA9 	// A Forward Harm. Energy
#define ATM90E32_APenergyBH 0xAA 	// B Forward Harm. Energy
#define ATM90E32_APenergyCH 0xAB 	// C Forward Harm. Energy
#define ATM90E32_ANenergyTH 0xAC 	// Total Reverse Harm. Energy
#define ATM90E32_ANenergyAH 0xAD  	// A Reverse Harm. Energy
#define ATM90E32_ANenergyBH 0xAE  	// B Reverse Harm. Energy
#define ATM90E32_ANenergyCH 0xAF  	// C Reverse Harm. Energy

/* POWER & P.F. REGISTERS */
#define ATM90E32_PmeanA 0xB1 		// A Mean Power (P)
#define ATM90E32_PmeanT 0xB0 		// Total Mean Power (P)
#define ATM90E32_PmeanB 0xB2 		// B Mean Power (P)
#define ATM90E32_PmeanC 0xB3 		// C Mean Power (P)
#define ATM90E32_QmeanT 0xB4 		// Total Mean Power (Q)
#define ATM90E32_QmeanA 0xB5 		// A Mean Power (Q)
#define ATM90E32_QmeanB 0xB6 		// B Mean Power (Q)
#define ATM90E32_QmeanC 0xB7 		// C Mean Power (Q)
#define ATM90E32_SmeanT 0xB8 		// Total Mean Power (S)
#define ATM90E32_SmeanA 0xB9 		// A Mean Power (S)
#define ATM90E32_SmeanB 0xBA 		// B Mean Power (S)
#define ATM90E32_SmeanC 0xBB 		// C Mean Power (S)
#define ATM90E32_PFmeanT 0xBC 		// Mean Power Factor
#define ATM90E32_PFmeanA 0xBD 		// A Power Factor
#define ATM90E32_PFmeanB 0xBE 		// B Power Factor
#define ATM90E32_PFmeanC 0xBF 		// C Power Factor

#define ATM90E32_PmeanTLSB 0xC0 		// Lower Word (Tot. Act. Power)
#define ATM90E32_PmeanALSB 0xC1 		// Lower Word (A Act. Power)
#define ATM90E32_PmeanBLSB 0xC2 		// Lower Word (B Act. Power)
#define ATM90E32_PmeanCLSB 0xC3 		// Lower Word (C Act. Power)
#define ATM90E32_QmeanTLSB 0xC4 		// Lower Word (Tot. React. Power)
#define ATM90E32_QmeanALSB 0xC5 		// Lower Word (A React. Power)
#define ATM90E32_QmeanBLSB 0xC6 		// Lower Word (B React. Power)
#define ATM90E32_QmeanCLSB 0xC7 		// Lower Word (C React. Power)
#define ATM90E32_SAmeanTLSB 0xC8 	    // Lower Word (Tot. App. Power)
#define ATM90E32_SmeanALSB 0xC9 		// Lower Word (A App. Power)
#define ATM90E32_SmeanBLSB 0xCA 		// Lower Word (B App. Power)
#define ATM90E32_SmeanCLSB 0xCB 		// Lower Word (C App. Power)

/* FUND/HARM POWER & V/I RMS REGISTERS */
#define ATM90E32_PmeanTF 0xD0 		// Total Active Fund. Power
#define ATM90E32_PmeanAF 0xD1 		// A Active Fund. Power
#define ATM90E32_PmeanBF 0xD2 		// B Active Fund. Power
#define ATM90E32_PmeanCF 0xD3 		// C Active Fund. Power
#define ATM90E32_PmeanTH 0xD4 		// Total Active Harm. Power
#define ATM90E32_PmeanAH 0xD5 		// A Active Harm. Power
#define ATM90E32_PmeanBH 0xD6 		// B Active Harm. Power
#define ATM90E32_PmeanCH 0xD7 		// C Active Harm. Power
#define ATM90E32_UrmsA 0xD9 		// A RMS Voltage
#define ATM90E32_UrmsB 0xDA 		// B RMS Voltage
#define ATM90E32_UrmsC 0xDB 		// C RMS Voltage
#define ATM90E32_IrmsN 0xDC 		// Calculated N RMS Current
#define ATM90E32_IrmsA 0xDD 		// A RMS Current
#define ATM90E32_IrmsB 0xDE 		// B RMS Current
#define ATM90E32_IrmsC 0xDF 		// C RMS Current

#define ATM90E32_PmeanTFLSB 0xE0	// Lower Word (Tot. Act. Fund. Power)
#define ATM90E32_PmeanAFLSB 0xE1	// Lower Word (A Act. Fund. Power) 
#define ATM90E32_PmeanBFLSB 0xE2	// Lower Word (B Act. Fund. Power)
#define ATM90E32_PmeanCFLSB 0xE3	// Lower Word (C Act. Fund. Power)
#define ATM90E32_PmeanTHLSB 0xE4	// Lower Word (Tot. Act. Harm. Power)
#define ATM90E32_PmeanAHLSB 0xE5	// Lower Word (A Act. Harm. Power)
#define ATM90E32_PmeanBHLSB 0xE6	// Lower Word (B Act. Harm. Power)
#define ATM90E32_PmeanCHLSB 0xE7	// Lower Word (C Act. Harm. Power)
///////////////// 0xE8	    // Reserved Register 
#define ATM90E32_UrmsALSB 0xE9		// Lower Word (A RMS Voltage)
#define ATM90E32_UrmsBLSB 0xEA		// Lower Word (B RMS Voltage)
#define ATM90E32_UrmsCLSB 0xEB		// Lower Word (C RMS Voltage)
///////////////// 0xEC	    // Reserved Register	
#define ATM90E32_IrmsALSB 0xED		// Lower Word (A RMS Current)
#define ATM90E32_IrmsBLSB 0xEE		// Lower Word (B RMS Current)
#define ATM90E32_IrmsCLSB 0xEF		// Lower Word (C RMS Current)

/* PEAK, FREQUENCY, ANGLE & TEMP REGISTERS*/
#define ATM90E32_UPeakA 0xF1 		// A Voltage Peak - THD+N on ATM90E36
#define ATM90E32_UPeakB 0xF2 		// B Voltage Peak
#define ATM90E32_UPeakC 0xF3 		// C Voltage Peak
///////////////// 0xF4	    // Reserved Register	
#define ATM90E32_IPeakA 0xF5 		// A Current Peak
#define ATM90E32_IPeakB 0xF6 		// B Current Peak
#define ATM90E32_IPeakC 0xF7 		// C Current Peak
#define ATM90E32_Freq 0xF8 			// Frequency
#define ATM90E32_PAngleA 0xF9 		// A Mean Phase Angle
#define ATM90E32_PAngleB 0xFA 		// B Mean Phase Angle
#define ATM90E32_PAngleC 0xFB 		// C Mean Phase Angle
#define ATM90E32_Temp 0xFC			// Measured Temperature
#define ATM90E32_UangleA 0xFD		// A Voltage Phase Angle
#define ATM90E32_UangleB 0xFE		// B Voltage Phase Angle
#define ATM90E32_UangleC 0xFF		// C Voltage Phase Angle

void atm_init(void);
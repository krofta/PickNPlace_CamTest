/*
 * ov7670Reg.h
 *
 *  Created on: 2017/08/25
 *      Author: take-iwiw
 */

#ifndef OV7670_OV7670REG_H_
#define OV7670_OV7670REG_H_

#define REG_BATT 0xFF

#define GAIN		0x00	// gain control register
#define GAIN_BLUE	0x01	// gain for blue color
#define GAIN_RED	0x02	// gain for red color
#define VREF		0x03	// vertical frame control
#define COM1		0x04	// Common control 1
#define BAVE		0x05	// U/B average level
#define GBAVE		0x06	// Y/Gb average level
#define AECHH		0x07	// Exposure Value
#define RAVE		0x08	// V/R average level
#define COM2		0x09	// common control 2
#define PID			0x0A	// Product ID number msb READ ONLY
#define VER			0x0B	// product ID numver lsb READ ONLY
#define COM3		0x0C	// common control 3 - scale dwc tristate
#define COM4		0x0D	// common control 4 - windowing
#define COM5		0x0E	// common control 5	- reserved
#define COM6		0x0F	// common control 6
#define AECH		0x10	// exposure value
#define CLKRC		0x11	// internal clock
#define COM7		0x12	// common control 7 -  image format vga qvga cif rgb
#define COM8		0x13	// common control 8 - AGC/AEC
#define COM9		0x14	// common control 9 - automatic gain ceiling
#define COM10		0x15	// common control 10 - vsync hsync pxclk
//#define RSVD				// reserved
#define HSTART		0x17	// output format - horizontal frame	start
#define HSTOP		0x18	// output format - horizontal frame end
#define VSTART		0x19	// output format - vertical frame start
#define VSTOP		0x1A	// output format - vertical frame end
#define PSHIFT		0x1B	// pixel delay select (delay timing d0-d7 relative to href)
#define MIDH		0x1C	// manufacturer id high byte READ ONLY
#define MIDL		0x1D	// manufacturere id low byte READ ONLY
#define MVFP		0x1E	// mirror/flip
#define LAEC		0x1F	// reserved
#define ADCCTR0		0x20	// ADC control 0
#define ADCCTR1		0x21	// ADC control 1
#define ADCCTR2		0x22	// ADC control 2
#define ADCCTR3		0x23	// ADC control 3
#define AEW			0x24	// AGC/AEC stable operation region upper limit
#define AEB			0x25	// AGC/AEC	stable operation region lower limit
#define VPT			0x26	// AGC/AEC fast mode operation region
#define BBIAS		0x27
#define GBBIAS		0x28
#define RSVD		0x29
#define EXHCH		0x2A
#define EXHCL		0x2B
#define RBIAS		0x2C
#define ADVFL		0x2D
#define ADVFH		0x2E
#define YAVE		0x2F
#define HSYST		0x30
#define HYSEN		0x31
#define HREF		0x32
#define CHLF		0x33
#define ARBLM		0x34
//#define RSVD		0x35	// reserved
#define OVADC		0x37	// adc control
#define ACOM		0x38	// adc and analog common control
#define AFON		0x39	// adc offset control
#define TSLB		0x3A	// line buffer test option
#define COM11		0x3B	// common control 11 - night mode/frame rate night mode
#define COM12		0x3C	// common control 12 - HREF option
#define COM13		0x3D	// common control 13 - gamma enable
#define COM14		0x3E	// common control 14 - pclk devider / manual scaling enable
#define EDGE		0x3F	// edge enhancement adjustment
#define COM15		0x40	// common control 15 - data format rgb565/rgb555
#define COM16		0x41	// common control 16 - Enable edge enhancement auto-adjustment for YUV
#define COM17		0x42	// common control 17 -  AEC window must be the same value as
#define AWB1		0x43	// AWBx control
#define AWB2		0x44	// AWBx control
#define AWB3		0x45	// AWBx control
#define AWB4		0x46	// AWBx control
#define AWB5		0x47	// AWBx control
#define AWB6		0x48	// AWBx control
#define REG4B		0x4B	// register 4b
#define DNSTH		0x4C	// de noise strength
#define DM_POS		0x4D	// dummy row position
//#define RSVD		0x4E
#define MTX1		0x4F	// matrix coefficient 1
#define MTX2		0x50	// matrix coefficient 2
#define MTX3		0x51	// matrix coefficient 3
#define MTX4		0x52	// matrix coefficient 4
#define MTX5		0x53	// matrix coefficient 5
#define MTX6		0x54	// matrix coefficient 6
#define BRIGHT		0x55	// brightness control
#define CONT		0x56	// contrast control
#define CONT_C		0x57	// contrast center
#define MTXS		0x58	// matric coefficient sign 0 to 5
#define AWBC7		0x59	// AWB control 7
#define AWBC8		0x5A	// AWB control 8
#define AWBC9		0x5B	// AWB control 9
#define AWBC10		0x5C	// AWB control 10
#define AWBC11		0x5D	// AWB control 11
#define AWBC12		0x5E	// AWB control 12
#define B_LMT		0x5F	// AWB blue gain range
#define R_LMT		0x60	// AWB red gain range
#define G_LMT		0x61	// AWB green gain range
#define LCC1		0x62	// lenc correction option 1
#define LCC2		0x63	// lenc correction option 2
#define LCC3		0x64	// lenc correction option 3
#define LCC4		0x65	// lenc correction option 4
#define LCC5		0x66	// lenc correction option 5
#define MANU		0x67	// manual u value
#define manv		0x68	// manual v value
#define GFIX		0x69	// fix gain control
#define GGAIN		0x6A	// gamma channel AWB gain
#define DBLV		0x6B	// pll control
#define AWBCTR3		0x6C	// AWB Contrl 3
#define AWBCTR2		0x6D	// AWB Contrl 2
#define AWBCTR1		0x6E	// AWB Contrl 1
#define AWBCTR0		0x6F	// AWB Contrl 0
#define SCAL_XSC	0x70	// test pattern
#define SCAL_YSC	0x71	// test pattern
#define SCAL_DCW	0x72	// vertival average calculation option
#define SCAL_PCLK	0x73	// clock devider fpr dsp
//#define REG74		0x74	// reserved
#define SLOP		0x7A	// gamma curve highest segment slop
#define GAM1		0x7B	// gamma 1. segment
#define GAM2		0x7C	// gamma 2. segment
#define GAM3		0x7D	// gamma 3. segment
#define GAM4		0x7E	// gamma 4. segment
#define GAM5		0x7F	// gamma 5. segment
#define GAM6		0x80	// gamma 6. segment
#define GAM7		0x81	// gamma 7. segment
#define GAM8		0x82	// gamma 8. segment
#define GAM9		0x83	// gamma 9. segment
#define GAM10		0x84	// gamma 10. segment
#define GAM11		0x85	// gamma 11. segment
#define GAM12		0x86	// gamma 12. segment
#define GAM13		0x87	// gamma 13. segment
#define GAM14		0x88	// gamma 14. segment
#define GAM15		0x89	// gamma 15. segment
#define DM_LNL		0x92	// dummy row low 8 bits
#define DN_LNH		0x93	// dummy row high 8 bits
#define LCC6		0x94	// lens correction option 6
#define LCC7		0x95	// lens correction option 7
//#define RSVD		0x95	// reserved
#define BD50ST		0x9D	// 50Hz banding filter value
#define BD60ST		0x9E	// 60Hz banding filter value
#define HRL			0x9F	// high reference luminance
#define LRL			0xA0	// low reference luminance
#define DSPC3		0xA1	// DSP control 3
#define SCAL_PCLK_DL	0xA2// pixel clock delay
//#define RSVD		0xA3	// reserved
#define NT_CTRL		0xA4	// Auto frame rate adjustment
#define AECGMAX		0xA5	// maximum banding filter step
#define LPH			0xA6	// Lower Limit of Probability for HRL, after exposure/gain stabilizes
#define UPL			0xA7	// Upper Limit of Probability for LRL, after exposure/gain stabilizes
#define TPL			0xA8	// Probability Threshold for LRL to control AEC/AGC speed
#define TPH			0xA9	// Probability Threshold for HRL to control AEC/AGC speed
#define NALG		0xAA	// AEC algorithm selection
//#define RSVD		0xAB	// reserved
#define STR_OPT		0xAC	// strobe xenen led1 led2
#define STR_R		0xAD	// R Gain for LED Output Frame
#define STR_G		0xAE	// G Gain for LED Output Frame
#define STR_B		0xAF	// B Gain for LED Output Frame
//#define RSVD		0xB0	// reserved
#define ABLC1		0xB1	// ablc enable
//#define RSVD		0xB2	// reserved
#define THL_ST		0xB3	// ABLCTarget
//#define RSVD		0xB4	// reserved
#define THL_DLT		0xB5	// ABLC stable range
//#define RSVD		0xB6-BD	// reserved
#define AD_CHB		0xBE	// Blue Channel Black Level Compensation
#define AD_CHR		0xBF	// Red Channel Black Level Compensation
#define AD_CHGB		0xC0	// Gb Channel Black Level Compensation
#define AD_CHGR		0xC1	// Gr Channel Black Level Compensation
//#define RSVD		0xC1-C8	// reserved
#define SATCTR		0xC9	// Saturation Control


//uint8_t OV7670_bak[0xFF];
#if 1
const uint8_t OV7670_reg[][2] =
{
   //Color mode related
  {COM7, 0x14},   // QVGA, RGB
  {0x8C, 0x00},   // RGB444 Disable
  {COM15, 0x10 + 0xc0},   // RGB565, 00 - FF
  //{0x3A, 0x04 + 8},   // UYVY (why?)
  {0x3A, 0b00000100},   // Y V Y U
  {COM13, 0x80 + 0x00},   // gamma enable, UV auto adjust, UYVY
  {0xB0, 0x84}, // Color mode (Not documented??)

  // clock related
  {COM3, 0x04},  // DCW enable
  {COM14, 0x19},  // manual scaling, pclk/=2
  {SCAL_XSC, 0x3A},  // scaling_xsc
  {SCAL_YSC, 0x35},  // scaling_ysc
  {SCAL_DCW, 0x11}, // down sample by 2
  {SCAL_PCLK, 0xf1}, // DSP clock /= 2

  // funzt
  // windowing (empirically decided...)
  // fps
  {HSTART, 0x16},   // HSTART
  {HSTOP, 0x04},   // HSTOP
  {HREF, 0x80},   // HREF
  {VSTART, 0x03},   // VSTART =  14 ( = 3 * 4 + 2)
  {VSTOP, 0x7b},   // VSTOP  = 494 ( = 123 * 4 + 2)
  {VREF, 0x0a},   // VREF (VSTART_LOW = 2, VSTOP_LOW = 2)

  // To achieve the best image quality, using "maximum" exposure and "minimum" gain for the
  // highest S/N ratio is recommended. When operating in low-light condition,
  {COM8, 0b00100010}, // disable automatic gain control - disable automatic exposure control
  {GAIN, 0x00},	// 0dB GAIN
  {AECHH, 0x3F}, // exposure bits 10-15
  {AECH, 0xFF},  // exposure bits 2-9
  {COM1, 0x03},	 // bit 0,1 - exposure bits 0-1



  // color matrix coefficient
#if 0
  {MTX1, 0xb3},
  {MTX2, 0xb3},
  {MTX3, 0x00},
  {MTX4, 0x3d},
  {MTX5, 0xa7},
  {MTX6, 0xe4},
  {MTXS, 0x9e},
#else
  {MTX1, 0x80},
  {MTX2, 0x80},
  {MTX3, 0x00},
  {MTX4, 0x22},
  {MTX5, 0x5e},
  {MTX6, 0x80},
  {MTXS, 0x9e},
#endif

  //
//  {COM8, 0x84},
//  {COM9, 0x0a},   // AGC Ceiling = 2x
//  {B_LMT, 0x2f},   // AWB B Gain Range (empirically decided)
//                  // without this bright scene becomes yellow (purple). might be because of color matrix
//  {R_LMT, 0x98},   // AWB R Gain Range (empirically decided)
//  {G_LMT, 0x70},   // AWB G Gain Range (empirically decided)
 {COM16, 0x38},   // edge enhancement, de-noise, AWG gain enabled


  // gamma curve
  {GAM1,  4},
  {GAM2,  8},
  {GAM3,  16},
  {GAM4,  32},
  {GAM5,  40},
  {GAM6,  48},
  {GAM7,  56},
  {GAM8,  64},
  {GAM9,  72},
  {GAM10, 80},
  {GAM11, 96},
  {GAM12, 112},
  {GAM13, 144},
  {GAM14, 176},
  {GAM15, 208},
  //{0x7a, ((256 - GAM15)* 40/30)},
  {0x7a, 64},

  // fps
  {DBLV, 0x4a}, //PLL  x4
  {CLKRC, 0x00}, // pre-scalar = 1/1

  // others
  //{MVFP, 0x31}, //mirror flip


  {REG_BATT, REG_BATT},
};
#else
const uint8_t OV7670_reg[][2] = {

  {0x12, 0x14},   // QVGA, RGB
  {0x8C, 0x00},   // RGB444 Disable
  {0x40, 0x10 + 0xc0},   // RGB565, 00 - FF
  //{0x3A, 0x04 + 8},   // UYVY (why?)
  {0x3A, 0b00000100},   // Y V Y U
  {0x3D, 0x80 + 0x00},   // gamma enable, UV auto adjust, UYVY
  {0xB0, 0x84}, // important


  {0x0C, 0x04},  // DCW enable
  {0x3E, 0x19},  // manual scaling, pclk/=2
  {0x70, 0x3A},  // scaling_xsc
  {0x71, 0x35},  // scaling_ysc
  {0x72, 0x11}, // down sample by 2
  {0x73, 0xf1}, // DSP clock /= 2


  {0x17, 0x16},   // HSTART
  {0x18, 0x04},   // HSTOP
  {0x32, 0x80},   // HREF
  {0x19, 0x03},   // VSTART =  14 ( = 3 * 4 + 2)
  {0x1a, 0x7b},   // VSTOP  = 494 ( = 123 * 4 + 2)
  {0x03, 0x0a},   // VREF (VSTART_LOW = 2, VSTOP_LOW = 2)


#if 0
  {0x4f, 0xb3},
  {0x50, 0xb3},
  {0x51, 0x00},
  {0x52, 0x3d},
  {0x53, 0xa7},
  {0x54, 0xe4},
  {0x58, 0x9e},
#else
  {0x4f, 0x80},
  {0x50, 0x80},
  {0x51, 0x00},
  {0x52, 0x22},
  {0x53, 0x5e},
  {0x54, 0x80},
  {0x58, 0x9e},
#endif

//  {0x13, 0x84},
//  {0x14, 0x0a},   // AGC Ceiling = 2x
//  {0x5F, 0x2f},   // AWB B Gain Range (empirically decided)
//                  // without this bright scene becomes yellow (purple). might be because of color matrix
//  {0x60, 0x98},   // AWB R Gain Range (empirically decided)
//  {0x61, 0x70},   // AWB G Gain Range (empirically decided)
  {0x41, 0x38},   // edge enhancement, de-noise, AWG gain enabled



#if 1
  {0x7b, 16},
  {0x7c, 30},
  {0x7d, 53},
  {0x7e, 90},
  {0x7f, 105},
  {0x80, 118},
  {0x81, 130},
  {0x82, 140},
  {0x83, 150},
  {0x84, 160},
  {0x85, 180},
  {0x86, 195},
  {0x87, 215},
  {0x88, 230},
  {0x89, 244},
  {0x7a, 16},
#else

  {0x7b, 4},
  {0x7c, 8},
  {0x7d, 16},
  {0x7e, 32},
  {0x7f, 40},
  {0x80, 48},
  {0x81, 56},
  {0x82, 64},
  {0x83, 72},
  {0x84, 80},
  {0x85, 96},
  {0x86, 112},
  {0x87, 144},
  {0x88, 176},
  {0x89, 208},
  {0x7a, 64},
#endif


  {0x6B, 0x4a}, //PLL  x4
  {0x11, 0x00}, // pre-scalar = 1/1


  //{0x1E, 0x31}, //mirror flip
//  {0x42, 0x08}, // color bar

  {REG_BATT, REG_BATT},
};

#endif



#endif /* OV7670_OV7670REG_H_ */

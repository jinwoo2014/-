#include "MPC5604P_M26V.h"
#include "freemaster.h"
#include "init_base.h"


/****************** Variable ******************/
volatile int i = 0, test=0;
int duty_traction=0, duty_servo=0, R_adc=0,T_adc=0;
unsigned int distance=0,echo=0 ,Pit0cnt=0, Pit1cnt=0;
int testduty=0;
int tra=720;
int ser=0;
char SW[4] = {0, 0,0,0};
char SWold[4] = {0,0,0,0};

/*
char ss[4]={0};
*/
union
{
 vuint8_t R;
 struct 
 { vuint8_t :4;
  vuint8_t sensor1:1;
  vuint8_t sensor2:1;
  vuint8_t sensor3:1;
  vuint8_t sensor4:1;
  
 }s;
}sen;




/****************** Function ******************/
void init_PIN(void);
void init_FlexPWM0(void);
void ISR_FlexPWM0(void);
void init_eTimer(void);
void ISR_ETC0(void);
void init_PIT(void);
void ISR_PIT0(void);
void infrared(void);
void ADCRead_1(void);
void line(void);
void switc();



int main(void)
{
 SIU.PCR[48].R=0x0100;
 SIU.PCR[49].R=0x0100;
 SIU.PCR[50].R=0x0100;
 SIU.PCR[51].R=0x0100;
 initModesAndClock();
 disableWatchdog();
 enableIrq();
 FMSTR_Init();
 init_PIN();
 init_INTC();
 init_Linflex0();
 init_eTimer();
 init_FlexPWM0();
 init_PIT();
	ADC_1.MCR.B.ABORT=1;
	ADC_1.MCR.B.PWDN=0;
	ADC_1.CTR[0].R=0x8208;
	ADC_1.NCMR[0].R=0x00000020;
	ADC_1.CDR[5].R=0x0000;
	ADC_1.MCR.B.ABORT=0;	
 
 INTC_InstallINTCInterruptHandler(ISR_FlexPWM0,183,6);
 INTC_InstallINTCInterruptHandler(ISR_ETC0,157,7);
 INTC_InstallINTCInterruptHandler(ISR_PIT0,59,7);
 
 
 for (;;) 
 {
  FMSTR_Recorder();
  FMSTR_Poll();
  ADCRead_1();
  /*if(R_adc<=250)
		{
			T_adc=700;
			duty_traction=T_adc;
		}
		else if(R_adc<=500)
		{
			T_adc=650;
			duty_traction=T_adc;
		}
		else if(R_adc<=750)
		{
			T_adc=550;
			duty_traction=T_adc;
		}
		else
		{
			T_adc=500;
			duty_traction=T_adc;	
		}
		*/
	//duty_traction=R_adc;
	switc();
	infrared();
	duty_traction=tra;
	
	if(distance<=60)
 	{
 		if(Pit1cnt <= 500000)
 		{
 			duty_servo=960;
			duty_traction=725;
 		}
 		else 
 		{
 		duty_servo=660;
		duty_traction=725;
 		}
 	}
 	else if(distance>60)
 	{
 		line();	
 	}
  	i++;
 }
}

void line(void)
{
	switch(sen.R)
 {
 	
 	
 	case 0b1100 :  //right
 	duty_servo=696;
 	duty_traction=tra;
 	break;
 	
 	case 0b1101 :	//little right
 	duty_servo=729;
 	duty_traction=tra;
 	break;
 	
 	case 0b1110 :  // full right +5
 	duty_servo=660;
 	duty_traction=tra;
 	break;
 	
 	case 0b0011 :	//left
 	duty_servo=908;
 	duty_traction=tra;
 	break;
 	
 	case 0b1011 : // little left
 	duty_servo=865;
 	duty_traction=tra;
 	break;
 	
 	case 0b0111 :	// full left+5
 	duty_servo=960;
 	duty_traction=tra;
 	break;
 
 	case 0b1001 :
 	duty_servo=800;
 	duty_traction=tra;
 	break;
 	
 }
}


void switc()
{
	SWold[0] = SW[0];
	SWold[1] = SW[1];
	SWold[2] = SW[2];
	SWold[3] = SW[3];
	
	SW[0] = SIU.GPDI[48].B.PDI; // put 48 to 51 input in SW0 to SW3
	SW[1] = SIU.GPDI[49].B.PDI;
	SW[2] = SIU.GPDI[50].B.PDI;
	SW[3] = SIU.GPDI[51].B.PDI;
		
	if(!SWold[0] && SW[0]) tra = 716; // 
	if(!SWold[1] && SW[1]) tra = 717;
	if(!SWold[2] && SW[2]) tra = 718;
	if(!SWold[3] && SW[3]) tra = 719;
	

	
}




/**************************** PIN initiation ****************************/
void init_PIN(void)
{
// ================== 적외선센서 ==============
 SIU.PCR[44].R=0x0100;   //sensor1 C[12]
 SIU.PCR[45].R=0x0100;   //sensor2 C[13]
 SIU.PCR[46].R=0x0100;   //sensor3 C[14]
 SIU.PCR[47].R=0x0100;   //sensor4 C[15]
 
// ================== 초음파센서 ============
 SIU.PCR[59].R = 0x0200;  // Trig GPIO[59]
 SIU.PCR[0].R = 0x0500;  // Echo eTimer_0

// ================== PWM Pin ======================
 SIU.PCR[58].R = 0x0600;  // FlexPWM0_A[0]
 SIU.PCR[61].R = 0x0600;  // FlexPWM0_A[1]

 
 SIU.PCR[64].R = 0x2C00;     // E[0] (Potentiometer)
}

/*************************** 적외선센서 ******************************/
void infrared(void)
{
 /*
 ss[0] = SIU.GPDI[44].B.PDI;
 ss[1] = SIU.GPDI[45].B.PDI;
 ss[2] = SIU.GPDI[46].B.PDI;
 ss[3] = SIU.GPDI[47].B.PDI;
*/
 sen.s.sensor1 = SIU.GPDI[44].B.PDI;
 sen.s.sensor2 = SIU.GPDI[45].B.PDI;
 sen.s.sensor3 = SIU.GPDI[46].B.PDI;
 sen.s.sensor4 = SIU.GPDI[47].B.PDI;


}

/***************************** Flex_PWM ***********************************/
#define PWM_period_traction 2500     // freq: 200Hz
#define PWM_period_servo 2500        // freq: 200Hz

void init_FlexPWM0(void)
{
 // ============== General PWM initialization =======
 FLEXPWM_0.OUTEN.R  = 0x0000;
// FLEXPWM_0.MASK.R = 0x0000;
// FLEXPWM_0.SWCOUT.R = 0x00F3;
// FLEXPWM_0.DTSRCSEL.R = 0x0000;
// FLEXPWM_0.FCTRL.R = 0x0000;
// FLEXPWM_0.FFILT.R = 0x0000;
 
 
 // =============== PWM Sub[0] initialization ======= ***** servo ***** 
 // =========================== Set PWM Frequency ===
 FLEXPWM_0.SUB[0].INIT.R = 0xffff - PWM_period_servo +1;
 FLEXPWM_0.SUB[0].VAL[0].R = 0;
 FLEXPWM_0.SUB[0].VAL[1].R = PWM_period_servo;
 FLEXPWM_0.SUB[0].VAL[2].R = 0;
 FLEXPWM_0.SUB[0].VAL[3].R = 0;
 FLEXPWM_0.SUB[0].VAL[4].R = 0;
 FLEXPWM_0.SUB[0].VAL[5].R = 0;
 // ============================== Set PWM Option ===
 FLEXPWM_0.SUB[0].CTRL2.R = 0x0000;
 FLEXPWM_0.SUB[0].CTRL2.B.INDEP = 1;  // Pair Operation : 1-Independent, 0-Pair
 FLEXPWM_0.SUB[0].CTRL.R = 0x0000;
 FLEXPWM_0.SUB[0].CTRL.B.HALF = 1;  // Half Cycle Reload : Enable
 FLEXPWM_0.SUB[0].CTRL.B.FULL = 1;  // Full Cycle Reload : Enable
 FLEXPWM_0.SUB[0].CTRL.B.PRSC = 0b110;   // CLK/64
 FLEXPWM_0.SUB[0].OCTRL.R = 0x0000;
 FLEXPWM_0.SUB[0].OCTRL.B.POLA = 0;  // PWM A Polarity : not Inverted
 FLEXPWM_0.SUB[0].OCTRL.B.POLB = 0;  // PWM B Polarity : not Inverted
 // =========================== Set PWM Interrupt ===
// FLEXPWM_0.SUB[0].TCTRL.R = 0x0004;
 FLEXPWM_0.SUB[0].TCTRL.R = 0x0000;
 // ======================= Set PWM Fault Disable ===
 FLEXPWM_0.SUB[0].DISMAP.R = 0xFF00;
 // ============================ Set PWM Deadtime ===
 FLEXPWM_0.SUB[0].DTCNT0.R = 0x0000;
 FLEXPWM_0.SUB[0].DTCNT1.R = 0x0000;
 
  
 // =============== PWM Sub[1] initialization ======= ***** traction *****
 // =========================== Set PWM Frequency ===
 FLEXPWM_0.SUB[1].INIT.R = 0xffff - PWM_period_traction +1;
 FLEXPWM_0.SUB[1].VAL[0].R = 0;
 FLEXPWM_0.SUB[1].VAL[1].R = PWM_period_traction;
 FLEXPWM_0.SUB[1].VAL[2].R = 0;
 FLEXPWM_0.SUB[1].VAL[3].R = 0;
 FLEXPWM_0.SUB[1].VAL[4].R = 0;
 FLEXPWM_0.SUB[1].VAL[5].R = 0;
 // ============================== Set PWM Option ===
 FLEXPWM_0.SUB[1].CTRL2.R = 0x0000;
 FLEXPWM_0.SUB[1].CTRL2.B.INDEP = 0;  // Pair Operation : 1-Independent, 0-Pair
 FLEXPWM_0.SUB[1].CTRL.R = 0x0000;
 FLEXPWM_0.SUB[1].CTRL.B.HALF = 1;  // Half Cycle Reload : Enable
 FLEXPWM_0.SUB[1].CTRL.B.FULL = 1;  // Full Cycle Reload : Enable
 FLEXPWM_0.SUB[1].CTRL.B.PRSC = 0b110;   // CLK/64
 FLEXPWM_0.SUB[1].OCTRL.R = 0x0000;
 FLEXPWM_0.SUB[1].OCTRL.B.POLA = 0;  // PWM A Polarity : not Inverted
 FLEXPWM_0.SUB[1].OCTRL.B.POLB = 0;  // PWM B Polarity : not Inverted
 // =========================== Set PWM Interrupt ===
// FLEXPWM_0.SUB[1].TCTRL.R = 0x0004;
 FLEXPWM_0.SUB[1].TCTRL.R = 0x0000;
 FLEXPWM_0.SUB[1].INTEN.R = 0x0001;
 // ======================= Set PWM Fault Disable ===
 FLEXPWM_0.SUB[1].DISMAP.R = 0xFF00;
 // ============================ Set PWM Deadtime ===
 FLEXPWM_0.SUB[1].DTCNT0.R = 0x0000;
 FLEXPWM_0.SUB[1].DTCNT1.R = 0x0000;
 
 // ============== General PWM initialization re =======
 FLEXPWM_0.OUTEN.B.PWMA_EN = 0b0011; 
 FLEXPWM_0.OUTEN.B.PWMB_EN = 0b0000;
 FLEXPWM_0.OUTEN.B.PWMX_EN = 0b0000; 
 FLEXPWM_0.MCTRL.B.LDOK |= 0xF;       // Load config values into buffers 
 FLEXPWM_0.MCTRL.B.RUN |=0xF;

}

void ISR_FlexPWM0(void)
{

  FLEXPWM_0.SUB[1].STS.B.CMPF = 0x0001;
  
  
  FLEXPWM_0.SUB[0].VAL[2].R = (unsigned short)-duty_servo;
  FLEXPWM_0.SUB[0].VAL[3].R = (unsigned short) duty_servo;
  FLEXPWM_0.SUB[1].VAL[2].R = (unsigned short)-duty_traction;
  FLEXPWM_0.SUB[1].VAL[3].R = (unsigned short) duty_traction;


  FLEXPWM_0.MCTRL.B.LDOK |= 0xF; // Load config values into buffers 
  FLEXPWM_0.MCTRL.B.RUN |=0xF;

  test++;
}




/************************ 초음파센서 Trig ****************************/
void init_PIT(void)
{
 PIT.PITMCR.R = 0x00000001;             // PIT Enabled and Config stop in debug mode
 PIT.CH[0].LDVAL.R = 0x00000280;     // 10 us
 PIT.CH[0].TCTRL.R = 0x3;               // Timer interrupt enabled & start
}




void ISR_PIT0(void)
{
 PIT.CH[0].TFLG.B.TIF = 1;              // Clear PIT0 Flag
 Pit0cnt++;
 Pit1cnt++;
 
 if(Pit0cnt!=10000)
 {
  SIU.GPDO[59].B.PDO = 0;
 }
 else 
 {
  SIU.GPDO[59].B.PDO = 1;
  Pit0cnt=0;
 }

}


/************************* 초음파센서 Echo ********************************/
void init_eTimer(void)
{

// ============== eTimer_0 initialization =======
    ETIMER_0.ENBL.R = 0b000000;
    
    ETIMER_0.CHANNEL[0].CTRL.B.CNTMODE = 0b001;    // Count rising edges of primary source
    ETIMER_0.CHANNEL[0].CTRL.B.PRISRC  = 0b11111;  // From Table 'Count source values'
    ETIMER_0.CHANNEL[0].CTRL.B.ONCE    = 0;        // Count repeatedly
    ETIMER_0.CHANNEL[0].CTRL.B.LENGTH  = 0;        // Continue counting to roll over
    ETIMER_0.CHANNEL[0].CTRL.B.DIR     = 0;        // Count up
    ETIMER_0.CHANNEL[0].CTRL.B.SECSRC  = 0b00000;  // From Table 'Count source values'
    
    ETIMER_0.CHANNEL[0].CCCTRL.B.CPT2MODE = 0b01;  // Capture falling edges
    ETIMER_0.CHANNEL[0].CCCTRL.B.CPT1MODE = 0b10;  // Capture rising edges
    ETIMER_0.CHANNEL[0].CCCTRL.B.ONESHOT = 0;      // Free-running mode is selected
    ETIMER_0.CHANNEL[0].CCCTRL.B.ARM = 1;          // Input capture operation as specified by the CPT1MODE and CPT2MODE bits is enabled
    
    ETIMER_0.CHANNEL[0].CTRL3.B.ROC   = 0b01;      // Reload the counter on a capture 1 event
     
 ETIMER_0.CHANNEL[0].INTDMA.B.ICF2IE = 1;       // Input Capture 2 flag interrupt Enable
 
    ETIMER_0.ENBL.R = 0b000001;                    // ETC[0] Enabled
    
}

void ISR_ETC0(void)
{

 ETIMER_0.CHANNEL[0].STS.B.ICF2 = 1;
 
 echo = ETIMER_0.CHANNEL[0].CAPT2.R;
 /*
 time_high  = 128/64M sec x echo (2 micro sec per 1 count)
 distance   = 340 x time_high / 2
            = 340 x echo
 */
 distance = 340 * echo / 10000; // centimeter
}

void ADCRead_1(void)
{
	ADC_1.MCR.B.NSTART=1;
	asm("nop");
	while(ADC_1.MCR.B.NSTART)
	{
		asm("nop");
	}
	R_adc=ADC_1.CDR[5].B.CDATA;
	
}

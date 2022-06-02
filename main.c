#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File


void InitEPwm1Example(void);
void Adc_Config(void);
void MakeSure_sampling_frequency(void);
void protection(void);
void ADC_Offset(void);
void ADC_Calculate(void);


__interrupt void ADC_interrupt_service_routine(void);

#define BUFFER_LENGTH 256
float ADC_buffer[BUFFER_LENGTH];
unsigned int index = 0;

#define sample_period_PWM 0.000025
#define V_load_scale = 10.;
#define I_ind_scale = 2.;


/* Global Variables used in this example: */
Uint16 LoopCount;
Uint16 ConversionCount;
Uint16 Voltage1[10];
Uint16 Voltage2[10];
int16 PWMPeriod;
int16 initial_CMPA_1 = 2200;
int16 initial_CMPA_2 = 50;

/*---------------------------------------
            load voltage
---------------------------------------*/
float32 setpoint                = 20.;
float32 result__V               = 0.;
float32 V_sampling_time         = 0.00005;
float32 V_err                   = 0.;
float32 V_Kp                    = 0.058318;
float32 V_Kp_err                = 0.;
float32 V_err_int               = 0.;
float32 V_err_int_old           = 0.;
float32 V_Ki                    = 1.09127;
float32 V_Ki_err                = 0.;

float32 load_voltage            = 0.;

/*--------------------------------------
            inductor current
--------------------------------------*/
float32 result__I               = 0.;
float32 I_sampling_time         = 0.00005;
float32 I_ref                   = 0.;
float32 I_err                   = 0.;
float32 I_Kp                    = 0.025780;
float32 I_Kp_err                = 0.;
float32 I_err_int               = 0.;
float32 I_err_int_old           = 0.;
float32 I_Ki                    = 2.0130;
float32 I_Ki_err                = 0.;

float32 inductor_current        = 0.;

float32 duty_cmd                = 0.;

/*--------------------------------------
              ADC-related
--------------------------------------*/
float32 V_load_Offset = 0.;
float32 V_load_OffsetSum = 0.;
float32 V_load_signal = 0.;
float32 V_load_adc = 0.;

float32 I_ind_Offset = 0.;
float32 I_ind_OffsetSum = 0.;
float32 I_ind_signal = 0.;
float32 I_ind_adc = 0.;

int16 ADC_cnt = 0;

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

__interrupt void ADC_interrupt_service_routine(void) // __interrupt void VS. interrupt ???
{
    GpioDataRegs.GPASET.bit.GPIO2 = 1;
    GpioDataRegs.GPASET.bit.GPIO3 = 1;

    result__V = (float32)(0xffff & AdcResult.ADCRESULT0);
    load_voltage = result__V * 3.3/4096.*10.0;
    V_err = setpoint - load_voltage;
    V_err_int = V_err_int_old + (V_err*0.00005);
    V_err_int_old = V_err_int;
    V_Ki_err = V_Ki*V_err_int;
    V_Kp_err = V_err*V_Kp;
    I_ref = V_Kp_err + V_Ki_err;

//    ADC_buffer[index] = (float)AdcResult.ADCRESULT0;
//    index++;
//    index &= 0x00ff;
//
//    Voltage1[ConversionCount] = (float)AdcResult.ADCRESULT0;
//
//    if(ConversionCount==9)
//    {
//        ConversionCount=0;
//    }
//    else ConversionCount++;

    result__I = (float32)(0xffff & AdcResult.ADCRESULT1);
    inductor_current = result__I * 3.3/4096.*2.0;
    I_err = I_ref - inductor_current;
    I_err_int = I_err_int_old + (I_err*0.00005);

//    if(I_err_int > 2.)
//    {
//        I_err_int = 2.;
//    }
//    else if(I_err_int < -2.)
//    {
//        I_err_int = -2.;
//    }
//    else;

    I_err_int_old = I_err_int;

    I_Ki_err = I_Ki*I_err_int;
    I_Kp_err = I_err*I_Kp;

    duty_cmd = I_Kp_err + I_Ki_err;

    if(duty_cmd > 1.0)
    {
        duty_cmd = 0.90;
    }
    else if(duty_cmd < 0.0)
    {
        duty_cmd = 0.10;
    }
    else;


    EPwm1Regs.CMPA.half.CMPA = (float32)duty_cmd*PWMPeriod;           // 여기서 듀티 생성

    if(EPwm1Regs.CMPA.half.CMPA > EPwm1Regs.TBPRD)
    {
        EPwm1Regs.CMPA.half.CMPA = initial_CMPA_1;
    }
    else if(EPwm1Regs.CMPA.half.CMPA < 0.0)
    {
        EPwm1Regs.CMPA.half.CMPA = initial_CMPA_2;
    }
    else;

    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;       //Clear ADCINT1 flag reinitialize for next SOC
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;     // Acknowledge interrupt to PIE


    GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;

    return;
}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

void main(void)
{
   InitSysCtrl();
   InitEPwm1Gpio();
   DINT;
   InitPieCtrl();
   IER = 0x0000;
   IFR = 0x0000;
   InitPieVectTable();

   //////////////////////////////////////
   //////////////////////////////////////
   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
   EDIS;

   InitEPwm1Example();

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;
   //////////////////////////////////////
   //////////////////////////////////////

   EALLOW;
   PieVectTable.ADCINT1 = &ADC_interrupt_service_routine;
   EDIS;
   InitAdc();                           // power 관련 레지스터 설정, F2806x_Adc.c에 있음

   // Enable ADCINT1 in PIE
   PieCtrlRegs.PIEIER1.bit.INTx1 = 1;   // Enable INT 1.1 in the PIE
   IER |= M_INT1;                       // Enable CPU Interrupt 1
   EINT;                                // Enable Global interrupt INTM
   ERTM;                                // Enable Global realtime interrupt DBGM


   // Configure ADC
   EALLOW;
   AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1;  // Enable non-overlap mode
   AdcRegs.ADCCTL1.bit.INTPULSEPOS   = 1;    // ADCINT1 trips after AdcResults latch
   AdcRegs.INTSEL1N2.bit.INT1E       = 1;    // Enabled ADCINT1
   AdcRegs.INTSEL1N2.bit.INT1CONT    = 0;    // Disable ADCINT1 Continuous mode
   AdcRegs.INTSEL1N2.bit.INT1SEL     = 0;    // setup EOC0 to trigger ADCINT1 to fire

   AdcRegs.ADCSOC0CTL.bit.CHSEL      = 2;    // set SOC0 channel select to ADCINA2(load voltage)
   AdcRegs.ADCSOC1CTL.bit.CHSEL      = 4;    // set SOC1 channel select to ADCINA4(inductor current)

   AdcRegs.ADCSOC0CTL.bit.TRIGSEL    = 5;    // set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
   AdcRegs.ADCSOC1CTL.bit.TRIGSEL    = 5;    // set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1

   AdcRegs.ADCSOC0CTL.bit.ACQPS      = 6;    // set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
   AdcRegs.ADCSOC1CTL.bit.ACQPS      = 6;    // set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)

//   AdcRegs.ADCINTSOCSEL1.bit.SOC0    = 1;
//   AdcRegs.ADCINTSOCSEL2.
   EDIS;

   // Assumes ePWM1 clock is already enabled in InitSysCtrl();
   EPwm1Regs.ETSEL.bit.SOCAEN   = 1;        // Enable SOC on A group
   EPwm1Regs.ETSEL.bit.SOCASEL  = 4;        // Select SOC from CMPA on upcount
   EPwm1Regs.ETPS.bit.SOCAPRD   = 1;        // Generate pulse on 1st event
//    EPwm1Regs.CMPA.half.CMPA     = 0x0080;   // Set compare A value
//    EPwm1Regs.TBPRD              = 0xFFFF;   // Set period for ePWM1
//    EPwm1Regs.TBCTL.bit.CTRMODE  = 0;        // count up and start

   for(;;)
   {
       LoopCount++;
   }
}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

//void ADC_Offset(void)
//{
//    if(ADC_cnt == 1000)
//    {
//        V_load_Offset = (V_load_OffsetSum * 0.001) * V_load_scale;
//        I_ind_Offset = (I_ind_OffsetSum  * 0.001) * I_ind_scale;
//    }
//    else
//    {
//        V_load_OffsetSum += (float32)(0xffff & AdcResult.ADCRESULT0);
//        I_ind_OffsetSum  += (float32)(0xffff & AdcResult.ADCRESULT1);
//    }
//}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

//void ADC_Calculate(void)
//{
//    V_load_signal = (float32)(0xffff & AdcResult.ADCRESULT0);
//    V_load_adc = (V_load_signal * V_load_scale) - V_load_Offset;
//
//    I_ind_signal = (float32)(0xffff & AdcResult.ADCRESULT1);
//    I_ind_adc = (I_ind_signal * I_ind_scale) - I_dc_Offset;
//}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

//void protection(void)
//{
//
//}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

void MakeSure_sampling_frequency(void)
{
    GpioCtrlRegs.GPAMUX1.bit.GPIO2      = 0;    // GPIO2 = GPIO2
    GpioCtrlRegs.GPAMUX1.bit.GPIO3      = 0;    // GPIO3 = GPIO3
    GpioCtrlRegs.GPADIR.bit.GPIO2       = 1;    // GPIO2 output
    GpioCtrlRegs.GPADIR.bit.GPIO3       = 1;    // GPIO3 output
    GpioCtrlRegs.GPAQSEL1.bit.GPIO2     = 0;
    GpioCtrlRegs.GPAQSEL1.bit.GPIO3     = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO2       = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO3       = 0;
}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

void InitEPwm1Example(void)
{
    PWMPeriod = sample_period_PWM * 90000000;

   // Setup TBCLK
   EPwm1Regs.TBPRD = PWMPeriod;           // Set timer period 801 TBCLKs
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                      // Clear counter

   // Set Compare values
   EPwm1Regs.CMPA.half.CMPA = initial_CMPA_1;     // Set compare A value    // 인터럽트 서비스 루틴에서 듀티 생성
//   EPwm1Regs.CMPB = 8000;               // Set Compare B value    // 인터럽트 서비스 루틴에서 듀티 생성

   // Setup counter mode
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up-down
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   // Setup shadowing
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set actions
   EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on event A, up count
   EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM1A on event A, down count

   EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM1B on event B, up count
   EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;           // Clear PWM1B on event B, down count
}

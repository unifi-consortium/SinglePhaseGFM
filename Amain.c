//###########################################################################
//Droop
//! - ePWM1A is on GPIO0 -- Q1
//! - ePWM1B is on GPIO1 -- Q2
//! - ePWM2A is on GPIO2 -- Q3
//! - ePWM2B is on GPIO3 -- Q4
// Included Files
#include "Amain.h"

// Key Flags
Uint16 SYNC_STATUS = 0; // if true(=1), keep in presync mode, else go to power transfer mode
Uint16 FLT_STATUS  = 0; // if false means no fault is there, means FLT_STATUS pin is high, should normally be false
Uint16 READY = 0; // if false means READY pin input is zero, it should normally be true
// RESET: false = normally false; true = reset!
Uint16 RESET = 0; // if false means, I am not reseting it, this should normally be false, but the GPIO logic should be inverted since it is active low
Uint16 RELAY = 0; // if false means, I am not connecting to grid
Uint16 ClearTripFlag = 0;
Uint16 FaultTypeFlag = 3;

// System & Control Params - Init in main func for non-zero values!
float Pref, Qref;
float PreSyncRatioDroop;
float PreSyncRatioVOC;
float PreSyncRatioVSM;
float wP;
float Imax;
float omega;
PIDQ_CONTROLLER pi_idq = PIDQ_CONTROLLER_DEFAULTS;
PIDQ_CONTROLLER pi_vdq = PIDQ_CONTROLLER_DEFAULTS;
//Compensator_PI pi_id;
//Compensator_PI pi_iq;
//Compensator_PI pi_vd;
//Compensator_PI pi_vq;
//MinMaxLimit MinMax_id;
//MinMaxLimit MinMax_iq;
//MinMaxLimit MinMax_vd;
//MinMaxLimit MinMax_vq;
//PI_CONTROLLER   pi_id   = PI_CONTROLLER_DEFAULTS;
//PI_CONTROLLER   pi_iq   = PI_CONTROLLER_DEFAULTS;
//PI_CONTROLLER   pi_vd   = PI_CONTROLLER_DEFAULTS;
//PI_CONTROLLER   pi_vq   = PI_CONTROLLER_DEFAULTS;

#if (GfmType == GfmDroop)
float mP,mQ;
#elif (GfmType == GfmdVOC)
VOC_VARS voc_obj = VOC_DEFAULTS;
#elif (GfmType == GfmVSM)
VSM_VARS VSM_obj = VSM_DEFAULTS;
#endif

// System & Control Variables
float theta = 0.0, theta_grid = 0.0;
float P_samp[2] = {0.0, 0.0}, Q_samp[2] = {0.0, 0.0};
float P_filt[2] = {0.0, 0.0}, Q_filt[2] = {0.0, 0.0};
float dV_int = 0.0, dV_prop = 0.0; //Reactive Power Controller PI
float Vmag = 0.0;
float dV_Sync = 0.0, dw_Sync = 0.0;
float Vdref = 0.0, Vqref = 0.0;
float Idref = 0.0, Iqref = 0.0;
float Irms_TEST = 2.0, Vrms_TEST = 120.0;
float Vm_d = 0.0, Vm_q = 0.0;
float Vmod = 0.0, ModIndex = 0.1;
float BW_cur = 4.5e3, Tau_cur = 100.0;
float BW_vol = 2e3,   Tau_vol = 50.0;
float Kpi,Kii,Kpv,Kiv; // For traditional PI structure

// ADC Variables
float vcap = 0.0, vgrid =0.0 , iinv = 0.0, igrid = 0.0;
float magvc = 0.0, magvg = 0.0, magil = 0.0, magig = 0.0;
ORTHO_SIG Vcap  = OSG_DEFAULTS;
ORTHO_SIG Vgrid = OSG_DEFAULTS;
ORTHO_SIG Iinv  = OSG_DEFAULTS;
ORTHO_SIG Igrid = OSG_DEFAULTS;
float Idc = 0.0, Vdc = 0.0;
// ADC offset & calibration
float K1 = 0.998,          // Offset filter coefficient K1: 0.05/(T+0.05);
      K2 = 0.001999;       // Offset filter coefficient K2: T/(T+0.05);
int OffsetCalCounter = 0;
float offset_Iinv = 0.0;
float offset_Vcap = 0.0;
float offset_Igrid = 0.0;
float offset_Vgrid = 0.0;
float offset_Idc = 0.0;
float curr_sens_adj_factor = 1.0;
float volt_sens_adj_factor = 1.0;// 0.9833; //1.15;
float AdcTemp[100]={0.0};
float SumTemp = 0.0;

//////////////////////////////////////////////////////
// for sampling frequency, look up this in epwm.c file:
// EPwm1Regs.ETPS.bit.SOCPSSEL = 1;
// EPwm1Regs.ETSOCPS.bit.SOCAPRD2 = 0x5; ~ this means that we will sample once every 1/20kHz = 50 us

#define presync_gain 1.1 // Kp = wc/Vg = 2*pi*30/170- PLL design

//DAC - Comment next 8 lines out if not using DAC
//#define REFERENCE_VDAC      1
//#define REFERENCE_VREF      1
//#define DACA         1
//#define DACB         2
//#define REFERENCE            REFERENCE_VDAC
//#define DAC_NUM                DACB
//void configureDAC(Uint16 dac_num);
//volatile struct DAC_REGS* DAC_PTR[4] = {0x0,&DacaRegs,&DacbRegs,&DaccRegs};

//#define BUFF_SIZE                160

//DLOG CODES FROM RAHUL
//float DBUFF_4CH1[200],
//      DBUFF_4CH2[200],
//      DlogCh1,
//      DlogCh2;
// Create an instance of DATALOG Module
//DLOG_4CH_F dlog_4ch1;

//__interrupt void cpu_timer0_isr(void);

//
// Main
//
void main(void)
{
// Initialize System Control:
// PLL, WatchDog, Enable Peripheral Clocks
    InitSysCtrl();

// Initialize GPIO:
// Set the GPIO to its default state.
    InitGpio();

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
    DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags are cleared.
    InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
    InitPieVectTable();

    //
    // Map ISR functions
    //
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
//    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    EDIS;
//    CpuTimer0Regs.PRD.all =  500000000;     // A tasks every 1 sec= 100000000
//    CpuTimer0Regs.TCR.bit.TIE = 1;

// Weiqian: Temp uncomment this
// Disable TBCLK when configuring Device Peripherals:
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

//    Weiqian: comment out , then using default value =1, which means divby2, since recommened value in manual is divby2;
//    EALLOW;
//    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0; // divby1
//    EDIS;


    //20240403 Weiqian: Config. All Gpio Pins
    GpioSetup();
    //20240404 Weiqian: Config. Epwms
    EpwmSetup(INV_PWM_TBPRD, EPWM_MIN_DB);
    // HALF_TBPRD_inv = (SYSTEM_FREQUENCY*1000000/f_inv)/4; // divby2 for triangular carrier, divby2 for half tbprd
    //20240404 Weiqian: Config. All Adcs
    AdcSetup();
    // DAC
    //configureDAC(2); //DAC_NUM
    //configureDAC(1); //DAC_NUM

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    for ( OffsetCalCounter = 0; OffsetCalCounter < 20000; )
    {
        if(AdcaRegs.ADCINTFLG.bit.ADCINT1 == 1) //   AdcaRegs.ADCINTFLG.bit.ADCINT2 == 1)EPwm11Regs.ETFLG.bit.INT == 1
        {
            if(OffsetCalCounter>1000) // Calculate the offset from zero-drifting -- should be around 0.5.
            {
                offset_Iinv   = K1*offset_Iinv  + K2 * IINV  * ADC_PU_SCALE_FACTOR;
                offset_Vcap   = K1*offset_Vcap  + K2 * VCAP  * ADC_PU_SCALE_FACTOR;
                offset_Igrid  = K1*offset_Igrid + K2 * IGRID * ADC_PU_SCALE_FACTOR;
                offset_Vgrid  = K1*offset_Vgrid + K2 * VGRID * ADC_PU_SCALE_FACTOR;
                offset_Idc    = K1*offset_Idc   + K2 * IDC   * ADC_PU_SCALE_FACTOR;
            }
            AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;//AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;//EPwm11Regs.ETCLR.bit.INT=1;
            OffsetCalCounter++;
        }
    }


    //-------------------------------------------------------------------------
    // Variable Initialization
    KeyInit();
    //-------------------------------------------------------------------------
    //
    // enable PIE interrupt
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // PieCtrlRegs.PIEIER3.bit.INTx11 = 1;
    PieCtrlRegs.PIEIER10.bit.INTx2 = 1;

    IER |= M_INT1; //Enable group 1 interrupts // adc A1 and timer0
    // IER |= M_INT3; // enable group 3 interrupt epwm 11
    IER |= M_INT10; // enable group 10 interrrupts adcaint2
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

//
// Step 6. IDLE loop. Just sit and loop forever:
//
    do
    {
        // GATE DRIVER
        FLT_STATUS = (GpioDataRegs.GPCDAT.bit.GPIO92==0?1:0); // TRUE: fault detected; FALSE: no fault is there, this pin at 3.3V;
        READY = GpioDataRegs.GPCDAT.bit.GPIO94; // TRUE: all good; FALSE: MOSFET not ready, pin at 0V;
        //SW1 = SYNC_STATUS
        // Weiqian 20240520 Stop using relay to do grid connection .
        // SYNC_STATUS = SW1; // if 1, then we keep it in the presync state, keep relay open/off
        //SW2 = RELAY
        RELAY = SW2;
        GpioDataRegs.GPBDAT.bit.GPIO35 = RELAY;
        if (RELAY==1)
        {
            SYNC_STATUS = 0;
        }
        //SW5 = ClearAllTrip & RESET, SW6 = Manual Trip
        EALLOW;
        if (SW5 == 1 || ClearTripFlag == 1)
        {
            ClearTripFlag = 0;
            FaultTypeFlag = 0;
            KeyInit();
            // clear CBC flags
            EPwm1Regs.TZCLR.bit.CBC = 1;
            EPwm2Regs.TZCLR.bit.CBC = 1;
            // clear OST flags
            EPwm1Regs.TZCLR.bit.OST = 1;
            EPwm2Regs.TZCLR.bit.OST = 1;

            // clear DCAEVT1 flags
            EPwm1Regs.TZCLR.bit.DCAEVT1 = 1;
            EPwm2Regs.TZCLR.bit.DCAEVT1 = 1;
            EPwm1Regs.TZCLR.bit.DCBEVT1 = 1;
            EPwm2Regs.TZCLR.bit.DCBEVT1 = 1;

            //RESET GATE DRIVER
            if (FLT_STATUS == 1 || READY == 0)
            {
                RESET = 1;
                GpioDataRegs.GPCDAT.bit.GPIO90 = (RESET==0?1:0); //to restart -> RESET==1 -> GPIO pin = 0;
                RESET = 0;
                GpioDataRegs.GPCDAT.bit.GPIO90 = (RESET==0?1:0); //to restart -> RESET==1 -> GPIO pin = 0;
            }
        }
        if (SW6 == 1 || iinv > Imax || FLT_STATUS == 1)
        {
            if (FaultTypeFlag == 0)
            {
                if (iinv>Imax)
                {
                    FaultTypeFlag = 1;
                }
                else if (FLT_STATUS == 1)
                {
                    FaultTypeFlag = 2;
                }
                else if (SW6 == 1)
                {
                    FaultTypeFlag = 3;
                }
            }
            // set OST flags
            EPwm1Regs.TZFRC.bit.OST = 1;
            EPwm2Regs.TZFRC.bit.OST = 1;

//            KeyInit();
        }
        EDIS;
    }while(1);

}

//
// GPIO Setup Function
//
void GpioSetup(void)
{
    // These can be combined into single statements for improved
    // code efficiency.
    //
    // Enable PWM1-2 on GPIO0-GPIO3
    //
    InitEPwm1Gpio();
    InitEPwm2Gpio();

    EALLOW;
    //
    // /RST: Make GPIO90 an output, low active
    //
    GpioCtrlRegs.GPCPUD.bit.GPIO90 = 0;   // Enable pullup
    GpioCtrlRegs.GPCMUX2.bit.GPIO90 = 0;  // Pin as GPIO
    GpioCtrlRegs.GPCDIR.bit.GPIO90 = GPIO_OUTPUT;   // 1 as output
    //
    // /FLT_STATUS: Make GPIO92 an input, low active
    //
    GpioCtrlRegs.GPCPUD.bit.GPIO92 = 0;  // Enable pullup
    GpioCtrlRegs.GPCMUX2.bit.GPIO92 = 0; // Pin as GPIO
    GpioCtrlRegs.GPCDIR.bit.GPIO92 = GPIO_INPUT;  // 0 = input
    //
    // RDY_STATUS: Make GPIO94 an input, high active
    //
    GpioCtrlRegs.GPCPUD.bit.GPIO94 = 1;  // Disable pullup
    GpioCtrlRegs.GPCMUX2.bit.GPIO94 = 0; // Pin as GPIO
    GpioCtrlRegs.GPCDIR.bit.GPIO94 = GPIO_INPUT;  // 0 = input
    //
    // RELAY: Make GPIO35 an output, high active
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO35 = 1;   // Disable pullup
    GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0;  // Pin as GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO35 = GPIO_OUTPUT;   // 1 as output
    //
    // J48-53: Make GPIO36,38,61,63,65,67,69 as outputs, high active
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO36 = 1;   // Disable pullup
    GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 0;  // Pin as GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO36 = GPIO_OUTPUT;   // 1 as output

    GpioCtrlRegs.GPBPUD.bit.GPIO38 = 1;   // Disable pullup
    GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 0;  // Pin as GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO38 = GPIO_OUTPUT;   // 1 as output

    GpioCtrlRegs.GPBPUD.bit.GPIO61 = 1;   // Disable pullup
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;  // Pin as GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO61 = GPIO_OUTPUT;   // 1 as output

    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 1;   // Disable pullup
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0;  // Pin as GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO63 = GPIO_OUTPUT;   // 1 as output

    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 1;   // Disable pullup
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 0;  // Pin as GPIO
    GpioCtrlRegs.GPCDIR.bit.GPIO65 = GPIO_OUTPUT;   // 1 as output

    GpioCtrlRegs.GPCPUD.bit.GPIO67 = 1;   // Disable pullup
    GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;  // Pin as GPIO
    GpioCtrlRegs.GPCDIR.bit.GPIO67 = GPIO_OUTPUT;   // 1 as output

    //
    // SW1-6: Make GPIO91,93,133,121,162,164 as inputs, high active
    //
    //SW1 - syncflag
    GpioCtrlRegs.GPCPUD.bit.GPIO91 = 1;   // Disable pullup
    GpioCtrlRegs.GPCMUX2.bit.GPIO91 = 0;  // Pin as GPIO
    GpioCtrlRegs.GPCDIR.bit.GPIO91 = GPIO_INPUT;   // 0 as input
    //SW2 - RELAY
    GpioCtrlRegs.GPCPUD.bit.GPIO93 = 1;   // Disable pullup
    GpioCtrlRegs.GPCMUX2.bit.GPIO93 = 0;  // Pin as GPIO
    GpioCtrlRegs.GPCDIR.bit.GPIO93 = GPIO_INPUT;   // 0 as input
    //SW3
    GpioCtrlRegs.GPEPUD.bit.GPIO133 = 1;   // Disable pullup
    GpioCtrlRegs.GPEMUX1.bit.GPIO133 = 0;  // Pin as GPIO
    GpioCtrlRegs.GPEDIR.bit.GPIO133 = GPIO_INPUT;   // 0 as input
    //SW4
    GpioCtrlRegs.GPDPUD.bit.GPIO121 = 1;   // Disable pullup
    GpioCtrlRegs.GPDMUX2.bit.GPIO121 = 0;  // Pin as GPIO
    GpioCtrlRegs.GPDDIR.bit.GPIO121 = GPIO_INPUT;   // 0 as input
    //SW5
    GpioCtrlRegs.GPFPUD.bit.GPIO162 = 1;   // Disable pullup
    GpioCtrlRegs.GPFMUX1.bit.GPIO162 = 0;  // Pin as GPIO
    GpioCtrlRegs.GPFDIR.bit.GPIO162 = GPIO_INPUT;   // 0 as input
    //SW6
    GpioCtrlRegs.GPFPUD.bit.GPIO164 = 1;   // Disable pullup
    GpioCtrlRegs.GPFMUX1.bit.GPIO164 = 0;  // Pin as GPIO
    GpioCtrlRegs.GPFDIR.bit.GPIO164 = GPIO_INPUT;   // 0 as input

    EDIS;
}

//
// EPWM Setup Function
//
void EpwmSetup(Uint16 TBPRD, Uint16 MIN_DB)
{
    EALLOW;

    //
    // EPWM1
    //
    // Time Base SubModule Registers
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;   // TBCLK = EPWMCLK
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Up-down Count Mode
    EPwm1Regs.TBPRD = TBPRD; // In Up-down Count Mode, PWM Frequency = 1/(2*TBPRD)
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW; // Shadow Load TBPRD Register
    EPwm1Regs.TBPHS.all = 0;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; //Sync Event at CTR=0
    EPwm1Regs.TBCTL.bit.PHSDIR = 0; // Count Down On Sync
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    //EPwm1Regs.TBCTL.bit.FREE_SOFT = 11;
    EPwm1Regs.TBCTR = 0; //Init Counter=0
    // Counter Compare Submodule Registers
    EPwm1Regs.CMPA.bit.CMPA = 0;   // Init Duty=0%
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // LOAD CMPA on CTR = 0
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    // Action Qualifier SubModule Registers
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // CLEAR if CTR == CMPA When CTR is Up
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
    // Dead Band SubModule Registers
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;// Enable Both RED&FED
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // PWMA=PWM Intput , PWMB= Inverted PWM Input
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL; // Both PWMA&B Inputs are from PWM1A
    EPwm1Regs.DBRED.bit.DBRED = MIN_DB; // Rising Edge Delay
    EPwm1Regs.DBFED.bit.DBFED = MIN_DB; // Falling Edge Delay
    //------------------------------------------------------------------------------------
    //
    // EPWM2
    //
    // Time Base SubModule Registers
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;   // TBCLK = EPWMCLK
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Up-down Count Mode
    EPwm2Regs.TBPRD = TBPRD; // In Up-down Count Mode, PWM Frequency = 1/(2*TBPRD)
    EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW; // Shadow Load TBPRD Register
    EPwm2Regs.TBPHS.all = 0;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; //Output Sync Event at CTR=0
    EPwm2Regs.TBCTL.bit.PHSDIR = 0; // Count Down On Sync
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    //EPwm2Regs.TBCTL.bit.FREE_SOFT = 11;
    EPwm2Regs.TBCTR = 0; //Init Counter=0
    // Counter Compare Submodule Registers
    EPwm2Regs.CMPA.bit.CMPA = 0;   // Init Duty=0%
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // LOAD CMPA on CTR = 0
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    // Action Qualifier SubModule Registers
    #if PwmMethod == BipolarPwm
    // Bipolar PWM
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;  // SET if CTR == CMPA When CTR is Up
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    #endif
    #if PwmMethod == UnipolarPwm
    // Unipolar PWM
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // CLEAR if CTR == CMPA When CTR is Up
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
    #endif
    // Dead Band SubModule Registers
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;// Enable Both RED&FED
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // PWMA=PWM Intput , PWMB= Inverted PWM Input
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL; // Both PWMA&B Inputs are from PWM2A
    EPwm2Regs.DBRED.bit.DBRED = MIN_DB; // Rising Edge Delay
    EPwm2Regs.DBFED.bit.DBFED = MIN_DB; // Falling Edge Delay
    //-----------------------------------------------------------------------------------
    // Configure EPWM 2 as Slave and Let It Pass The Sync In Pulse From EPWM1
    EPwm2Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_IN;
    EPwm2Regs.TBCTL.bit.PHSEN=TB_ENABLE; //Enable
    EPwm2Regs.TBPHS.bit.TBPHS=2; //When Sync Comes, Replace Current TBCTR With TBPHS
    EPwm2Regs.TBCTL.bit.PHSDIR=TB_UP; //Count Up After Sync
    //-----------------------------------------------------------------------------------
    // Trigger ADCSOCA from EPWM1 EVERY 5 EPWMSOC EVENTS
    EPwm1Regs.ETSEL.bit.SOCAEN = 0;    // Disable SOCA
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;    // Generate Events When TBCTR = PRD
    EPwm1Regs.ETPS.bit.SOCPSSEL = 1; // Let SOCAPRD2 Take Control Instead of SOCAPRD
    EPwm1Regs.ETSOCPS.bit.SOCAPRD2 = 5; // Generate SOCA Every 5 Events
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //Enable SOCA

    // One-shot Trip EPWM Initially
    EPwm1Regs.TZFRC.bit.OST = 1;
    EPwm2Regs.TZFRC.bit.OST = 1;

    EDIS;
}

//
// ADC Setup Function
//
void AdcSetup(void)
{
    EALLOW;

    // ADCA
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; //power up the ADC
    DELAY_US(1000); // (US==us, 1000 = 1000us)delay for 1ms to allow ADC time to power up

    // ADCB
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1; //Set pulse positions to late
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1; //power up the ADC
    DELAY_US(1000); // (US==us, 1000 = 1000us)delay for 1ms to allow ADC time to power up

    // ADCC
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1; //Set pulse positions to late
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1; //power up the ADC
    DELAY_US(1000); // (US==us, 1000 = 1000us)delay for 1ms to allow ADC time to power up

    //-----------------------------------------------------------------------------------
    // ADCASOC0 -- A2 -- Iinv
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert ADCA2
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 ADCSOCA
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 30; //sample window is 30 SYSCLK cycles > 1 ADCCLK
    // ADCEOC0 & Interrupt -- Codes from old TI boards
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //EOC0 Triggers ADCINT1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //Enable ADCINT1 Flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0;//No ADCINT1 pulses are generated until ADCINT1 Flag is cleared.
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //CLear ADCINT1 Flag

    // ADCASOC1 -- A3 -- Igrid
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC0 will convert ADCA2
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5; //trigger on ePWM1 ADCSOCA
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 30; //sample window is 30 SYSCLK cycles > 1 ADCCLK

    // ADCBSOC0 -- B0 -- Idc (neg.?)
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert ADCA2
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 ADCSOCA
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 30; //sample window is 30 SYSCLK cycles > 1 ADCCLK

    // ADCBSOC1 -- B1 -- Vdc
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1;  //SOC0 will convert ADCA2
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 5; //trigger on ePWM1 ADCSOCA
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 30; //sample window is 30 SYSCLK cycles > 1 ADCCLK

    // ADCCSOC0 -- C2 -- Vgrid
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert ADCA2
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 ADCSOCA
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 30; //sample window is 30 SYSCLK cycles > 1 ADCCLK

    // ADCCSOC1 -- C3 -- Vcap
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC0 will convert ADCA2
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 5; //trigger on ePWM1 ADCSOCA
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 30; //sample window is 30 SYSCLK cycles > 1 ADCCLK

    EDIS;
}

//
// PWM Generation
//
void UnipolarPWM(float mod)
{
    EPwm1Regs.CMPA.bit.CMPA = + ( INV_PWM_HALF_TBPRD * mod )   + INV_PWM_HALF_TBPRD;
    EPwm2Regs.CMPA.bit.CMPA = - ( INV_PWM_HALF_TBPRD * mod ) + INV_PWM_HALF_TBPRD;
}
void BipolarPWM(float mod)
{
    EPwm1Regs.CMPA.bit.CMPA = ( INV_PWM_HALF_TBPRD * mod ) + INV_PWM_HALF_TBPRD;
    EPwm2Regs.CMPA.bit.CMPA = ( INV_PWM_HALF_TBPRD * mod ) + INV_PWM_HALF_TBPRD;
}

//
// Orthogonal Signal Generator via Hilbert Transformation
//
void OsgExe(ORTHO_SIG * sig, float input, float omega) {
    sig->f_beta_prev  = sig->f_beta;
    sig->f_alpha_prev = sig->f_alpha;
    sig->f_alpha      = input;
    sig->f_beta       = (2-omega*T_ISR) / (2+omega*T_ISR) * (sig->f_beta_prev - sig->f_alpha) + sig->f_alpha_prev;
}

//dq transform and inverse transform
void ab2dq(ORTHO_SIG * sig, float theta) {
    sig->f_d = cosf(theta) * sig->f_alpha + sinf(theta) * sig->f_beta;
    sig->f_q = cosf(theta) * sig->f_beta  - sinf(theta) * sig->f_alpha;
}

#if ( GfmType == GfmdVOC)
//
// VOC Update Function, discretized via Heun's method
//
void RunVOC(VOC_VARS *VOC_obj, float Qerr, float Perr)
{
    VOC_obj->dV[1]     = dV_Sync + VOC_obj->Kesi * VOC_obj->Vmag[0] *( 2 * Vnom * Vnom - VOC_obj->Vmag[0] * VOC_obj->Vmag[0]) - VOC_obj->Kappa / VOC_obj->Vmag[0] * Qerr;
    VOC_obj->dtheta[1] = dw_Sync + wnom - VOC_obj->Kappa / (VOC_obj->Vmag[0] * VOC_obj->Vmag[0]) * Perr;

    VOC_obj->Vmag[1]   = VOC_obj->Vmag[0]   + VOC_obj->dV[1]     * T_ISR;
    VOC_obj->Vtheta[1] = VOC_obj->Vtheta[0] + VOC_obj->dtheta[1] * T_ISR;

    VOC_obj->dV[0]     = VOC_obj->dV[1];
    VOC_obj->dtheta[0] = VOC_obj->dtheta[1];

    VOC_obj->dV[1]     = dV_Sync + VOC_obj->Kesi * VOC_obj->Vmag[1] *( 2* Vnom * Vnom - VOC_obj->Vmag[1] * VOC_obj->Vmag[1]) - VOC_obj->Kappa / VOC_obj->Vmag[1] * Qerr;
    VOC_obj->dtheta[1] = dw_Sync + wnom - VOC_obj->Kappa / (VOC_obj->Vmag[1] * VOC_obj->Vmag[1]) * Perr;

    VOC_obj->Vmag[1]   = VOC_obj->Vmag[0]   + 0.5*(VOC_obj->dV[1]     + VOC_obj->dV[0]    ) * T_ISR;
    VOC_obj->Vtheta[1] = VOC_obj->Vtheta[0] + 0.5*(VOC_obj->dtheta[1] + VOC_obj->dtheta[0]) * T_ISR;

    VOC_obj->Vmag[0]   = VOC_obj->Vmag[1];
    VOC_obj->Vtheta[0] = VOC_obj->Vtheta[1];

    if (VOC_obj->Vtheta[0] > 2*PI)
    {
        VOC_obj->Vtheta[0] = VOC_obj->Vtheta[0] - 2*PI;
    }
}
#endif

#if ( GfmType == GfmVSM)
//
// VSM Update Function, discretized via Forward Euler
//
void  RunVSM(VSM_VARS *VSM_obj, float Qerr, float Perr)
{
    VSM_obj->VSM_V[1]         = Vnom*SQRT2 - VSM_obj->mq * Qerr;
    VSM_obj->VSM_dot_w[1]     = 1 / VSM_obj->J * ( wnom - VSM_obj->VSM_w[0] - VSM_obj->mp * Perr - VSM_obj->D * (VSM_obj->VSM_w[0] - wnom));
    VSM_obj->VSM_dot_theta[1] = VSM_obj->VSM_w[0] + dw_Sync ;

    VSM_obj->VSM_w[1]     = VSM_obj->VSM_w[0]     + VSM_obj->VSM_dot_w[1]     * T_ISR;
    VSM_obj->VSM_theta[1] = VSM_obj->VSM_theta[0] + VSM_obj->VSM_dot_theta[1] * T_ISR;

    if (VSM_obj->VSM_theta[1] > 2*PI)
    {
        VSM_obj->VSM_theta[1] =  VSM_obj->VSM_theta[1] - 2*PI;
    }

    VSM_obj->VSM_V[0]     = VSM_obj->VSM_V[1];
    VSM_obj->VSM_w[0]     = VSM_obj->VSM_w[1];
    VSM_obj->VSM_theta[0] = VSM_obj->VSM_theta[1];
}
#endif

//
// Two-axes Anti-Windup PI
//
void RunPIDQ(PIDQ_CONTROLLER * v, float FFd, float FFq)
{
    // Integrator Input Saturation
    v->magint = sqrtf((v->Outd - FFd) * (v->Outd - FFd) + (v->Outq - FFq) * (v->Outq - FFq));
    if (v->magint > v->max) {
        v->uid = (v->Outd - FFd) * v->max / v->magint;
        v->uiq = (v->Outq - FFq) * v->max / v->magint;
     }
     else {
        v->uid = v->Outd - FFd;
        v->uiq = v->Outq - FFq;
     }
    // Integration
    v->i1d = v->i1d + (v->uid - v->i1d) * v->Ki;
    v->i1q = v->i1q + (v->uiq - v->i1q) * v->Ki; // Here Ki contains T_ISR!
    // PI
    v->v1d  = v->Kp * (v->Refd - v->Fbkd) + v->i1d + FFd;
    v->v1q  = v->Kp * (v->Refq - v->Fbkq) + v->i1q + FFq;
    // Output Saturation
    v->magout = sqrtf(v->v1d * v->v1d + v->v1q * v->v1q);
    if (v->magout > v->max) {
        v->Outd = v->v1d * v->max / v->magout;
        v->Outq = v->v1q * v->max / v->magout;
    }
    else {
        v->Outd = v->v1d;
        v->Outq = v->v1q;
    }
}

//Key Integrator Initialization for restart
void KeyInit(void) {
    // Parameter Initialization, Copied from main.c
    Pref = 0.0, Qref = 0.0;
    PreSyncRatioDroop = 500.0;
    PreSyncRatioVOC   = 10.0;
    PreSyncRatioVSM   = 500.0;
    wP = 2*PI * 20.0; // Power Filter Bandwidth
    Imax = 15.0 * SQRT2;
    //////////////////////
    // For traditional PI
//    MinMax_id = initiateMinMax( -0.95*Vdc_nom, 0.95*Vdc_nom);
//    MinMax_iq = initiateMinMax( -0.95*Vdc_nom, 0.95*Vdc_nom);
//    MinMax_vd = initiateMinMax( -8.0 *SQRT2, 8.0 *SQRT2);
//    MinMax_vq = initiateMinMax( -8.0 *SQRT2, 8.0 *SQRT2);
//    Kpi = 2*PI*BW_cur*Lf;
//    Kii = Kpi*Tau_cur;
//    Kpv = 2*PI*BW_vol*Cf;
//    Kiv = Kpv*Tau_vol;
//    pi_id = InitiateCompensator_PI(T_ISR, Kpi, Kii, 1000/Kii, MinMax_id);  //0.4*BW_cur*L*BW_cur
//    pi_iq = InitiateCompensator_PI(T_ISR, Kpi, Kii, 1000/Kii, MinMax_iq);  //0.4*BW_cur*L*BW_cur
//    pi_vd = InitiateCompensator_PI(T_ISR, Kpv, Kiv, 1000/Kiv, MinMax_vd);
//    pi_vq = InitiateCompensator_PI(T_ISR, Kpv, Kiv, 1000/Kiv, MinMax_vq);
    //////////////////////
    #if (GfmType == GfmDroop)
    mP = 0.5 * 2*PI  / Snom;
    mQ = 0.1 * SQRT2 * Vnom / Snom;
    #elif (GfmType == GfmdVOC)
    voc_obj.Kappa = (2 * PI *0.5) * (Vnom * Vnom * 2) / Snom; //Vnom^2 * 2 -> Vnom here is RMS value
    voc_obj.Kesi  = (2 * PI *0.5) / (0.9 * 0.9 * (1 - 0.9 * 0.9) * (Vnom * Vnom * 2));
    #elif (GfmType == GfmVSM)
    VSM_obj.J  = 2.0 * 0.02 * Snom / wnom;
    VSM_obj.D  = 50.0 * Snom / wnom;
    VSM_obj.mp = (2 * PI * 0.5 / Snom) * (VSM_obj.D + 1);
    VSM_obj.mq = 0.1 * Vnom * SQRT2 / Snom;
    #endif
    omega = wnom;
    // Clear All Integrators
    // PI Integrator
    pi_idq.i1d = 0.0;
    pi_idq.i1q = 0.0;
    pi_vdq.i1d = 0.0;
    pi_vdq.i1q = 0.0;
    //OSG Integrator
    Vcap.f_alpha = 0.0;
    Vcap.f_alpha_prev = 0.0;
    Vcap.f_beta = 0.0;
    Vcap.f_beta_prev = 0.0;
    Vgrid.f_alpha = 0.0;
    Vgrid.f_alpha_prev = 0.0;
    Vgrid.f_beta = 0.0;
    Vgrid.f_beta_prev = 0.0;
    Igrid.f_alpha = 0.0;
    Igrid.f_alpha_prev = 0.0;
    Igrid.f_beta = 0.0;
    Igrid.f_beta_prev = 0.0;
    Iinv.f_alpha = 0.0;
    Iinv.f_alpha_prev = 0.0;
    Iinv.f_beta = 0.0;
    Iinv.f_beta_prev = 0.0;
    //Power Filter Integrator
    P_samp[0] = 0.0;
    Q_samp[0] = 0.0;
    P_filt[0] = 0.0;
    Q_filt[0] = 0.0;
}

//
// cpu_timer0_isr - CPU Timer1 ISR
//
//__interrupt void cpu_timer0_isr(void)
//{
//   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//   if(CpuTimer0Regs.TCR.bit.TIF == 1)
//      {
//          CpuTimer0Regs.TCR.bit.TIF = 1;  // clear flag
//
//      }
//}

interrupt void adca1_isr(void)
{
    volatile int16 temp;
//    int cnt;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    // J53: check if the codes fit the interrupt
    J53 = 1;
    //PI Param Update - SPT Design Based
    pi_idq.Kp   = 2*PI*BW_cur*Lf;
    pi_idq.Ki   = Tau_cur*T_ISR;
    pi_vdq.Kp   = 2*PI*BW_vol*Cf;
    pi_vdq.Ki   = Tau_vol*T_ISR;
    pi_idq.max =  0.98*Vdc_nom;
    pi_vdq.max =  12.5 *SQRT2;

    // Read ADC Sampled Variables
    Vdc   = ((temp=VDC)   * ADC_PU_SCALE_FACTOR               ) * 3.0 * VDC_AMCC1100_GAIN_RECIPROCAL;
    Idc   = ((temp=IDC)   * ADC_PU_SCALE_FACTOR - offset_Idc  ) * 3.0 * IDC_8K2_3K3_RATIO; //direction: dc supply -> inv.

//    iinv  = ((temp=IINV)  * ADC_PU_SCALE_FACTOR - offset_Iinv ) * 3.0 * LESR_15_NP_GAIN_RECIPROCAL * curr_sens_adj_factor;//direction: inv. -> load/grid
//    igrid = ((temp=IGRID) * ADC_PU_SCALE_FACTOR - offset_Igrid) * 3.0 * LESR_15_NP_GAIN_RECIPROCAL * curr_sens_adj_factor;//direction: inv. -> load/grid
//    vcap  = ((temp=VCAP)  * ADC_PU_SCALE_FACTOR - offset_Vcap ) * 3.0 * VAC_SENSING_RES_4M30K_RATIO * volt_sens_adj_factor;
//    vgrid = ((temp=VGRID) * ADC_PU_SCALE_FACTOR - offset_Vgrid) * 3.0 * VAC_SENSING_RES_4M30K_RATIO * volt_sens_adj_factor;
    // Newly tuned sensor
//    vcap  = ((temp=VCAP)  * ADC_PU_SCALE_FACTOR * 3.0 - VAC_DRIFT_TUNED ) * VAC_SENSING_TUNED_RATIO * volt_sens_adj_factor;
    vcap  = (((temp=VCAP)  * ADC_PU_SCALE_FACTOR * 3.0 - 1.5 ) * VAC_SENSING_RES_4M30K_RATIO)* 0.984453579809824 - 3.263084981077332;
    vgrid  = (((temp=VGRID)  * ADC_PU_SCALE_FACTOR * 3.0 - 1.5 ) * VAC_SENSING_RES_4M30K_RATIO)* 0.984453579809824 - 3.263084981077332;
//    vgrid = ((temp=VGRID) * ADC_PU_SCALE_FACTOR * 3.0 - VAC_DRIFT_TUNED ) * VAC_SENSING_TUNED_RATIO * volt_sens_adj_factor;
//    iinv  = ((temp=IINV)  * ADC_PU_SCALE_FACTOR * 3.0 - IAC_DRIFT_TUNED ) * IAC_SENSING_TUNED_RATIO * curr_sens_adj_factor;//direction: inv. -> load/grid
    iinv = (((temp=IINV)  * ADC_PU_SCALE_FACTOR * 3.0 - 1.5 ) * LESR_15_NP_GAIN_RECIPROCAL) * 0.988344121554945 - 0.517468743642696;
    igrid = (((temp=IGRID)  * ADC_PU_SCALE_FACTOR * 3.0 - 1.5 ) * LESR_15_NP_GAIN_RECIPROCAL) * 0.988344121554945 - 0.517468743642696;
//    for (cnt=0;cnt<99;cnt++)
//    {
//        AdcTemp[cnt]=AdcTemp[cnt+1];
//    }
//    AdcTemp[99] = (((temp=VCAP)  * ADC_PU_SCALE_FACTOR * 3.0 - 1.5 ) * VAC_SENSING_RES_4M30K_RATIO)*0.984453579809824 - 3.263084981077332;
//    SumTemp = 0.0;
//    for (cnt=0;cnt<100;cnt++)
//    {
//        SumTemp = SumTemp + AdcTemp[cnt];
//    }
//    vcap = SumTemp/100;

//    igrid = ((temp=IGRID) * ADC_PU_SCALE_FACTOR * 3.0 - IAC_DRIFT_TUNED ) * IAC_SENSING_TUNED_RATIO * curr_sens_adj_factor;//direction: inv. -> load/grid


    // Angle Increment
    #if (ControlType == OpenLoopSine || ControlType == VoltLoopTest || ControlType == CurrLoopTest)
    {
        theta += wnom * T_ISR;
    }
    #elif ( (ControlType == SingleLoopGfm || ControlType == DualLoopGfm) && (GfmType == GfmDroop) )
    {
        theta += omega * T_ISR;
    }
    #endif
    if ( theta > (2*PI) )
    {
        theta -= 2*PI;
    }

    // Grid Angle Calc
    theta_grid = atan2(Vgrid.f_beta, Vgrid.f_alpha) + 2*PI;// Sequence needs check again;
    if ( theta_grid > (2*PI) )
    {
        theta_grid -= 2*PI;
    }

    // OSG via Hilbert Transformation.
    OsgExe(&Vcap , vcap , wnom);
    OsgExe(&Vgrid, vgrid, wnom);
    OsgExe(&Igrid, igrid, wnom);
    OsgExe(&Iinv , iinv , wnom);

    // ab2dq
    ab2dq(&Vcap,  theta);
    ab2dq(&Vgrid, theta);
    ab2dq(&Igrid, theta);
    ab2dq(&Iinv,  theta);
    magvc= sqrtf(Vcap.f_d  * Vcap.f_d  + Vcap.f_q  * Vcap.f_q );
    magvg= sqrtf(Vgrid.f_d * Vgrid.f_d + Vgrid.f_q * Vgrid.f_q);
    magig= sqrtf(Igrid.f_d * Igrid.f_d + Igrid.f_q * Igrid.f_q);
    magil= sqrtf(Iinv.f_d  * Iinv.f_d  + Iinv.f_q  * Iinv.f_q );

    //  Power Filter
    P_samp[1] = ( Vcap.f_d * Iinv.f_d + Vcap.f_q * Iinv.f_q ) / 2 ;
    Q_samp[1] = ( Vcap.f_q * Iinv.f_d - Vcap.f_d * Iinv.f_q ) / 2 ;
    P_filt[1] = P_filt[0]*(2-wP*T_ISR)/(2+wP*T_ISR) + (P_samp[1]+P_samp[0])*wP*T_ISR/(2+wP*T_ISR);
    Q_filt[1] = Q_filt[0]*(2-wP*T_ISR)/(2+wP*T_ISR) + (Q_samp[1]+Q_samp[0])*wP*T_ISR/(2+wP*T_ISR);
    P_filt[0] = P_filt[1];
    P_samp[0] = P_samp[1];
    Q_filt[0] = Q_filt[1];
    Q_samp[0] = Q_samp[1];

    // PWM Gate Driver Test: fixed duty ratio
    if (FaultTypeFlag == 0)
//    if(READY == 1 && FLT_STATUS == 0)//&& EPwm1Regs.TZFLG.all == 0 && EPwm2Regs.TZFLG.all == 0)
    {
#if (ControlType == SingleLoopGfm || ControlType == DualLoopGfm )
#if ( GfmType == GfmDroop)
        // GFM Relation - Droop
        if (SYNC_STATUS == 1)
        {
            omega = wnom - PreSyncRatioDroop * sinf(theta - theta_grid);
        }
        else if (SYNC_STATUS == 0)
        {
            omega = wnom - mP * (P_filt[1] - Pref);
        }
        //Vmag =  Vnom*SQRT2 - mq *  (Q_filt[1] - Qref);
        //dV_int += 50.0 * T_ISR * mQ * (Qref - Q_filt[1]);
        dV_int = 0.0;
        dV_prop = mQ * (Qref - Q_filt[1]);
        Vmag =  Vnom * SQRT2 + (dV_int + dV_prop);

#elif ( GfmType == GfmdVOC)
        // GFM Relation - dVOC
        if (SYNC_STATUS == 1) {
            dV_Sync = 0.0;
            dw_Sync = - PreSyncRatioVOC * sinf(theta - theta_grid);
            RunVOC(&voc_obj, 0.0, 0.0);
        }
        else if (SYNC_STATUS == 0) {
            dV_Sync = 0.0;
            dw_Sync = 0.0;
            RunVOC(&voc_obj, Q_filt[1] - Qref, P_filt[1] - Pref);
        }
        else {}
        omega = voc_obj.dtheta[0];
        theta = voc_obj.Vtheta[0];
        Vmag  = voc_obj.Vmag[0];
#elif ( GfmType == GfmVSM)
        // GFM Relation - VSM
        if (SYNC_STATUS == 1) {
            dV_Sync = 0.0;
            dw_Sync = - PreSyncRatioVSM * sinf(theta - theta_grid);
            RunVSM(&VSM_obj, 0.0, 0.0);
        }
        else if (SYNC_STATUS == 0) {
            dV_Sync = 0.0;
            dw_Sync = 0.0;
            RunVSM(&VSM_obj, Q_filt[1] - Qref, P_filt[1] - Pref);
        }
        else{}
        Vmag  = VSM_obj.VSM_V[0];
        omega = VSM_obj.VSM_w[0];
        theta = VSM_obj.VSM_theta[0];
#endif
#endif

        #if ( ControlType == VoltLoopTest )
        {
            Vdref = Vrms_TEST*SQRT2;
            Vqref = 0.0 ;
        }
        #else
        {
            Vdref = Vmag; // - 1.5 * Igrid.f_d;
            Vqref = 0.0 ; // - 1.5 * Igrid.f_q;
        }
        #endif

        pi_vdq.Refd = Vdref;
        pi_vdq.Refq = Vqref;
        pi_vdq.Fbkd = Vcap.f_d;
        pi_vdq.Fbkq = Vcap.f_q;
        RunPIDQ(&pi_vdq, - wnom * Cf * Vcap.f_q + Igrid.f_d, + wnom * Cf * Vcap.f_d + Igrid.f_q);
//        RunPIDQ(&pi_vdq, - wnom * Cf * Vcap.f_q, + wnom * Cf * Vcap.f_d);
        Idref = pi_vdq.Outd;
        Iqref = pi_vdq.Outq;
//        Idref = CompensatorCalculation_PI(Vcap.f_d, Vdref, &pi_vd, - wnom * Cf * Vcap.f_q);
//        Iqref = CompensatorCalculation_PI(Vcap.f_q, Vqref, &pi_vq, + wnom * Cf * Vcap.f_d);

        #if (ControlType == CurrLoopTest)
        Idref = Irms_TEST*SQRT2;
        Iqref = 0.0;
        #endif

        //Sol 1: Traditional PI
//        pi_id.Ref = Idref;
//        pi_iq.Ref = Iqref;
//
//        pi_id.Fbk = Iinv.f_d;
//        pi_iq.Fbk = Iinv.f_q;
//
//        PI_MACRO(pi_id)
//        PI_MACRO(pi_iq)
////
//        Vm_d = pi_id.Out - wnom * Lf * Iinv.f_q + Vcap.f_d;
//        Vm_q = pi_iq.Out + wnom * Lf * Iinv.f_d + Vcap.f_q;
//          Vm_d = CompensatorCalculation_PI(Iinv.f_d, Idref, &pi_id, - wnom * Lf * Iinv.f_q + Vcap.f_d);
//          Vm_q = CompensatorCalculation_PI(Iinv.f_q, Iqref, &pi_iq, + wnom * Lf * Iinv.f_d + Vcap.f_q);
//        //Sol 2: New PI
        pi_idq.Refd = Idref;
        pi_idq.Refq = Iqref;
        pi_idq.Fbkd = Iinv.f_d;
        pi_idq.Fbkq = Iinv.f_q;
        RunPIDQ(&pi_idq, - wnom * Lf * Iinv.f_q + Vcap.f_d, + wnom * Lf * Iinv.f_d + Vcap.f_q);
        Vm_d = pi_idq.Outd;
        Vm_q = pi_idq.Outq;
    }
        #if (ControlType == OpenLoopSine)
        {
            Vmod = ModIndex*sinf(theta);
        }
        #elif (ControlType == DC2DC_SensorTest)
        {
            Vmod = ModIndex;
        }
        #elif (ControlType == SingleLoopGfm)
        {
            Vmod = (Vdref * cosf(theta) - sinf(theta) * Vqref)/Vdc_nom;
        }
        #elif (ControlType == DualLoopGfm || ControlType == VoltLoopTest || ControlType == CurrLoopTest)
        {
            Vmod = (Vm_d * cosf(theta) - Vm_q * sinf(theta))/Vdc_nom;
        }
        #endif

        #if PwmMethod == BipolarPwm
        BipolarPWM(Vmod);
        #endif
        #if PwmMethod == UnipolarPwm
        UnipolarPWM(Vmod); // Vgrid distorted
        #endif

    // J53: check if the codes fit the interrupt
    J53 = 0;

    return;
}

// COMMENT-UNCOMMENT DAC
//void configureDAC(Uint16 dac_num)
//{
//    EALLOW;
//    DAC_PTR[dac_num]->DACCTL.bit.DACREFSEL = 1; //REFERENCE
//    DAC_PTR[dac_num]->DACOUTEN.bit.DACOUTEN = 1;
//    DAC_PTR[dac_num]->DACVALS.all = 0;
//    DELAY_US(10); // Delay for buffered DAC to power up
//    EDIS;
//}


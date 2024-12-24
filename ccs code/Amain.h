#ifndef AMAIN_H_
#define AMAIN_H_

#include "F28x_Project.h"
#include "math.h"
//#include "IQmathLib.h"

/*********************************/
#define GfmDroop  0
#define GfmdVOC   1
#define GfmVSM    2
/*********************************/
#define GfmType GfmdVOC

/*********************************/
#define  OpenLoopSine      0
#define  SingleLoopGfm     1
#define  DualLoopGfm       2
#define  DC2DC_SensorTest  3
#define  VoltLoopTest      4
#define  CurrLoopTest      5
/*********************************/
#define  ControlType DualLoopGfm

/*********************************/
#define BipolarPwm   0
#define UnipolarPwm  1
/*********************************/
#define PwmMethod UnipolarPwm

// Math Macros
#define PI         3.141592653589793f
#define SQRT2      1.414213562373095f
#define ONEbySQRT3 0.577350269189626f  // 1/sqrt(3)
#define SQRT3      1.732050807568877f
// Hardware Macros
#define Cf   7e-6f
#define Rd   5.0f
#define Lf   300e-6f
#define Lg   30e-6f
#define Rf   0.1f
#define Snom 1500.0f
#define Vnom 120.0f
#define fnom 60.0f
#define wnom 2*PI*fnom
#define Vdc_nom 200.0f //Nominal Vdc 250V
//Time Base Macros
#define SYSTEM_FREQUENCY 200000000 // 200 MHz -- Weiqian 20241028 Is it so?
#define PWM_FREQUENCY       100000 // 100kHz inverter switching frequency
#define INV_PWM_TICKS ((SYSTEM_FREQUENCY/2)/PWM_FREQUENCY) //EPWMCLKDIV=1, HSPCLKDIV=0,CLKDIV = 0, TBCLK = SYSCLK/2
#define INV_PWM_TBPRD INV_PWM_TICKS/2  //500
#define INV_PWM_HALF_TBPRD INV_PWM_TICKS/4   //250
#define EPWM_MIN_DB  38 //45 //dead time = (EPWM_MIN_DB*10)ns
#define ISR_FREQUENCY        20000.0f //20kHz
#define T_ISR            1/ISR_FREQUENCY //1/20kHz=50us

#define ADC_PU_SCALE_FACTOR 0.000244140625 // (1/2^12)
#define IINV  AdcaResultRegs.ADCRESULT0
#define IGRID AdcaResultRegs.ADCRESULT1
#define IDC   AdcbResultRegs.ADCRESULT0
#define VDC   AdcbResultRegs.ADCRESULT1
#define VCAP  AdccResultRegs.ADCRESULT1
#define VGRID AdccResultRegs.ADCRESULT0
// Voltage sensing circuit sensing input resistance ratio R2/R1
#define VAC_DRIFT_TUNED              1.530019004218497f
#define VAC_SENSING_TUNED_RATIO      132.8867315447621f
#define VAC_SENSING_RES_4M30K_RATIO  133.333333333 // R2=4M¦¸, R1=30k¦¸
#define VAC_SENSING_RES_3M30K_RATIO  100.000000000 // R2=3M¦¸, R1=30k¦¸

// AC Current sensing circuit
#define IAC_DRIFT_TUNED              1.516776905326749f
#define IAC_SENSING_TUNED_RATIO      23.799692641049564f
//LESR 15-NP Hall sensor sensitivity
// Sensitivity G=41.67mV/A 's reciprocal
#define LESR_15_NP_GAIN_RECIPROCAL  23.99808015359

// DC Current sensing ratio R1/(G*R2),
//  R1=3K3, R2=8K2
#define IDC_8K2_3K3_RATIO  9.899208063355
//  R1=3K3, R2=8K0
#define IDC_8K0_3K3_RATIO  9.6577639642487
// DC Voltage sensing ratio
// Gain = voltage_divider_ratio * amplifier_ratio * Op-amp_ratio
// Gain = 56/(400k+56)  * 8 * (4.3k/2k) --> Take Its Reciprocal
#define VDC_AMCC1100_GAIN_RECIPROCAL  415.3405315614618

//GPIO Input (SW1-6) & Output(J48-53) Pins
#define J48 GpioDataRegs.GPBDAT.bit.GPIO36
#define J49 GpioDataRegs.GPBDAT.bit.GPIO38
#define J50 GpioDataRegs.GPBDAT.bit.GPIO61
#define J51 GpioDataRegs.GPBDAT.bit.GPIO63
#define J52 GpioDataRegs.GPCDAT.bit.GPIO65
#define J53 GpioDataRegs.GPCDAT.bit.GPIO67

#define SW1 GpioDataRegs.GPCDAT.bit.GPIO91
#define SW2 GpioDataRegs.GPCDAT.bit.GPIO93
#define SW3 GpioDataRegs.GPEDAT.bit.GPIO133
#define SW4 GpioDataRegs.GPDDAT.bit.GPIO121
#define SW5 GpioDataRegs.GPFDAT.bit.GPIO162
#define SW6 GpioDataRegs.GPFDAT.bit.GPIO164

// Orthogonal Signal Struct Def
typedef struct {
    float f_alpha; // input in this cycle
    float f_beta; // output
    float f_alpha_prev; // input from the last cycle
    float f_beta_prev; //
    float f_d;
    float f_q;
} ORTHO_SIG;
#define OSG_DEFAULTS {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}

// Two-axes Anti-Windup PI Controller Defs
typedef struct {
    float  Refd;             // Input: reference set-point
    float  Refq;             // Input: reference set-point
    float  Fbkd;             // Input: feedback
    float  Fbkq;             // Input: feedback
    float  v1d;              // Data: pre-saturated controller output
    float  v1q;              // Data: pre-saturated controller output
    float  Outd;             // Output: controller output
    float  Outq;             // Output: controller output
    float  Kp;              // Parameter: proportional loop gain
    float  Ki;              // Parameter: integral gain
    float  max;            // Parameter: upper saturation limit
    float  uid;              // Data: saturated integral input
    float  uiq;              // Data: saturated integral input
    float  i1d;              // Data: integral output
    float  i1q;              // Data: integral output
    float  magout;
    float  magint;
} PIDQ_CONTROLLER;
#define PIDQ_CONTROLLER_DEFAULTS {        \
                           0,           \
                           0,           \
                           0,           \
                           0.0,    \
                           0.0,    \
                           0.0,    \
                           0.0,   \
                           0.0,    \
                           0.0,    \
                           0.0,    \
                           0.0,    \
                           0.0,    \
                           0.0,    \
                           0.0,    \
                           0.0,     \
                           0.0,\
                           0.0\
}

#if ( GfmType == GfmdVOC)
typedef struct {
    float Vmag[2];     // V_amp[n-1]
    float Vtheta[2];     // V_theta[n-1]
    float dV[2]; // V_amp[n]
    float dtheta[2]; // V_theta[n]
    float Kesi;
    float Kappa;
}VOC_VARS;
#define VOC_DEFAULTS  {{0.1, 0.1}, {0.1, 0.1}, {0.0, 0.0}, {0.0, 0.0}, 0.0, 0.0}
void RunVOC(VOC_VARS *VOC_obj, float Qerr, float Perr);
#endif

#if ( GfmType == GfmVSM)
typedef struct {
    //x,y,z: V, omega, theta
    float VSM_V[2];
    float VSM_w[2];
    float VSM_theta[2];
    float VSM_dot_w[2];
    float VSM_dot_theta[2];
    float mq;
    float mp;
    float D;
    float J;
}VSM_VARS;
#define VSM_DEFAULTS  {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 0.0, 0.0, 0.0, 0.0}
void RunVSM(VSM_VARS *VSM_obj, float Qerr, float Perr); //VSM iteration func
#endif

// Function Prototypes
//interrupt void epwm11_isr(void);
interrupt void adca1_isr(void);
void GpioSetup(void);
void EpwmSetup(Uint16 TBPRD, Uint16 MIN_DB);
void AdcSetup(void);
void UnipolarPWM(float mod);
void BipolarPWM(float mod);
void OsgExe(ORTHO_SIG * sig, float input, float omega);
void ab2dq(ORTHO_SIG * sig, float theta);
void KeyInit(void);
void InitPIDQ(PIDQ_CONTROLLER * v);
void RunPIDQ(PIDQ_CONTROLLER * v, float FFd, float FFq);

#endif /* AMAIN_H_ */

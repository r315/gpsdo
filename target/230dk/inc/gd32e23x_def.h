#ifndef _gd32e23x_def_h_
#define _gd32e23x_def_h_

#include <stdint.h>

typedef struct {
    union{
        volatile uint32_t CTL0;
        struct {
            volatile uint32_t IRC8MEN : 1;
            volatile uint32_t IRC8MSTB : 1;
            volatile uint32_t rsvd0 : 1;
            volatile uint32_t IRC8MADJ : 5;
            volatile uint32_t IRC8MCALIB : 8;
            volatile uint32_t HXTALEN : 1;
            volatile uint32_t HXTALSTB : 1;
            volatile uint32_t HXTALBPS : 1;
            volatile uint32_t CLKMEN : 1;
            volatile uint32_t rsvd1 : 4;
            volatile uint32_t PLLEN : 1;
            volatile uint32_t PLLSTB : 1;
        };
    }CTL0_BIT;
    volatile uint32_t CFG0;
    volatile uint32_t INT;
    volatile uint32_t APB2RST;
    volatile uint32_t APB1RST;
    volatile uint32_t AHBEN;
    volatile uint32_t APB2EN;
    volatile uint32_t APB1EN;
    volatile uint32_t BDCTL;
    volatile uint32_t AHBRST;
    volatile uint32_t CFG1;
    volatile uint32_t CFG2;
    volatile uint32_t CTL1;
    uint32_t rsvd[0x100 - 0x34];
    volatile uint32_t VKEY;
    volatile uint32_t DSV;
}Rcu_Type;

typedef struct {
    volatile uint32_t CTL0;
    volatile uint32_t CTL1;
    volatile uint32_t SMCFG;
    volatile uint32_t DMAINTEN;
    volatile uint32_t INTF;
    volatile uint32_t SWEVG;
    volatile uint32_t CHCTL0;
    volatile uint32_t CHCTL1;
    volatile uint32_t CHCTL2;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t CAR;
    volatile uint32_t CREP;
    volatile uint32_t CH0CV;
    volatile uint32_t CH1CV;
    volatile uint32_t CH2CV;
    volatile uint32_t CH3CV;
    volatile uint32_t CCHP;
    volatile uint32_t DMACFG;
    volatile uint32_t DMATB;
    volatile uint32_t RSVD2[43];
    volatile uint32_t CFG;
}Timer_Advanced;

typedef struct {
    volatile uint32_t CTL0;
    volatile uint32_t CTL1;
    volatile uint32_t SMCFG;
    volatile uint32_t DMAINTEN;
    volatile uint32_t INTF;
    volatile uint32_t SWEVG;
    volatile uint32_t CHCTL0;
    volatile uint32_t CHCTL1;
    volatile uint32_t CHCTL2;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t CAR;
    volatile uint32_t RSV0;
    volatile uint32_t CH0CV;
    volatile uint32_t CH1CV;
    volatile uint32_t CH2CV;
    volatile uint32_t CH3CV;
    volatile uint32_t RSV1;
    volatile uint32_t DMACFG;
    volatile uint32_t DMATB;
    volatile uint32_t RSVD2[43];
    volatile uint32_t CFG;
}Timer_TypeL0;

typedef struct {
    volatile uint32_t CTL0;     // 00
    volatile uint32_t RSV1[2];
    volatile uint32_t DMAINTEN; // 0C
    volatile uint32_t INTF;     // 10
    volatile uint32_t SWEVG;    // 14
    volatile uint32_t CHCTL0;   // 18
    volatile uint32_t RSV2;
    volatile uint32_t CHCTL2;   // 20
    volatile uint32_t CNT;      // 24
    volatile uint32_t PSC;      // 28
    volatile uint32_t CAR;      // 2C
    volatile uint32_t RSV3;
    volatile uint32_t CH0CV;    // 34
    volatile uint32_t RSV4[7];
    volatile uint32_t IRMP;     // 50
    volatile uint32_t RSV5[43];
    volatile uint32_t CFG;
}Timer_TypeL2;

typedef struct {
    volatile uint32_t CTL0;
    volatile uint32_t CTL1;
    volatile uint32_t SMCFG;
    volatile uint32_t DMAINTEN;
    volatile uint32_t INTF;
    volatile uint32_t SWEVG;
    volatile uint32_t CHCTL0;
    volatile uint32_t RSV1;
    volatile uint32_t CHCTL2;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t CAR;
    volatile uint32_t CREP;
    volatile uint32_t CH0CV;
    volatile uint32_t CH1CV;
    volatile uint32_t RSV2[3];
    volatile uint32_t CCHP;
    volatile uint32_t DMACFG;
    volatile uint32_t DMATB;
    volatile uint32_t RSVD2[43];
    volatile uint32_t CFG;
}Timer_TypeL3;

typedef struct {
    volatile uint32_t CTL0;
    volatile uint32_t CTL1;
    volatile uint32_t RSV1;
    volatile uint32_t DMAINTEN;
    volatile uint32_t INTF;
    volatile uint32_t SWEVG;
    volatile uint32_t CHCTL0;
    volatile uint32_t RSV2;
    volatile uint32_t CHCTL2;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t CAR;
    volatile uint32_t CREP;
    volatile uint32_t CH0CV;
    volatile uint32_t RSV3[4];
    volatile uint32_t CCHP;
    volatile uint32_t DMACFG;
    volatile uint32_t DMATB;
    volatile uint32_t RSV4[43];
    volatile uint32_t CFG;
}Timer_TypeL4;

typedef struct {
    volatile uint32_t CTL0;
    volatile uint32_t CTL1;
    volatile uint32_t rsvd_1;
    volatile uint32_t DMAINTEN;
    volatile uint32_t INTF;
    volatile uint32_t SWEVG;
    volatile uint32_t rsvd_2[3];
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t CAR;
}TimerBasic_Type;

#define RCU_CMSIS       ((Rcu_Type *)RCU_BASE)
#define TMR0            ((Timer_Advanced*)TIMER0)
#define TMR2            ((Timer_TypeL0*)TIMER2)
#define TMR5            ((TimerBasic_Type*)TIMER5)
#define TMR13           ((Timer_TypeL2*)TIMER13)
#define TMR14           ((Timer_TypeL3*)TIMER14)
#define TMR15           ((Timer_TypeL4*)TIMER15)
#define TMR16           ((Timer_TypeL4*)TIMER16)


#define RCU_BDCTL_RTCSRC_LXTAL      (1 << 8)

#endif
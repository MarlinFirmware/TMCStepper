/**
 * TMCStepper library by @teemuatlut
 * TMC2240_bitfields.h
 *
 * TMC2240 hardware register bit fields:
 * CHOPCONF, COOLCONF, DRV_CONF, PWMCONF, NODECONF
 * TPOWERDOWN, TSTEP, TPWMTHRS, TCOOLTHRS, THIGH
 * SG4_THRS, SG4_RESULT, SG4_IND
 * GCONF, GSTAT
 * IOIN, GLOBAL_SCALER
 * IHOLD_IRUN
 * DRV_STATUS
 * PWM_SCALE, PWM_AUTO
 */
#pragma once
#pragma pack(push, 1)

namespace TMC2240_n {
  // 0x00 GCONF: Global Configuration Flags (RW)
  struct GCONF_t {
    constexpr static uint8_t address = 0x00;
    union {
      uint32_t sr;
      struct {
        bool                   : 1, // unused
              fast_standstill  : 1,
              en_pwm_mode      : 1,
              multistep_filt   : 1,
              shaft            : 1,
              diag0_error      : 1,
              diag0_otpw       : 1,
              diag0_stall      : 1,
              diag1_stall      : 1,
              diag1_index      : 1,
              diag1_onstate    : 1,
                               : 1, // unused
              diag0_pushpull   : 1,
              diag1_pushpull   : 1,
              small_hysteresis : 1,
              stop_enable      : 1,
              direct_mode      : 1;
        uint16_t               : 15; // unused
      };
    };
  };

  // 0x01 GSTAT: Global Status Flags (RO)
  // Write 1 to clear a flag
  struct GSTAT_t {
      constexpr static uint8_t address = 0x01;
      union {
        uint8_t sr;
        struct {
          bool reset          : 1,
               drv_err        : 1,
               uv_cp          : 1,
               register_reset : 1,
               vm_uvlo        : 1,
                              : 3; // unused
      };
    };
  };

  // 0x03 NODECONF: Node Configuration (RW)
  // Write 0 to reset values
  struct NODECONF_t {
    constexpr static uint8_t address = 0x03;
    union {
      uint16_t sr;
      struct {
        uint8_t   nodeaddr  : 8,
                  senddelay : 4,
                            : 4; // unused
      };
    };
  };

  // 0x04 IOIN: I/O In (RO, mostly)
  // Read the state of all input pins and return IC revision in high byte
  // Write 1 to 'output' to reset
  struct IOIN_t {
    constexpr static uint8_t address = 0x04;
    union {
      uint32_t sr;
      struct {
        bool  step          : 1,
              dir           : 1,
              encb          : 1,
              enca          : 1,
              drv_enn       : 1,
              encn          : 1,
              uart_en       : 1,
                            : 1, // reserved
              comp_a        : 1,
              comp_b        : 1,
              comp_a1_a2    : 1,
              comp_b1_b2    : 1,
              output        : 1, // RW
              ext_res_det   : 1,
              ext_clk       : 1,
              adc_err       : 1;
        uint8_t silicon_rv  : 3,
                            : 5, // unused
              version       : 8;
      };
    };
  };

  // 0x0A DRV_CONF: Driver Configuration (RW)
  // Write 0 to "reset" values
  struct DRV_CONF_t {
    constexpr static uint8_t address = 0x0A;
    union {
      uint8_t sr;
      struct {
        uint8_t current_range : 2,
                              : 2,  // unused
                slope_control : 2,
                              : 2;  // unused
      };
    };
  };

  // 0x0B GLOBAL_SCALER: Global Scaler (RW)
  // Write 0 to reset
  struct GLOBAL_SCALER_t {
    constexpr static uint8_t address = 0x0B;
    union {
      uint8_t sr;
      struct {
         uint8_t GLOBALSCALER : 8;
      };
    };
  };

  // 0x10 IHOLD_IRUN: Motor Current Control (RW)
  struct IHOLD_IRUN_t {
    constexpr static uint8_t address = 0x10;
    union {
      uint32_t sr;
      struct {
        uint8_t  ihold : 5,
                       : 3, // unused
                 irun  : 5,
                       : 3, // unused
            iholddelay : 4,
                       : 4, // unused
            irundelay  : 4,
                       : 4; // unused
      };
    };
  };

  // 0x11 TPOWERDOWN: Motor Power Down Time (RW)
  struct TPOWERDOWN_t {
    constexpr static uint8_t address = 0x11;
    union {
      uint8_t sr;
      struct {
        uint8_t TPOWERDOWN;
      };
    };
  };

  // 0x12 TSTEP: Actual Step Frequency (RO)
  struct TSTEP_t {
    constexpr static uint8_t address = 0x12;
    union {
      uint32_t sr;
      struct {
        uint32_t  tstep : 20;
        uint16_t        : 12; // unused
      };
    };
  };

  // 0x13 TPWMTHRS: StealthChop Threshold (RW)
  struct TPWMTHRS_t {
    constexpr static uint8_t address = 0x13;
    union {
      uint32_t sr;
      struct {
        uint32_t  tpwmthrs : 20;
        uint16_t           : 12; // unused
      };
    };
  };

  // 0x14 TCOOLTHRS: CoolStep Threshold (RW)
  struct TCOOLTHRS_t {
    constexpr static uint8_t address = 0x14;
    union {
      uint32_t sr;
      struct {
        uint32_t  tcoolthrs : 20;
        uint16_t            : 12; // unused
      };
    };
  };

  // 0x15 THIGH: High Velocity Mode Threshold (RW)
  struct THIGH_t {
    constexpr static uint8_t address = 0x15;
    union {
      uint32_t sr;
      struct {
        uint32_t  thigh : 20;
        uint16_t        : 12; // unused
      };
    };
  };

  // 0x6C CHOPCONF: Chopper Configuration (RW)
  struct CHOPCONF_t {
    constexpr static uint8_t address = 0x6C;
    union {
      uint32_t sr;
      struct {
        uint8_t toff     : 4,
                hstrt    : 3,
                hend     : 4;
        bool    fd3      : 1,
                disfdcc  : 1,
                         : 1, // unused
                chm      : 1;
        uint8_t TBL      : 2;
        bool             : 1, // unused
                vhighfs  : 1,
                vhighchm : 1;
        uint8_t tpfd     : 4,
                mres     : 4;
        bool    intpol   : 1,
                dedge    : 1,
                diss2g   : 1,
                diss2vs  : 1;
      };
    };
  };

  // 0x6D COOLCONF: CoolStep Configuration (RW)
  struct COOLCONF_t {
    constexpr static uint8_t address = 0x6D;
    union {
      uint32_t sr;
      struct {
        uint8_t    semin  : 4;
        bool              : 1;
        uint8_t    seup   : 2;
        bool              : 1;
        uint8_t    semax  : 4;
        bool              : 1;
        uint8_t    sedn   : 2;
        bool       seimin : 1;
        int8_t     sgt    : 7,
                          : 1; // unused
        bool       sfilt  : 1;
        uint8_t           : 7; // unused
      };
    };
  };

  // 0x6F DRV_STATUS: Driver Status Flags (RO)
  struct DRV_STATUS_t {
    constexpr static uint8_t address = 0x6F;
    union {
      uint32_t sr;
      struct {
        uint16_t SG_RESULT : 10;
        uint8_t            : 2; // unused
        uint8_t      s2vsa : 1,
                     s2vsb : 1,
                   stealth : 1,
                  fsactive : 1;
        uint16_t CS_ACTUAL : 5;
        uint8_t            : 3; // unused
        bool    stallguard : 1,
                        ot : 1,
                      otpw : 1,
                      s2ga : 1,
                      s2gb : 1,
                       ola : 1,
                       olb : 1,
                      stst : 1;
      };
    };
  };

  // 0x70 PWMCONF: PWM Configuration (RW)
  struct PWMCONF_t {
    constexpr static uint8_t address = 0x70;
    union {
      uint32_t sr;                      // 0xC40C1E1D
      struct {
        uint8_t pwm_ofs            : 8, // 29
                pwm_grad           : 8, // 30
                pwm_freq           : 2; // 0
        bool    pwm_autoscale      : 1, // true
                pwm_autograd       : 1; // true
        uint8_t freewheel          : 2; // 0
        bool    pwm_meas_sd_enable : 1, // false
                pwm_dis_reg_stst   : 1; // false
        uint8_t pwm_reg            : 4, // 4
                pwm_lim            : 4; // 12
      };
    };
  };

  // 0x71 PWM_SCALE: PWM Amplitude (RO)
  struct PWM_SCALE_t {
    constexpr static uint8_t address = 0x71;
    union {
      uint32_t sr;
      struct {
        uint16_t pwm_scale_sum  : 10;
        uint8_t                 : 6; // unused
        int16_t  pwm_scale_auto : 9;
        uint8_t                 : 7; // unused
      };
    };
  };

  // 0x72 PWM_AUTO: PWM Automatic Scaling (RO)
  struct PWM_AUTO_t {
    constexpr static uint8_t address = 0x72;
    union {
      uint32_t sr : 24;
      struct {
        uint8_t pwm_ofs_auto  : 8,
                              : 8, // unused
                pwm_grad_auto : 8,
                              : 8; // unused
      };
    };
  };

  // 0x74 SG4_THRS: StallGuard4 Threshold (RW)
  struct SG4_THRS_t {
    constexpr static uint8_t address = 0x74;
    union {
      uint32_t sr;
      struct {
        uint8_t  sg4_thrs;
        bool     sg4_filt_en      :  1;
        uint8_t  sg4_angle_offset :  1;
        uint32_t                  : 22; // unused
      };
    };
  };

  // 0x75 SG4_RESULT: StallGuard4 Result (RO)
  struct SG4_RESULT_t {
    constexpr static uint8_t address = 0x75;
    union {
      uint32_t sr;
      struct {
        uint16_t sg4_result : 10;
        uint32_t            : 22; // unused
      };
    };
  };

  // 0x76 SG4_IND: StallGuard4 Indicators (RO)
  struct SG4_IND_t {
    constexpr static uint8_t address = 0x76;
    union {
      uint32_t sr;
      struct {
        uint8_t sg4_ind_0, sg4_ind_1, sg4_ind_2, sg4_ind_3;
      };
    };
  };

  // 0x50 ADC_VSUPPLY_AIN: Supply & AIN ADC (RO)
  struct ADC_VSUPPLY_AIN_t {
    constexpr static uint8_t address = 0x50;
    union {
      uint32_t sr;
      struct {
        uint16_t adc_vsupply : 13;
        uint16_t adc_ain     : 13;
        uint8_t              : 6; // reserved / padding
      };
    };
  };

  // 0x51 ADC_TEMP: Temperature ADC (RO)
  struct ADC_TEMP_t {
    constexpr static uint8_t address = 0x51;
    union {
      uint32_t sr;
      struct {
        uint16_t adc_temp : 13;
        uint32_t          : 19; // reserved
      };
    };
  };

  // 0x52 OTW_OV_VTH: Overtemp/Overvoltage Thresholds (RW)
  struct OTW_OV_VTH_t {
    constexpr static uint8_t address = 0x52;
    union {
      uint32_t sr;
      struct {
        uint16_t overvoltage_vth       : 13;
        uint8_t                        : 3;
        uint16_t overtemp_prewarn_vth  : 13;
        uint8_t                        : 3;
      };
    };
  };

} // TMC2240_n

#pragma pack(pop)

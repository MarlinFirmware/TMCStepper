/**
 * TMCStepper library by @teemuatlut
 * TMC2240_bitfields.h
 *
 * TMC2240 hardware register bit fields:
 * CHOPCONF, DEVCONF, COOLCONF, PWMCONF
 * IHOLD_IRUN
 * TPOWERDOWN, TPWMTHRS, TCOOLTHRS
 * SG4_THRS
 * GCONF, GSTAT
 */
#pragma once
#pragma pack(push, 1)

namespace TMC2240_n {
  struct GCONF_t {
    constexpr static uint8_t address = 0x00;
    union {
      uint32_t sr;
      struct {
        bool                   : 1,
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
                               : 1,
              diag0_pushpull   : 1,
              diag1_pushpull   : 1,
              small_hysteresis : 1,
              stop_enable      : 1,
              direct_mode      : 1;
      };
    };
  };

  struct GSTAT_t {
      constexpr static uint8_t address = 0x01;
      union {
        uint32_t sr;
        struct {
          bool reset          : 1,
               drv_err        : 1,
               uv_cp          : 1,
               register_reset : 1,
               vm_uvlo        : 1;
      };
    };
  };

  struct TPOWERDOWN_t {
    constexpr static uint8_t address = 0x11;
    union {
      uint32_t sr;
      struct {
        uint8_t semin : 8;
      };
    };
  };

  struct DEVCONF_t {
    constexpr static uint8_t address = 0x0a;
    union {
      uint32_t sr;
      struct {
        uint8_t cur_range     : 2,
                              : 2,
                slope_control : 2;
      };
    };
  };

  struct TPWMTHRS_t {
    constexpr static uint8_t address = 0x13;
    union {
      uint32_t sr;
      struct {
        //uint8_t tpwmthrsb : 20;
        uint8_t tpwmthrsb;
      };
    };
  };
  struct TCOOLTHRS_t {
    constexpr static uint8_t address = 0x14;
    union {
      uint32_t sr;
      struct {
        //uint8_t tcoolthrs : 20;
        uint8_t tcoolthrs;
      };
    };
  };
  struct SG4_THRS_t {
    constexpr static uint8_t address = 0x74;
    union {
      uint32_t sr;
      struct {
        uint8_t sg4_thrsb       : 8;
        bool    sg4_filt_en     : 1,
                sg_angle_offset : 1;
      };
    };
  };

  struct IHOLD_IRUN_t {
    constexpr static uint8_t address = 0x10;
    union {
      uint32_t sr;
      struct {
        uint8_t  ihold : 5,
                       : 3,
                 irun  : 5,
                       : 3,
            iholddelay : 4,
                       : 4,
            irundelay  : 4;
      };
    };
  };

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
                         : 1,
                chm      : 1;
        uint8_t tbl      : 2;
        bool             : 1,
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
        uint8_t    sgt    : 7;
        bool              : 1;
        bool       sfilt  : 1;
        uint8_t           : 7;
      };
    };
  };

  struct PWMCONF_t {
    constexpr static uint8_t address = 0x70;
    union {
      uint32_t sr;
      struct {
        uint8_t pwm_ofs            : 8,
                pwm_grad           : 8,
                pwm_freq           : 2;
        bool    pwm_autoscale      : 1,
                pwm_autograd       : 1;
        uint8_t freewheel          : 2;
        bool    pwm_meas_sd_enable : 1,
                pwm_dis_reg_stst   : 1;
        uint8_t pwm_reg            : 4,
                pwm_lim            : 4;
      };
    };
  };

}

#pragma pack(pop)

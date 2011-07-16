#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 151 "/opt/tinyos-2.1.1/msp430-z1/lib/gcc-lib/msp430/3.2.3/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef int wchar_t;
# 8 "/usr/local/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
}  ;
#line 14
struct __nesc_attr_one_nok {
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
}  ;
# 38 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/sys/inttypes.h" 3
typedef signed char int8_t;
typedef unsigned char uint8_t;

typedef int int16_t;
typedef unsigned int uint16_t;

typedef long int32_t;
typedef unsigned long uint32_t;

typedef long long int64_t;
typedef unsigned long long uint64_t;




typedef int16_t intptr_t;
typedef uint16_t uintptr_t;
# 235 "/usr/local/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void * source)  ;




static __inline uint8_t __nesc_hton_uint8(void * target, uint8_t value)  ;
#line 257
static __inline int8_t __nesc_hton_int8(void * target, int8_t value)  ;






static __inline uint16_t __nesc_ntoh_uint16(const void * source)  ;




static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;
#line 301
static __inline uint32_t __nesc_hton_uint32(void * target, uint32_t value)  ;
#line 385
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 41 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 40 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/string.h" 3
extern void *memset(void *arg_0x2ae8cb341980, int arg_0x2ae8cb341be8, size_t arg_0x2ae8cb33f020);
#line 61
extern void *memset(void *arg_0x2ae8cb35ab10, int arg_0x2ae8cb35ad78, size_t arg_0x2ae8cb359060);
# 59 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/stdlib.h" 3
#line 55
typedef struct __nesc_unnamed4242 {

  int quot;
  int rem;
} div_t;







#line 63
typedef struct __nesc_unnamed4243 {

  long quot;
  long rem;
} ldiv_t;
# 122 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 28 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/sys/reent.h" 3
typedef __uint32_t __ULong;


struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x2ae8cb394b58);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x2ae8cb397c10);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 208
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 261
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 23 "/opt/tinyos-2.1.1/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 24
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;







struct __nesc_attr_atmostonce {
};
#line 35
struct __nesc_attr_atleastonce {
};
#line 36
struct __nesc_attr_exactlyonce {
};
# 40 "/opt/tinyos-2.1.1/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 39 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/msp430/iostructures.h" 3
#line 27
typedef union port {
  volatile unsigned char reg_p;
  volatile struct __nesc_unnamed4249 {
    unsigned char __p0 : 1, 
    __p1 : 1, 
    __p2 : 1, 
    __p3 : 1, 
    __p4 : 1, 
    __p5 : 1, 
    __p6 : 1, 
    __p7 : 1;
  } __pin;
} __attribute((packed))  ioregister_t;
#line 108
struct port_full_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t ifg;
  ioregister_t ies;
  ioregister_t ie;
  ioregister_t sel;

  ioregister_t ren;
};







struct port_simple_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};




struct port_full_t;



struct port_full_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;
# 254 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/msp430/usci.h" 3
volatile unsigned char UCA0CTL1 __asm ("0x0061");

volatile unsigned char UCA0BR0 __asm ("0x0062");

volatile unsigned char UCA0BR1 __asm ("0x0063");

volatile unsigned char UCA0MCTL __asm ("0x0064");



volatile unsigned char UCA0RXBUF __asm ("0x0066");
#line 277
volatile unsigned char UCB0CTL1 __asm ("0x0069");









volatile unsigned char UCB0RXBUF __asm ("0x006E");
#line 302
volatile unsigned char UCA1CTL1 __asm ("0x00D1");
#line 325
volatile unsigned char UCB1CTL1 __asm ("0x00D9");
# 28 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/msp430/timera.h" 3
volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0R __asm ("0x0170");


volatile unsigned int TA0CCTL0 __asm ("0x0162");

volatile unsigned int TA0CCTL1 __asm ("0x0164");
#line 71
volatile unsigned int TA0CCTL2 __asm ("0x0166");
#line 132
#line 123
typedef struct __nesc_unnamed4250 {
  volatile unsigned 
  taifg : 1, 
  taie : 1, 
  taclr : 1, 
  dummy : 1, 
  tamc : 2, 
  taid : 2, 
  tassel : 2;
} __attribute((packed))  tactl_t;
#line 148
#line 134
typedef struct __nesc_unnamed4251 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  dummy : 1, 
  scci : 1, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tacctl_t;



struct timera_t {
  tactl_t ctl;
  tacctl_t cctl0;
  tacctl_t cctl1;

  tacctl_t cctl2;



  volatile unsigned dummy2[4];
  volatile unsigned tar;
  volatile unsigned taccr0;
  volatile unsigned taccr1;

  volatile unsigned taccr2;
};




struct timera_t;
# 26 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/msp430/timerb.h" 3
volatile unsigned int TBR __asm ("0x0190");
#line 76
#line 64
typedef struct __nesc_unnamed4252 {
  volatile unsigned 
  tbifg : 1, 
  tbie : 1, 
  tbclr : 1, 
  dummy1 : 1, 
  tbmc : 2, 
  tbid : 2, 
  tbssel : 2, 
  dummy2 : 1, 
  tbcntl : 2, 
  tbclgrp : 2;
} __attribute((packed))  tbctl_t;
#line 91
#line 78
typedef struct __nesc_unnamed4253 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  clld : 2, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tbcctl_t;


struct timerb_t {
  tbctl_t ctl;
  tbcctl_t cctl0;
  tbcctl_t cctl1;
  tbcctl_t cctl2;

  tbcctl_t cctl3;
  tbcctl_t cctl4;
  tbcctl_t cctl5;
  tbcctl_t cctl6;



  volatile unsigned tbr;
  volatile unsigned tbccr0;
  volatile unsigned tbccr1;
  volatile unsigned tbccr2;

  volatile unsigned tbccr3;
  volatile unsigned tbccr4;
  volatile unsigned tbccr5;
  volatile unsigned tbccr6;
};





struct timerb_t;
# 20 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/msp430/basic_clock.h" 3
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");
# 18 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/msp430/adc12.h" 3
volatile unsigned int ADC12CTL0 __asm ("0x01A0");

volatile unsigned int ADC12CTL1 __asm ("0x01A2");
#line 42
#line 30
typedef struct __nesc_unnamed4254 {
  volatile unsigned 
  adc12sc : 1, 
  enc : 1, 
  adc12tovie : 1, 
  adc12ovie : 1, 
  adc12on : 1, 
  refon : 1, 
  r2_5v : 1, 
  msc : 1, 
  sht0 : 4, 
  sht1 : 4;
} __attribute((packed))  adc12ctl0_t;
#line 54
#line 44
typedef struct __nesc_unnamed4255 {
  volatile unsigned 
  adc12busy : 1, 
  conseq : 2, 
  adc12ssel : 2, 
  adc12div : 3, 
  issh : 1, 
  shp : 1, 
  shs : 2, 
  cstartadd : 4;
} __attribute((packed))  adc12ctl1_t;
#line 74
#line 56
typedef struct __nesc_unnamed4256 {
  volatile unsigned 
  bit0 : 1, 
  bit1 : 1, 
  bit2 : 1, 
  bit3 : 1, 
  bit4 : 1, 
  bit5 : 1, 
  bit6 : 1, 
  bit7 : 1, 
  bit8 : 1, 
  bit9 : 1, 
  bit10 : 1, 
  bit11 : 1, 
  bit12 : 1, 
  bit13 : 1, 
  bit14 : 1, 
  bit15 : 1;
} __attribute((packed))  adc12xflg_t;


struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};




struct adc12_t;
# 93 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/msp430x261x.h" 3
volatile unsigned char IFG2 __asm ("0x0003");
#line 156
volatile unsigned char CALDCO_8MHZ __asm ("0x10FC");

volatile unsigned char CALBC1_8MHZ __asm ("0x10FD");
# 195 "/opt/tinyos-2.1.1/tos/chips/msp430X/msp430hardware.h"
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4257 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
#line 250
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
# 48 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestCase.h"
static void assertEqualsFailed(char *failMsg, uint32_t expected, uint32_t actual, uint8_t assertionId);
static void assertNotEqualsFailed(char *failMsg, uint32_t actual, uint8_t assertionId);
static void assertResultIsBelowFailed(char *failMsg, uint32_t upperbound, uint32_t actual, uint8_t assertionId);
static void assertResultIsAboveFailed(char *failMsg, uint32_t lowerbound, uint32_t actual, uint8_t assertionId);
static void assertTunitSuccess(uint8_t assertionId);
static void assertTunitFail(char *failMsg, uint8_t assertionId);


static void setTUnitTestName(char *name);
# 6 "/opt/tinyos-2.1.1/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4258 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4259 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 32 "/opt/tinyos-2.1.1/tos/types/Leds.h"
enum __nesc_unnamed4260 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.h"
enum __nesc_unnamed4261 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 64
#line 51
typedef struct __nesc_unnamed4262 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 76
#line 66
typedef struct __nesc_unnamed4263 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 91
#line 78
typedef struct __nesc_unnamed4264 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 39 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 93
#line 87
typedef nx_struct security_header_t {
  unsigned char __nesc_filler0[1];


  nx_uint32_t frameCounter;
  nx_uint8_t keyID[1];
} __attribute__((packed)) security_header_t;
#line 113
#line 95
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;







  nxle_uint8_t network;


  nxle_uint8_t type;
} __attribute__((packed)) cc2420_header_t;





#line 118
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 143
#line 128
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;
} __attribute__((packed)) 





cc2420_metadata_t;





#line 146
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 179
enum __nesc_unnamed4265 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 28 + MAC_FOOTER_SIZE, 

  CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE, 

  AM_OVERHEAD = 2
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};

enum cc2420_security_enums {
  CC2420_NO_SEC = 0, 
  CC2420_CBC_MAC = 1, 
  CC2420_CTR = 2, 
  CC2420_CCM = 3, 
  NO_SEC = 0, 
  CBC_MAC_4 = 1, 
  CBC_MAC_8 = 2, 
  CBC_MAC_16 = 3, 
  CTR = 4, 
  CCM_4 = 5, 
  CCM_8 = 6, 
  CCM_16 = 7
};


enum __nesc_unnamed4266 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 72 "/opt/tinyos-2.1.1/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4267 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4268 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4269 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 110
#line 98
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 112
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 120
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;



#line 125
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute__((packed)) serial_metadata_t;
# 48 "/opt/tinyos-2.1.1/tos/platforms/z1/platform_message.h"
#line 45
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



#line 50
typedef union TOSRadioFooter {
  cc2420_footer_t cc2420;
} message_footer_t;




#line 54
typedef union TOSRadioMetadata {
  cc2420_metadata_t cc2420;
  serial_metadata_t serial;
} message_metadata_t;
# 19 "/opt/tinyos-2.1.1/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[28];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 59 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessing.h"
#line 50
typedef nx_struct TUnitProcessingMsg {
  nx_uint8_t cmd;
  nx_uint8_t id;
  nx_uint8_t assertionId;
  nx_uint32_t expected;
  nx_uint32_t actual;
  nx_bool lastMsg;
  nx_uint8_t failMsgLength;
  nx_uint8_t failMsg[28 - 13];
} __attribute__((packed)) TUnitProcessingMsg;


enum __nesc_unnamed4270 {
  TUNITPROCESSING_CMD_PING = 0, 
  TUNITPROCESSING_REPLY_PING = 1, 
  TUNITPROCESSING_CMD_RUN = 2, 
  TUNITPROCESSING_REPLY_RUN = 3, 
  TUNITPROCESSING_EVENT_PONG = 4, 
  TUNITPROCESSING_EVENT_TESTRESULT_SUCCESS = 5, 
  TUNITPROCESSING_EVENT_TESTRESULT_FAILED = 6, 
  TUNITPROCESSING_EVENT_TESTRESULT_EQUALS_FAILED = 7, 
  TUNITPROCESSING_EVENT_TESTRESULT_NOTEQUALS_FAILED = 8, 
  TUNITPROCESSING_EVENT_TESTRESULT_BELOW_FAILED = 9, 
  TUNITPROCESSING_EVENT_TESTRESULT_ABOVE_FAILED = 10, 
  TUNITPROCESSING_EVENT_ALLDONE = 11, 
  TUNITPROCESSING_CMD_TEARDOWNONETIME = 12
};

enum __nesc_unnamed4271 {
  AM_TUNITPROCESSINGMSG = 0xFF
};
# 80 "/opt/tinyos-2.1.1/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b);
# 94 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/msp430usci.h"
#line 89
typedef enum __nesc_unnamed4272 {
  USCI_NONE = 0, 
  USCI_UART = 1, 
  USCI_SPI = 2, 
  USCI_I2C = 3
} msp430_uscimode_t;
#line 115
#line 107
typedef struct __nesc_unnamed4273 {
  unsigned int ucsync : 1;
  unsigned int ucmode : 2;
  unsigned int ucspb : 1;
  unsigned int uc7bit : 1;
  unsigned int ucmsb : 1;
  unsigned int ucpar : 1;
  unsigned int ucpen : 1;
} __attribute((packed))  msp430_uctl0_t;
#line 130
#line 122
typedef struct __nesc_unnamed4274 {
  unsigned int ucswrst : 1;
  unsigned int uctxbrk : 1;
  unsigned int uctxaddr : 1;
  unsigned int ucdorm : 1;
  unsigned int ucbrkie : 1;
  unsigned int ucrxeie : 1;
  unsigned int ucssel : 2;
} __attribute((packed))  msp430_uctl1_t;
#line 189
#line 149
typedef enum __nesc_unnamed4275 {
  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x04, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x0c, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x0e, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x06, 

  UBR_1048MHZ_9600 = 0x006D, UMCTL_1048MHZ_9600 = 0x04, 
  UBR_1048MHZ_19200 = 0x0036, UMCTL_1048MHZ_19200 = 0x0a, 
  UBR_1048MHZ_38400 = 0x001B, UMCTL_1048MHZ_38400 = 0x04, 
  UBR_1048MHZ_57600 = 0x0012, UMCTL_1048MHZ_57600 = 0x0c, 
  UBR_1048MHZ_115200 = 0x0009, UMCTL_1048MHZ_115200 = 0x02, 
  UBR_1048MHZ_128000 = 0x0008, UMCTL_1048MHZ_128000 = 0x02, 
  UBR_1048MHZ_256000 = 0x0004, UMCTL_1048MHZ_230400 = 0x02, 








  UBR_1MHZ_9600 = 0x6, UMCTL_1MHZ_9600 = 0x81, 
  UBR_1MHZ_19200 = 0x3, UMCTL_1MHZ_19200 = 0x41, 
  UBR_1MHZ_57600 = 0x1, UMCTL_1MHZ_57600 = 0x0F, 

  UBR_8MHZ_4800 = 0x68, UMCTL_8MHZ_4800 = 0x31, 
  UBR_8MHZ_9600 = 0x34, UMCTL_8MHZ_9600 = 0x11, 
  UBR_8MHZ_19200 = 0x1A, UMCTL_8MHZ_19200 = 0x11, 
  UBR_8MHZ_38400 = 0x0D, UMCTL_8MHZ_38400 = 0x01, 
  UBR_8MHZ_57600 = 0x08, UMCTL_8MHZ_57600 = 0xB1, 
  UBR_8MHZ_115200 = 0x04, UMCTL_8MHZ_115200 = 0x3B, 
  UBR_8MHZ_230400 = 0x02, UMCTL_8MHZ_230400 = 0x27, 

  UBR_16MHZ_4800 = 0xD0, UMCTL_16MHZ_4800 = 0x51, 
  UBR_16MHZ_9600 = 0x68, UMCTL_16MHZ_9600 = 0x31, 
  UBR_16MHZ_19200 = 0x34, UMCTL_16MHZ_19200 = 0x11, 
  UBR_16MHZ_38400 = 0x1A, UMCTL_16MHZ_38400 = 0x11, 
  UBR_16MHZ_57600 = 0x11, UMCTL_16MHZ_57600 = 0x61, 
  UBR_16MHZ_115200 = 0x8, UMCTL_16MHZ_115200 = 0xB1, 
  UBR_16MHZ_230400 = 0x4, UMCTL_16MHZ_230400 = 0x3B
} msp430_uart_rate_t;
#line 215
#line 192
typedef struct __nesc_unnamed4276 {
  unsigned int ubr : 16;
  unsigned int umctl : 8;


  unsigned int  : 1;
  unsigned int ucmode : 2;
  unsigned int ucspb : 1;
  unsigned int uc7bit : 1;
  unsigned int  : 1;
  unsigned int ucpar : 1;
  unsigned int ucpen : 1;


  unsigned int  : 5;
  unsigned int ucrxeie : 1;
  unsigned int ucssel : 2;




  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;







#line 217
typedef struct __nesc_unnamed4277 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl0;
  uint8_t uctl1;
  uint8_t ume;
} msp430_uart_registers_t;




#line 225
typedef union __nesc_unnamed4278 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;


const msp430_uart_union_config_t msp430_uart_default_config = { { 
.ubr = UBR_8MHZ_115200, 
.umctl = UMCTL_8MHZ_115200, 
.ucmode = 0, 
.ucspb = 0, 
.uc7bit = 0, 
.ucpar = 0, 
.ucpen = 0, 
.ucrxeie = 0, 
.ucssel = 2, 
.utxe = 1, 
.urxe = 1 } };
#line 269
#line 252
typedef struct __nesc_unnamed4279 {
  unsigned int ubr : 16;


  unsigned int  : 1;
  unsigned int ucmode : 2;
  unsigned int ucmst : 1;
  unsigned int uc7bit : 1;
  unsigned int ucmsb : 1;
  unsigned int ucckpl : 1;
  unsigned int ucckph : 1;



  unsigned int  : 1;
  unsigned int  : 5;
  unsigned int ucssel : 2;
} msp430_spi_config_t;






#line 272
typedef struct __nesc_unnamed4280 {
  uint16_t ubr;
  uint8_t uctl0;
  uint8_t uctl1;
} msp430_spi_registers_t;




#line 278
typedef union __nesc_unnamed4281 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;
#line 310
#line 302
typedef struct __nesc_unnamed4282 {
  unsigned int  : 1;
  unsigned int ucmode : 2;
  unsigned int ucmst : 1;
  unsigned int  : 1;
  unsigned int ucmm : 1;
  unsigned int ucsla10 : 1;
  unsigned int uca10 : 1;
} __attribute((packed))  msp430_i2cctl0_t;
#line 325
#line 317
typedef struct __nesc_unnamed4283 {
  unsigned int ucswrst : 1;
  unsigned int uctxstt : 1;
  unsigned int uctxstp : 1;
  unsigned int uctxnack : 1;
  unsigned int uctr : 1;
  unsigned int  : 1;
  unsigned int ucssel : 2;
} __attribute((packed))  msp430_i2cctl1_t;
#line 353
#line 328
typedef struct __nesc_unnamed4284 {
  uint16_t ubr : 16;


  uint8_t  : 1;
  uint8_t ucmode : 2;
  uint8_t ucmst : 1;
  uint8_t  : 1;
  uint8_t ucmm : 1;
  uint8_t ucsla10 : 1;
  uint8_t uca10 : 1;


  uint8_t  : 1;
  uint8_t  : 1;
  uint8_t  : 1;
  uint8_t  : 1;
  uint8_t uctr : 1;
  uint8_t  : 1;
  uint8_t ucssel : 2;


  uint16_t i2coa : 10;
  uint8_t  : 5;
  uint8_t ucgcen : 1;
} msp430_i2c_config_t;






#line 355
typedef struct __nesc_unnamed4285 {
  uint16_t ubr;
  uint8_t uctl0;
  uint8_t uctl1;
  uint16_t ui2coa;
} msp430_i2c_registers_t;




#line 362
typedef union __nesc_unnamed4286 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
# 29 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4287 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4288 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4289 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 33 "/opt/tinyos-2.1.1/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
enum SerialAMQueueP____nesc_unnamed4290 {
  SerialAMQueueP__NUM_CLIENTS = 1U
};
enum /*PlatformSerialC.UartC*/Msp430Uart0C__0____nesc_unnamed4291 {
  Msp430Uart0C__0__CLIENT_ID = 0U
};
typedef T32khz /*Msp430Uart0P.UartP*/Msp430UartP__0__Counter__precision_tag;
typedef uint16_t /*Msp430Uart0P.UartP*/Msp430UartP__0__Counter__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
enum /*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0____nesc_unnamed4292 {
  Msp430UsciA0C__0__CLIENT_ID = 1U
};
enum /*FirstTestC.BasicAssertionTestC*/TestCaseC__0____nesc_unnamed4293 {
  TestCaseC__0__TUNIT_TEST_ID = 0U
};
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void TUnitP__waitForSendDone__runTask(void );
# 38 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
static void TUnitP__TearDownOneTime__default__run(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void TUnitP__begin__runTask(void );
# 38 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
static void TUnitP__TearDown__default__run(void );
# 39 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/TestCase.nc"
static void TUnitP__TestCase__default__run(
# 75 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
uint8_t arg_0x2ae8cb6666e8);
# 41 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/TestCase.nc"
static void TUnitP__TestCase__done(
# 75 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
uint8_t arg_0x2ae8cb6666e8);
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t TUnitP__Init__init(void );
# 61 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitProcessing.nc"
static void TUnitP__TUnitProcessing__tearDownOneTime(void );
#line 57
static void TUnitP__TUnitProcessing__run(void );

static void TUnitP__TUnitProcessing__ping(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void TUnitP__runDone__runTask(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static void TUnitP__SerialSplitControl__startDone(error_t error);
#line 117
static void TUnitP__SerialSplitControl__stopDone(error_t error);
# 38 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
static void TUnitP__SetUp__default__run(void );
#line 38
static void TUnitP__SetUpOneTime__default__run(void );
# 46 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/StatsQuery.nc"
static bool TUnitP__StatsQuery__default__isIdle(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
# 35 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 32
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 30
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 40 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
uint8_t arg_0x2ae8cb7b3690);
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 40 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
uint8_t arg_0x2ae8cb7b3690);
# 33 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 31 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 45 "/opt/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2ae8cb614a18);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 45 "/opt/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2ae8cb614a18);
# 46 "/opt/tinyos-2.1.1/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 61
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 54
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 54 "/opt/tinyos-2.1.1/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__default__lowestState(void );
# 59 "/opt/tinyos-2.1.1/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static void Link_TUnitProcessingP__SerialEventSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 49 "/opt/tinyos-2.1.1/tos/interfaces/Boot.nc"
static void Link_TUnitProcessingP__Boot__booted(void );
# 47 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitProcessing.nc"
static void Link_TUnitProcessingP__TUnitProcessing__testResultIsAboveFailed(uint8_t testId, char *failMsg, uint32_t lowerbound, uint32_t actual, uint8_t assertionId);
#line 41
static void Link_TUnitProcessingP__TUnitProcessing__testEqualsFailed(uint8_t testId, char *failMsg, uint32_t expected, uint32_t actual, uint8_t assertionId);



static void Link_TUnitProcessingP__TUnitProcessing__testResultIsBelowFailed(uint8_t testId, char *failMsg, uint32_t upperbound, uint32_t actual, uint8_t assertionId);



static void Link_TUnitProcessingP__TUnitProcessing__testFailed(uint8_t testId, char *failMsg, uint8_t assertionId);
#line 43
static void Link_TUnitProcessingP__TUnitProcessing__testNotEqualsFailed(uint8_t testId, char *failMsg, uint32_t actual, uint8_t assertionId);
#line 39
static void Link_TUnitProcessingP__TUnitProcessing__testSuccess(uint8_t testId, uint8_t assertionId);
#line 52
static void Link_TUnitProcessingP__TUnitProcessing__allDone(void );

static void Link_TUnitProcessingP__TUnitProcessing__pong(void );
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



Link_TUnitProcessingP__SerialReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static void Link_TUnitProcessingP__SerialSplitControl__startDone(error_t error);
#line 117
static void Link_TUnitProcessingP__SerialSplitControl__stopDone(error_t error);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void Link_TUnitProcessingP__sendEventMsg__runTask(void );
#line 64
static void Link_TUnitProcessingP__allDone__runTask(void );
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static error_t /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
#line 124
static 
#line 122
void * 

/*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__getPayload(
#line 121
message_t * msg, 


uint8_t len);
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(
# 40 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
am_id_t arg_0x2ae8cb9b4340, 
# 92 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(
# 38 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2ae8cb9b51c8, 
# 56 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 114
static 
#line 112
void * 

/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__getPayload(
# 38 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2ae8cb9b51c8, 
# 111 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 


uint8_t len);
#line 89
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(
# 38 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2ae8cb9b51c8, 
# 85 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );
#line 64
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(
# 36 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x2ae8cba0cba8, 
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
#line 124
static 
#line 122
void * 

/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__getPayload(
# 36 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x2ae8cba0cba8, 
# 121 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
message_t * msg, 


uint8_t len);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
static uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(
#line 63
message_t * msg);
#line 115
static 
#line 112
void * 


/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
#line 95
static uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void );
#line 83
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(
# 37 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x2ae8cba09cd0, 
# 60 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(
#line 63
message_t * amsg);
#line 92
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 136
static am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(
#line 132
message_t * amsg);
#line 151
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 83 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static error_t SerialP__SplitControl__start(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void SerialP__stopDoneTask__runTask(void );
#line 64
static void SerialP__RunTx__runTask(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t SerialP__Init__init(void );
# 43 "/opt/tinyos-2.1.1/tos/lib/serial/SerialFlush.nc"
static void SerialP__SerialFlush__flushDone(void );
#line 38
static void SerialP__SerialFlush__default__flush(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void SerialP__startDoneTask__runTask(void );
# 83 "/opt/tinyos-2.1.1/tos/lib/serial/SerialFrameComm.nc"
static void SerialP__SerialFrameComm__dataReceived(uint8_t data);





static void SerialP__SerialFrameComm__putDone(void );
#line 74
static void SerialP__SerialFrameComm__delimiterReceived(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void SerialP__defaultSerialFlushTask__runTask(void );
# 60 "/opt/tinyos-2.1.1/tos/lib/serial/SendBytePacket.nc"
static error_t SerialP__SendBytePacket__completeSend(void );
#line 51
static error_t SerialP__SendBytePacket__startSend(uint8_t first_byte);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(
# 40 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2ae8cbb56238, 
# 56 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 89
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(
# 40 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2ae8cbb56238, 
# 85 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void );
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(
# 39 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2ae8cbb576e8, 
# 60 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(
# 43 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2ae8cbb55570, 
# 31 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(
# 43 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2ae8cbb55570);
# 23 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(
# 43 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2ae8cbb55570, 
# 23 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 70 "/opt/tinyos-2.1.1/tos/lib/serial/SendBytePacket.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void );









static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error);
# 51 "/opt/tinyos-2.1.1/tos/lib/serial/ReceiveBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void );






static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t data);










static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result);
# 79 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
static void HdlcTranslateC__UartStream__receivedByte(uint8_t byte);
#line 99
static void HdlcTranslateC__UartStream__receiveDone(
#line 95
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void HdlcTranslateC__UartStream__sendDone(
#line 53
uint8_t * buf, 



uint16_t len, error_t error);
# 45 "/opt/tinyos-2.1.1/tos/lib/serial/SerialFrameComm.nc"
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void );
#line 68
static void HdlcTranslateC__SerialFrameComm__resetReceive(void );
#line 54
static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data);
# 55 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(
# 76 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4f9b0);
# 49 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__ResourceConfigure__configure(
# 76 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4f9b0);
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciInterrupts__rxDone(
# 83 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc5b930, 
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
uint8_t data);
#line 80
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciInterrupts__txDone(
# 83 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc5b930);
# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart0P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(
# 81 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4acd0);
# 48 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__send(
# 77 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4e7e0, 
# 44 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len);
#line 79
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(
# 77 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4e7e0, 
# 79 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(
# 77 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4e7e0, 
# 95 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__sendDone(
# 77 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4e7e0, 
# 53 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 71 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Counter__overflow(void );
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__release(
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc50688);
# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__immediateRequest(
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc50688);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__default__granted(
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc50688);
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__release(
# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4ba18);
# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__immediateRequest(
# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4ba18);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__granted(
# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4ba18);
# 118 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static bool /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__isOwner(
# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4ba18);
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
static void HplMsp430UsciA0P__UsciRawInterrupts__rxDone(uint8_t data);
#line 80
static void HplMsp430UsciA0P__UsciRawInterrupts__txDone(void );
# 158 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
static void HplMsp430UsciA0P__Usci__enableUart(void );
#line 92
static void HplMsp430UsciA0P__Usci__resetUsci(bool reset);
#line 121
static void HplMsp430UsciA0P__Usci__disableIntr(void );
#line 75
static void HplMsp430UsciA0P__Usci__setUmctl(uint8_t umctl);
#line 124
static void HplMsp430UsciA0P__Usci__enableIntr(void );





static void HplMsp430UsciA0P__Usci__clrIntr(void );
#line 71
static void HplMsp430UsciA0P__Usci__setUbr(uint16_t ubr);
#line 139
static void HplMsp430UsciA0P__Usci__tx(uint8_t data);
#line 159
static void HplMsp430UsciA0P__Usci__disableUart(void );





static void HplMsp430UsciA0P__Usci__setModeUart(msp430_uart_union_config_t *config);
#line 128
static void HplMsp430UsciA0P__Usci__clrTxIntr(void );
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectModuleFunc(void );
#line 71
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );
#line 71
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );
#line 71
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
static void HplMsp430UsciAB0RawInterruptsP__UsciB__default__rxDone(uint8_t data);
#line 80
static void HplMsp430UsciAB0RawInterruptsP__UsciB__default__txDone(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 35 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );





static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );





static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(
# 70 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x2ae8cbfc6410, 
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
uint8_t data);
#line 80
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(
# 70 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x2ae8cbfc6410);
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 80
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );
# 43 "/opt/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
static bool /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(
# 55 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ae8cc001020);
# 55 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(
# 60 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ae8cbfff340);
# 49 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ae8cbfff340);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 46
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );
#line 81
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void );
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(
# 54 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ae8cc003d40);
# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(
# 54 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ae8cc003d40);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ae8cc003d40);
# 118 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static bool /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(
# 54 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ae8cc003d40);
# 80 "/opt/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartConfigure.nc"
static msp430_uart_union_config_t *Z1SerialP__Msp430UartConfigure__getConfig(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void Z1SerialP__Resource__granted(void );
# 74 "/opt/tinyos-2.1.1/tos/interfaces/StdControl.nc"
static error_t Z1SerialP__StdControl__start(void );









static error_t Z1SerialP__StdControl__stop(void );
# 31 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t SerialPacketInfoActiveMessageP__Info__offset(void );







static uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen);
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t StateImplP__Init__init(void );
# 71 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static uint8_t StateImplP__State__getState(
# 67 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
uint8_t arg_0x2ae8cc0c3be8);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static void StateImplP__State__toIdle(
# 67 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
uint8_t arg_0x2ae8cc0c3be8);
# 66 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static bool StateImplP__State__isState(
# 67 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
uint8_t arg_0x2ae8cc0c3be8, 
# 66 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
uint8_t myState);
#line 61
static bool StateImplP__State__isIdle(
# 67 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
uint8_t arg_0x2ae8cc0c3be8);
# 45 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static error_t StateImplP__State__requestState(
# 67 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
uint8_t arg_0x2ae8cc0c3be8, 
# 45 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
uint8_t reqState);





static void StateImplP__State__forceState(
# 67 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
uint8_t arg_0x2ae8cc0c3be8, 
# 51 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
uint8_t reqState);
# 44 "/opt/tinyos-2.1.1/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 50 "/opt/tinyos-2.1.1/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );
# 39 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/TestCase.nc"
static void FirstTestP__BasicAssertionTest__run(void );
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t TUnitP__waitForSendDone__postTask(void );
# 38 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
static void TUnitP__TearDownOneTime__run(void );
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t TUnitP__begin__postTask(void );
# 61 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static bool TUnitP__SendState__isIdle(void );
# 38 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
static void TUnitP__TearDown__run(void );
# 50 "/opt/tinyos-2.1.1/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t TUnitP__ActiveMessageAddress__amAddress(void );
# 39 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/TestCase.nc"
static void TUnitP__TestCase__run(
# 75 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
uint8_t arg_0x2ae8cb6666e8);
# 47 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitProcessing.nc"
static void TUnitP__TUnitProcessing__testResultIsAboveFailed(uint8_t testId, char *failMsg, uint32_t lowerbound, uint32_t actual, uint8_t assertionId);
#line 41
static void TUnitP__TUnitProcessing__testEqualsFailed(uint8_t testId, char *failMsg, uint32_t expected, uint32_t actual, uint8_t assertionId);



static void TUnitP__TUnitProcessing__testResultIsBelowFailed(uint8_t testId, char *failMsg, uint32_t upperbound, uint32_t actual, uint8_t assertionId);



static void TUnitP__TUnitProcessing__testFailed(uint8_t testId, char *failMsg, uint8_t assertionId);
#line 43
static void TUnitP__TUnitProcessing__testNotEqualsFailed(uint8_t testId, char *failMsg, uint32_t actual, uint8_t assertionId);
#line 39
static void TUnitP__TUnitProcessing__testSuccess(uint8_t testId, uint8_t assertionId);
#line 52
static void TUnitP__TUnitProcessing__allDone(void );

static void TUnitP__TUnitProcessing__pong(void );
# 71 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static uint8_t TUnitP__TestState__getState(void );
#line 56
static void TUnitP__TestState__toIdle(void );
#line 45
static error_t TUnitP__TestState__requestState(uint8_t reqState);





static void TUnitP__TestState__forceState(uint8_t reqState);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t TUnitP__runDone__postTask(void );
# 71 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static uint8_t TUnitP__TUnitState__getState(void );
#line 61
static bool TUnitP__TUnitState__isIdle(void );
#line 51
static void TUnitP__TUnitState__forceState(uint8_t reqState);
# 38 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
static void TUnitP__SetUp__run(void );
#line 38
static void TUnitP__SetUpOneTime__run(void );
# 46 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/StatsQuery.nc"
static bool TUnitP__StatsQuery__isIdle(void );
# 131 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
enum TUnitP____nesc_unnamed4294 {
#line 131
  TUnitP__begin = 0U
};
#line 131
typedef int TUnitP____nesc_sillytask_begin[TUnitP__begin];
enum TUnitP____nesc_unnamed4295 {
#line 132
  TUnitP__waitForSendDone = 1U
};
#line 132
typedef int TUnitP____nesc_sillytask_waitForSendDone[TUnitP__waitForSendDone];
enum TUnitP____nesc_unnamed4296 {
#line 133
  TUnitP__runDone = 2U
};
#line 133
typedef int TUnitP____nesc_sillytask_runDone[TUnitP__runDone];
#line 98
uint8_t TUnitP__currentTest;





bool TUnitP__driver;




enum TUnitP____nesc_unnamed4297 {
  TUnitP__S_NOT_BOOTED, 
  TUnitP__S_READY, 
  TUnitP__S_RUNNING
};




enum TUnitP____nesc_unnamed4298 {
  TUnitP__S_IDLE, 
  TUnitP__S_SETUP_ONETIME, 

  TUnitP__S_SETUP, 
  TUnitP__S_RUN, 
  TUnitP__S_TEARDOWN, 

  TUnitP__S_TEARDOWN_ONETIME
};







static void TUnitP__setUpOneTimeDone(void );
static inline void TUnitP__setUpDone(void );
static inline void TUnitP__tearDownDone(void );
static inline void TUnitP__tearDownOneTimeDone(void );
static void TUnitP__attemptTest(void );
static inline void TUnitP__dummyCalls(void );


static inline error_t TUnitP__Init__init(void );










static inline void TUnitP__SerialSplitControl__startDone(error_t error);
#line 170
static inline void TUnitP__SerialSplitControl__stopDone(error_t error);



static inline void TUnitP__TUnitProcessing__run(void );





static inline void TUnitP__TUnitProcessing__ping(void );



static inline void TUnitP__TUnitProcessing__tearDownOneTime(void );







static inline void TUnitP__TestCase__done(uint8_t testId);




void assertEqualsFailed(char *failMsg, uint32_t expected, uint32_t actual, uint8_t assertionId) __attribute((noinline))   ;





void assertNotEqualsFailed(char *failMsg, uint32_t actual, uint8_t assertionId) __attribute((noinline))   ;





void assertResultIsBelowFailed(char *failMsg, uint32_t upperbound, uint32_t actual, uint8_t assertionId) __attribute((noinline))   ;





void assertResultIsAboveFailed(char *failMsg, uint32_t lowerbound, uint32_t actual, uint8_t assertionId) __attribute((noinline))   ;





void assertTunitSuccess(uint8_t assertionId) __attribute((noinline))   ;





void assertTunitFail(char *failMsg, uint8_t assertionId) __attribute((noinline))   ;








void setTUnitTestName(char *name) __attribute((noinline))   ;
#line 267
static void TUnitP__setUpOneTimeDone(void );
#line 279
static inline void TUnitP__setUpDone(void );







static inline void TUnitP__tearDownDone(void );







static inline void TUnitP__tearDownOneTimeDone(void );





static void TUnitP__attemptTest(void );
#line 319
static inline void TUnitP__dummyCalls(void );
#line 339
static inline void TUnitP__begin__runTask(void );









static inline void TUnitP__waitForSendDone__runTask(void );
#line 361
static inline void TUnitP__runDone__runTask(void );








static inline void TUnitP__SetUpOneTime__default__run(void );



static inline void TUnitP__SetUp__default__run(void );



static inline void TUnitP__TestCase__default__run(uint8_t testId);



static inline void TUnitP__TearDown__default__run(void );



static inline void TUnitP__TearDownOneTime__default__run(void );







static inline bool TUnitP__StatsQuery__default__isIdle(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t PlatformP__Msp430ClockInit__init(void );
#line 51
static error_t PlatformP__LedsInit__init(void );
# 9 "/opt/tinyos-2.1.1/tos/platforms/z1/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 32 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 30
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 73 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TA0CTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TA0IV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4299 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};
#line 98
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 133
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 148
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 168
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 196
static inline void Msp430ClockP__startTimerB(void );
#line 262
static inline error_t Msp430ClockP__Init__init(void );
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 40 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
uint8_t arg_0x2ae8cb7b3690);
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 115 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 40 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
uint8_t arg_0x2ae8cb7b3690);
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 115 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;


static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;


static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;


static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(50)))  ;









void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(48)))  ;









void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(58)))  ;









void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(56)))  ;
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 49 "/opt/tinyos-2.1.1/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 46 "/opt/tinyos-2.1.1/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 61
static void RealMainP__Scheduler__taskLoop(void );
#line 54
static bool RealMainP__Scheduler__runNextTask(void );
# 52 "/opt/tinyos-2.1.1/tos/system/RealMainP.nc"
int main(void )   ;
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 45 "/opt/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2ae8cb614a18);
# 59 "/opt/tinyos-2.1.1/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 50 "/opt/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4300 {

  SchedulerBasicP__NUM_TASKS = 14U, 
  SchedulerBasicP__NO_TASK = 255
};

uint8_t SchedulerBasicP__m_head;
uint8_t SchedulerBasicP__m_tail;
uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static __inline uint8_t SchedulerBasicP__popTask(void );
#line 86
static inline bool SchedulerBasicP__isWaiting(uint8_t id);




static inline bool SchedulerBasicP__pushTask(uint8_t id);
#line 113
static inline void SchedulerBasicP__Scheduler__init(void );









static bool SchedulerBasicP__Scheduler__runNextTask(void );
#line 138
static inline void SchedulerBasicP__Scheduler__taskLoop(void );
#line 159
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 54 "/opt/tinyos-2.1.1/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 53 "/opt/tinyos-2.1.1/tos/chips/msp430X/McuSleepC.nc"
bool McuSleepC__dirty = TRUE;
mcu_power_t McuSleepC__powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC__msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC__getPowerState(void );
#line 102
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
#line 133
static inline mcu_power_t McuSleepC__McuPowerOverride__default__lowestState(void );
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static error_t Link_TUnitProcessingP__SerialEventSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
#line 124
static 
#line 122
void * 

Link_TUnitProcessingP__SerialEventSend__getPayload(
#line 121
message_t * msg, 


uint8_t len);
# 71 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static uint8_t Link_TUnitProcessingP__SerialState__getState(void );
#line 51
static void Link_TUnitProcessingP__SerialState__forceState(uint8_t reqState);




static void Link_TUnitProcessingP__SendState__toIdle(void );




static bool Link_TUnitProcessingP__SendState__isIdle(void );
#line 51
static void Link_TUnitProcessingP__SendState__forceState(uint8_t reqState);
# 61 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitProcessing.nc"
static void Link_TUnitProcessingP__TUnitProcessing__tearDownOneTime(void );
#line 57
static void Link_TUnitProcessingP__TUnitProcessing__run(void );

static void Link_TUnitProcessingP__TUnitProcessing__ping(void );
# 83 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static error_t Link_TUnitProcessingP__SerialSplitControl__start(void );
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t Link_TUnitProcessingP__sendEventMsg__postTask(void );
#line 56
static error_t Link_TUnitProcessingP__allDone__postTask(void );
# 85 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
enum Link_TUnitProcessingP____nesc_unnamed4301 {
#line 85
  Link_TUnitProcessingP__sendEventMsg = 3U
};
#line 85
typedef int Link_TUnitProcessingP____nesc_sillytask_sendEventMsg[Link_TUnitProcessingP__sendEventMsg];
enum Link_TUnitProcessingP____nesc_unnamed4302 {
#line 86
  Link_TUnitProcessingP__allDone = 4U
};
#line 86
typedef int Link_TUnitProcessingP____nesc_sillytask_allDone[Link_TUnitProcessingP__allDone];
#line 55
message_t Link_TUnitProcessingP__eventMsg[5];


uint8_t Link_TUnitProcessingP__writingEventMsg;


uint8_t Link_TUnitProcessingP__sendingEventMsg;




enum Link_TUnitProcessingP____nesc_unnamed4303 {
  Link_TUnitProcessingP__S_OFF, 
  Link_TUnitProcessingP__S_ON
};




enum Link_TUnitProcessingP____nesc_unnamed4304 {
  Link_TUnitProcessingP__S_IDLE, 
  Link_TUnitProcessingP__S_BUSY
};

enum Link_TUnitProcessingP____nesc_unnamed4305 {
  Link_TUnitProcessingP__EMPTY = 0xFF
};


static inline void Link_TUnitProcessingP__execute(TUnitProcessingMsg *inMsg);



static error_t Link_TUnitProcessingP__insert(uint8_t cmd, uint8_t testId, char *failMsg, uint32_t expected, uint32_t actual, uint8_t assertionId);
static void Link_TUnitProcessingP__attemptEventSend(void );


static inline void Link_TUnitProcessingP__Boot__booted(void );










static inline void Link_TUnitProcessingP__SerialSplitControl__startDone(error_t error);



static inline void Link_TUnitProcessingP__SerialSplitControl__stopDone(error_t error);




static inline message_t *Link_TUnitProcessingP__SerialReceive__receive(message_t *msg, void *payload, uint8_t len);





static void Link_TUnitProcessingP__SerialEventSend__sendDone(message_t *msg, error_t error);










static inline void Link_TUnitProcessingP__TUnitProcessing__testSuccess(uint8_t testId, uint8_t assertionId);



static inline void Link_TUnitProcessingP__TUnitProcessing__testEqualsFailed(uint8_t testId, char *failMsg, uint32_t expected, uint32_t actual, uint8_t assertionId);



static inline void Link_TUnitProcessingP__TUnitProcessing__testNotEqualsFailed(uint8_t testId, char *failMsg, uint32_t actual, uint8_t assertionId);



static inline void Link_TUnitProcessingP__TUnitProcessing__testResultIsBelowFailed(uint8_t testId, char *failMsg, uint32_t upperbound, uint32_t actual, uint8_t assertionId);



static inline void Link_TUnitProcessingP__TUnitProcessing__testResultIsAboveFailed(uint8_t testId, char *failMsg, uint32_t lowerbound, uint32_t actual, uint8_t assertionId);



static inline void Link_TUnitProcessingP__TUnitProcessing__testFailed(uint8_t testId, char *failMsg, uint8_t assertionId);




static inline void Link_TUnitProcessingP__TUnitProcessing__allDone(void );



static inline void Link_TUnitProcessingP__TUnitProcessing__pong(void );





static inline void Link_TUnitProcessingP__sendEventMsg__runTask(void );








static inline void Link_TUnitProcessingP__allDone__runTask(void );






static inline void Link_TUnitProcessingP__execute(TUnitProcessingMsg *inMsg);
#line 203
static error_t Link_TUnitProcessingP__insert(uint8_t cmd, uint8_t testId, char *failMsg, uint32_t expected, uint32_t actual, uint8_t assertionId);
#line 269
static void Link_TUnitProcessingP__attemptEventSend(void );
# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static void /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(
#line 56
message_t * msg, 







uint8_t len);
#line 114
static 
#line 112
void * 

/*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__Send__getPayload(
#line 111
message_t * msg, 


uint8_t len);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
static void /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 151
static void /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 45 "/opt/tinyos-2.1.1/tos/system/AMQueueEntryP.nc"
static inline error_t /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err);







static inline void */*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__getPayload(message_t *m, uint8_t len);
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(
# 40 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
am_id_t arg_0x2ae8cb9b4340, 
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
#line 124
static 
#line 122
void * 

/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__getPayload(
# 40 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
am_id_t arg_0x2ae8cb9b4340, 
# 121 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
message_t * msg, 


uint8_t len);
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(
# 38 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2ae8cb9b51c8, 
# 85 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
static uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(
#line 63
message_t * msg);
#line 83
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void );
# 67 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
static am_addr_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(
#line 63
message_t * amsg);
#line 136
static am_id_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(
#line 132
message_t * amsg);
# 118 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
enum /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4306 {
#line 118
  AMQueueImplP__0__CancelTask = 5U
};
#line 118
typedef int /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_CancelTask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask];
#line 161
enum /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4307 {
#line 161
  AMQueueImplP__0__errorTask = 6U
};
#line 161
typedef int /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_errorTask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask];
#line 49
#line 47
typedef struct /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4308 {
  message_t * msg;
} /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t;

uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[1];
uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[1 / 8 + 1];

static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );

static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void );
#line 82
static inline error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len);
#line 118
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
#line 155
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err);





static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );




static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );
#line 181
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err);
#line 203
static inline void */*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__getPayload(uint8_t id, message_t *m, uint8_t len);



static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(
# 36 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x2ae8cba0cba8, 
# 92 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(
# 37 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x2ae8cba09cd0, 
# 60 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 49 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(message_t * msg);







static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len);
#line 86
static inline void */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__getPayload(am_id_t id, message_t *m, uint8_t len);



static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(message_t *msg, error_t result);







static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(uint8_t id, message_t *msg, void *payload, uint8_t len);



static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(message_t *msg, void *payload, uint8_t len);








static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(message_t *msg);




static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void );



static void */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(message_t *msg, uint8_t len);
#line 137
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(message_t *amsg);









static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(message_t *amsg, am_addr_t addr);
#line 161
static inline am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(message_t *amsg);




static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(message_t *amsg, am_id_t type);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static void SerialP__SplitControl__startDone(error_t error);
#line 117
static void SerialP__SplitControl__stopDone(error_t error);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t SerialP__stopDoneTask__postTask(void );
# 74 "/opt/tinyos-2.1.1/tos/interfaces/StdControl.nc"
static error_t SerialP__SerialControl__start(void );









static error_t SerialP__SerialControl__stop(void );
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t SerialP__RunTx__postTask(void );
# 38 "/opt/tinyos-2.1.1/tos/lib/serial/SerialFlush.nc"
static void SerialP__SerialFlush__flush(void );
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t SerialP__startDoneTask__postTask(void );
# 45 "/opt/tinyos-2.1.1/tos/lib/serial/SerialFrameComm.nc"
static error_t SerialP__SerialFrameComm__putDelimiter(void );
#line 68
static void SerialP__SerialFrameComm__resetReceive(void );
#line 54
static error_t SerialP__SerialFrameComm__putData(uint8_t data);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t SerialP__defaultSerialFlushTask__postTask(void );
# 70 "/opt/tinyos-2.1.1/tos/lib/serial/SendBytePacket.nc"
static uint8_t SerialP__SendBytePacket__nextByte(void );









static void SerialP__SendBytePacket__sendCompleted(error_t error);
# 51 "/opt/tinyos-2.1.1/tos/lib/serial/ReceiveBytePacket.nc"
static error_t SerialP__ReceiveBytePacket__startPacket(void );






static void SerialP__ReceiveBytePacket__byteReceived(uint8_t data);










static void SerialP__ReceiveBytePacket__endPacket(error_t result);
# 189 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
enum SerialP____nesc_unnamed4309 {
#line 189
  SerialP__RunTx = 7U
};
#line 189
typedef int SerialP____nesc_sillytask_RunTx[SerialP__RunTx];
#line 320
enum SerialP____nesc_unnamed4310 {
#line 320
  SerialP__startDoneTask = 8U
};
#line 320
typedef int SerialP____nesc_sillytask_startDoneTask[SerialP__startDoneTask];





enum SerialP____nesc_unnamed4311 {
#line 326
  SerialP__stopDoneTask = 9U
};
#line 326
typedef int SerialP____nesc_sillytask_stopDoneTask[SerialP__stopDoneTask];








enum SerialP____nesc_unnamed4312 {
#line 335
  SerialP__defaultSerialFlushTask = 10U
};
#line 335
typedef int SerialP____nesc_sillytask_defaultSerialFlushTask[SerialP__defaultSerialFlushTask];
#line 79
enum SerialP____nesc_unnamed4313 {
  SerialP__RX_DATA_BUFFER_SIZE = 2, 
  SerialP__TX_DATA_BUFFER_SIZE = 4, 
  SerialP__SERIAL_MTU = 255, 
  SerialP__SERIAL_VERSION = 1, 
  SerialP__ACK_QUEUE_SIZE = 5
};

enum SerialP____nesc_unnamed4314 {
  SerialP__RXSTATE_NOSYNC, 
  SerialP__RXSTATE_PROTO, 
  SerialP__RXSTATE_TOKEN, 
  SerialP__RXSTATE_INFO, 
  SerialP__RXSTATE_INACTIVE
};

enum SerialP____nesc_unnamed4315 {
  SerialP__TXSTATE_IDLE, 
  SerialP__TXSTATE_PROTO, 
  SerialP__TXSTATE_SEQNO, 
  SerialP__TXSTATE_INFO, 
  SerialP__TXSTATE_FCS1, 
  SerialP__TXSTATE_FCS2, 
  SerialP__TXSTATE_ENDFLAG, 
  SerialP__TXSTATE_ENDWAIT, 
  SerialP__TXSTATE_FINISH, 
  SerialP__TXSTATE_ERROR, 
  SerialP__TXSTATE_INACTIVE
};





#line 109
typedef enum SerialP____nesc_unnamed4316 {
  SerialP__BUFFER_AVAILABLE, 
  SerialP__BUFFER_FILLING, 
  SerialP__BUFFER_COMPLETE
} SerialP__tx_data_buffer_states_t;

enum SerialP____nesc_unnamed4317 {
  SerialP__TX_ACK_INDEX = 0, 
  SerialP__TX_DATA_INDEX = 1, 
  SerialP__TX_BUFFER_COUNT = 2
};






#line 122
typedef struct SerialP____nesc_unnamed4318 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP__RX_DATA_BUFFER_SIZE + 1];
} SerialP__rx_buf_t;




#line 128
typedef struct SerialP____nesc_unnamed4319 {
  uint8_t state;
  uint8_t buf;
} SerialP__tx_buf_t;





#line 133
typedef struct SerialP____nesc_unnamed4320 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP__ACK_QUEUE_SIZE + 1];
} SerialP__ack_queue_t;



SerialP__rx_buf_t SerialP__rxBuf;
SerialP__tx_buf_t SerialP__txBuf[SerialP__TX_BUFFER_COUNT];



uint8_t SerialP__rxState;
uint8_t SerialP__rxByteCnt;
uint8_t SerialP__rxProto;
uint8_t SerialP__rxSeqno;
uint16_t SerialP__rxCRC;



uint8_t SerialP__txState;
uint8_t SerialP__txByteCnt;
uint8_t SerialP__txProto;
uint8_t SerialP__txSeqno;
uint16_t SerialP__txCRC;
uint8_t SerialP__txPending;
uint8_t SerialP__txIndex;


SerialP__ack_queue_t SerialP__ackQ;

bool SerialP__offPending = FALSE;



static __inline void SerialP__txInit(void );
static __inline void SerialP__rxInit(void );
static __inline void SerialP__ackInit(void );

static __inline bool SerialP__ack_queue_is_full(void );
static __inline bool SerialP__ack_queue_is_empty(void );
static __inline void SerialP__ack_queue_push(uint8_t token);
static __inline uint8_t SerialP__ack_queue_top(void );
static inline uint8_t SerialP__ack_queue_pop(void );




static __inline void SerialP__rx_buffer_push(uint8_t data);
static __inline uint8_t SerialP__rx_buffer_top(void );
static __inline uint8_t SerialP__rx_buffer_pop(void );
static __inline uint16_t SerialP__rx_current_crc(void );

static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data);
static void SerialP__MaybeScheduleTx(void );




static __inline void SerialP__txInit(void );
#line 205
static __inline void SerialP__rxInit(void );








static __inline void SerialP__ackInit(void );



static inline error_t SerialP__Init__init(void );
#line 232
static __inline bool SerialP__ack_queue_is_full(void );









static __inline bool SerialP__ack_queue_is_empty(void );





static __inline void SerialP__ack_queue_push(uint8_t token);









static __inline uint8_t SerialP__ack_queue_top(void );









static inline uint8_t SerialP__ack_queue_pop(void );
#line 295
static __inline void SerialP__rx_buffer_push(uint8_t data);



static __inline uint8_t SerialP__rx_buffer_top(void );



static __inline uint8_t SerialP__rx_buffer_pop(void );





static __inline uint16_t SerialP__rx_current_crc(void );










static inline void SerialP__startDoneTask__runTask(void );





static inline void SerialP__stopDoneTask__runTask(void );



static inline void SerialP__SerialFlush__flushDone(void );




static inline void SerialP__defaultSerialFlushTask__runTask(void );


static inline void SerialP__SerialFlush__default__flush(void );



static inline error_t SerialP__SplitControl__start(void );




static void SerialP__testOff(void );
#line 384
static inline void SerialP__SerialFrameComm__delimiterReceived(void );


static inline void SerialP__SerialFrameComm__dataReceived(uint8_t data);



static inline bool SerialP__valid_rx_proto(uint8_t proto);










static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data);
#line 502
static void SerialP__MaybeScheduleTx(void );










static inline error_t SerialP__SendBytePacket__completeSend(void );








static inline error_t SerialP__SendBytePacket__startSend(uint8_t b);
#line 539
static inline void SerialP__RunTx__runTask(void );
#line 642
static inline void SerialP__SerialFrameComm__putDone(void );
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask(void );
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(
# 40 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2ae8cbb56238, 
# 85 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask(void );
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(
# 39 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2ae8cbb576e8, 
# 60 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(
# 43 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2ae8cbb55570, 
# 31 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(
# 43 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2ae8cbb55570);
# 23 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(
# 43 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2ae8cbb55570, 
# 23 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 60 "/opt/tinyos-2.1.1/tos/lib/serial/SendBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend(void );
#line 51
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(uint8_t first_byte);
# 147 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4321 {
#line 147
  SerialDispatcherP__0__signalSendDone = 11U
};
#line 147
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_signalSendDone[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone];
#line 264
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4322 {
#line 264
  SerialDispatcherP__0__receiveTask = 12U
};
#line 264
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_receiveTask[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask];
#line 55
#line 51
typedef enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4323 {
  SerialDispatcherP__0__SEND_STATE_IDLE = 0, 
  SerialDispatcherP__0__SEND_STATE_BEGIN = 1, 
  SerialDispatcherP__0__SEND_STATE_DATA = 2
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__send_state_t;

enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4324 {
  SerialDispatcherP__0__RECV_STATE_IDLE = 0, 
  SerialDispatcherP__0__RECV_STATE_BEGIN = 1, 
  SerialDispatcherP__0__RECV_STATE_DATA = 2
};






#line 63
typedef struct /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4325 {
  uint8_t which : 1;
  uint8_t bufZeroLocked : 1;
  uint8_t bufOneLocked : 1;
  uint8_t state : 2;
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recv_state_t;



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recv_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState = { 0, 0, 0, /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE };
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = TOS_SERIAL_UNKNOWN_ID;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = 0;


message_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[2];
message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[2] = { &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[0], &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[1] };




uint8_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer = (uint8_t * )&/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[0];

uint8_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer = (void *)0;
/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__send_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex = 0;
error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = SUCCESS;
bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled = FALSE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId = 0;


uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = FALSE;
uart_id_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich;
message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf = (void *)0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize = 0;

static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(uint8_t id, message_t *msg, uint8_t len);
#line 147
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void );
#line 167
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void );
#line 183
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error);




static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked(void );



static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer(void );








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(uint8_t which);








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap(void );




static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void );
#line 233
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t b);
#line 264
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void );
#line 285
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result);
#line 347
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(uart_id_t id);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen);




static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len);


static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(uart_id_t idxxx, message_t *msg, error_t error);
# 48 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
static error_t HdlcTranslateC__UartStream__send(
#line 44
uint8_t * buf, 



uint16_t len);
# 83 "/opt/tinyos-2.1.1/tos/lib/serial/SerialFrameComm.nc"
static void HdlcTranslateC__SerialFrameComm__dataReceived(uint8_t data);





static void HdlcTranslateC__SerialFrameComm__putDone(void );
#line 74
static void HdlcTranslateC__SerialFrameComm__delimiterReceived(void );
# 47 "/opt/tinyos-2.1.1/tos/lib/serial/HdlcTranslateC.nc"
#line 44
typedef struct HdlcTranslateC____nesc_unnamed4326 {
  uint8_t sendEscape : 1;
  uint8_t receiveEscape : 1;
} HdlcTranslateC__HdlcState;


HdlcTranslateC__HdlcState HdlcTranslateC__state = { 0, 0 };
uint8_t HdlcTranslateC__txTemp;
uint8_t HdlcTranslateC__m_data;


static inline void HdlcTranslateC__SerialFrameComm__resetReceive(void );





static inline void HdlcTranslateC__UartStream__receivedByte(uint8_t data);
#line 86
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void );





static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data);
#line 104
static void HdlcTranslateC__UartStream__sendDone(uint8_t *buf, uint16_t len, 
error_t error);










static inline void HdlcTranslateC__UartStream__receiveDone(uint8_t *buf, uint16_t len, error_t error);
# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart0P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(
# 81 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4acd0);
# 79 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__receivedByte(
# 77 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4e7e0, 
# 79 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__receiveDone(
# 77 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4e7e0, 
# 95 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__sendDone(
# 77 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4e7e0, 
# 53 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__granted(
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc50688);
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__release(
# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4ba18);
# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__immediateRequest(
# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4ba18);
# 118 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static bool /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__isOwner(
# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2ae8cbc4ba18);
# 92 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__resetUsci(bool reset);
#line 121
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__disableIntr(void );


static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__enableIntr(void );
#line 139
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__tx(uint8_t data);
#line 159
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__disableUart(void );





static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__setModeUart(msp430_uart_union_config_t *config);
#line 128
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__clrTxIntr(void );
# 91 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint16_t /*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_len;
#line 91
uint16_t /*Msp430Uart0P.UartP*/Msp430UartP__0__m_rx_len;
uint8_t * /*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_buf;
#line 92
uint8_t * /*Msp430Uart0P.UartP*/Msp430UartP__0__m_rx_buf;
uint16_t /*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_pos;
#line 93
uint16_t /*Msp430Uart0P.UartP*/Msp430UartP__0__m_rx_pos;
uint8_t /*Msp430Uart0P.UartP*/Msp430UartP__0__m_byte_time;
uint8_t /*Msp430Uart0P.UartP*/Msp430UartP__0__current_owner;

static inline error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__immediateRequest(uint8_t id);
#line 109
static inline error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__release(uint8_t id);







static void /*Msp430Uart0P.UartP*/Msp430UartP__0__ResourceConfigure__configure(uint8_t id);






static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(uint8_t id);





static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__granted(uint8_t id);
#line 163
static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciInterrupts__rxDone(uint8_t id, uint8_t data);
#line 177
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__send(uint8_t id, uint8_t *buf, uint16_t len);
#line 192
static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciInterrupts__txDone(uint8_t id);
#line 239
static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__Counter__overflow(void );

static inline error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__isOwner(uint8_t id);

static inline error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__release(uint8_t id);
static inline msp430_uart_union_config_t */*Msp430Uart0P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__sendDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error);
static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(uint8_t id, uint8_t byte);
static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error);
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void HplMsp430UsciA0P__Interrupts__rxDone(uint8_t data);
#line 80
static void HplMsp430UsciA0P__Interrupts__txDone(void );
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void HplMsp430UsciA0P__URXD__selectIOFunc(void );
#line 78
static void HplMsp430UsciA0P__URXD__selectModuleFunc(void );






static void HplMsp430UsciA0P__UTXD__selectIOFunc(void );
#line 78
static void HplMsp430UsciA0P__UTXD__selectModuleFunc(void );
# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static volatile uint8_t HplMsp430UsciA0P__IE2 __asm ("0x0001");
static volatile uint8_t HplMsp430UsciA0P__IFG2 __asm ("0x0003");
static volatile uint8_t HplMsp430UsciA0P__UCA0CTL0 __asm ("0x0060");
static volatile uint8_t HplMsp430UsciA0P__UCA0CTL1 __asm ("0x0061");
static volatile uint8_t HplMsp430UsciA0P__UCA0TXBUF __asm ("0x0067");

static inline void HplMsp430UsciA0P__UsciRawInterrupts__rxDone(uint8_t temp);



static inline void HplMsp430UsciA0P__UsciRawInterrupts__txDone(void );
#line 111
static inline void HplMsp430UsciA0P__Usci__setUbr(uint16_t control);










static inline void HplMsp430UsciA0P__Usci__setUmctl(uint8_t control);
#line 139
static inline void HplMsp430UsciA0P__Usci__resetUsci(bool reset);
#line 228
static inline void HplMsp430UsciA0P__Usci__clrTxIntr(void );







static inline void HplMsp430UsciA0P__Usci__clrIntr(void );
#line 248
static inline void HplMsp430UsciA0P__Usci__disableIntr(void );
#line 266
static inline void HplMsp430UsciA0P__Usci__enableIntr(void );
#line 291
static inline void HplMsp430UsciA0P__Usci__tx(uint8_t data);
#line 303
static inline void HplMsp430UsciA0P__Usci__enableUart(void );






static inline void HplMsp430UsciA0P__Usci__disableUart(void );






static inline void HplMsp430UsciA0P__configUart(msp430_uart_union_config_t *config);






static inline void HplMsp430UsciA0P__Usci__setModeUart(msp430_uart_union_config_t *config);
# 54 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
static void HplMsp430UsciAB0RawInterruptsP__UsciA__rxDone(uint8_t data);
#line 80
static void HplMsp430UsciAB0RawInterruptsP__UsciA__txDone(void );




static void HplMsp430UsciAB0RawInterruptsP__UsciB__rxDone(uint8_t data);
#line 80
static void HplMsp430UsciAB0RawInterruptsP__UsciB__txDone(void );
# 46 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciAB0RawInterruptsP.nc"
void sig_USCIAB0RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(46)))  ;
#line 58
void sig_USCIAB0TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(44)))  ;
#line 77
static inline void HplMsp430UsciAB0RawInterruptsP__UsciB__default__txDone(void );



static inline void HplMsp430UsciAB0RawInterruptsP__UsciB__default__rxDone(uint8_t temp);
# 71 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 53 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 35 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__makeOutput(void );
#line 29
static void LedsP__Led0__set(void );





static void LedsP__Led1__makeOutput(void );
#line 29
static void LedsP__Led1__set(void );





static void LedsP__Led2__makeOutput(void );
#line 29
static void LedsP__Led2__set(void );
# 45 "/opt/tinyos-2.1.1/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 34
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 34
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 34
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__rxDone(
# 70 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x2ae8cbfc6410, 
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
uint8_t data);
#line 80
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__txDone(
# 70 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x2ae8cbfc6410);
# 80 "/opt/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId(void );
# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
# 39 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0____nesc_unnamed4327 {
#line 39
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[2U];
uint8_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );




static inline bool /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );







static inline resource_client_id_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(
# 55 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ae8cc001020);
# 55 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(
# 60 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ae8cbfff340);
# 49 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ae8cbfff340);
# 43 "/opt/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
static bool /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void );
# 46 "/opt/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2ae8cc003d40);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void );
# 75 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
enum /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4328 {
#line 75
  ArbiterP__0__grantedTask = 13U
};
#line 75
typedef int /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4329 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4330 {
#line 68
  ArbiterP__0__default_owner_id = 2U
};
#line 69
enum /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4331 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
#line 90
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id);
#line 108
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id);
#line 130
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 150
static bool /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );
#line 163
static uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );










static uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id);
#line 187
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
#line 199
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id);



static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id);

static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );




static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void );


static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t Z1SerialP__Resource__release(void );
#line 87
static error_t Z1SerialP__Resource__immediateRequest(void );
# 8 "/opt/tinyos-2.1.1/tos/platforms/z1/chips/msp430X/usci/Z1SerialP.nc"
msp430_uart_union_config_t Z1SerialP__msp430_uart_z1_config = { { 
.ubr = UBR_8MHZ_115200, 
.umctl = UMCTL_8MHZ_115200, 
.ucssel = 2 } };




static inline error_t Z1SerialP__StdControl__start(void );


static inline error_t Z1SerialP__StdControl__stop(void );



static inline void Z1SerialP__Resource__granted(void );

static inline msp430_uart_union_config_t *Z1SerialP__Msp430UartConfigure__getConfig(void );
# 40 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__offset(void );


static inline uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen);


static inline uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen);
# 74 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
uint8_t StateImplP__state[4U];

enum StateImplP____nesc_unnamed4332 {
  StateImplP__S_IDLE = 0
};


static inline error_t StateImplP__Init__init(void );
#line 96
static inline error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState);
#line 111
static inline void StateImplP__State__forceState(uint8_t id, uint8_t reqState);






static inline void StateImplP__State__toIdle(uint8_t id);







static inline bool StateImplP__State__isIdle(uint8_t id);






static bool StateImplP__State__isState(uint8_t id, uint8_t myState);









static uint8_t StateImplP__State__getState(uint8_t id);
# 51 "/opt/tinyos-2.1.1/tos/system/ActiveMessageAddressC.nc"
am_addr_t ActiveMessageAddressC__addr = TOS_AM_ADDRESS;









static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );
#line 95
static inline am_addr_t ActiveMessageAddressC__amAddress(void );
# 41 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/TestCase.nc"
static void FirstTestP__BasicAssertionTest__done(void );
# 9 "FirstTestP.nc"
static inline void FirstTestP__BasicAssertionTest__run(void );
# 214 "/opt/tinyos-2.1.1/tos/chips/msp430X/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
   __asm volatile ("eint");}

# 126 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
static inline bool StateImplP__State__isIdle(uint8_t id)
#line 126
{
  return StateImplP__State__isState(id, StateImplP__S_IDLE);
}

# 61 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static bool TUnitP__TUnitState__isIdle(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = StateImplP__State__isIdle(2U);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 133 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static inline void Link_TUnitProcessingP__TUnitProcessing__testEqualsFailed(uint8_t testId, char *failMsg, uint32_t expected, uint32_t actual, uint8_t assertionId)
#line 133
{
  Link_TUnitProcessingP__insert(TUNITPROCESSING_EVENT_TESTRESULT_EQUALS_FAILED, testId, failMsg, expected, actual, assertionId);
}

# 41 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitProcessing.nc"
inline static void TUnitP__TUnitProcessing__testEqualsFailed(uint8_t testId, char *failMsg, uint32_t expected, uint32_t actual, uint8_t assertionId){
#line 41
  Link_TUnitProcessingP__TUnitProcessing__testEqualsFailed(testId, failMsg, expected, actual, assertionId);
#line 41
}
#line 41
# 61 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static bool Link_TUnitProcessingP__SendState__isIdle(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = StateImplP__State__isIdle(1U);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 111 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
static inline void StateImplP__State__forceState(uint8_t id, uint8_t reqState)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    StateImplP__state[id] = reqState;
#line 112
    __nesc_atomic_end(__nesc_atomic); }
}

# 51 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static void Link_TUnitProcessingP__SendState__forceState(uint8_t reqState){
#line 51
  StateImplP__State__forceState(1U, reqState);
#line 51
}
#line 51
# 86 "/opt/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP__isWaiting(uint8_t id)
{
  return SchedulerBasicP__m_next[id] != SchedulerBasicP__NO_TASK || SchedulerBasicP__m_tail == id;
}

static inline bool SchedulerBasicP__pushTask(uint8_t id)
{
  if (!SchedulerBasicP__isWaiting(id)) 
    {
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_head = id;
          SchedulerBasicP__m_tail = id;
        }
      else 
        {
          SchedulerBasicP__m_next[SchedulerBasicP__m_tail] = id;
          SchedulerBasicP__m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 301 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint32_t __nesc_hton_uint32(void * target, uint32_t value)
#line 301
{
  uint8_t *base = target;

#line 303
  base[3] = value;
  base[2] = value >> 8;
  base[1] = value >> 16;
  base[0] = value >> 24;
  return value;
}

#line 240
static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
#line 240
{
  uint8_t *base = target;

#line 242
  base[0] = value;
  return value;
}

#line 257
static __inline  int8_t __nesc_hton_int8(void * target, int8_t value)
#line 257
{
#line 257
  __nesc_hton_uint8(target, value);
#line 257
  return value;
}

# 137 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static inline void Link_TUnitProcessingP__TUnitProcessing__testNotEqualsFailed(uint8_t testId, char *failMsg, uint32_t actual, uint8_t assertionId)
#line 137
{
  Link_TUnitProcessingP__insert(TUNITPROCESSING_EVENT_TESTRESULT_NOTEQUALS_FAILED, testId, failMsg, actual, actual, assertionId);
}

# 43 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitProcessing.nc"
inline static void TUnitP__TUnitProcessing__testNotEqualsFailed(uint8_t testId, char *failMsg, uint32_t actual, uint8_t assertionId){
#line 43
  Link_TUnitProcessingP__TUnitProcessing__testNotEqualsFailed(testId, failMsg, actual, assertionId);
#line 43
}
#line 43
# 141 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static inline void Link_TUnitProcessingP__TUnitProcessing__testResultIsBelowFailed(uint8_t testId, char *failMsg, uint32_t upperbound, uint32_t actual, uint8_t assertionId)
#line 141
{
  Link_TUnitProcessingP__insert(TUNITPROCESSING_EVENT_TESTRESULT_BELOW_FAILED, testId, failMsg, upperbound, actual, assertionId);
}

# 45 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitProcessing.nc"
inline static void TUnitP__TUnitProcessing__testResultIsBelowFailed(uint8_t testId, char *failMsg, uint32_t upperbound, uint32_t actual, uint8_t assertionId){
#line 45
  Link_TUnitProcessingP__TUnitProcessing__testResultIsBelowFailed(testId, failMsg, upperbound, actual, assertionId);
#line 45
}
#line 45
# 145 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static inline void Link_TUnitProcessingP__TUnitProcessing__testResultIsAboveFailed(uint8_t testId, char *failMsg, uint32_t lowerbound, uint32_t actual, uint8_t assertionId)
#line 145
{
  Link_TUnitProcessingP__insert(TUNITPROCESSING_EVENT_TESTRESULT_ABOVE_FAILED, testId, failMsg, lowerbound, actual, assertionId);
}

# 47 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitProcessing.nc"
inline static void TUnitP__TUnitProcessing__testResultIsAboveFailed(uint8_t testId, char *failMsg, uint32_t lowerbound, uint32_t actual, uint8_t assertionId){
#line 47
  Link_TUnitProcessingP__TUnitProcessing__testResultIsAboveFailed(testId, failMsg, lowerbound, actual, assertionId);
#line 47
}
#line 47
# 129 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static inline void Link_TUnitProcessingP__TUnitProcessing__testSuccess(uint8_t testId, uint8_t assertionId)
#line 129
{
  Link_TUnitProcessingP__insert(TUNITPROCESSING_EVENT_TESTRESULT_SUCCESS, testId, (void *)0, 0, 0, assertionId);
}

# 39 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitProcessing.nc"
inline static void TUnitP__TUnitProcessing__testSuccess(uint8_t testId, uint8_t assertionId){
#line 39
  Link_TUnitProcessingP__TUnitProcessing__testSuccess(testId, assertionId);
#line 39
}
#line 39
# 149 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static inline void Link_TUnitProcessingP__TUnitProcessing__testFailed(uint8_t testId, char *failMsg, uint8_t assertionId)
#line 149
{
  Link_TUnitProcessingP__insert(TUNITPROCESSING_EVENT_TESTRESULT_FAILED, testId, failMsg, 0, 0, assertionId);
}

# 49 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitProcessing.nc"
inline static void TUnitP__TUnitProcessing__testFailed(uint8_t testId, char *failMsg, uint8_t assertionId){
#line 49
  Link_TUnitProcessingP__TUnitProcessing__testFailed(testId, failMsg, assertionId);
#line 49
}
#line 49
# 185 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 37
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 37
}
#line 37
# 126 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x2ae8cb7b3690){
#line 28
  switch (arg_0x2ae8cb7b3690) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x2ae8cb7b3690);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 115 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 28
}
#line 28
# 47 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4333 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(* (volatile uint16_t * )354U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired();
#line 34
}
#line 34
# 47 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4334 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(* (volatile uint16_t * )356U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired();
#line 34
}
#line 34
# 47 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4335 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(* (volatile uint16_t * )358U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 34
}
#line 34
# 120 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 123
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 28
}
#line 28
# 115 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 28
}
#line 28
# 185 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void )
{
}

# 239 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__Counter__overflow(void )
#line 239
{
}

# 71 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 71
  /*Msp430Uart0P.UartP*/Msp430UartP__0__Counter__overflow();
#line 71
}
#line 71
# 53 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 37
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow();
#line 37
}
#line 37
# 126 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 181 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4336 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(* (volatile uint16_t * )386U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4337 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(* (volatile uint16_t * )388U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4338 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(* (volatile uint16_t * )390U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4339 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(* (volatile uint16_t * )392U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4340 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(* (volatile uint16_t * )394U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4341 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(* (volatile uint16_t * )396U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void )
{
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4342 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(* (volatile uint16_t * )398U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired();
    }
}

# 120 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 123
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 28
}
#line 28
# 113 "/opt/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 46 "/opt/tinyos-2.1.1/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 46
  SchedulerBasicP__Scheduler__init();
#line 46
}
#line 46
# 45 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set();
}

# 29 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 29
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set();
#line 29
}
#line 29
# 45 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 6;
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set();
}

# 29 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 29
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set();
#line 29
}
#line 29
# 45 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set();
}

# 29 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 29
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set();
#line 29
}
#line 29
# 52 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput();
#line 35
}
#line 35
# 45 "/opt/tinyos-2.1.1/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 46
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__set();
    LedsP__Led1__set();
    LedsP__Led2__set();
  }
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
inline static error_t PlatformP__LedsInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = LedsP__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 196 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 184
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TA0CTL = 0x0020 | (Msp430ClockP__TA0CTL & ~(0x0020 | 0x0010));
}

#line 148
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP__TBCTL = 0x0100 | 0x0002;
}

#line 178
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 32 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 32
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 32
}
#line 32
# 133 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TA0R = 0;









  Msp430ClockP__TA0CTL = 0x0200 | 0x0002;
}

#line 173
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 31 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 31
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 31
}
#line 31
# 98 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{


  if (CALBC1_8MHZ != 0xFF) {
      DCOCTL = 0x00;
      BCSCTL1 = CALBC1_8MHZ;
      DCOCTL = CALDCO_8MHZ;
    }
  else 
#line 106
    {
      DCOCTL = 0x00;
      BCSCTL1 = 0x8D;
      DCOCTL = 0x88;
    }







  BCSCTL1 = 0x80 | BCSCTL1;
#line 130
  Msp430ClockP__IE1 &= ~(1 << 1);
}

#line 168
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 30 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 30
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 30
}
#line 30
# 262 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430ClockP.nc"
static inline error_t Msp430ClockP__Init__init(void )
{


  Msp430ClockP__TA0CTL = 0x0004;
  Msp430ClockP__TA0IV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {



    Msp430ClockP__Msp430ClockInit__initClocks();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__startTimerA();
    Msp430ClockP__startTimerB();
  }

  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
inline static error_t PlatformP__Msp430ClockInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = Msp430ClockP__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 9 "/opt/tinyos-2.1.1/tos/platforms/z1/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 9
{
  PlatformP__Msp430ClockInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = PlatformP__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 54 "/opt/tinyos-2.1.1/tos/interfaces/Scheduler.nc"
inline static bool RealMainP__Scheduler__runNextTask(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = SchedulerBasicP__Scheduler__runNextTask();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 23 "/opt/tinyos-2.1.1/tos/platforms/z1/chips/msp430X/usci/Z1SerialP.nc"
static inline void Z1SerialP__Resource__granted(void )
#line 23
{
}

# 249 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__default__granted(uint8_t id)
#line 249
{
}

# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__granted(uint8_t arg_0x2ae8cbc50688){
#line 92
  switch (arg_0x2ae8cbc50688) {
#line 92
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 92
      Z1SerialP__Resource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__default__granted(arg_0x2ae8cbc50688);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 130 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__granted(uint8_t id)
#line 130
{
  /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__granted(id);
}

# 199 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id)
#line 199
{
}

# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x2ae8cc003d40){
#line 92
  switch (arg_0x2ae8cc003d40) {
#line 92
    case /*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID:
#line 92
      /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__granted(/*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0x2ae8cc003d40);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 213 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 213
{
}

# 49 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x2ae8cbfff340){
#line 49
  switch (arg_0x2ae8cbfff340) {
#line 49
    case /*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID:
#line 49
      /*Msp430Uart0P.UartP*/Msp430UartP__0__ResourceConfigure__configure(/*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x2ae8cbfff340);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 187 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void )
#line 187
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
    }
#line 191
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
}

# 25 "/opt/tinyos-2.1.1/tos/platforms/z1/chips/msp430X/usci/Z1SerialP.nc"
static inline msp430_uart_union_config_t *Z1SerialP__Msp430UartConfigure__getConfig(void )
#line 25
{
  return &Z1SerialP__msp430_uart_z1_config;
}

# 245 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline msp430_uart_union_config_t */*Msp430Uart0P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(uint8_t id)
#line 245
{
  return (msp430_uart_union_config_t *)&msp430_uart_default_config;
}

# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartConfigure.nc"
inline static msp430_uart_union_config_t */*Msp430Uart0P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(uint8_t arg_0x2ae8cbc4acd0){
#line 71
  union __nesc_unnamed4278 *__nesc_result;
#line 71

#line 71
  switch (arg_0x2ae8cbc4acd0) {
#line 71
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 71
      __nesc_result = Z1SerialP__Msp430UartConfigure__getConfig();
#line 71
      break;
#line 71
    default:
#line 71
      __nesc_result = /*Msp430Uart0P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(arg_0x2ae8cbc4acd0);
#line 71
      break;
#line 71
    }
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 139 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__Usci__resetUsci(bool reset)
#line 139
{
  if (reset) {
    HplMsp430UsciA0P__UCA0CTL1 |= 0x01;
    }
  else {
#line 143
    HplMsp430UsciA0P__UCA0CTL1 &= ~0x01;
    }
}

#line 122
static inline void HplMsp430UsciA0P__Usci__setUmctl(uint8_t control)
#line 122
{
  UCA0MCTL = control;
}

#line 111
static inline void HplMsp430UsciA0P__Usci__setUbr(uint16_t control)
#line 111
{
  /* atomic removed: atomic calls only */
#line 112
  {
    UCA0BR0 = control & 0x00FF;
    UCA0BR1 = (control >> 8) & 0x00FF;
  }
}

#line 317
static inline void HplMsp430UsciA0P__configUart(msp430_uart_union_config_t *config)
#line 317
{
  HplMsp430UsciA0P__UCA0CTL1 = config->uartRegisters.uctl1 | 0x01;
  HplMsp430UsciA0P__UCA0CTL0 = config->uartRegisters.uctl0;
  HplMsp430UsciA0P__Usci__setUbr(config->uartRegisters.ubr);
  HplMsp430UsciA0P__Usci__setUmctl(config->uartRegisters.umctl);
}

# 54 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 5;
}

# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciA0P__URXD__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectModuleFunc();
#line 78
}
#line 78
# 54 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 4;
}

# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciA0P__UTXD__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectModuleFunc();
#line 78
}
#line 78
# 303 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__Usci__enableUart(void )
#line 303
{
  /* atomic removed: atomic calls only */
#line 304
  {
    HplMsp430UsciA0P__UTXD__selectModuleFunc();
    HplMsp430UsciA0P__URXD__selectModuleFunc();
  }
}

#line 236
static inline void HplMsp430UsciA0P__Usci__clrIntr(void )
#line 236
{
  HplMsp430UsciA0P__IFG2 &= ~((1 << 1) | (1 << 0));
}









static inline void HplMsp430UsciA0P__Usci__disableIntr(void )
#line 248
{
  HplMsp430UsciA0P__IE2 &= ~((1 << 1) | (1 << 0));
}

#line 324
static inline void HplMsp430UsciA0P__Usci__setModeUart(msp430_uart_union_config_t *config)
#line 324
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 325
    {
      HplMsp430UsciA0P__Usci__disableIntr();
      HplMsp430UsciA0P__Usci__clrIntr();
      HplMsp430UsciA0P__Usci__resetUsci(TRUE);
      HplMsp430UsciA0P__Usci__enableUart();
      HplMsp430UsciA0P__configUart(config);
      HplMsp430UsciA0P__Usci__resetUsci(FALSE);
    }
#line 332
    __nesc_atomic_end(__nesc_atomic); }
}

# 165 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
inline static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__setModeUart(msp430_uart_union_config_t *config){
#line 165
  HplMsp430UsciA0P__Usci__setModeUart(config);
#line 165
}
#line 165
# 266 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__Usci__enableIntr(void )
#line 266
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 267
    {
      HplMsp430UsciA0P__IFG2 &= ~((1 << 1) | (1 << 0));
      HplMsp430UsciA0P__IE2 |= (1 << 1) | (1 << 0);
    }
#line 270
    __nesc_atomic_end(__nesc_atomic); }
}

# 124 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
inline static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__enableIntr(void ){
#line 124
  HplMsp430UsciA0P__Usci__enableIntr();
#line 124
}
#line 124
# 235 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 235
{
  const uint8_t *base = source;

#line 237
  return base[0];
}

# 49 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(message_t * msg)
#line 49
{
  return (serial_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(serial_header_t ));
}

#line 161
static inline am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(message_t *amsg)
#line 161
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 163
  return __nesc_ntoh_uint8(header->type.data);
}

# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static void /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(message_t * msg, error_t error){
#line 99
  Link_TUnitProcessingP__SerialEventSend__sendDone(msg, error);
#line 99
}
#line 99
# 57 "/opt/tinyos-2.1.1/tos/system/AMQueueEntryP.nc"
static inline void /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err)
#line 57
{
  /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(m, err);
}

# 207 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err)
#line 207
{
}

# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(uint8_t arg_0x2ae8cb9b51c8, message_t * msg, error_t error){
#line 89
  switch (arg_0x2ae8cb9b51c8) {
#line 89
    case 0U:
#line 89
      /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(arg_0x2ae8cb9b51c8, msg, error);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
# 155 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err)
#line 155
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend();
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, err);
}

#line 181
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err)
#line 181
{





  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      return;
    }
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == msg) {
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current, msg, err);
    }
  else {
      ;
    }
}

# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(am_id_t arg_0x2ae8cba0cba8, message_t * msg, error_t error){
#line 99
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(arg_0x2ae8cba0cba8, msg, error);
#line 99
}
#line 99
# 90 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(message_t *msg, error_t result)
#line 90
{
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(msg), msg, result);
}

# 365 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(uart_id_t idxxx, message_t *msg, error_t error)
#line 365
{
  return;
}

# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(uart_id_t arg_0x2ae8cbb56238, message_t * msg, error_t error){
#line 89
  switch (arg_0x2ae8cbb56238) {
#line 89
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 89
      /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(arg_0x2ae8cbb56238, msg, error);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
# 147 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void )
#line 147
{
  error_t error;

  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
    error = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError;
#line 151
    __nesc_atomic_end(__nesc_atomic); }

  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled) {
#line 153
    error = ECANCEL;
    }
#line 154
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId, (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer, error);
}

# 57 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void )
#line 57
{
  uint8_t i;

#line 59
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
  for (i = 0; i < 1; i++) {
      if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == (void *)0 || 
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current / 8] & (1 << /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current % 8)) 
        {
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
        }
      else {
          break;
        }
    }
  if (i >= 1) {
#line 70
    /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
    }
}

# 264 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_uint16(const void * source)
#line 264
{
  const uint8_t *base = source;

#line 266
  return ((uint16_t )base[0] << 8) | base[1];
}

# 111 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(message_t *msg)
#line 111
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg);

#line 113
  return __nesc_ntoh_uint8(header->length.data);
}

# 67 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
inline static uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(message_t * msg){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(msg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 120 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void )
#line 120
{
  return 28;
}

# 522 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SendBytePacket__startSend(uint8_t b)
#line 522
{
  bool not_busy = FALSE;

#line 524
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 524
    {
      if (SerialP__txBuf[SerialP__TX_DATA_INDEX].state == SerialP__BUFFER_AVAILABLE) {
          SerialP__txBuf[SerialP__TX_DATA_INDEX].state = SerialP__BUFFER_FILLING;
          SerialP__txBuf[SerialP__TX_DATA_INDEX].buf = b;
          not_busy = TRUE;
        }
    }
#line 530
    __nesc_atomic_end(__nesc_atomic); }
  if (not_busy) {
      SerialP__MaybeScheduleTx();
      return SUCCESS;
    }
  return EBUSY;
}

# 51 "/opt/tinyos-2.1.1/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(uint8_t first_byte){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = SerialP__SendBytePacket__startSend(first_byte);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 43 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen)
#line 43
{
  return upperLen + sizeof(serial_header_t );
}

# 350 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen)
#line 351
{
  return 0;
}

# 23 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(uart_id_t arg_0x2ae8cbb55570, message_t *msg, uint8_t upperLen){
#line 23
  unsigned char __nesc_result;
#line 23

#line 23
  switch (arg_0x2ae8cbb55570) {
#line 23
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 23
      __nesc_result = SerialPacketInfoActiveMessageP__Info__dataLinkLength(msg, upperLen);
#line 23
      break;
#line 23
    default:
#line 23
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(arg_0x2ae8cbb55570, msg, upperLen);
#line 23
      break;
#line 23
    }
#line 23

#line 23
  return __nesc_result;
#line 23
}
#line 23
# 40 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__offset(void )
#line 40
{
  return (uint8_t )(sizeof(message_header_t ) - sizeof(serial_header_t ));
}

# 347 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(uart_id_t id)
#line 347
{
  return 0;
}

# 15 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(uart_id_t arg_0x2ae8cbb55570){
#line 15
  unsigned char __nesc_result;
#line 15

#line 15
  switch (arg_0x2ae8cbb55570) {
#line 15
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 15
      __nesc_result = SerialPacketInfoActiveMessageP__Info__offset();
#line 15
      break;
#line 15
    default:
#line 15
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(arg_0x2ae8cbb55570);
#line 15
      break;
#line 15
    }
#line 15

#line 15
  return __nesc_result;
#line 15
}
#line 15
# 100 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(uint8_t id, message_t *msg, uint8_t len)
#line 100
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState != /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE) {
      return EBUSY;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 105
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(id);
      if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex > sizeof(message_header_t )) {
          {
            unsigned char __nesc_temp = 
#line 108
            ESIZE;

            {
#line 108
              __nesc_atomic_end(__nesc_atomic); 
#line 108
              return __nesc_temp;
            }
          }
        }
#line 111
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = SUCCESS;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer = (uint8_t *)msg;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_DATA;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId = id;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled = FALSE;






      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(id, msg, len) + /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex;
    }
#line 123
    __nesc_atomic_end(__nesc_atomic); }
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(id) == SUCCESS) {
      return SUCCESS;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
      return 0x0080;
    }
}

# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(TOS_SERIAL_ACTIVE_MESSAGE_ID, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__RunTx__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__RunTx);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
inline static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 118 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
static inline void StateImplP__State__toIdle(uint8_t id)
#line 118
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 119
    StateImplP__state[id] = StateImplP__S_IDLE;
#line 119
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static void Link_TUnitProcessingP__SendState__toIdle(void ){
#line 56
  StateImplP__State__toIdle(1U);
#line 56
}
#line 56
# 201 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(uint8_t which)
#line 201
{
  if (which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked = 0;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked = 0;
    }
}

# 51 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static void TUnitP__TUnitState__forceState(uint8_t reqState){
#line 51
  StateImplP__State__forceState(2U, reqState);
#line 51
}
#line 51
# 295 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__tearDownOneTimeDone(void )
#line 295
{
  TUnitP__TUnitState__forceState(TUnitP__S_READY);
  TUnitP__TestState__toIdle();
}

#line 386
static inline void TUnitP__TearDownOneTime__default__run(void )
#line 386
{
  TUnitP__tearDownOneTimeDone();
}

# 38 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
inline static void TUnitP__TearDownOneTime__run(void ){
#line 38
  TUnitP__TearDownOneTime__default__run();
#line 38
}
#line 38
# 184 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__TUnitProcessing__tearDownOneTime(void )
#line 184
{
  TUnitP__TUnitState__forceState(TUnitP__S_TEARDOWN_ONETIME);
  TUnitP__TearDownOneTime__run();
}

# 61 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitProcessing.nc"
inline static void Link_TUnitProcessingP__TUnitProcessing__tearDownOneTime(void ){
#line 61
  TUnitP__TUnitProcessing__tearDownOneTime();
#line 61
}
#line 61
# 158 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static inline void Link_TUnitProcessingP__TUnitProcessing__pong(void )
#line 158
{
  Link_TUnitProcessingP__insert(TUNITPROCESSING_EVENT_PONG, 0xFF, (void *)0, 0, 0, 0xFF);
}

# 54 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitProcessing.nc"
inline static void TUnitP__TUnitProcessing__pong(void ){
#line 54
  Link_TUnitProcessingP__TUnitProcessing__pong();
#line 54
}
#line 54
# 180 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__TUnitProcessing__ping(void )
#line 180
{
  TUnitP__TUnitProcessing__pong();
}

# 59 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitProcessing.nc"
inline static void Link_TUnitProcessingP__TUnitProcessing__ping(void ){
#line 59
  TUnitP__TUnitProcessing__ping();
#line 59
}
#line 59
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t TUnitP__begin__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(TUnitP__begin);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 174 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__TUnitProcessing__run(void )
#line 174
{


  TUnitP__begin__postTask();
}

# 57 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitProcessing.nc"
inline static void Link_TUnitProcessingP__TUnitProcessing__run(void ){
#line 57
  TUnitP__TUnitProcessing__run();
#line 57
}
#line 57
# 180 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static inline void Link_TUnitProcessingP__execute(TUnitProcessingMsg *inMsg)
#line 180
{
  switch (__nesc_ntoh_uint8(inMsg->cmd.data)) {
      case TUNITPROCESSING_CMD_RUN: 
        Link_TUnitProcessingP__TUnitProcessing__run();
      break;

      case TUNITPROCESSING_CMD_PING: 
        Link_TUnitProcessingP__TUnitProcessing__ping();
      break;

      case TUNITPROCESSING_CMD_TEARDOWNONETIME: 
        Link_TUnitProcessingP__TUnitProcessing__tearDownOneTime();
      break;

      default: ;
    }
}

#line 112
static inline message_t *Link_TUnitProcessingP__SerialReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 112
{
  Link_TUnitProcessingP__execute(payload);
  return msg;
}

# 98 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(uint8_t id, message_t *msg, void *payload, uint8_t len)
#line 98
{
  return msg;
}

# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
inline static message_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(am_id_t arg_0x2ae8cba09cd0, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  switch (arg_0x2ae8cba09cd0) {
#line 67
    case 255:
#line 67
      __nesc_result = Link_TUnitProcessingP__SerialReceive__receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(arg_0x2ae8cba09cd0, msg, payload, len);
#line 67
      break;
#line 67
    }
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 102 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 102
{
  return /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(msg), msg, msg->data, len);
}

# 360 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len)
#line 362
{
  return msg;
}

# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
inline static message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(uart_id_t arg_0x2ae8cbb576e8, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  switch (arg_0x2ae8cbb576e8) {
#line 67
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 67
      __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(arg_0x2ae8cbb576e8, msg, payload, len);
#line 67
      break;
#line 67
    }
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 46 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen)
#line 46
{
  return dataLinkLen - sizeof(serial_header_t );
}

# 354 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen)
#line 355
{
  return 0;
}

# 31 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(uart_id_t arg_0x2ae8cbb55570, message_t *msg, uint8_t dataLinkLen){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  switch (arg_0x2ae8cbb55570) {
#line 31
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 31
      __nesc_result = SerialPacketInfoActiveMessageP__Info__upperLength(msg, dataLinkLen);
#line 31
      break;
#line 31
    default:
#line 31
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(arg_0x2ae8cbb55570, msg, dataLinkLen);
#line 31
      break;
#line 31
    }
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 264 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void )
#line 264
{
  uart_id_t myType;
  message_t *myBuf;
  uint8_t mySize;
  uint8_t myWhich;

#line 269
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 269
    {
      myType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType;
      myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf;
      mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize;
      myWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich;
    }
#line 274
    __nesc_atomic_end(__nesc_atomic); }
  mySize -= /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(myType);
  mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(myType, myBuf, mySize);
  myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(myType, myBuf, myBuf, mySize);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 278
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[myWhich] = myBuf;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(myWhich);
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = FALSE;
    }
#line 282
    __nesc_atomic_end(__nesc_atomic); }
}

# 170 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__SerialSplitControl__stopDone(error_t error)
#line 170
{
}

# 51 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static void Link_TUnitProcessingP__SerialState__forceState(uint8_t reqState){
#line 51
  StateImplP__State__forceState(0U, reqState);
#line 51
}
#line 51
# 107 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static inline void Link_TUnitProcessingP__SerialSplitControl__stopDone(error_t error)
#line 107
{
  Link_TUnitProcessingP__SerialState__forceState(Link_TUnitProcessingP__S_OFF);
}

# 117 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static void SerialP__SplitControl__stopDone(error_t error){
#line 117
  Link_TUnitProcessingP__SerialSplitControl__stopDone(error);
#line 117
  TUnitP__SerialSplitControl__stopDone(error);
#line 117
}
#line 117
# 205 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void )
#line 205
{
}

# 46 "/opt/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted();
#line 46
}
#line 46
# 56 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 5);
}

# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciA0P__URXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 4);
}

# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciA0P__UTXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc();
#line 85
}
#line 85
# 310 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__Usci__disableUart(void )
#line 310
{
  /* atomic removed: atomic calls only */
#line 311
  {
    HplMsp430UsciA0P__UTXD__selectIOFunc();
    HplMsp430UsciA0P__URXD__selectIOFunc();
  }
}

# 159 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
inline static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__disableUart(void ){
#line 159
  HplMsp430UsciA0P__Usci__disableUart();
#line 159
}
#line 159
#line 121
inline static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__disableIntr(void ){
#line 121
  HplMsp430UsciA0P__Usci__disableIntr();
#line 121
}
#line 121
#line 92
inline static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__resetUsci(bool reset){
#line 92
  HplMsp430UsciA0P__Usci__resetUsci(reset);
#line 92
}
#line 92
# 124 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 124
{
  /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__resetUsci(TRUE);
  /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__disableIntr();
  /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__disableUart();
}

# 215 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 215
{
}

# 55 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x2ae8cbfff340){
#line 55
  switch (arg_0x2ae8cbfff340) {
#line 55
    case /*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID:
#line 55
      /*Msp430Uart0P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(/*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID);
#line 55
      break;
#line 55
    default:
#line 55
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x2ae8cbfff340);
#line 55
      break;
#line 55
    }
#line 55
}
#line 55
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 58 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead != /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
        uint8_t id = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead;

#line 62
        /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[/*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead];
        if (/*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
          /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
          }
#line 65
        /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[id] = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 66
          id;

#line 66
          return __nesc_temp;
        }
      }
#line 68
    {
      unsigned char __nesc_temp = 
#line 68
      /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/opt/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 50 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 51
  {
    unsigned char __nesc_temp = 
#line 51
    /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 51
    return __nesc_temp;
  }
}

# 43 "/opt/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 108 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id)
#line 108
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY && /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
          if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty() == FALSE) {
              /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue();
              /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;
              /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
              /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
            }
          else {
              /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
              /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
              /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
              /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted();
            }
          {
            unsigned char __nesc_temp = 
#line 124
            SUCCESS;

            {
#line 124
              __nesc_atomic_end(__nesc_atomic); 
#line 124
              return __nesc_temp;
            }
          }
        }
    }
#line 128
    __nesc_atomic_end(__nesc_atomic); }
#line 127
  return 0x0080;
}

# 244 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__release(uint8_t id)
#line 244
{
#line 244
  return 0x0080;
}

# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__release(uint8_t arg_0x2ae8cbc4ba18){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0x2ae8cbc4ba18) {
#line 110
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 110
      __nesc_result = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(/*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__release(arg_0x2ae8cbc4ba18);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 241 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__isOwner(uint8_t id)
#line 241
{
#line 241
  return 0x0080;
}

# 118 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static bool /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__isOwner(uint8_t arg_0x2ae8cbc4ba18){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  switch (arg_0x2ae8cbc4ba18) {
#line 118
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 118
      __nesc_result = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(/*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID);
#line 118
      break;
#line 118
    default:
#line 118
      __nesc_result = /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__isOwner(arg_0x2ae8cbc4ba18);
#line 118
      break;
#line 118
    }
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 109 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__release(uint8_t id)
#line 109
{
  if (/*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__isOwner(id) == FALSE) {
    return 0x0080;
    }
#line 112
  if (/*Msp430Uart0P.UartP*/Msp430UartP__0__m_rx_buf || /*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_buf) {
    return EBUSY;
    }
#line 114
  return /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__release(id);
}

# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t Z1SerialP__Resource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__release(/*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 19 "/opt/tinyos-2.1.1/tos/platforms/z1/chips/msp430X/usci/Z1SerialP.nc"
static inline error_t Z1SerialP__StdControl__stop(void )
#line 19
{
  Z1SerialP__Resource__release();
  return SUCCESS;
}

# 84 "/opt/tinyos-2.1.1/tos/interfaces/StdControl.nc"
inline static error_t SerialP__SerialControl__stop(void ){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  __nesc_result = Z1SerialP__StdControl__stop();
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 330 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFlush__flushDone(void )
#line 330
{
  SerialP__SerialControl__stop();
  SerialP__SplitControl__stopDone(SUCCESS);
}

static inline void SerialP__defaultSerialFlushTask__runTask(void )
#line 335
{
  SerialP__SerialFlush__flushDone();
}

# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__defaultSerialFlushTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__defaultSerialFlushTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 338 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFlush__default__flush(void )
#line 338
{
  SerialP__defaultSerialFlushTask__postTask();
}

# 38 "/opt/tinyos-2.1.1/tos/lib/serial/SerialFlush.nc"
inline static void SerialP__SerialFlush__flush(void ){
#line 38
  SerialP__SerialFlush__default__flush();
#line 38
}
#line 38
# 326 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static inline void SerialP__stopDoneTask__runTask(void )
#line 326
{
  SerialP__SerialFlush__flush();
}

# 370 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__SetUpOneTime__default__run(void )
#line 370
{
  TUnitP__setUpOneTimeDone();
}

# 38 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
inline static void TUnitP__SetUpOneTime__run(void ){
#line 38
  TUnitP__SetUpOneTime__default__run();
#line 38
}
#line 38
# 51 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static void TUnitP__TestState__forceState(uint8_t reqState){
#line 51
  StateImplP__State__forceState(3U, reqState);
#line 51
}
#line 51
#line 71
inline static uint8_t TUnitP__TUnitState__getState(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = StateImplP__State__getState(2U);
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 154 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__SerialSplitControl__startDone(error_t error)
#line 154
{
  if (TUnitP__TUnitState__getState() == TUnitP__S_NOT_BOOTED) {
      TUnitP__TUnitState__forceState(TUnitP__S_READY);

      if (!TUnitP__driver) {




          TUnitP__TUnitState__forceState(TUnitP__S_RUNNING);
          TUnitP__TestState__forceState(TUnitP__S_SETUP_ONETIME);
          TUnitP__SetUpOneTime__run();
        }
    }
}

# 103 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static inline void Link_TUnitProcessingP__SerialSplitControl__startDone(error_t error)
#line 103
{
  Link_TUnitProcessingP__SerialState__forceState(Link_TUnitProcessingP__S_ON);
}

# 92 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static void SerialP__SplitControl__startDone(error_t error){
#line 92
  Link_TUnitProcessingP__SerialSplitControl__startDone(error);
#line 92
  TUnitP__SerialSplitControl__startDone(error);
#line 92
}
#line 92
# 130 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void )
#line 130
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 131
    {
      if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId == /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id) {
          if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING) {
              /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              {
                unsigned char __nesc_temp = 
#line 135
                SUCCESS;

                {
#line 135
                  __nesc_atomic_end(__nesc_atomic); 
#line 135
                  return __nesc_temp;
                }
              }
            }
          else {
#line 137
            if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING) {
                /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
                /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
                {
                  unsigned char __nesc_temp = 
#line 140
                  SUCCESS;

                  {
#line 140
                    __nesc_atomic_end(__nesc_atomic); 
#line 140
                    return __nesc_temp;
                  }
                }
              }
            }
        }
    }
#line 146
    __nesc_atomic_end(__nesc_atomic); }
#line 144
  return 0x0080;
}

#line 210
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void )
#line 210
{
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
}

# 81 "/opt/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested();
#line 81
}
#line 81
# 203 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id)
#line 203
{
}

# 51 "/opt/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0x2ae8cc001020){
#line 51
    /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(arg_0x2ae8cc001020);
#line 51
}
#line 51
# 90 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id)
#line 90
{
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 92
    {
      if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
          /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING;
          /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
        }
      else {
          unsigned char __nesc_temp = 
#line 97
          0x0080;

          {
#line 97
            __nesc_atomic_end(__nesc_atomic); 
#line 97
            return __nesc_temp;
          }
        }
    }
#line 100
    __nesc_atomic_end(__nesc_atomic); }
#line 99
  /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
      return SUCCESS;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 104
    /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
#line 104
    __nesc_atomic_end(__nesc_atomic); }
  return 0x0080;
}

# 243 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__immediateRequest(uint8_t id)
#line 243
{
#line 243
  return 0x0080;
}

# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__immediateRequest(uint8_t arg_0x2ae8cbc4ba18){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  switch (arg_0x2ae8cbc4ba18) {
#line 87
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 87
      __nesc_result = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(/*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID);
#line 87
      break;
#line 87
    default:
#line 87
      __nesc_result = /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__immediateRequest(arg_0x2ae8cbc4ba18);
#line 87
      break;
#line 87
    }
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 97 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__immediateRequest(uint8_t id)
#line 97
{
  return /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__immediateRequest(id);
}

# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t Z1SerialP__Resource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__immediateRequest(/*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 16 "/opt/tinyos-2.1.1/tos/platforms/z1/chips/msp430X/usci/Z1SerialP.nc"
static inline error_t Z1SerialP__StdControl__start(void )
#line 16
{
  return Z1SerialP__Resource__immediateRequest();
}

# 74 "/opt/tinyos-2.1.1/tos/interfaces/StdControl.nc"
inline static error_t SerialP__SerialControl__start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = Z1SerialP__StdControl__start();
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 320 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static inline void SerialP__startDoneTask__runTask(void )
#line 320
{
  SerialP__SerialControl__start();
  SerialP__SplitControl__startDone(SUCCESS);
}

# 96 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
static inline error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState)
#line 96
{
  error_t returnVal = 0x0080;

#line 98
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 98
    {
      if (reqState == StateImplP__S_IDLE || StateImplP__state[id] == StateImplP__S_IDLE) {
          StateImplP__state[id] = reqState;
          returnVal = SUCCESS;
        }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return returnVal;
}

# 45 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static error_t TUnitP__TestState__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(3U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t TUnitP__runDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(TUnitP__runDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 192 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__TestCase__done(uint8_t testId)
#line 192
{
  TUnitP__runDone__postTask();
}

# 41 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/TestCase.nc"
inline static void FirstTestP__BasicAssertionTest__done(void ){
#line 41
  TUnitP__TestCase__done(/*FirstTestC.BasicAssertionTestC*/TestCaseC__0__TUNIT_TEST_ID);
#line 41
}
#line 41
# 9 "FirstTestP.nc"
static inline void FirstTestP__BasicAssertionTest__run(void )
{
  if (!TRUE) {
#line 11
      assertTunitFail("Not false", 18U);
#line 11
      ;
    }
  else 
#line 11
    {
#line 11
      assertTunitSuccess(19U);
#line 11
      ;
    }
#line 11
  ;
  if (FALSE) {
#line 12
      assertTunitFail("Not true", 20U);
#line 12
      ;
    }
  else 
#line 12
    {
#line 12
      assertTunitSuccess(21U);
#line 12
      ;
    }
#line 12
  ;
  if ((float )1.2 == (int )1.2) {
#line 13
      assertNotEqualsFailed("Should not be equal", (uint32_t )(int )1.2, 22U);
    }
  else 
#line 13
    {
#line 13
      assertTunitSuccess(23U);
#line 13
      ;
    }
#line 13
  ;
  FirstTestP__BasicAssertionTest__done();
}

# 378 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__TestCase__default__run(uint8_t testId)
#line 378
{
  TUnitP__runDone__postTask();
}

# 39 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/TestCase.nc"
inline static void TUnitP__TestCase__run(uint8_t arg_0x2ae8cb6666e8){
#line 39
  switch (arg_0x2ae8cb6666e8) {
#line 39
    case /*FirstTestC.BasicAssertionTestC*/TestCaseC__0__TUNIT_TEST_ID:
#line 39
      FirstTestP__BasicAssertionTest__run();
#line 39
      break;
#line 39
    default:
#line 39
      TUnitP__TestCase__default__run(arg_0x2ae8cb6666e8);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 71 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static uint8_t TUnitP__TestState__getState(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = StateImplP__State__getState(3U);
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 279 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__setUpDone(void )
#line 279
{
  if (TUnitP__TestState__getState() == TUnitP__S_SETUP) {
      TUnitP__TestState__forceState(TUnitP__S_RUN);
      TUnitP__TestCase__run(TUnitP__currentTest);
    }
}

#line 374
static inline void TUnitP__SetUp__default__run(void )
#line 374
{
  TUnitP__setUpDone();
}

# 38 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
inline static void TUnitP__SetUp__run(void ){
#line 38
  TUnitP__SetUp__default__run();
#line 38
}
#line 38
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t Link_TUnitProcessingP__allDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(Link_TUnitProcessingP__allDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 154 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static inline void Link_TUnitProcessingP__TUnitProcessing__allDone(void )
#line 154
{
  Link_TUnitProcessingP__allDone__postTask();
}

# 52 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitProcessing.nc"
inline static void TUnitP__TUnitProcessing__allDone(void ){
#line 52
  Link_TUnitProcessingP__TUnitProcessing__allDone();
#line 52
}
#line 52
# 45 "/opt/tinyos-2.1.1/tos/lib/serial/SerialFrameComm.nc"
inline static error_t SerialP__SerialFrameComm__putDelimiter(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = HdlcTranslateC__SerialFrameComm__putDelimiter();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 183 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error)
#line 183
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 184
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = error;
#line 184
    __nesc_atomic_end(__nesc_atomic); }
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask();
}

# 80 "/opt/tinyos-2.1.1/tos/lib/serial/SendBytePacket.nc"
inline static void SerialP__SendBytePacket__sendCompleted(error_t error){
#line 80
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error);
#line 80
}
#line 80
# 242 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static __inline bool SerialP__ack_queue_is_empty(void )
#line 242
{
  bool ret;

#line 244
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 244
    ret = SerialP__ackQ.writePtr == SerialP__ackQ.readPtr;
#line 244
    __nesc_atomic_end(__nesc_atomic); }
  return ret;
}











static __inline uint8_t SerialP__ack_queue_top(void )
#line 258
{
  uint8_t tmp = 0;

  /* atomic removed: atomic calls only */
#line 260
  {
    if (!SerialP__ack_queue_is_empty()) {
        tmp = SerialP__ackQ.buf[SerialP__ackQ.readPtr];
      }
  }
  return tmp;
}

static inline uint8_t SerialP__ack_queue_pop(void )
#line 268
{
  uint8_t retval = 0;

#line 270
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 270
    {
      if (SerialP__ackQ.writePtr != SerialP__ackQ.readPtr) {
          retval = SerialP__ackQ.buf[SerialP__ackQ.readPtr];
          if (++ SerialP__ackQ.readPtr > SerialP__ACK_QUEUE_SIZE) {
#line 273
            SerialP__ackQ.readPtr = 0;
            }
        }
    }
#line 276
    __nesc_atomic_end(__nesc_atomic); }
#line 276
  return retval;
}

#line 539
static inline void SerialP__RunTx__runTask(void )
#line 539
{
  uint8_t idle;
  uint8_t done;
  uint8_t fail;









  error_t result = SUCCESS;
  bool send_completed = FALSE;
  bool start_it = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 556
    {
      SerialP__txPending = 0;
      idle = SerialP__txState == SerialP__TXSTATE_IDLE;
      done = SerialP__txState == SerialP__TXSTATE_FINISH;
      fail = SerialP__txState == SerialP__TXSTATE_ERROR;
      if (done || fail) {
          SerialP__txState = SerialP__TXSTATE_IDLE;
          SerialP__txBuf[SerialP__txIndex].state = SerialP__BUFFER_AVAILABLE;
        }
    }
#line 565
    __nesc_atomic_end(__nesc_atomic); }


  if (done || fail) {
      SerialP__txSeqno++;
      if (SerialP__txProto == SERIAL_PROTO_ACK) {
          SerialP__ack_queue_pop();
        }
      else {
          result = done ? SUCCESS : 0x0080;
          send_completed = TRUE;
        }
      idle = TRUE;
    }


  if (idle) {
      bool goInactive;

#line 583
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 583
        goInactive = SerialP__offPending;
#line 583
        __nesc_atomic_end(__nesc_atomic); }
      if (goInactive) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 585
            SerialP__txState = SerialP__TXSTATE_INACTIVE;
#line 585
            __nesc_atomic_end(__nesc_atomic); }
        }
      else {

          uint8_t myAckState;
          uint8_t myDataState;

#line 591
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 591
            {
              myAckState = SerialP__txBuf[SerialP__TX_ACK_INDEX].state;
              myDataState = SerialP__txBuf[SerialP__TX_DATA_INDEX].state;
            }
#line 594
            __nesc_atomic_end(__nesc_atomic); }
          if (!SerialP__ack_queue_is_empty() && myAckState == SerialP__BUFFER_AVAILABLE) {
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 596
                {
                  SerialP__txBuf[SerialP__TX_ACK_INDEX].state = SerialP__BUFFER_COMPLETE;
                  SerialP__txBuf[SerialP__TX_ACK_INDEX].buf = SerialP__ack_queue_top();
                }
#line 599
                __nesc_atomic_end(__nesc_atomic); }
              SerialP__txProto = SERIAL_PROTO_ACK;
              SerialP__txIndex = SerialP__TX_ACK_INDEX;
              start_it = TRUE;
            }
          else {
#line 604
            if (myDataState == SerialP__BUFFER_FILLING || myDataState == SerialP__BUFFER_COMPLETE) {
                SerialP__txProto = SERIAL_PROTO_PACKET_NOACK;
                SerialP__txIndex = SerialP__TX_DATA_INDEX;
                start_it = TRUE;
              }
            else {
              }
            }
        }
    }
  else {
    }


  if (send_completed) {
      SerialP__SendBytePacket__sendCompleted(result);
    }

  if (SerialP__txState == SerialP__TXSTATE_INACTIVE) {
      SerialP__testOff();
      return;
    }

  if (start_it) {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 629
        {
          SerialP__txCRC = 0;
          SerialP__txByteCnt = 0;
          SerialP__txState = SerialP__TXSTATE_PROTO;
        }
#line 633
        __nesc_atomic_end(__nesc_atomic); }
      if (SerialP__SerialFrameComm__putDelimiter() != SUCCESS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 635
            SerialP__txState = SerialP__TXSTATE_ERROR;
#line 635
            __nesc_atomic_end(__nesc_atomic); }
          SerialP__MaybeScheduleTx();
        }
    }
}

# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__stopDoneTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__stopDoneTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 48 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
inline static error_t HdlcTranslateC__UartStream__send(uint8_t * buf, uint16_t len){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__send(/*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID, buf, len);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 118 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void )
#line 118
{
  uint8_t i;
#line 119
  uint8_t j;
#line 119
  uint8_t mask;
#line 119
  uint8_t last;
  message_t *msg;

#line 121
  for (i = 0; i < 1 / 8 + 1; i++) {
      if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i]) {
          for (mask = 1, j = 0; j < 8; j++) {
              if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] & mask) {
                  last = i * 8 + j;
                  msg = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg;
                  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
                  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] &= ~mask;
                  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, ECANCEL);
                }
              mask <<= 1;
            }
        }
    }
}

#line 161
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void )
#line 161
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current, /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg, 0x0080);
}

# 173 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static inline void Link_TUnitProcessingP__allDone__runTask(void )
#line 173
{
  if (Link_TUnitProcessingP__insert(TUNITPROCESSING_EVENT_ALLDONE, 0xFF, (void *)0, 0, 0, 0xFF) != SUCCESS) {
      Link_TUnitProcessingP__allDone__postTask();
    }
}

# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t Link_TUnitProcessingP__sendEventMsg__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(Link_TUnitProcessingP__sendEventMsg);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
inline static error_t SerialP__startDoneTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__startDoneTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 342 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SplitControl__start(void )
#line 342
{
  SerialP__startDoneTask__postTask();
  return SUCCESS;
}

# 83 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static error_t Link_TUnitProcessingP__SerialSplitControl__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = SerialP__SplitControl__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 71 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static uint8_t Link_TUnitProcessingP__SerialState__getState(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = StateImplP__State__getState(0U);
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(am_id_t arg_0x2ae8cb9b4340, am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(arg_0x2ae8cb9b4340, addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 67 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(message_t * amsg){
#line 67
  unsigned int __nesc_result;
#line 67

#line 67
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(amsg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
#line 136
inline static am_id_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(message_t * amsg){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(amsg);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 116 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(message_t *msg, uint8_t len)
#line 116
{
  __nesc_hton_uint8(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg)->length.data, len);
}

# 83 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
inline static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(message_t * msg, uint8_t len){
#line 83
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(msg, len);
#line 83
}
#line 83
# 82 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static inline error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len)
#line 83
{
  if (clientId >= 1) {
      return 0x0080;
    }
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg != (void *)0) {
      return EBUSY;
    }
  ;

  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = msg;
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(msg, len);

  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      error_t err;
      am_id_t amId = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(msg);
      am_addr_t dest = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(msg);

      ;
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = clientId;

      err = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(amId, dest, msg, len);
      if (err != SUCCESS) {
          ;
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = (void *)0;
        }

      return err;
    }
  else {
      ;
    }
  return SUCCESS;
}

# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static error_t /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(0U, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 166 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(message_t *amsg, am_id_t type)
#line 166
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 168
  __nesc_hton_uint8(header->type.data, type);
}

# 151 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static void /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(amsg, t);
#line 151
}
#line 151
# 269 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_uint16(void * target, uint16_t value)
#line 269
{
  uint8_t *base = target;

#line 271
  base[1] = value;
  base[0] = value >> 8;
  return value;
}

# 147 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(message_t *amsg, am_addr_t addr)
#line 147
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 149
  __nesc_hton_uint16(header->dest.data, addr);
}

# 92 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static void /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
# 45 "/opt/tinyos-2.1.1/tos/system/AMQueueEntryP.nc"
static inline error_t /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 47
{
  /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(msg, dest);
  /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(msg, 255);
  return /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(msg, len);
}

# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static error_t Link_TUnitProcessingP__SerialEventSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 164 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static inline void Link_TUnitProcessingP__sendEventMsg__runTask(void )
#line 164
{
  if (Link_TUnitProcessingP__SerialEventSend__send(0, &Link_TUnitProcessingP__eventMsg[Link_TUnitProcessingP__sendingEventMsg], sizeof(TUnitProcessingMsg )) != SUCCESS) {
      if (Link_TUnitProcessingP__SerialState__getState() == Link_TUnitProcessingP__S_OFF) {
          Link_TUnitProcessingP__SerialSplitControl__start();
        }
      Link_TUnitProcessingP__sendEventMsg__postTask();
    }
}

# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t TUnitP__waitForSendDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(TUnitP__waitForSendDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 287 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__tearDownDone(void )
#line 287
{
  if (TUnitP__TestState__getState() == TUnitP__S_TEARDOWN) {
      TUnitP__TestState__toIdle();
      TUnitP__waitForSendDone__postTask();
    }
}

#line 382
static inline void TUnitP__TearDown__default__run(void )
#line 382
{
  TUnitP__tearDownDone();
}

# 38 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
inline static void TUnitP__TearDown__run(void ){
#line 38
  TUnitP__TearDown__default__run();
#line 38
}
#line 38
# 361 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__runDone__runTask(void )
#line 361
{
  if (TUnitP__TestState__getState() == TUnitP__S_RUN) {
      TUnitP__TestState__forceState(TUnitP__S_TEARDOWN);
      TUnitP__TearDown__run();
    }
}

#line 394
static inline bool TUnitP__StatsQuery__default__isIdle(void )
#line 394
{
  return TRUE;
}

# 46 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/StatsQuery.nc"
inline static bool TUnitP__StatsQuery__isIdle(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = TUnitP__StatsQuery__default__isIdle();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 61 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static bool TUnitP__SendState__isIdle(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = StateImplP__State__isIdle(1U);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 349 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__waitForSendDone__runTask(void )
#line 349
{
  if (TUnitP__SendState__isIdle() && TUnitP__StatsQuery__isIdle()) {

      TUnitP__currentTest++;
      TUnitP__attemptTest();
    }
  else {

      TUnitP__waitForSendDone__postTask();
    }
}

#line 339
static inline void TUnitP__begin__runTask(void )
#line 339
{
  if (TUnitP__TUnitState__getState() == TUnitP__S_READY) {
      TUnitP__TUnitState__forceState(TUnitP__S_RUNNING);
      TUnitP__TestState__forceState(TUnitP__S_SETUP_ONETIME);
      TUnitP__currentTest = 0;
      TUnitP__SetUpOneTime__run();
    }
}

# 58 "/opt/tinyos-2.1.1/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 95 "/opt/tinyos-2.1.1/tos/system/ActiveMessageAddressC.nc"
static inline am_addr_t ActiveMessageAddressC__amAddress(void )
#line 95
{
  am_addr_t myAddr;

  /* atomic removed: atomic calls only */
#line 97
  myAddr = ActiveMessageAddressC__addr;
  return myAddr;
}

#line 61
static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void )
#line 61
{
  return ActiveMessageAddressC__amAddress();
}

# 50 "/opt/tinyos-2.1.1/tos/interfaces/ActiveMessageAddress.nc"
inline static am_addr_t TUnitP__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned int __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 319 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__dummyCalls(void )
#line 319
{
  assertTunitSuccess(0U);
#line 320
  ;
  assertTunitFail("", 1U);
#line 321
  ;
  if (0 != 0) {
#line 322
      assertEqualsFailed("", (uint32_t )0, (uint32_t )0, 2U);
    }
  else 
#line 322
    {
#line 322
      assertTunitSuccess(3U);
#line 322
      ;
    }
#line 322
  ;
  if (0 == 0) {
#line 323
      assertNotEqualsFailed("", (uint32_t )0, 4U);
    }
  else 
#line 323
    {
#line 323
      assertTunitSuccess(5U);
#line 323
      ;
    }
#line 323
  ;
  if (0 <= 0) {
#line 324
      assertResultIsBelowFailed("", (uint32_t )0, (uint32_t )0, 6U);
    }
  else 
#line 324
    {
#line 324
      assertTunitSuccess(7U);
#line 324
      ;
    }
#line 324
  ;
  if (0 >= 0) {
#line 325
      assertResultIsAboveFailed("", (uint32_t )0, (uint32_t )0, 8U);
    }
  else 
#line 325
    {
#line 325
      assertTunitSuccess(9U);
#line 325
      ;
    }
#line 325
  ;

  if (!TRUE) {
#line 327
      assertTunitFail("", 10U);
#line 327
      ;
    }
  else 
#line 327
    {
#line 327
      assertTunitSuccess(11U);
#line 327
      ;
    }
#line 327
  ;
  if (FALSE) {
#line 328
      assertTunitFail("", 12U);
#line 328
      ;
    }
  else 
#line 328
    {
#line 328
      assertTunitSuccess(13U);
#line 328
      ;
    }
#line 328
  ;
  if ((void *)0 != (void *)0) {
#line 329
      assertTunitFail("NULL"" was not null.", 14U);
#line 329
      ;
    }
  else 
#line 329
    {
#line 329
      assertTunitSuccess(15U);
#line 329
      ;
    }
#line 329
  ;
  if ((void *)0 == (void *)0) {
#line 330
      assertTunitFail("NULL"" was null", 16U);
#line 330
      ;
    }
  else 
#line 330
    {
#line 330
      assertTunitSuccess(17U);
#line 330
      ;
    }
#line 330
  ;
  setTUnitTestName((void *)0);
#line 331
  ;
}

#line 143
static inline error_t TUnitP__Init__init(void )
#line 143
{
  TUnitP__dummyCalls();
  TUnitP__driver = TUnitP__ActiveMessageAddress__amAddress() == 0;
  return SUCCESS;
}

# 214 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static __inline void SerialP__ackInit(void )
#line 214
{
  SerialP__ackQ.writePtr = SerialP__ackQ.readPtr = 0;
}

#line 205
static __inline void SerialP__rxInit(void )
#line 205
{
  SerialP__rxBuf.writePtr = SerialP__rxBuf.readPtr = 0;
  SerialP__rxState = SerialP__RXSTATE_NOSYNC;
  SerialP__rxByteCnt = 0;
  SerialP__rxProto = 0;
  SerialP__rxSeqno = 0;
  SerialP__rxCRC = 0;
}

#line 193
static __inline void SerialP__txInit(void )
#line 193
{
  uint8_t i;

  /* atomic removed: atomic calls only */
#line 195
  for (i = 0; i < SerialP__TX_BUFFER_COUNT; i++) SerialP__txBuf[i].state = SerialP__BUFFER_AVAILABLE;
  SerialP__txState = SerialP__TXSTATE_IDLE;
  SerialP__txByteCnt = 0;
  SerialP__txProto = 0;
  SerialP__txSeqno = 0;
  SerialP__txCRC = 0;
  SerialP__txPending = FALSE;
  SerialP__txIndex = 0;
}

#line 218
static inline error_t SerialP__Init__init(void )
#line 218
{

  SerialP__txInit();
  SerialP__rxInit();
  SerialP__ackInit();

  return SUCCESS;
}

# 45 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void )
#line 45
{
  memset(/*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ, /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 81 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
static inline error_t StateImplP__Init__init(void )
#line 81
{
  int i;

#line 83
  for (i = 0; i < 4U; i++) {
      StateImplP__state[i] = StateImplP__S_IDLE;
    }
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = StateImplP__Init__init();
#line 51
  __nesc_result = ecombine(__nesc_result, /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, SerialP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, TUnitP__Init__init());
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 86 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
static inline void */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__getPayload(am_id_t id, message_t *m, uint8_t len)
#line 86
{
  return /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(m, len);
}

# 124 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static void * /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__getPayload(am_id_t arg_0x2ae8cb9b4340, message_t * msg, uint8_t len){
#line 124
  void *__nesc_result;
#line 124

#line 124
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__getPayload(arg_0x2ae8cb9b4340, msg, len);
#line 124

#line 124
  return __nesc_result;
#line 124
}
#line 124
# 203 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static inline void */*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__getPayload(uint8_t id, message_t *m, uint8_t len)
#line 203
{
  return /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__getPayload(0, m, len);
}

# 114 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static void * /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__Send__getPayload(message_t * msg, uint8_t len){
#line 114
  void *__nesc_result;
#line 114

#line 114
  __nesc_result = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__getPayload(0U, msg, len);
#line 114

#line 114
  return __nesc_result;
#line 114
}
#line 114
# 65 "/opt/tinyos-2.1.1/tos/system/AMQueueEntryP.nc"
static inline void */*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__getPayload(message_t *m, uint8_t len)
#line 65
{
  return /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__Send__getPayload(m, len);
}

# 124 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static void * Link_TUnitProcessingP__SerialEventSend__getPayload(message_t * msg, uint8_t len){
#line 124
  void *__nesc_result;
#line 124

#line 124
  __nesc_result = /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__getPayload(msg, len);
#line 124

#line 124
  return __nesc_result;
#line 124
}
#line 124
# 92 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static inline void Link_TUnitProcessingP__Boot__booted(void )
#line 92
{
  int i;

#line 94
  for (i = 0; i < 5; i++) {
      __nesc_hton_uint8(((TUnitProcessingMsg *)Link_TUnitProcessingP__SerialEventSend__getPayload(&Link_TUnitProcessingP__eventMsg[i], 28))->cmd.data, 0xFF);
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 97
    Link_TUnitProcessingP__writingEventMsg = 0;
#line 97
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 98
    Link_TUnitProcessingP__sendingEventMsg = 0;
#line 98
    __nesc_atomic_end(__nesc_atomic); }
  Link_TUnitProcessingP__SerialSplitControl__start();
}

# 49 "/opt/tinyos-2.1.1/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 49
  Link_TUnitProcessingP__Boot__booted();
#line 49
}
#line 49
# 208 "/opt/tinyos-2.1.1/tos/chips/msp430X/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

# 133 "/opt/tinyos-2.1.1/tos/chips/msp430X/McuSleepC.nc"
static inline mcu_power_t McuSleepC__McuPowerOverride__default__lowestState(void )
#line 133
{
  return MSP430_POWER_LPM4;
}

# 54 "/opt/tinyos-2.1.1/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = McuSleepC__McuPowerOverride__default__lowestState();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 68 "/opt/tinyos-2.1.1/tos/chips/msp430X/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 68
{
  mcu_power_t pState = MSP430_POWER_LPM3;





  if (((((((
#line 71
  TA0CCTL0 & 0x0010 || TA0CCTL1 & 0x0010) || TA0CCTL2 & 0x0010) && (
  TA0CTL & (3 << 8)) == 2 << 8) || (
  UCA0CTL1 & (3 << 6)) != 0 << 6) || (
  UCA1CTL1 & (3 << 6)) != 0 << 6) || (
  UCB0CTL1 & (3 << 6)) != 0 << 6) || (
  UCB1CTL1 & (3 << 6)) != 0 << 6) {

    pState = MSP430_POWER_LPM1;
    }



  if (ADC12CTL0 & 0x0010) {
      if (ADC12CTL1 & (2 << 3)) {

          if (ADC12CTL1 & (1 << 3)) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 89
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 90
        if (ADC12CTL1 & 0x0400 && (TA0CTL & (3 << 8)) == 2 << 8) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 196 "/opt/tinyos-2.1.1/tos/chips/msp430X/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 196
{
  return m1 < m2 ? m1 : m2;
}

# 102 "/opt/tinyos-2.1.1/tos/chips/msp430X/McuSleepC.nc"
static inline void McuSleepC__computePowerState(void )
#line 102
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 107
{
  uint16_t temp;

#line 109
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }










  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 59 "/opt/tinyos-2.1.1/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 59
  McuSleepC__McuSleep__sleep();
#line 59
}
#line 59
# 67 "/opt/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP__popTask(void )
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) 
    {
      uint8_t id = SchedulerBasicP__m_head;

#line 72
      SchedulerBasicP__m_head = SchedulerBasicP__m_next[SchedulerBasicP__m_head];
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
        }
      SchedulerBasicP__m_next[id] = SchedulerBasicP__NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP__NO_TASK;
    }
}

#line 138
static inline void SchedulerBasicP__Scheduler__taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP__popTask()) == SchedulerBasicP__NO_TASK) 
            {
              SchedulerBasicP__McuSleep__sleep();
            }
        }
#line 150
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

# 61 "/opt/tinyos-2.1.1/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 61
  SchedulerBasicP__Scheduler__taskLoop();
#line 61
}
#line 61
# 88 "/opt/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId();
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 387 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__dataReceived(uint8_t data)
#line 387
{
  SerialP__rx_state_machine(FALSE, data);
}

# 83 "/opt/tinyos-2.1.1/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__dataReceived(uint8_t data){
#line 83
  SerialP__SerialFrameComm__dataReceived(data);
#line 83
}
#line 83
# 384 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__delimiterReceived(void )
#line 384
{
  SerialP__rx_state_machine(TRUE, 0);
}

# 74 "/opt/tinyos-2.1.1/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__delimiterReceived(void ){
#line 74
  SerialP__SerialFrameComm__delimiterReceived();
#line 74
}
#line 74
# 61 "/opt/tinyos-2.1.1/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__UartStream__receivedByte(uint8_t data)
#line 61
{






  if (data == HDLC_FLAG_BYTE) {

      HdlcTranslateC__SerialFrameComm__delimiterReceived();
      return;
    }
  else {
#line 73
    if (data == HDLC_CTLESC_BYTE) {

        HdlcTranslateC__state.receiveEscape = 1;
        return;
      }
    else {
#line 78
      if (HdlcTranslateC__state.receiveEscape) {

          HdlcTranslateC__state.receiveEscape = 0;
          data = data ^ 0x20;
        }
      }
    }
#line 83
  HdlcTranslateC__SerialFrameComm__dataReceived(data);
}

# 252 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(uint8_t id, uint8_t byte)
#line 252
{
}

# 79 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__receivedByte(uint8_t arg_0x2ae8cbc4e7e0, uint8_t byte){
#line 79
  switch (arg_0x2ae8cbc4e7e0) {
#line 79
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 79
      HdlcTranslateC__UartStream__receivedByte(byte);
#line 79
      break;
#line 79
    default:
#line 79
      /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(arg_0x2ae8cbc4e7e0, byte);
#line 79
      break;
#line 79
    }
#line 79
}
#line 79
# 116 "/opt/tinyos-2.1.1/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__UartStream__receiveDone(uint8_t *buf, uint16_t len, error_t error)
#line 116
{
}

# 253 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error)
#line 253
{
}

# 99 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__receiveDone(uint8_t arg_0x2ae8cbc4e7e0, uint8_t * buf, uint16_t len, error_t error){
#line 99
  switch (arg_0x2ae8cbc4e7e0) {
#line 99
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 99
      HdlcTranslateC__UartStream__receiveDone(buf, len, error);
#line 99
      break;
#line 99
    default:
#line 99
      /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(arg_0x2ae8cbc4e7e0, buf, len, error);
#line 99
      break;
#line 99
    }
#line 99
}
#line 99
# 163 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciInterrupts__rxDone(uint8_t id, uint8_t data)
#line 163
{

  if (/*Msp430Uart0P.UartP*/Msp430UartP__0__m_rx_buf) {
      /*Msp430Uart0P.UartP*/Msp430UartP__0__m_rx_buf[/*Msp430Uart0P.UartP*/Msp430UartP__0__m_rx_pos++] = data;
      if (/*Msp430Uart0P.UartP*/Msp430UartP__0__m_rx_pos >= /*Msp430Uart0P.UartP*/Msp430UartP__0__m_rx_len) {
          uint8_t *buf = /*Msp430Uart0P.UartP*/Msp430UartP__0__m_rx_buf;

#line 169
          /*Msp430Uart0P.UartP*/Msp430UartP__0__m_rx_buf = (void *)0;
          /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__receiveDone(id, buf, /*Msp430Uart0P.UartP*/Msp430UartP__0__m_rx_len, SUCCESS);
        }
    }
  else 
#line 172
    {
      /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__receivedByte(id, data);
    }
}

# 89 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 89
{
}

# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
inline static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__rxDone(uint8_t arg_0x2ae8cbfc6410, uint8_t data){
#line 85
  switch (arg_0x2ae8cbfc6410) {
#line 85
    case /*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID:
#line 85
      /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciInterrupts__rxDone(/*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID, data);
#line 85
      break;
#line 85
    default:
#line 85
      /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(arg_0x2ae8cbfc6410, data);
#line 85
      break;
#line 85
    }
#line 85
}
#line 85
# 80 "/opt/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse(void ){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse();
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 83 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 83
{
  if (/*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__rxDone(/*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId(), data);
    }
}

# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
inline static void HplMsp430UsciA0P__Interrupts__rxDone(uint8_t data){
#line 85
  /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(data);
#line 85
}
#line 85
# 86 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__UsciRawInterrupts__rxDone(uint8_t temp)
#line 86
{
  HplMsp430UsciA0P__Interrupts__rxDone(temp);
}

# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB0RawInterruptsP__UsciA__rxDone(uint8_t data){
#line 85
  HplMsp430UsciA0P__UsciRawInterrupts__rxDone(data);
#line 85
}
#line 85
# 391 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static inline bool SerialP__valid_rx_proto(uint8_t proto)
#line 391
{
  switch (proto) {
      case SERIAL_PROTO_PACKET_ACK: 
        return TRUE;
      case SERIAL_PROTO_ACK: 
        case SERIAL_PROTO_PACKET_NOACK: 
          default: 
            return FALSE;
    }
}

# 192 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer(void )
#line 192
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked = 1;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked = 1;
    }
}

#line 188
static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked(void )
#line 188
{
  return /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which ? /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked : /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked;
}

#line 215
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void )
#line 215
{
  error_t result = SUCCESS;

  /* atomic removed: atomic calls only */
#line 217
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked()) {


        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_BEGIN;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = 0;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = TOS_SERIAL_UNKNOWN_ID;
      }
    else {
        result = EBUSY;
      }
  }
  return result;
}

# 51 "/opt/tinyos-2.1.1/tos/lib/serial/ReceiveBytePacket.nc"
inline static error_t SerialP__ReceiveBytePacket__startPacket(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 309 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static __inline uint16_t SerialP__rx_current_crc(void )
#line 309
{
  uint16_t crc;
  uint8_t tmp = SerialP__rxBuf.writePtr;

#line 312
  tmp = tmp == 0 ? SerialP__RX_DATA_BUFFER_SIZE : tmp - 1;
  crc = SerialP__rxBuf.buf[tmp] & 0x00ff;
  crc = (crc << 8) & 0xFF00;
  tmp = tmp == 0 ? SerialP__RX_DATA_BUFFER_SIZE : tmp - 1;
  crc |= SerialP__rxBuf.buf[tmp] & 0x00FF;
  return crc;
}

# 69 "/opt/tinyos-2.1.1/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP__ReceiveBytePacket__endPacket(error_t result){
#line 69
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(result);
#line 69
}
#line 69
# 210 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap(void )
#line 210
{
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which ? 0 : 1;
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer = (uint8_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which];
}

# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 232 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static __inline bool SerialP__ack_queue_is_full(void )
#line 232
{
  uint8_t tmp;
#line 233
  uint8_t tmp2;

  /* atomic removed: atomic calls only */
#line 234
  {
    tmp = SerialP__ackQ.writePtr;
    tmp2 = SerialP__ackQ.readPtr;
  }
  if (++tmp > SerialP__ACK_QUEUE_SIZE) {
#line 238
    tmp = 0;
    }
#line 239
  return tmp == tmp2;
}







static __inline void SerialP__ack_queue_push(uint8_t token)
#line 248
{
  if (!SerialP__ack_queue_is_full()) {
      /* atomic removed: atomic calls only */
#line 250
      {
        SerialP__ackQ.buf[SerialP__ackQ.writePtr] = token;
        if (++ SerialP__ackQ.writePtr > SerialP__ACK_QUEUE_SIZE) {
#line 252
          SerialP__ackQ.writePtr = 0;
          }
      }
#line 254
      SerialP__MaybeScheduleTx();
    }
}

# 233 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t b)
#line 233
{
  /* atomic removed: atomic calls only */
#line 234
  {
    switch (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state) {
        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_BEGIN: 
          /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_DATA;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(b);
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = b;
        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_DATA: 
          if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex < sizeof(message_t )) {
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex] = b;
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex++;
            }
          else {
            }




        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE: 
          default: 
#line 255
            ;
      }
  }
}

# 58 "/opt/tinyos-2.1.1/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP__ReceiveBytePacket__byteReceived(uint8_t data){
#line 58
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(data);
#line 58
}
#line 58
# 299 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static __inline uint8_t SerialP__rx_buffer_top(void )
#line 299
{
  uint8_t tmp = SerialP__rxBuf.buf[SerialP__rxBuf.readPtr];

#line 301
  return tmp;
}

#line 303
static __inline uint8_t SerialP__rx_buffer_pop(void )
#line 303
{
  uint8_t tmp = SerialP__rxBuf.buf[SerialP__rxBuf.readPtr];

#line 305
  if (++ SerialP__rxBuf.readPtr > SerialP__RX_DATA_BUFFER_SIZE) {
#line 305
    SerialP__rxBuf.readPtr = 0;
    }
#line 306
  return tmp;
}

#line 295
static __inline void SerialP__rx_buffer_push(uint8_t data)
#line 295
{
  SerialP__rxBuf.buf[SerialP__rxBuf.writePtr] = data;
  if (++ SerialP__rxBuf.writePtr > SerialP__RX_DATA_BUFFER_SIZE) {
#line 297
    SerialP__rxBuf.writePtr = 0;
    }
}

# 55 "/opt/tinyos-2.1.1/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__SerialFrameComm__resetReceive(void )
#line 55
{
  HdlcTranslateC__state.receiveEscape = 0;
}

# 68 "/opt/tinyos-2.1.1/tos/lib/serial/SerialFrameComm.nc"
inline static void SerialP__SerialFrameComm__resetReceive(void ){
#line 68
  HdlcTranslateC__SerialFrameComm__resetReceive();
#line 68
}
#line 68
# 81 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciAB0RawInterruptsP.nc"
static inline void HplMsp430UsciAB0RawInterruptsP__UsciB__default__rxDone(uint8_t temp)
#line 81
{
  return;
}

# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB0RawInterruptsP__UsciB__rxDone(uint8_t data){
#line 85
  HplMsp430UsciAB0RawInterruptsP__UsciB__default__rxDone(data);
#line 85
}
#line 85
# 251 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__sendDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error)
#line 251
{
}

# 57 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__sendDone(uint8_t arg_0x2ae8cbc4e7e0, uint8_t * buf, uint16_t len, error_t error){
#line 57
  switch (arg_0x2ae8cbc4e7e0) {
#line 57
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 57
      HdlcTranslateC__UartStream__sendDone(buf, len, error);
#line 57
      break;
#line 57
    default:
#line 57
      /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__sendDone(arg_0x2ae8cbc4e7e0, buf, len, error);
#line 57
      break;
#line 57
    }
#line 57
}
#line 57
# 291 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__Usci__tx(uint8_t data)
#line 291
{
  HplMsp430UsciA0P__UCA0TXBUF = data;
}

# 139 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
inline static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__tx(uint8_t data){
#line 139
  HplMsp430UsciA0P__Usci__tx(data);
#line 139
}
#line 139
# 228 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__Usci__clrTxIntr(void )
#line 228
{
  HplMsp430UsciA0P__IFG2 &= ~(1 << 1);
}

# 128 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA.nc"
inline static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__clrTxIntr(void ){
#line 128
  HplMsp430UsciA0P__Usci__clrTxIntr();
#line 128
}
#line 128
# 192 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciInterrupts__txDone(uint8_t id)
#line 192
{
  /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__clrTxIntr();
  if (/*Msp430Uart0P.UartP*/Msp430UartP__0__current_owner != id) {
      uint8_t *buf = /*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_buf;

#line 196
      /*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_buf = (void *)0;
      /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__sendDone(id, buf, /*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_len, 0x0080);
    }
  else {
#line 199
    if (/*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_pos < /*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_len) {
        /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__tx(/*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_buf[/*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_pos++]);
      }
    else {
        uint8_t *buf = /*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_buf;

#line 204
        /*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_buf = (void *)0;
        /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__sendDone(id, buf, /*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_len, SUCCESS);
      }
    }
}

# 88 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(uint8_t id)
#line 88
{
}

# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
inline static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__txDone(uint8_t arg_0x2ae8cbfc6410){
#line 80
  switch (arg_0x2ae8cbfc6410) {
#line 80
    case /*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID:
#line 80
      /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciInterrupts__txDone(/*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID);
#line 80
      break;
#line 80
    default:
#line 80
      /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(arg_0x2ae8cbfc6410);
#line 80
      break;
#line 80
    }
#line 80
}
#line 80
# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone(void )
#line 78
{
  if (/*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__txDone(/*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId());
    }
}

# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
inline static void HplMsp430UsciA0P__Interrupts__txDone(void ){
#line 80
  /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone();
#line 80
}
#line 80
# 90 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciA0P.nc"
static inline void HplMsp430UsciA0P__UsciRawInterrupts__txDone(void )
#line 90
{
  HplMsp430UsciA0P__Interrupts__txDone();
}

# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB0RawInterruptsP__UsciA__txDone(void ){
#line 80
  HplMsp430UsciA0P__UsciRawInterrupts__txDone();
#line 80
}
#line 80
# 54 "/opt/tinyos-2.1.1/tos/lib/serial/SerialFrameComm.nc"
inline static error_t SerialP__SerialFrameComm__putData(uint8_t data){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = HdlcTranslateC__SerialFrameComm__putData(data);
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 513 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SendBytePacket__completeSend(void )
#line 513
{
  bool ret = 0x0080;

  /* atomic removed: atomic calls only */
#line 515
  {
    SerialP__txBuf[SerialP__TX_DATA_INDEX].state = SerialP__BUFFER_COMPLETE;
    ret = SUCCESS;
  }
  return ret;
}

# 60 "/opt/tinyos-2.1.1/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = SerialP__SendBytePacket__completeSend();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 167 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void )
#line 167
{
  uint8_t b;
  uint8_t indx;

  /* atomic removed: atomic calls only */
#line 170
  {
    b = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex];
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex++;
    indx = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex;
  }
  if (indx > /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend();
      return 0;
    }
  else {
      return b;
    }
}

# 70 "/opt/tinyos-2.1.1/tos/lib/serial/SendBytePacket.nc"
inline static uint8_t SerialP__SendBytePacket__nextByte(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 642 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__putDone(void )
#line 642
{
  {
    error_t txResult = SUCCESS;

    switch (SerialP__txState) {

        case SerialP__TXSTATE_PROTO: 

          txResult = SerialP__SerialFrameComm__putData(SerialP__txProto);

        SerialP__txState = SerialP__TXSTATE_INFO;



        SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txProto);
        break;

        case SerialP__TXSTATE_SEQNO: 
          txResult = SerialP__SerialFrameComm__putData(SerialP__txSeqno);
        SerialP__txState = SerialP__TXSTATE_INFO;
        SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txSeqno);
        break;

        case SerialP__TXSTATE_INFO: /* atomic removed: atomic calls only */
          {
            txResult = SerialP__SerialFrameComm__putData(SerialP__txBuf[SerialP__txIndex].buf);
            SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txBuf[SerialP__txIndex].buf);
            ++SerialP__txByteCnt;

            if (SerialP__txIndex == SerialP__TX_DATA_INDEX) {
                uint8_t nextByte;

#line 673
                nextByte = SerialP__SendBytePacket__nextByte();
                if (SerialP__txBuf[SerialP__txIndex].state == SerialP__BUFFER_COMPLETE || SerialP__txByteCnt >= SerialP__SERIAL_MTU) {
                    SerialP__txState = SerialP__TXSTATE_FCS1;
                  }
                else {
                    SerialP__txBuf[SerialP__txIndex].buf = nextByte;
                  }
              }
            else {
                SerialP__txState = SerialP__TXSTATE_FCS1;
              }
          }
        break;

        case SerialP__TXSTATE_FCS1: 
          txResult = SerialP__SerialFrameComm__putData(SerialP__txCRC & 0xff);
        SerialP__txState = SerialP__TXSTATE_FCS2;
        break;

        case SerialP__TXSTATE_FCS2: 
          txResult = SerialP__SerialFrameComm__putData((SerialP__txCRC >> 8) & 0xff);
        SerialP__txState = SerialP__TXSTATE_ENDFLAG;
        break;

        case SerialP__TXSTATE_ENDFLAG: 
          txResult = SerialP__SerialFrameComm__putDelimiter();
        SerialP__txState = SerialP__TXSTATE_ENDWAIT;
        break;

        case SerialP__TXSTATE_ENDWAIT: 
          SerialP__txState = SerialP__TXSTATE_FINISH;
        case SerialP__TXSTATE_FINISH: 
          SerialP__MaybeScheduleTx();
        break;
        case SerialP__TXSTATE_ERROR: 
          default: 
            txResult = 0x0080;
        break;
      }

    if (txResult != SUCCESS) {
        SerialP__txState = SerialP__TXSTATE_ERROR;
        SerialP__MaybeScheduleTx();
      }
  }
}

# 89 "/opt/tinyos-2.1.1/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__putDone(void ){
#line 89
  SerialP__SerialFrameComm__putDone();
#line 89
}
#line 89
# 77 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciAB0RawInterruptsP.nc"
static inline void HplMsp430UsciAB0RawInterruptsP__UsciB__default__txDone(void )
#line 77
{
  return;
}

# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB0RawInterruptsP__UsciB__txDone(void ){
#line 80
  HplMsp430UsciAB0RawInterruptsP__UsciB__default__txDone();
#line 80
}
#line 80
# 228 "/opt/tinyos-2.1.1/tos/chips/msp430X/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (({
#line 230
    uint16_t __x;

#line 230
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 230
   & 0x0008) != 0;

#line 231
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 197 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
__attribute((noinline))   void assertEqualsFailed(char *failMsg, uint32_t expected, uint32_t actual, uint8_t assertionId)
#line 197
{
  if (!TUnitP__TUnitState__isIdle()) {
      TUnitP__TUnitProcessing__testEqualsFailed(TUnitP__currentTest, failMsg, expected, actual, assertionId);
    }
}

# 133 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
static bool StateImplP__State__isState(uint8_t id, uint8_t myState)
#line 133
{
  bool isState;

#line 135
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 135
    isState = StateImplP__state[id] == myState;
#line 135
    __nesc_atomic_end(__nesc_atomic); }
  return isState;
}

# 203 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static error_t Link_TUnitProcessingP__insert(uint8_t cmd, uint8_t testId, char *failMsg, uint32_t expected, uint32_t actual, uint8_t assertionId)
#line 203
{
  unsigned char __nesc_temp43;
  unsigned char *__nesc_temp42;
#line 204
  TUnitProcessingMsg *tunitMsg;
  bool failed = (((cmd == TUNITPROCESSING_EVENT_TESTRESULT_FAILED
   || cmd == TUNITPROCESSING_EVENT_TESTRESULT_EQUALS_FAILED)
   || cmd == TUNITPROCESSING_EVENT_TESTRESULT_NOTEQUALS_FAILED)
   || cmd == TUNITPROCESSING_EVENT_TESTRESULT_BELOW_FAILED)
   || cmd == TUNITPROCESSING_EVENT_TESTRESULT_ABOVE_FAILED;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 211
    {
      while (TRUE) {
          if (__nesc_ntoh_uint8((tunitMsg = (TUnitProcessingMsg *)(&Link_TUnitProcessingP__eventMsg[Link_TUnitProcessingP__writingEventMsg])->data)->cmd.data) == Link_TUnitProcessingP__EMPTY) {


              __nesc_hton_uint8(tunitMsg->cmd.data, cmd);
              __nesc_hton_uint8(tunitMsg->id.data, testId);
              __nesc_hton_uint8(tunitMsg->failMsgLength.data, 0);
              __nesc_hton_uint32(tunitMsg->expected.data, expected);
              __nesc_hton_uint32(tunitMsg->actual.data, actual);
              __nesc_hton_uint8(tunitMsg->assertionId.data, assertionId);

              memset(tunitMsg->failMsg, 0x0, 28 - 13);

              if (failed && failMsg != (void *)0) {
                  while (*failMsg && __nesc_ntoh_uint8(tunitMsg->failMsgLength.data) < 28 - 13) {
                      __nesc_hton_uint8(tunitMsg->failMsg[__nesc_ntoh_uint8(tunitMsg->failMsgLength.data)].data, * failMsg++);
                      (__nesc_temp42 = tunitMsg->failMsgLength.data, __nesc_hton_uint8(__nesc_temp42, (__nesc_temp43 = __nesc_ntoh_uint8(__nesc_temp42)) + 1), __nesc_temp43);
                    }
                }

              Link_TUnitProcessingP__writingEventMsg++;
              Link_TUnitProcessingP__writingEventMsg %= 5;

              if (failed && failMsg != (void *)0 && *failMsg) {

                  if (__nesc_ntoh_uint8(((TUnitProcessingMsg *)(&Link_TUnitProcessingP__eventMsg[Link_TUnitProcessingP__writingEventMsg])->data)->cmd.data) != Link_TUnitProcessingP__EMPTY) {

                      __nesc_hton_int8(tunitMsg->lastMsg.data, TRUE);
                      break;
                    }
                  else {

                      __nesc_hton_int8(tunitMsg->lastMsg.data, FALSE);
                    }
                }
              else {

                  __nesc_hton_int8(tunitMsg->lastMsg.data, TRUE);
                  break;
                }
            }
          else {

              Link_TUnitProcessingP__attemptEventSend();
              {
                unsigned char __nesc_temp = 
#line 256
                0x0080;

                {
#line 256
                  __nesc_atomic_end(__nesc_atomic); 
#line 256
                  return __nesc_temp;
                }
              }
            }
        }
    }
#line 261
    __nesc_atomic_end(__nesc_atomic); }
#line 261
  Link_TUnitProcessingP__attemptEventSend();
  return SUCCESS;
}





static void Link_TUnitProcessingP__attemptEventSend(void )
#line 269
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 270
    {
      if (Link_TUnitProcessingP__SendState__isIdle()) {

          if (__nesc_ntoh_uint8(((TUnitProcessingMsg *)(&Link_TUnitProcessingP__eventMsg[Link_TUnitProcessingP__sendingEventMsg])->data)->cmd.data) != Link_TUnitProcessingP__EMPTY) {

              Link_TUnitProcessingP__SendState__forceState(Link_TUnitProcessingP__S_BUSY);
              Link_TUnitProcessingP__sendEventMsg__postTask();
            }
        }
    }
#line 279
    __nesc_atomic_end(__nesc_atomic); }
}

# 159 "/opt/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 161
    {
#line 161
      {
        unsigned char __nesc_temp = 
#line 161
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
#line 161
          __nesc_atomic_end(__nesc_atomic); 
#line 161
          return __nesc_temp;
        }
      }
    }
#line 164
    __nesc_atomic_end(__nesc_atomic); }
}

# 203 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
__attribute((noinline))   void assertNotEqualsFailed(char *failMsg, uint32_t actual, uint8_t assertionId)
#line 203
{
  if (!TUnitP__TUnitState__isIdle()) {
      TUnitP__TUnitProcessing__testNotEqualsFailed(TUnitP__currentTest, failMsg, actual, assertionId);
    }
}

__attribute((noinline))   void assertResultIsBelowFailed(char *failMsg, uint32_t upperbound, uint32_t actual, uint8_t assertionId)
#line 209
{
  if (!TUnitP__TUnitState__isIdle()) {
      TUnitP__TUnitProcessing__testResultIsBelowFailed(TUnitP__currentTest, failMsg, upperbound, actual, assertionId);
    }
}

__attribute((noinline))   void assertResultIsAboveFailed(char *failMsg, uint32_t lowerbound, uint32_t actual, uint8_t assertionId)
#line 215
{
  if (!TUnitP__TUnitState__isIdle()) {
      TUnitP__TUnitProcessing__testResultIsAboveFailed(TUnitP__currentTest, failMsg, lowerbound, actual, assertionId);
    }
}

__attribute((noinline))   void assertTunitSuccess(uint8_t assertionId)
#line 221
{
  if (!TUnitP__TUnitState__isIdle()) {
      TUnitP__TUnitProcessing__testSuccess(TUnitP__currentTest, assertionId);
    }
}

__attribute((noinline))   void assertTunitFail(char *failMsg, uint8_t assertionId)
#line 227
{
  if (!TUnitP__TUnitState__isIdle()) {
      TUnitP__TUnitProcessing__testFailed(TUnitP__currentTest, failMsg, assertionId);
    }
}




__attribute((noinline))   void setTUnitTestName(char *name)
#line 236
{
}

# 11 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(50)))  void sig_TIMERA0_VECTOR(void )
#line 11
{



  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 169 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired();
    }
}

# 21 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(48)))  void sig_TIMERA1_VECTOR(void )
#line 21
{



  Msp430TimerCommonP__VectorTimerA1__fired();
}




__attribute((wakeup)) __attribute((interrupt(58)))  void sig_TIMERB0_VECTOR(void )
#line 31
{



  Msp430TimerCommonP__VectorTimerB0__fired();
}

# 135 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x2ae8cb7b3690){
#line 28
  switch (arg_0x2ae8cb7b3690) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired();
#line 28
      break;
#line 28
    case 3:
#line 28
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired();
#line 28
      break;
#line 28
    case 4:
#line 28
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired();
#line 28
      break;
#line 28
    case 6:
#line 28
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired();
#line 28
      break;
#line 28
    case 7:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x2ae8cb7b3690);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 41 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(56)))  void sig_TIMERB1_VECTOR(void )
#line 41
{



  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 52 "/opt/tinyos-2.1.1/tos/system/RealMainP.nc"
  int main(void )
#line 52
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 60
      ;

      RealMainP__Scheduler__init();





      RealMainP__PlatformInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;





      RealMainP__SoftwareInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP__Boot__booted();


  RealMainP__Scheduler__taskLoop();




  return -1;
}

# 123 "/opt/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__Scheduler__runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 127
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 131
          FALSE;

#line 131
          return __nesc_temp;
        }
      }
  }
#line 134
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

#line 164
static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x2ae8cb614a18){
#line 64
  switch (arg_0x2ae8cb614a18) {
#line 64
    case TUnitP__begin:
#line 64
      TUnitP__begin__runTask();
#line 64
      break;
#line 64
    case TUnitP__waitForSendDone:
#line 64
      TUnitP__waitForSendDone__runTask();
#line 64
      break;
#line 64
    case TUnitP__runDone:
#line 64
      TUnitP__runDone__runTask();
#line 64
      break;
#line 64
    case Link_TUnitProcessingP__sendEventMsg:
#line 64
      Link_TUnitProcessingP__sendEventMsg__runTask();
#line 64
      break;
#line 64
    case Link_TUnitProcessingP__allDone:
#line 64
      Link_TUnitProcessingP__allDone__runTask();
#line 64
      break;
#line 64
    case /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask:
#line 64
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask();
#line 64
      break;
#line 64
    case /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask:
#line 64
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask();
#line 64
      break;
#line 64
    case SerialP__RunTx:
#line 64
      SerialP__RunTx__runTask();
#line 64
      break;
#line 64
    case SerialP__startDoneTask:
#line 64
      SerialP__startDoneTask__runTask();
#line 64
      break;
#line 64
    case SerialP__stopDoneTask:
#line 64
      SerialP__stopDoneTask__runTask();
#line 64
      break;
#line 64
    case SerialP__defaultSerialFlushTask:
#line 64
      SerialP__defaultSerialFlushTask__runTask();
#line 64
      break;
#line 64
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone:
#line 64
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask();
#line 64
      break;
#line 64
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask:
#line 64
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask();
#line 64
      break;
#line 64
    case /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask:
#line 64
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x2ae8cb614a18);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 117 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__ResourceConfigure__configure(uint8_t id)
#line 117
{
  msp430_uart_union_config_t *config = /*Msp430Uart0P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(id);

#line 119
  /*Msp430Uart0P.UartP*/Msp430UartP__0__m_byte_time = config->uartConfig.ubr / 2;
  /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__setModeUart(config);
  /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__enableIntr();
}

# 166 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void )
#line 166
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket();
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current < 1) {
      error_t nextErr;
      message_t *nextMsg = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg;
      am_id_t nextId = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(nextMsg);
      am_addr_t nextDest = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(nextMsg);
      uint8_t len = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(nextMsg);

#line 174
      nextErr = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(nextId, nextDest, nextMsg, len);
      if (nextErr != SUCCESS) {
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask();
        }
    }
}

# 137 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(message_t *amsg)
#line 137
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 139
  return __nesc_ntoh_uint16(header->dest.data);
}

#line 57
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 59
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg);

  if (len > /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength()) {
      return ESIZE;
    }

  __nesc_hton_uint16(header->dest.data, dest);





  __nesc_hton_uint8(header->type.data, id);
  __nesc_hton_uint8(header->length.data, len);

  return /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(msg, len);
}

# 502 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static void SerialP__MaybeScheduleTx(void )
#line 502
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 503
    {
      if (SerialP__txPending == 0) {
          if (SerialP__RunTx__postTask() == SUCCESS) {
              SerialP__txPending = 1;
            }
        }
    }
#line 509
    __nesc_atomic_end(__nesc_atomic); }
}

# 118 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/Link_TUnitProcessingP.nc"
static void Link_TUnitProcessingP__SerialEventSend__sendDone(message_t *msg, error_t error)
#line 118
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 119
    {
      __nesc_hton_uint8(((TUnitProcessingMsg *)(&Link_TUnitProcessingP__eventMsg[Link_TUnitProcessingP__sendingEventMsg])->data)->cmd.data, Link_TUnitProcessingP__EMPTY);
      Link_TUnitProcessingP__sendingEventMsg++;
      Link_TUnitProcessingP__sendingEventMsg %= 5;
    }
#line 123
    __nesc_atomic_end(__nesc_atomic); }
  Link_TUnitProcessingP__SendState__toIdle();
  Link_TUnitProcessingP__attemptEventSend();
}

# 56 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static void TUnitP__TestState__toIdle(void ){
#line 56
  StateImplP__State__toIdle(3U);
#line 56
}
#line 56
# 174 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id)
#line 174
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 175
    {
      if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id && /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) {
          unsigned char __nesc_temp = 
#line 176
          TRUE;

          {
#line 176
            __nesc_atomic_end(__nesc_atomic); 
#line 176
            return __nesc_temp;
          }
        }
      else 
#line 177
        {
          unsigned char __nesc_temp = 
#line 177
          FALSE;

          {
#line 177
            __nesc_atomic_end(__nesc_atomic); 
#line 177
            return __nesc_temp;
          }
        }
    }
#line 180
    __nesc_atomic_end(__nesc_atomic); }
}

# 143 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
static uint8_t StateImplP__State__getState(uint8_t id)
#line 143
{
  uint8_t theState;

#line 145
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 145
    theState = StateImplP__state[id];
#line 145
    __nesc_atomic_end(__nesc_atomic); }
  return theState;
}

# 267 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static void TUnitP__setUpOneTimeDone(void )
#line 267
{
  if (TUnitP__TestState__getState() == TUnitP__S_SETUP_ONETIME) {
      TUnitP__TestState__toIdle();
      if (!TUnitP__driver) {
          TUnitP__TestState__forceState(TUnitP__S_RUN);
        }
      else {
          TUnitP__attemptTest();
        }
    }
}

#line 301
static void TUnitP__attemptTest(void )
#line 301
{
  if (TUnitP__currentTest < 1U) {
      if (TUnitP__TestState__requestState(TUnitP__S_SETUP) == SUCCESS) {

          TUnitP__SetUp__run();
        }
    }
  else 
    {
      TUnitP__TUnitProcessing__allDone();
    }
}

# 347 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static void SerialP__testOff(void )
#line 347
{
  bool turnOff = FALSE;

#line 349
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 349
    {
      if (SerialP__txState == SerialP__TXSTATE_INACTIVE && 
      SerialP__rxState == SerialP__RXSTATE_INACTIVE) {
          turnOff = TRUE;
        }
    }
#line 354
    __nesc_atomic_end(__nesc_atomic); }
  if (turnOff) {
      SerialP__stopDoneTask__postTask();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 357
        SerialP__offPending = FALSE;
#line 357
        __nesc_atomic_end(__nesc_atomic); }
    }
  else {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
        SerialP__offPending = TRUE;
#line 360
        __nesc_atomic_end(__nesc_atomic); }
    }
}

# 86 "/opt/tinyos-2.1.1/tos/lib/serial/HdlcTranslateC.nc"
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void )
#line 86
{
  HdlcTranslateC__state.sendEscape = 0;
  HdlcTranslateC__m_data = HDLC_FLAG_BYTE;
  return HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
}

# 177 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__send(uint8_t id, uint8_t *buf, uint16_t len)
#line 177
{
  if (/*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__isOwner(id) == FALSE) {
    return 0x0080;
    }
#line 180
  if (len == 0) {
    return 0x0080;
    }
  else {
#line 182
    if (/*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_buf) {
      return EBUSY;
      }
    }
#line 184
  /*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_buf = buf;
  /*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_len = len;
  /*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_pos = 0;
  /*Msp430Uart0P.UartP*/Msp430UartP__0__current_owner = id;
  /*Msp430Uart0P.UartP*/Msp430UartP__0__Usci__tx(buf[/*Msp430Uart0P.UartP*/Msp430UartP__0__m_tx_pos++]);
  return SUCCESS;
}

# 124 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
static void */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__getPayload(message_t *msg, uint8_t len)
#line 124
{
  if (len > /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength()) {
      return (void *)0;
    }
  else {
      return (void * )msg->data;
    }
}

# 46 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciAB0RawInterruptsP.nc"
__attribute((wakeup)) __attribute((interrupt(46)))  void sig_USCIAB0RX_VECTOR(void )
#line 46
{
  uint8_t temp;

#line 48
  if (IFG2 & (1 << 0)) {
      temp = UCA0RXBUF;
      HplMsp430UsciAB0RawInterruptsP__UsciA__rxDone(temp);
    }
  if (IFG2 & (1 << 2)) {
      temp = UCB0RXBUF;
      HplMsp430UsciAB0RawInterruptsP__UsciB__rxDone(temp);
    }
}

# 150 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static bool /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void )
#line 150
{
  /* atomic removed: atomic calls only */
#line 151
  {
    if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 153
        FALSE;

#line 153
        return __nesc_temp;
      }
  }
#line 155
  return TRUE;
}

# 402 "/opt/tinyos-2.1.1/tos/lib/serial/SerialP.nc"
static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data)
#line 402
{

  switch (SerialP__rxState) {

      case SerialP__RXSTATE_NOSYNC: 
        if (isDelimeter) {
            SerialP__rxInit();
            SerialP__rxState = SerialP__RXSTATE_PROTO;
          }
      break;

      case SerialP__RXSTATE_PROTO: 
        if (!isDelimeter) {
            SerialP__rxCRC = crcByte(SerialP__rxCRC, data);
            SerialP__rxState = SerialP__RXSTATE_TOKEN;
            SerialP__rxProto = data;
            if (!SerialP__valid_rx_proto(SerialP__rxProto)) {
              goto nosync;
              }
            if (SerialP__rxProto != SERIAL_PROTO_PACKET_ACK) {
                goto nosync;
              }
            if (SerialP__ReceiveBytePacket__startPacket() != SUCCESS) {
                goto nosync;
              }
          }
      break;

      case SerialP__RXSTATE_TOKEN: 
        if (isDelimeter) {
            goto nosync;
          }
        else {
            SerialP__rxSeqno = data;
            SerialP__rxCRC = crcByte(SerialP__rxCRC, SerialP__rxSeqno);
            SerialP__rxState = SerialP__RXSTATE_INFO;
          }
      break;

      case SerialP__RXSTATE_INFO: 
        if (SerialP__rxByteCnt < SerialP__SERIAL_MTU) {
            if (isDelimeter) {
                if (SerialP__rxByteCnt >= 2) {
                    if (SerialP__rx_current_crc() == SerialP__rxCRC) {
                        SerialP__ReceiveBytePacket__endPacket(SUCCESS);
                        SerialP__ack_queue_push(SerialP__rxSeqno);
                        goto nosync;
                      }
                    else {
                        goto nosync;
                      }
                  }
                else {
                    goto nosync;
                  }
              }
            else {
                if (SerialP__rxByteCnt >= 2) {
                    SerialP__ReceiveBytePacket__byteReceived(SerialP__rx_buffer_top());
                    SerialP__rxCRC = crcByte(SerialP__rxCRC, SerialP__rx_buffer_pop());
                  }
                SerialP__rx_buffer_push(data);
                SerialP__rxByteCnt++;
              }
          }
        else 

          {
            goto nosync;
          }
      break;

      default: 
        goto nosync;
    }
  goto done;

  nosync: 

    SerialP__rxInit();
  SerialP__SerialFrameComm__resetReceive();
  SerialP__ReceiveBytePacket__endPacket(0x0080);
  if (SerialP__offPending) {
      SerialP__rxState = SerialP__RXSTATE_INACTIVE;
      SerialP__testOff();
    }
  else {
    if (isDelimeter) {
        SerialP__rxState = SerialP__RXSTATE_PROTO;
      }
    }
  done: ;
}

# 80 "/opt/tinyos-2.1.1/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b)
#line 80
{
  crc = (uint8_t )(crc >> 8) | (crc << 8);
  crc ^= b;
  crc ^= (uint8_t )(crc & 0xff) >> 4;
  crc ^= crc << 12;
  crc ^= (crc & 0xff) << 5;
  return crc;
}

# 285 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result)
#line 285
{
  uint8_t postsignalreceive = FALSE;

  /* atomic removed: atomic calls only */
#line 287
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending && result == SUCCESS) {
        postsignalreceive = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf = (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE;
      }
    else 
#line 297
      {

        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which);
      }
  }
  if (postsignalreceive) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask();
    }
}

# 163 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static uint8_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void )
#line 163
{
  /* atomic removed: atomic calls only */
#line 164
  {
    if (/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__state != /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 166
        /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;

#line 166
        return __nesc_temp;
      }
#line 167
    {
      unsigned char __nesc_temp = 
#line 167
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__resId;

#line 167
      return __nesc_temp;
    }
  }
}

# 58 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciAB0RawInterruptsP.nc"
__attribute((wakeup)) __attribute((interrupt(44)))  void sig_USCIAB0TX_VECTOR(void )
#line 58
{
  if ((IFG2 & (1 << 1)) | (IFG2 & (1 << 0))) {
      HplMsp430UsciAB0RawInterruptsP__UsciA__txDone();
    }
  if ((IFG2 & (1 << 3)) | (IFG2 & (1 << 2))) {
      HplMsp430UsciAB0RawInterruptsP__UsciB__txDone();
    }
}

# 104 "/opt/tinyos-2.1.1/tos/lib/serial/HdlcTranslateC.nc"
static void HdlcTranslateC__UartStream__sendDone(uint8_t *buf, uint16_t len, 
error_t error)
#line 105
{
  if (HdlcTranslateC__state.sendEscape) {
      HdlcTranslateC__state.sendEscape = 0;
      HdlcTranslateC__m_data = HdlcTranslateC__txTemp;
      HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
    }
  else {
      HdlcTranslateC__SerialFrameComm__putDone();
    }
}

#line 92
static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data)
#line 92
{
  if (data == HDLC_CTLESC_BYTE || data == HDLC_FLAG_BYTE) {
      HdlcTranslateC__state.sendEscape = 1;
      HdlcTranslateC__txTemp = data ^ 0x20;
      HdlcTranslateC__m_data = HDLC_CTLESC_BYTE;
    }
  else {
      HdlcTranslateC__m_data = data;
    }
  return HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
}


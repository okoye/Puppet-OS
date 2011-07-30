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





static __inline uint8_t __nesc_ntoh_leuint8(const void * source)  ;




static __inline uint8_t __nesc_hton_leuint8(void * target, uint8_t value)  ;





static __inline int8_t __nesc_ntoh_int8(const void * source)  ;
#line 257
static __inline int8_t __nesc_hton_int8(void * target, int8_t value)  ;






static __inline uint16_t __nesc_ntoh_uint16(const void * source)  ;




static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;






static __inline uint16_t __nesc_ntoh_leuint16(const void * source)  ;




static __inline uint16_t __nesc_hton_leuint16(void * target, uint16_t value)  ;
#line 294
static __inline uint32_t __nesc_ntoh_uint32(const void * source)  ;






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
# 38 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/string.h" 3
extern void *memcpy(void *arg_0x2b99bc4ffbf0, const void *arg_0x2b99bc505020, size_t arg_0x2b99bc5052c8);

extern void *memset(void *arg_0x2b99bc504980, int arg_0x2b99bc504be8, size_t arg_0x2b99bc502020);
#line 61
extern void *memset(void *arg_0x2b99bc51db10, int arg_0x2b99bc51dd78, size_t arg_0x2b99bc51c060);
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
#line 93
void *malloc(size_t size);
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

  void (*__cleanup)(struct _reent *arg_0x2b99bc557b58);


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


  void (**_sig_func)(int arg_0x2b99bc55ac10);




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
uint16_t TOS_NODE_ID = 1;






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
# 128 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/msp430/gpio.h" 3
volatile unsigned char P1IFG __asm ("0x0023");

volatile unsigned char P1IES __asm ("0x0024");

volatile unsigned char P1IE __asm ("0x0025");
#line 149
volatile unsigned char P2IFG __asm ("0x002B");



volatile unsigned char P2IE __asm ("0x002D");
# 254 "/opt/tinyos-2.1.1/msp430-z1/msp430/include/msp430/usci.h" 3
volatile unsigned char UCA0CTL1 __asm ("0x0061");

volatile unsigned char UCA0BR0 __asm ("0x0062");

volatile unsigned char UCA0BR1 __asm ("0x0063");

volatile unsigned char UCA0MCTL __asm ("0x0064");



volatile unsigned char UCA0RXBUF __asm ("0x0066");
#line 277
volatile unsigned char UCB0CTL1 __asm ("0x0069");

volatile unsigned char UCB0BR0 __asm ("0x006A");

volatile unsigned char UCB0BR1 __asm ("0x006B");





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
# 8 "/home/chuka/projects/puppet-os/system/HomeCommManager.h"
#line 4
typedef struct sensor_info {

  char *id;
  char *measurement_unit;
} sensor_info_t;

enum __nesc_unnamed4258 {

  BASE = 1, 
  AM_REGISTER_REQUEST = 0x10
};






#line 16
typedef struct manufacturer_info {

  char *name;
  char *id;
  char *device_id;
} manufacturer_info_t;






#line 23
typedef struct register_request {

  nx_uint32_t device_type_id;
  nx_uint16_t *sensor_ids;
  nx_uint32_t man_id;
} register_request_t;
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

enum __nesc_unnamed4259 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4260 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 32 "/opt/tinyos-2.1.1/tos/types/Leds.h"
enum __nesc_unnamed4261 {
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
enum __nesc_unnamed4262 {
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
typedef struct __nesc_unnamed4263 {

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
typedef struct __nesc_unnamed4264 {

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
typedef struct __nesc_unnamed4265 {

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
enum __nesc_unnamed4266 {

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


enum __nesc_unnamed4267 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 72 "/opt/tinyos-2.1.1/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4268 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4269 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4270 {
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


enum __nesc_unnamed4271 {
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

enum __nesc_unnamed4272 {
  AM_TUNITPROCESSINGMSG = 0xFF
};
# 80 "/opt/tinyos-2.1.1/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b);
# 94 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/msp430usci.h"
#line 89
typedef enum __nesc_unnamed4273 {
  USCI_NONE = 0, 
  USCI_UART = 1, 
  USCI_SPI = 2, 
  USCI_I2C = 3
} msp430_uscimode_t;
#line 115
#line 107
typedef struct __nesc_unnamed4274 {
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
typedef struct __nesc_unnamed4275 {
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
typedef enum __nesc_unnamed4276 {
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
typedef struct __nesc_unnamed4277 {
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
typedef struct __nesc_unnamed4278 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl0;
  uint8_t uctl1;
  uint8_t ume;
} msp430_uart_registers_t;




#line 225
typedef union __nesc_unnamed4279 {
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
typedef struct __nesc_unnamed4280 {
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
typedef struct __nesc_unnamed4281 {
  uint16_t ubr;
  uint8_t uctl0;
  uint8_t uctl1;
} msp430_spi_registers_t;




#line 278
typedef union __nesc_unnamed4282 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;
#line 310
#line 302
typedef struct __nesc_unnamed4283 {
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
typedef struct __nesc_unnamed4284 {
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
typedef struct __nesc_unnamed4285 {
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
typedef struct __nesc_unnamed4286 {
  uint16_t ubr;
  uint8_t uctl0;
  uint8_t uctl1;
  uint16_t ui2coa;
} msp430_i2c_registers_t;




#line 362
typedef union __nesc_unnamed4287 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
# 29 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4288 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4289 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4290 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 33 "/opt/tinyos-2.1.1/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 49 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunitstats/Statistics.h"
#line 44
typedef nx_struct StatisticsMsg {
  nx_int32_t value;
  nx_uint8_t statsId;
  nx_uint8_t unitLength;
  nx_uint8_t units[28 - 6];
} __attribute__((packed)) StatisticsMsg;

enum __nesc_unnamed4291 {
  AM_STATISTICSMSG = 0xFE
};
# 31 "/opt/tinyos-2.1.1/tos/lib/net/ctp/Collection.h"
enum __nesc_unnamed4292 {
  AM_COLLECTION_DATA = 20, 
  AM_COLLECTION_CONTROL = 21, 
  AM_COLLECTION_DEBUG = 22
};

typedef uint8_t collection_id_t;
typedef nx_uint8_t nx_collection_id_t;
# 51 "/opt/tinyos-2.1.1/tos/lib/net/ctp/Ctp.h"
enum __nesc_unnamed4293 {

  AM_CTP_ROUTING = 0x70, 
  AM_CTP_DATA = 0x71, 
  AM_CTP_DEBUG = 0x72, 


  CTP_OPT_PULL = 0x80, 
  CTP_OPT_ECN = 0x40, 
  CTP_OPT_ALL = 0xff
};

typedef nx_uint8_t nx_ctp_options_t;
typedef uint8_t ctp_options_t;









#line 66
typedef nx_struct __nesc_unnamed4294 {
  nx_ctp_options_t options;
  nx_uint8_t thl;
  nx_uint16_t etx;
  nx_am_addr_t origin;
  nx_uint8_t originSeqNo;
  nx_collection_id_t type;
  nx_uint8_t ( data)[0];
} __attribute__((packed)) ctp_data_header_t;






#line 76
typedef nx_struct __nesc_unnamed4295 {
  nx_ctp_options_t options;
  nx_am_addr_t parent;
  nx_uint16_t etx;
  nx_uint8_t ( data)[0];
} __attribute__((packed)) ctp_routing_header_t;
# 33 "/opt/tinyos-2.1.1/tos/types/Ieee154.h"
typedef uint16_t ieee154_panid_t;
typedef uint16_t ieee154_saddr_t;

enum __nesc_unnamed4296 {
  IEEE154_BROADCAST_ADDR = 0xffff
};
# 38 "/opt/tinyos-2.1.1/tos/chips/cc2420/IEEE802154.h"
enum ieee154_fcf_enums {
  IEEE154_FCF_FRAME_TYPE = 0, 
  IEEE154_FCF_SECURITY_ENABLED = 3, 
  IEEE154_FCF_FRAME_PENDING = 4, 
  IEEE154_FCF_ACK_REQ = 5, 
  IEEE154_FCF_INTRAPAN = 6, 
  IEEE154_FCF_DEST_ADDR_MODE = 10, 
  IEEE154_FCF_SRC_ADDR_MODE = 14
};

enum ieee154_fcf_type_enums {
  IEEE154_TYPE_BEACON = 0, 
  IEEE154_TYPE_DATA = 1, 
  IEEE154_TYPE_ACK = 2, 
  IEEE154_TYPE_MAC_CMD = 3
};

enum iee154_fcf_addr_mode_enums {
  IEEE154_ADDR_NONE = 0, 
  IEEE154_ADDR_SHORT = 2, 
  IEEE154_ADDR_EXT = 3
};
# 32 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420TimeSyncMessage.h"
typedef nx_uint32_t timesync_radio_t;





#line 34
typedef struct timesync_footer_t {

  nx_am_id_t type;
  timesync_radio_t timestamp;
} timesync_footer_t;
# 60 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngine.h"
enum __nesc_unnamed4297 {



  FORWARD_PACKET_TIME = 32
};


enum __nesc_unnamed4298 {
  SENDDONE_OK_OFFSET = FORWARD_PACKET_TIME, 
  SENDDONE_OK_WINDOW = FORWARD_PACKET_TIME, 
  SENDDONE_NOACK_OFFSET = FORWARD_PACKET_TIME, 
  SENDDONE_NOACK_WINDOW = FORWARD_PACKET_TIME, 
  SENDDONE_FAIL_OFFSET = FORWARD_PACKET_TIME << 2, 
  SENDDONE_FAIL_WINDOW = SENDDONE_FAIL_OFFSET, 
  LOOPY_OFFSET = FORWARD_PACKET_TIME << 2, 
  LOOPY_WINDOW = LOOPY_OFFSET, 
  CONGESTED_WAIT_OFFSET = FORWARD_PACKET_TIME << 2, 
  CONGESTED_WAIT_WINDOW = CONGESTED_WAIT_OFFSET, 
  NO_ROUTE_RETRY = 10000
};








enum __nesc_unnamed4299 {
  MAX_RETRIES = 30
};
#line 104
#line 100
typedef struct __nesc_unnamed4300 {
  message_t * msg;
  uint8_t client;
  uint8_t retries;
} fe_queue_entry_t;
# 7 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpDebugMsg.h"
enum __nesc_unnamed4301 {
  NET_C_DEBUG_STARTED = 0xDE, 

  NET_C_FE_MSG_POOL_EMPTY = 0x10, 
  NET_C_FE_SEND_QUEUE_FULL = 0x11, 
  NET_C_FE_NO_ROUTE = 0x12, 
  NET_C_FE_SUBSEND_OFF = 0x13, 
  NET_C_FE_SUBSEND_BUSY = 0x14, 
  NET_C_FE_BAD_SENDDONE = 0x15, 
  NET_C_FE_QENTRY_POOL_EMPTY = 0x16, 
  NET_C_FE_SUBSEND_SIZE = 0x17, 
  NET_C_FE_LOOP_DETECTED = 0x18, 
  NET_C_FE_SEND_BUSY = 0x19, 

  NET_C_FE_SENDQUEUE_EMPTY = 0x50, 
  NET_C_FE_PUT_MSGPOOL_ERR = 0x51, 
  NET_C_FE_PUT_QEPOOL_ERR = 0x52, 
  NET_C_FE_GET_MSGPOOL_ERR = 0x53, 
  NET_C_FE_GET_QEPOOL_ERR = 0x54, 
  NET_C_FE_QUEUE_SIZE = 0x55, 

  NET_C_FE_SENT_MSG = 0x20, 
  NET_C_FE_RCV_MSG = 0x21, 
  NET_C_FE_FWD_MSG = 0x22, 
  NET_C_FE_DST_MSG = 0x23, 
  NET_C_FE_SENDDONE_FAIL = 0x24, 
  NET_C_FE_SENDDONE_WAITACK = 0x25, 
  NET_C_FE_SENDDONE_FAIL_ACK_SEND = 0x26, 
  NET_C_FE_SENDDONE_FAIL_ACK_FWD = 0x27, 
  NET_C_FE_DUPLICATE_CACHE = 0x28, 
  NET_C_FE_DUPLICATE_QUEUE = 0x29, 
  NET_C_FE_DUPLICATE_CACHE_AT_SEND = 0x2A, 
  NET_C_FE_CONGESTION_SENDWAIT = 0x2B, 
  NET_C_FE_CONGESTION_BEGIN = 0x2C, 
  NET_C_FE_CONGESTION_END = 0x2D, 



  NET_C_FE_CONGESTED = 0x2E, 

  NET_C_TREE_NO_ROUTE = 0x30, 
  NET_C_TREE_NEW_PARENT = 0x31, 
  NET_C_TREE_ROUTE_INFO = 0x32, 
  NET_C_TREE_SENT_BEACON = 0x33, 
  NET_C_TREE_RCV_BEACON = 0x34, 

  NET_C_DBG_1 = 0x40, 
  NET_C_DBG_2 = 0x41, 
  NET_C_DBG_3 = 0x42
};
#line 79
#line 58
typedef nx_struct CollectionDebugMsg {
  nx_uint8_t type;
  nx_union __nesc_unnamed4302 {
    nx_uint16_t arg;
    nx_struct __nesc_unnamed4303 {
      nx_uint16_t msg_uid;
      nx_am_addr_t origin;
      nx_am_addr_t other_node;
    } __attribute__((packed)) msg;
    nx_struct __nesc_unnamed4304 {
      nx_am_addr_t parent;
      nx_uint8_t hopcount;
      nx_uint16_t metric;
    } __attribute__((packed)) route_info;
    nx_struct __nesc_unnamed4305 {
      nx_uint16_t a;
      nx_uint16_t b;
      nx_uint16_t c;
    } __attribute__((packed)) dbg;
  } __attribute__((packed)) data;
  nx_uint16_t seqno;
} __attribute__((packed)) CollectionDebugMsg;
# 38 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/./LinkEstimator.h"
enum __nesc_unnamed4306 {


  NUM_ENTRIES_FLAG = 15
};
#line 54
#line 51
typedef nx_struct linkest_header {
  nx_uint8_t flags;
  nx_uint8_t seq;
} __attribute__((packed)) linkest_header_t;







#line 59
typedef nx_struct neighbor_stat_entry {
  nx_am_addr_t ll_addr;
  nx_uint8_t inquality;
} __attribute__((packed)) neighbor_stat_entry_t;




#line 65
typedef nx_struct linkest_footer {
  neighbor_stat_entry_t neighborList[1];
} __attribute__((packed)) linkest_footer_t;



enum __nesc_unnamed4307 {
  VALID_ENTRY = 0x1, 


  MATURE_ENTRY = 0x2, 


  INIT_ENTRY = 0x4, 


  PINNED_ENTRY = 0x8
};
#line 111
#line 86
typedef struct neighbor_table_entry {

  am_addr_t ll_addr;

  uint8_t lastseq;


  uint8_t rcvcnt;

  uint8_t failcnt;

  uint8_t flags;


  uint8_t inquality;


  uint16_t etx;



  uint8_t data_success;


  uint8_t data_total;
} neighbor_table_entry_t;
# 4 "/opt/tinyos-2.1.1/tos/lib/net/ctp/TreeRouting.h"
enum __nesc_unnamed4308 {
  AM_TREE_ROUTING_CONTROL = 0xCE, 
  BEACON_INTERVAL = 8192, 
  INVALID_ADDR = 0xFFFF, 
  ETX_THRESHOLD = 50, 
  PARENT_SWITCH_THRESHOLD = 15, 
  MAX_METRIC = 0xFFFF
};







#line 14
typedef struct __nesc_unnamed4309 {
  am_addr_t parent;
  uint16_t etx;
  bool haveHeard;
  bool congested;
} route_info_t;




#line 21
typedef struct __nesc_unnamed4310 {
  am_addr_t neighbor;
  route_info_t info;
} routing_table_entry;

static __inline void routeInfoInit(route_info_t *ri);
enum SerialAMQueueP____nesc_unnamed4311 {
  SerialAMQueueP__NUM_CLIENTS = 2U
};
enum /*PlatformSerialC.UartC*/Msp430Uart0C__0____nesc_unnamed4312 {
  Msp430Uart0C__0__CLIENT_ID = 0U
};
typedef T32khz /*Msp430Uart0P.UartP*/Msp430UartP__0__Counter__precision_tag;
typedef uint16_t /*Msp430Uart0P.UartP*/Msp430UartP__0__Counter__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
enum /*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0____nesc_unnamed4313 {
  Msp430UsciA0C__0__CLIENT_ID = 1U
};
enum /*TestAPIComplexC.TestRegisterC*/TestCaseC__0____nesc_unnamed4314 {
  TestCaseC__0__TUNIT_TEST_ID = 0U
};
typedef StatisticsMsg *StatisticsP__FifoQueue__t;
typedef StatisticsMsg */*WireStatisticsC.FifoQueueC*/FifoQueueC__0__q_element_t;
typedef /*WireStatisticsC.FifoQueueC*/FifoQueueC__0__q_element_t /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__q_element_t;
typedef /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__q_element_t /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__FifoQueue__t;
enum /*TestAPIComplexC.Stats*/StatisticsC__0____nesc_unnamed4315 {
  StatisticsC__0__STATISTICS_ID = 0U
};
enum CtpP____nesc_unnamed4316 {
  CtpP__CLIENT_COUNT = 1U, CtpP__FORWARD_COUNT = 12, CtpP__TREE_ROUTING_TABLE_SIZE = 10, CtpP__QUEUE_SIZE = CtpP__CLIENT_COUNT + CtpP__FORWARD_COUNT, CtpP__CACHE_SIZE = 4
};
enum CC2420ActiveMessageC____nesc_unnamed4317 {
  CC2420ActiveMessageC__CC2420_AM_SEND_ID = 0U
};
typedef T32khz CC2420ControlP__StartupTimer__precision_tag;
typedef uint32_t CC2420ControlP__StartupTimer__size_type;
typedef uint16_t CC2420ControlP__ReadRssi__val_t;
enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4318 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__from_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__size_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__to_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_precision_tag;
typedef uint32_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__size_type;
enum /*CC2420ControlC.Spi*/CC2420SpiC__0____nesc_unnamed4319 {
  CC2420SpiC__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0____nesc_unnamed4320 {
  Msp430SpiB0C__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsciC*/Msp430UsciB0C__0____nesc_unnamed4321 {
  Msp430UsciB0C__0__CLIENT_ID = 1U
};
enum /*CC2420ControlC.SyncSpiC*/CC2420SpiC__1____nesc_unnamed4322 {
  CC2420SpiC__1__CLIENT_ID = 1U
};
enum /*CC2420ControlC.RssiResource*/CC2420SpiC__2____nesc_unnamed4323 {
  CC2420SpiC__2__CLIENT_ID = 2U
};
typedef T32khz CC2420TransmitP__PacketTimeStamp__precision_tag;
typedef uint32_t CC2420TransmitP__PacketTimeStamp__size_type;
typedef T32khz CC2420TransmitP__BackoffTimer__precision_tag;
typedef uint32_t CC2420TransmitP__BackoffTimer__size_type;
enum /*CC2420TransmitC.Spi*/CC2420SpiC__3____nesc_unnamed4324 {
  CC2420SpiC__3__CLIENT_ID = 3U
};
typedef T32khz CC2420ReceiveP__PacketTimeStamp__precision_tag;
typedef uint32_t CC2420ReceiveP__PacketTimeStamp__size_type;
typedef T32khz CC2420PacketP__PacketTimeStamp32khz__precision_tag;
typedef uint32_t CC2420PacketP__PacketTimeStamp32khz__size_type;
typedef T32khz CC2420PacketP__LocalTime32khz__precision_tag;
typedef TMilli CC2420PacketP__LocalTimeMilli__precision_tag;
typedef TMilli CC2420PacketP__PacketTimeStampMilli__precision_tag;
typedef uint32_t CC2420PacketP__PacketTimeStampMilli__size_type;
typedef T32khz /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__precision_tag;
typedef uint32_t /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__1____nesc_unnamed4325 {
  Msp430Timer32khzC__1__ALARM_ID = 1U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC__1__to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC__1__from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type;
enum /*CC2420ReceiveC.Spi*/CC2420SpiC__4____nesc_unnamed4326 {
  CC2420SpiC__4__CLIENT_ID = 4U
};
typedef uint16_t RandomMlcgC__SeedInit__parameter;
enum CC2420TinyosNetworkC____nesc_unnamed4327 {
  CC2420TinyosNetworkC__TINYOS_N_NETWORKS = 1U
};
typedef message_t */*CtpP.Forwarder*/CtpForwardingEngineP__0__SentCache__t;
typedef TMilli /*CtpP.Forwarder*/CtpForwardingEngineP__0__RetxmitTimer__precision_tag;
typedef fe_queue_entry_t */*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__t;
typedef fe_queue_entry_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__t;
typedef message_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__t;
typedef message_t /*CtpP.MessagePoolP*/PoolC__0__pool_t;
typedef /*CtpP.MessagePoolP*/PoolC__0__pool_t /*CtpP.MessagePoolP.PoolP*/PoolP__0__pool_t;
typedef /*CtpP.MessagePoolP.PoolP*/PoolP__0__pool_t /*CtpP.MessagePoolP.PoolP*/PoolP__0__Pool__t;
typedef fe_queue_entry_t /*CtpP.QEntryPoolP*/PoolC__1__pool_t;
typedef /*CtpP.QEntryPoolP*/PoolC__1__pool_t /*CtpP.QEntryPoolP.PoolP*/PoolP__1__pool_t;
typedef /*CtpP.QEntryPoolP.PoolP*/PoolP__1__pool_t /*CtpP.QEntryPoolP.PoolP*/PoolP__1__Pool__t;
typedef fe_queue_entry_t */*CtpP.SendQueueP*/QueueC__0__queue_t;
typedef /*CtpP.SendQueueP*/QueueC__0__queue_t /*CtpP.SendQueueP*/QueueC__0__Queue__t;
typedef message_t */*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__Cache__t;
enum AMQueueP____nesc_unnamed4328 {
  AMQueueP__NUM_CLIENTS = 2U
};
typedef TMilli /*CtpP.Router*/CtpRoutingEngineP__0__BeaconTimer__precision_tag;
typedef TMilli /*CtpP.Router*/CtpRoutingEngineP__0__RouteTimer__precision_tag;
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void TUnitP__waitForSendDone__runTask(void );
# 40 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
static void TUnitP__TearDownOneTime__done(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void TUnitP__begin__runTask(void );
# 38 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
static void TUnitP__TearDown__default__run(void );
# 39 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/TestCase.nc"
static void TUnitP__TestCase__default__run(
# 75 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
uint8_t arg_0x2b99bc837e00);
# 41 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/TestCase.nc"
static void TUnitP__TestCase__done(
# 75 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
uint8_t arg_0x2b99bc837e00);
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

static void TUnitP__SetUpOneTime__done(void );
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
uint8_t arg_0x2b99bc98c690);
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 40 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
uint8_t arg_0x2b99bc98c690);
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
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
#line 46
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
#line 36
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 30 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 33 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );
# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 31
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
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
#line 46
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );
#line 36
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );
# 28 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 30 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t delta);
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
uint8_t arg_0x2b99bc7d7a18);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 45 "/opt/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2b99bc7d7a18);
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
am_id_t arg_0x2b99bcb95340, 
# 92 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(
# 38 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2b99bcb961c8, 
# 56 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 114
static 
#line 112
void * 

/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__getPayload(
# 38 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2b99bcb961c8, 
# 111 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 


uint8_t len);
#line 89
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(
# 38 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2b99bcb961c8, 
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
am_id_t arg_0x2b99bcbeeba8, 
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
am_id_t arg_0x2b99bcbeeba8, 
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
am_id_t arg_0x2b99bcbebcd0, 
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
uart_id_t arg_0x2b99bcd15238, 
# 56 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 89
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(
# 40 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2b99bcd15238, 
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
uart_id_t arg_0x2b99bcd166e8, 
# 60 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(
# 43 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2b99bcd14570, 
# 31 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(
# 43 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2b99bcd14570);
# 23 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(
# 43 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2b99bcd14570, 
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
uint8_t arg_0x2b99bce0e9b0);
# 49 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__ResourceConfigure__configure(
# 76 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0e9b0);
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciInterrupts__rxDone(
# 83 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce1a930, 
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
uint8_t data);
#line 80
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciInterrupts__txDone(
# 83 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce1a930);
# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart0P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(
# 81 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce09cd0);
# 48 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__send(
# 77 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0d7e0, 
# 44 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len);
#line 79
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(
# 77 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0d7e0, 
# 79 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(
# 77 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0d7e0, 
# 95 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__sendDone(
# 77 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0d7e0, 
# 53 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 71 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Counter__overflow(void );
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__release(
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0f688);
# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__immediateRequest(
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0f688);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__default__granted(
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0f688);
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__release(
# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0aa18);
# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__immediateRequest(
# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0aa18);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__granted(
# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0aa18);
# 118 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static bool /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__isOwner(
# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0aa18);
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
# 59 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static bool /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__getRaw(void );






static bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
#line 64
static void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
#line 71
static void /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__set(void );




static void /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectModuleFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void );
#line 85
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void );
#line 78
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );
#line 71
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );




static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void );
#line 71
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );




static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );
#line 71
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );
#line 71
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );




static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void );
#line 71
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 53 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 78 "/opt/tinyos-2.1.1/tos/interfaces/Leds.nc"
static void LedsP__Leds__led2On(void );
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
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(
# 70 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x2b99bd185410, 
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
uint8_t data);
#line 80
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(
# 70 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x2b99bd185410);
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 80
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );
# 43 "/opt/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
static bool /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(
# 55 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c0020);
# 55 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(
# 60 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1be340);
# 49 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1be340);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 46
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );
#line 81
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void );
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(
# 54 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c2d40);
# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(
# 54 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c2d40);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c2d40);
# 118 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static bool /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(
# 54 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c2d40);
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
uint8_t arg_0x2b99bd282be8);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static void StateImplP__State__toIdle(
# 67 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
uint8_t arg_0x2b99bd282be8);
# 66 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static bool StateImplP__State__isState(
# 67 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
uint8_t arg_0x2b99bd282be8, 
# 66 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
uint8_t myState);
#line 61
static bool StateImplP__State__isIdle(
# 67 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
uint8_t arg_0x2b99bd282be8);
# 45 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static error_t StateImplP__State__requestState(
# 67 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
uint8_t arg_0x2b99bd282be8, 
# 45 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
uint8_t reqState);





static void StateImplP__State__forceState(
# 67 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
uint8_t arg_0x2b99bd282be8, 
# 51 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
uint8_t reqState);
# 44 "/opt/tinyos-2.1.1/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 50 "/opt/tinyos-2.1.1/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );




static am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static void StatisticsP__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 46 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/StatsQuery.nc"
static bool StatisticsP__StatsQuery__isIdle(void );
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static error_t /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__Init__init(void );
# 66 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/FifoQueue.nc"
static bool /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__FifoQueue__isEmpty(void );









static void /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__FifoQueue__reset(void );
#line 51
static error_t /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__FifoQueue__dequeue(void *location, uint8_t maxSize);
# 83 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static error_t CC2420CsmaP__SplitControl__start(void );
#line 109
static error_t CC2420CsmaP__SplitControl__stop(void );
# 81 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420CsmaP__SubBackoff__requestInitialBackoff(message_t * msg);






static void CC2420CsmaP__SubBackoff__requestCongestionBackoff(message_t * msg);
# 73 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static void CC2420CsmaP__CC2420Transmit__sendDone(message_t * p_msg, error_t error);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t CC2420CsmaP__Send__send(
#line 56
message_t * msg, 







uint8_t len);
#line 101
static uint8_t CC2420CsmaP__Send__maxPayloadLength(void );
# 76 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Power.nc"
static void CC2420CsmaP__CC2420Power__startOscillatorDone(void );
#line 56
static void CC2420CsmaP__CC2420Power__startVRegDone(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void CC2420CsmaP__Resource__granted(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void CC2420CsmaP__sendDone_task__runTask(void );
#line 64
static void CC2420CsmaP__stopDone_task__runTask(void );
#line 64
static void CC2420CsmaP__startDone_task__runTask(void );
# 86 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Config.nc"
static bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void );
#line 110
static bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void );
#line 105
static bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void );
#line 64
static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void );
#line 52
static error_t CC2420ControlP__CC2420Config__sync(void );
#line 70
static uint16_t CC2420ControlP__CC2420Config__getPanAddr(void );
# 67 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupTimer__fired(void );
# 63 "/opt/tinyos-2.1.1/tos/interfaces/Read.nc"
static void CC2420ControlP__ReadRssi__default__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP__syncDone__runTask(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t CC2420ControlP__Init__init(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void CC2420ControlP__SpiResource__granted(void );
#line 92
static void CC2420ControlP__SyncResource__granted(void );
# 71 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Power.nc"
static error_t CC2420ControlP__CC2420Power__startOscillator(void );
#line 90
static error_t CC2420ControlP__CC2420Power__rxOn(void );
#line 51
static error_t CC2420ControlP__CC2420Power__startVReg(void );
#line 63
static error_t CC2420ControlP__CC2420Power__stopVReg(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP__sync__runTask(void );
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__Resource__release(void );
#line 78
static error_t CC2420ControlP__Resource__request(void );
# 57 "/opt/tinyos-2.1.1/tos/interfaces/GpioInterrupt.nc"
static void CC2420ControlP__InterruptCCA__fired(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void CC2420ControlP__RssiResource__granted(void );
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 92 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 62
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 71 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static void /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 53
static /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get(void );
# 98 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 92
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 55
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);






static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 71 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 33 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void );
#line 32
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get(void );


static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
#line 29
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set(void );
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void );

static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void );
#line 32
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void );


static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void );
#line 29
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void );
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void );


static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void );
#line 32
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get(void );


static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
#line 29
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set(void );
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr(void );
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 43 "/opt/tinyos-2.1.1/tos/interfaces/GpioCapture.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void );
#line 55
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void );
#line 42
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void );
# 41 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__clear(void );
#line 36
static void HplMsp430InterruptP__Port14__disable(void );
#line 56
static void HplMsp430InterruptP__Port14__edge(bool low_to_high);
#line 31
static void HplMsp430InterruptP__Port14__enable(void );









static void HplMsp430InterruptP__Port26__clear(void );
#line 61
static void HplMsp430InterruptP__Port26__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port17__clear(void );
#line 61
static void HplMsp430InterruptP__Port17__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port21__clear(void );
#line 61
static void HplMsp430InterruptP__Port21__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port12__clear(void );
#line 36
static void HplMsp430InterruptP__Port12__disable(void );
#line 56
static void HplMsp430InterruptP__Port12__edge(bool low_to_high);
#line 31
static void HplMsp430InterruptP__Port12__enable(void );









static void HplMsp430InterruptP__Port24__clear(void );
#line 61
static void HplMsp430InterruptP__Port24__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port15__clear(void );
#line 61
static void HplMsp430InterruptP__Port15__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port27__clear(void );
#line 61
static void HplMsp430InterruptP__Port27__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port10__clear(void );
#line 61
static void HplMsp430InterruptP__Port10__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port22__clear(void );
#line 61
static void HplMsp430InterruptP__Port22__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port13__clear(void );
#line 61
static void HplMsp430InterruptP__Port13__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port25__clear(void );
#line 61
static void HplMsp430InterruptP__Port25__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port16__clear(void );
#line 61
static void HplMsp430InterruptP__Port16__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port20__clear(void );
#line 61
static void HplMsp430InterruptP__Port20__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port11__clear(void );
#line 61
static void HplMsp430InterruptP__Port11__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port23__clear(void );
#line 61
static void HplMsp430InterruptP__Port23__default__fired(void );
#line 61
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 50 "/opt/tinyos-2.1.1/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void );
#line 42
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );
# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 50 "/opt/tinyos-2.1.1/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void );
#line 43
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void );
# 71 "/opt/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
static void CC2420SpiP__SpiPacket__sendDone(
#line 64
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 62 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420SpiP__Fifo__continueRead(
# 46 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd778328, 
# 62 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 91
static void CC2420SpiP__Fifo__default__writeDone(
# 46 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd778328, 
# 91 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 82
static cc2420_status_t CC2420SpiP__Fifo__write(
# 46 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd778328, 
# 82 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420SpiP__Fifo__beginRead(
# 46 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd778328, 
# 51 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 71
static void CC2420SpiP__Fifo__default__readDone(
# 46 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd778328, 
# 71 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 31 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420SpiP__ChipSpiResource__abortRelease(void );







static error_t CC2420SpiP__ChipSpiResource__attemptRelease(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void CC2420SpiP__SpiResource__granted(void );
# 63 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420SpiP__Ram__write(
# 47 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint16_t arg_0x2b99bd7774d8, 
# 63 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Ram.nc"
uint8_t offset, uint8_t * data, uint8_t length);
# 47 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420SpiP__Reg__read(
# 48 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd776220, 
# 47 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t *data);







static cc2420_status_t CC2420SpiP__Reg__write(
# 48 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd776220, 
# 55 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t data);
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__release(
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd77a158);
# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__immediateRequest(
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd77a158);
# 78 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__request(
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd77a158);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void CC2420SpiP__Resource__default__granted(
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd77a158);
# 118 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static bool CC2420SpiP__Resource__isOwner(
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd77a158);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void CC2420SpiP__grant__runTask(void );
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(
# 49 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd775020);
# 55 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__ResourceConfigure__unconfigure(
# 74 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7e8060);
# 49 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__ResourceConfigure__configure(
# 74 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7e8060);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task__runTask(void );
# 59 "/opt/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__send(
# 76 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7e7368, 
# 48 "/opt/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
#line 71
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__default__sendDone(
# 76 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7e7368, 
# 64 "/opt/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 34 "/opt/tinyos-2.1.1/tos/interfaces/SpiByte.nc"
static uint8_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiByte__write(uint8_t tx);
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__rxDone(uint8_t data);
#line 80
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__txDone(void );
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__release(
# 73 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7ece00);
# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__immediateRequest(
# 73 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7ece00);
# 78 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__request(
# 73 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7ece00);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__default__granted(
# 73 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7ece00);
# 118 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__isOwner(
# 73 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7ece00);
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__release(
# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7e6108);
# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__immediateRequest(
# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7e6108);
# 78 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__request(
# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7e6108);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__granted(
# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7e6108);
# 118 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__isOwner(
# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7e6108);
# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDmaB0P.Z1UsciP*/Z1UsciP__0__Msp430SpiConfigure__getConfig(
# 2 "/opt/tinyos-2.1.1/tos/platforms/z1/chips/msp430X/usci/Z1UsciP.nc"
uint8_t arg_0x2b99bd867408);
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
static void HplMsp430UsciB0P__UsciRawInterrupts__rxDone(uint8_t data);
#line 80
static void HplMsp430UsciB0P__UsciRawInterrupts__txDone(void );
# 100 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB.nc"
static void HplMsp430UsciB0P__Usci__enableRxIntr(void );
#line 88
static void HplMsp430UsciB0P__Usci__resetUsci(bool reset);
#line 106
static void HplMsp430UsciB0P__Usci__clrRxIntr(void );
#line 99
static void HplMsp430UsciB0P__Usci__disableIntr(void );
#line 97
static void HplMsp430UsciB0P__Usci__disableRxIntr(void );










static void HplMsp430UsciB0P__Usci__clrIntr(void );
#line 71
static void HplMsp430UsciB0P__Usci__setUbr(uint16_t ubr);
#line 115
static void HplMsp430UsciB0P__Usci__tx(uint8_t data);
#line 136
static void HplMsp430UsciB0P__Usci__enableSpi(void );
#line 122
static uint8_t HplMsp430UsciB0P__Usci__rx(void );
#line 148
static void HplMsp430UsciB0P__Usci__setModeSpi(msp430_spi_union_config_t *config);
#line 105
static bool HplMsp430UsciB0P__Usci__isRxIntrPending(void );
#line 137
static void HplMsp430UsciB0P__Usci__disableSpi(void );
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__Interrupts__default__rxDone(
# 70 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x2b99bd185410, 
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
uint8_t data);
#line 80
static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__Interrupts__default__txDone(
# 70 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x2b99bd185410);
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__RawInterrupts__rxDone(uint8_t data);
#line 80
static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__RawInterrupts__txDone(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__Init__init(void );
# 69 "/opt/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty(void );








static bool /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue(void );
# 43 "/opt/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(
# 55 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c0020);
# 51 "/opt/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(
# 55 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c0020);
# 55 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(
# 60 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1be340);
# 49 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(
# 60 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1be340);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
#line 73
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested(void );
#line 46
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted(void );
#line 81
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested(void );
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(
# 54 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c2d40);
# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(
# 54 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c2d40);
# 78 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(
# 54 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c2d40);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(
# 54 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c2d40);
# 118 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static bool /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(
# 54 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c2d40);
# 80 "/opt/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
# 66 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420TransmitP__RadioBackoff__setCongestionBackoff(uint16_t backoffTime);
#line 60
static void CC2420TransmitP__RadioBackoff__setInitialBackoff(uint16_t backoffTime);
# 50 "/opt/tinyos-2.1.1/tos/interfaces/GpioCapture.nc"
static void CC2420TransmitP__CaptureSFD__captured(uint16_t time);
# 67 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void CC2420TransmitP__BackoffTimer__fired(void );
# 63 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t * message);
# 51 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static error_t CC2420TransmitP__Send__send(message_t * p_msg, bool useCca);
# 24 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP__ChipSpiResource__releasing(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t CC2420TransmitP__Init__init(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void CC2420TransmitP__SpiResource__granted(void );
# 74 "/opt/tinyos-2.1.1/tos/interfaces/StdControl.nc"
static error_t CC2420TransmitP__StdControl__start(void );









static error_t CC2420TransmitP__StdControl__stop(void );
# 91 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420TransmitP__TXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420TransmitP__TXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 53 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ReceiveP__CC2420Config__syncDone(error_t error);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void CC2420ReceiveP__receiveDone_task__runTask(void );
# 55 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420ReceiveP__CC2420Receive__sfd_dropped(void );
#line 49
static void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time);
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t CC2420ReceiveP__Init__init(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void CC2420ReceiveP__SpiResource__granted(void );
# 91 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420ReceiveP__RXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420ReceiveP__RXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 57 "/opt/tinyos-2.1.1/tos/interfaces/GpioInterrupt.nc"
static void CC2420ReceiveP__InterruptFIFOP__fired(void );
# 74 "/opt/tinyos-2.1.1/tos/interfaces/StdControl.nc"
static error_t CC2420ReceiveP__StdControl__start(void );









static error_t CC2420ReceiveP__StdControl__stop(void );
# 77 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Packet.nc"
static void CC2420PacketP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId);
#line 75
static uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t *p_msg);
# 59 "/opt/tinyos-2.1.1/tos/interfaces/PacketTimeStamp.nc"
static void CC2420PacketP__PacketTimeStamp32khz__clear(
#line 55
message_t * msg);
#line 67
static void CC2420PacketP__PacketTimeStamp32khz__set(
#line 62
message_t * msg, 




CC2420PacketP__PacketTimeStamp32khz__size_type value);
# 42 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg);




static cc2420_metadata_t * CC2420PacketP__CC2420PacketBody__getMetadata(message_t * msg);
# 47 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
static uint8_t CC2420PacketP__PacketTimeSyncOffset__get(
#line 42
message_t * msg);
#line 39
static bool CC2420PacketP__PacketTimeSyncOffset__isSet(
#line 35
message_t * msg);
# 48 "/opt/tinyos-2.1.1/tos/interfaces/PacketAcknowledgements.nc"
static error_t CC2420PacketP__Acks__requestAck(
#line 42
message_t * msg);
#line 74
static bool CC2420PacketP__Acks__wasAcked(
#line 69
message_t * msg);
# 71 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 92 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );
# 71 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static void /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
#line 53
static /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get(void );
# 98 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
# 71 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 67 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 125 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 118
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 72 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 72
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 37 "/opt/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b99bdc492f8);
# 81 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(
# 37 "/opt/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b99bdc492f8);
# 53 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(
# 37 "/opt/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b99bdc492f8, 
# 53 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
uint32_t dt);








static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 37 "/opt/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b99bdc492f8, 
# 62 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(
# 37 "/opt/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b99bdc492f8);
# 71 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 41 "/opt/tinyos-2.1.1/tos/interfaces/Random.nc"
static uint16_t RandomMlcgC__Random__rand16(void );
#line 35
static uint32_t RandomMlcgC__Random__rand32(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t RandomMlcgC__Init__init(void );
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void UniqueSendP__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
#line 64
static error_t UniqueSendP__Send__send(
#line 56
message_t * msg, 







uint8_t len);
#line 101
static uint8_t UniqueSendP__Send__maxPayloadLength(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t UniqueSendP__Init__init(void );
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



UniqueReceiveP__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t UniqueReceiveP__Init__init(void );
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



UniqueReceiveP__DuplicateReceive__default__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420TinyosNetworkP__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void CC2420TinyosNetworkP__grantTask__runTask(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t CC2420TinyosNetworkP__ActiveSend__send(
#line 56
message_t * msg, 







uint8_t len);
#line 114
static 
#line 112
void * 

CC2420TinyosNetworkP__ActiveSend__getPayload(
#line 111
message_t * msg, 


uint8_t len);
#line 101
static uint8_t CC2420TinyosNetworkP__ActiveSend__maxPayloadLength(void );
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420TinyosNetworkP__BareReceive__default__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP__Resource__release(
# 46 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x2b99bdd37550);
# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP__Resource__immediateRequest(
# 46 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x2b99bdd37550);
# 78 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP__Resource__request(
# 46 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x2b99bdd37550);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void CC2420TinyosNetworkP__Resource__default__granted(
# 46 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x2b99bdd37550);
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__BareSend__default__sendDone(
#line 85
message_t * msg, 



error_t error);
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void );
# 69 "/opt/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
static error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );








static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420ActiveMessageP__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void CC2420ActiveMessageP__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 53 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ActiveMessageP__CC2420Config__syncDone(error_t error);
# 95 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420ActiveMessageP__RadioBackoff__default__requestCca(
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b99bddf1d60, 
# 95 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
#line 81
static void CC2420ActiveMessageP__RadioBackoff__default__requestInitialBackoff(
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b99bddf1d60, 
# 81 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);






static void CC2420ActiveMessageP__RadioBackoff__default__requestCongestionBackoff(
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b99bddf1d60, 
# 88 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
# 59 "/opt/tinyos-2.1.1/tos/interfaces/SendNotifier.nc"
static void CC2420ActiveMessageP__SendNotifier__default__aboutToSend(
# 44 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b99bddf1258, 
# 59 "/opt/tinyos-2.1.1/tos/interfaces/SendNotifier.nc"
am_addr_t dest, 
#line 57
message_t * msg);
# 95 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420ActiveMessageP__SubBackoff__requestCca(message_t * msg);
#line 81
static void CC2420ActiveMessageP__SubBackoff__requestInitialBackoff(message_t * msg);






static void CC2420ActiveMessageP__SubBackoff__requestCongestionBackoff(message_t * msg);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
static uint8_t CC2420ActiveMessageP__Packet__payloadLength(
#line 63
message_t * msg);
#line 115
static 
#line 112
void * 


CC2420ActiveMessageP__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
#line 95
static uint8_t CC2420ActiveMessageP__Packet__maxPayloadLength(void );
#line 83
static void CC2420ActiveMessageP__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static error_t CC2420ActiveMessageP__AMSend__send(
# 39 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b99bddf66c0, 
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
#line 112
static uint8_t CC2420ActiveMessageP__AMSend__maxPayloadLength(
# 39 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b99bddf66c0);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420ActiveMessageP__Snoop__default__receive(
# 41 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b99bddf4480, 
# 60 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
#line 67
static 
#line 63
message_t * 



CC2420ActiveMessageP__Receive__default__receive(
# 40 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b99bddf5908, 
# 60 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 77 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
static am_addr_t CC2420ActiveMessageP__AMPacket__source(
#line 73
message_t * amsg);
#line 57
static am_addr_t CC2420ActiveMessageP__AMPacket__address(void );









static am_addr_t CC2420ActiveMessageP__AMPacket__destination(
#line 63
message_t * amsg);
#line 92
static void CC2420ActiveMessageP__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 136
static am_id_t CC2420ActiveMessageP__AMPacket__type(
#line 132
message_t * amsg);
#line 151
static void CC2420ActiveMessageP__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
#line 125
static bool CC2420ActiveMessageP__AMPacket__isForMe(
#line 122
message_t * amsg);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void CC2420ActiveMessageP__RadioResource__granted(void );
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*CtpP.Forwarder*/CtpForwardingEngineP__0__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 50 "/opt/tinyos-2.1.1/tos/lib/net/CollectionDebug.nc"
static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__default__logEvent(uint8_t type);
#line 62
static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__default__logEventMsg(uint8_t type, uint16_t msg, am_addr_t origin, am_addr_t node);
# 43 "/opt/tinyos-2.1.1/tos/lib/net/CollectionPacket.nc"
static am_addr_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getOrigin(message_t *msg);





static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getSequenceNumber(message_t *msg);
# 61 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimator.nc"
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__LinkEstimator__evicted(am_addr_t neighbor);
# 31 "/opt/tinyos-2.1.1/tos/interfaces/Intercept.nc"
static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__Intercept__default__forward(
# 114 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
collection_id_t arg_0x2b99bde92da8, 
# 20 "/opt/tinyos-2.1.1/tos/interfaces/Intercept.nc"
message_t * msg, 

void * payload, 








uint8_t len);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__payloadLength(
#line 63
message_t * msg);
#line 115
static 
#line 112
void * 


/*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
#line 95
static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__maxPayloadLength(void );
#line 83
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*CtpP.Forwarder*/CtpForwardingEngineP__0__Snoop__default__receive(
# 113 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
collection_id_t arg_0x2b99bde92270, 
# 60 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__send(
# 111 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
uint8_t arg_0x2b99bde943f0, 
# 56 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 114
static 
#line 112
void * 

/*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__getPayload(
# 111 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
uint8_t arg_0x2b99bde943f0, 
# 111 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 


uint8_t len);
#line 101
static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__maxPayloadLength(
# 111 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
uint8_t arg_0x2b99bde943f0);
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__default__sendDone(
# 111 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
uint8_t arg_0x2b99bde943f0, 
# 85 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 72 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__RetxmitTimer__fired(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__RadioControl__startDone(error_t error);
#line 117
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__RadioControl__stopDone(error_t error);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask__runTask(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Init__init(void );
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*CtpP.Forwarder*/CtpForwardingEngineP__0__Receive__default__receive(
# 112 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
collection_id_t arg_0x2b99bde93648, 
# 60 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 7 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpCongestion.nc"
static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpCongestion__isCongested(void );
# 51 "/opt/tinyos-2.1.1/tos/lib/net/UnicastNameFreeRouting.nc"
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__UnicastNameFreeRouting__routeFound(void );
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__UnicastNameFreeRouting__noRoute(void );
# 52 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpPacket.nc"
static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__option(message_t *msg, ctp_options_t opt);





static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__setEtx(message_t *msg, uint16_t etx);
#line 49
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__clearOption(message_t *msg, ctp_options_t option);







static uint16_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getEtx(message_t *msg);


static am_addr_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getOrigin(message_t *msg);
#line 46
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__setOption(message_t *msg, ctp_options_t option);







static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getThl(message_t *msg);








static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getSequenceNumber(message_t *msg);





static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__matchInstance(message_t *m1, message_t *m2);
#line 66
static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getType(message_t *msg);
#line 55
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__setThl(message_t *msg, uint8_t thl);
# 74 "/opt/tinyos-2.1.1/tos/interfaces/StdControl.nc"
static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__StdControl__start(void );
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*CtpP.Forwarder*/CtpForwardingEngineP__0__SubSnoop__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 46 "/opt/tinyos-2.1.1/tos/lib/net/CollectionId.nc"
static collection_id_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionId__default__fetch(
# 146 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
uint8_t arg_0x2b99bdecf968);
# 97 "/opt/tinyos-2.1.1/tos/interfaces/Pool.nc"
static 
#line 94
/*CtpP.MessagePoolP.PoolP*/PoolP__0__Pool__t * 


/*CtpP.MessagePoolP.PoolP*/PoolP__0__Pool__get(void );
#line 61
static bool /*CtpP.MessagePoolP.PoolP*/PoolP__0__Pool__empty(void );
#line 89
static error_t /*CtpP.MessagePoolP.PoolP*/PoolP__0__Pool__put(
#line 85
/*CtpP.MessagePoolP.PoolP*/PoolP__0__Pool__t * newVal);
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*CtpP.MessagePoolP.PoolP*/PoolP__0__Init__init(void );
# 97 "/opt/tinyos-2.1.1/tos/interfaces/Pool.nc"
static 
#line 94
/*CtpP.QEntryPoolP.PoolP*/PoolP__1__Pool__t * 


/*CtpP.QEntryPoolP.PoolP*/PoolP__1__Pool__get(void );
#line 61
static bool /*CtpP.QEntryPoolP.PoolP*/PoolP__1__Pool__empty(void );
#line 89
static error_t /*CtpP.QEntryPoolP.PoolP*/PoolP__1__Pool__put(
#line 85
/*CtpP.QEntryPoolP.PoolP*/PoolP__1__Pool__t * newVal);
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*CtpP.QEntryPoolP.PoolP*/PoolP__1__Init__init(void );
# 73 "/opt/tinyos-2.1.1/tos/interfaces/Queue.nc"
static 
#line 71
/*CtpP.SendQueueP*/QueueC__0__Queue__t  

/*CtpP.SendQueueP*/QueueC__0__Queue__head(void );
#line 90
static error_t /*CtpP.SendQueueP*/QueueC__0__Queue__enqueue(
#line 86
/*CtpP.SendQueueP*/QueueC__0__Queue__t  newVal);
#line 101
static 
#line 99
/*CtpP.SendQueueP*/QueueC__0__Queue__t  

/*CtpP.SendQueueP*/QueueC__0__Queue__element(uint8_t idx);
#line 65
static uint8_t /*CtpP.SendQueueP*/QueueC__0__Queue__maxSize(void );
#line 81
static 
#line 79
/*CtpP.SendQueueP*/QueueC__0__Queue__t  

/*CtpP.SendQueueP*/QueueC__0__Queue__dequeue(void );
#line 50
static bool /*CtpP.SendQueueP*/QueueC__0__Queue__empty(void );







static uint8_t /*CtpP.SendQueueP*/QueueC__0__Queue__size(void );
# 40 "/opt/tinyos-2.1.1/tos/interfaces/Cache.nc"
static void /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__Cache__insert(/*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__Cache__t item);







static bool /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__Cache__lookup(/*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__Cache__t item);
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__Init__init(void );
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



LinkEstimatorP__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 51 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimator.nc"
static error_t LinkEstimatorP__LinkEstimator__txAck(am_addr_t neighbor);
#line 44
static error_t LinkEstimatorP__LinkEstimator__pinNeighbor(am_addr_t neighbor);










static error_t LinkEstimatorP__LinkEstimator__txNoAck(am_addr_t neighbor);
#line 41
static error_t LinkEstimatorP__LinkEstimator__insertNeighbor(am_addr_t neighbor);
#line 58
static error_t LinkEstimatorP__LinkEstimator__clearDLQ(am_addr_t neighbor);
#line 47
static error_t LinkEstimatorP__LinkEstimator__unpinNeighbor(am_addr_t neighbor);
#line 38
static uint16_t LinkEstimatorP__LinkEstimator__getLinkQuality(uint16_t neighbor);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
static uint8_t LinkEstimatorP__Packet__payloadLength(
#line 63
message_t * msg);
#line 115
static 
#line 112
void * 


LinkEstimatorP__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
#line 95
static uint8_t LinkEstimatorP__Packet__maxPayloadLength(void );
# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static void LinkEstimatorP__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
#line 69
static error_t LinkEstimatorP__Send__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
#line 124
static 
#line 122
void * 

LinkEstimatorP__Send__getPayload(
#line 121
message_t * msg, 


uint8_t len);
#line 112
static uint8_t LinkEstimatorP__Send__maxPayloadLength(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t LinkEstimatorP__Init__init(void );
# 74 "/opt/tinyos-2.1.1/tos/interfaces/StdControl.nc"
static error_t LinkEstimatorP__StdControl__start(void );
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static error_t /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
#line 112
static uint8_t /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__maxPayloadLength(void );
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__sendDone(
# 40 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
am_id_t arg_0x2b99bcb95340, 
# 92 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__send(
# 38 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2b99bcb961c8, 
# 56 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 101
static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__maxPayloadLength(
# 38 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2b99bcb961c8);
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__default__sendDone(
# 38 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2b99bcb961c8, 
# 85 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask__runTask(void );
#line 64
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__CancelTask__runTask(void );
# 43 "/opt/tinyos-2.1.1/tos/lib/net/RootControl.nc"
static bool /*CtpP.Router*/CtpRoutingEngineP__0__RootControl__isRoot(void );
#line 41
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__RootControl__setRoot(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*CtpP.Router*/CtpRoutingEngineP__0__updateRouteTask__runTask(void );
# 68 "/opt/tinyos-2.1.1/tos/lib/net/CollectionDebug.nc"
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__CollectionDebug__default__logEventRoute(uint8_t type, am_addr_t parent, uint8_t hopcount, uint16_t metric);
#line 56
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__CollectionDebug__default__logEventDbg(uint8_t type, uint16_t arg1, uint16_t arg2, uint16_t arg3);
# 61 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimator.nc"
static void /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__evicted(am_addr_t neighbor);
# 46 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingPacket.nc"
static bool /*CtpP.Router*/CtpRoutingEngineP__0__CtpRoutingPacket__getOption(message_t * msg, ctp_options_t opt);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static void /*CtpP.Router*/CtpRoutingEngineP__0__RadioControl__startDone(error_t error);
#line 117
static void /*CtpP.Router*/CtpRoutingEngineP__0__RadioControl__stopDone(error_t error);
# 71 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpInfo.nc"
static void /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__recomputeRoutes(void );
#line 59
static void /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__triggerRouteUpdate(void );
#line 52
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__getEtx(uint16_t *etx);
#line 66
static void /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__triggerImmediateRouteUpdate(void );









static void /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__setNeighborCongested(am_addr_t n, bool congested);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static void /*CtpP.Router*/CtpRoutingEngineP__0__sendBeaconTask__runTask(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__Init__init(void );
# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static void /*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 72 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void /*CtpP.Router*/CtpRoutingEngineP__0__BeaconTimer__fired(void );
#line 72
static void /*CtpP.Router*/CtpRoutingEngineP__0__RouteTimer__fired(void );
# 43 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CompareBit.nc"
static bool /*CtpP.Router*/CtpRoutingEngineP__0__CompareBit__shouldInsert(message_t * msg, void * payload, uint8_t len);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*CtpP.Router*/CtpRoutingEngineP__0__BeaconReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 74 "/opt/tinyos-2.1.1/tos/interfaces/StdControl.nc"
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__StdControl__start(void );
# 49 "/opt/tinyos-2.1.1/tos/lib/net/UnicastNameFreeRouting.nc"
static bool /*CtpP.Router*/CtpRoutingEngineP__0__Routing__hasRoute(void );
#line 48
static am_addr_t /*CtpP.Router*/CtpRoutingEngineP__0__Routing__nextHop(void );
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static error_t /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 47 "/opt/tinyos-2.1.1/tos/interfaces/LinkPacketMetadata.nc"
static bool DummyActiveMessageP__LinkPacketMetadata__highChannelQuality(
#line 44
message_t * msg);
# 46 "/opt/tinyos-2.1.1/tos/lib/net/CollectionId.nc"
static collection_id_t /*HomeCommManagerC.CollectionSenderC.CollectionSenderP.CollectionIdP*/CollectionIdP__0__CollectionId__fetch(void );
# 83 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static error_t HomeCommManagerP__SplitControl__start(void );
#line 109
static error_t HomeCommManagerP__SplitControl__stop(void );
#line 92
static void HomeCommManagerP__RadioControl__startDone(error_t error);
#line 117
static void HomeCommManagerP__RadioControl__stopDone(error_t error);
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void HomeCommManagerP__RadioSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 6 "/home/chuka/projects/puppet-os/interfaces/PuppetAPI.nc"
static error_t HomeCommManagerP__PuppetAPI__registerDeviceRequest(register_request_t *reg);
# 39 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/TestCase.nc"
static void TestAPIComplexP__TestRegister__run(void );
# 38 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
static void TestAPIComplexP__TearDownOneTime__run(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static void TestAPIComplexP__SplitControl__startDone(error_t error);
#line 117
static void TestAPIComplexP__SplitControl__stopDone(error_t error);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



TestAPIComplexP__Snoop__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
#line 67
static 
#line 63
message_t * 



TestAPIComplexP__Receive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 38 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
static void TestAPIComplexP__SetUpOneTime__run(void );
# 8 "/home/chuka/projects/puppet-os/interfaces/PuppetAPI.nc"
static void TestAPIComplexP__PuppetAPI__registerRequestDone(message_t *msg, error_t err);
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
uint8_t arg_0x2b99bc837e00);
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
enum TUnitP____nesc_unnamed4329 {
#line 131
  TUnitP__begin = 0U
};
#line 131
typedef int TUnitP____nesc_sillytask_begin[TUnitP__begin];
enum TUnitP____nesc_unnamed4330 {
#line 132
  TUnitP__waitForSendDone = 1U
};
#line 132
typedef int TUnitP____nesc_sillytask_waitForSendDone[TUnitP__waitForSendDone];
enum TUnitP____nesc_unnamed4331 {
#line 133
  TUnitP__runDone = 2U
};
#line 133
typedef int TUnitP____nesc_sillytask_runDone[TUnitP__runDone];
#line 98
uint8_t TUnitP__currentTest;





bool TUnitP__driver;




enum TUnitP____nesc_unnamed4332 {
  TUnitP__S_NOT_BOOTED, 
  TUnitP__S_READY, 
  TUnitP__S_RUNNING
};




enum TUnitP____nesc_unnamed4333 {
  TUnitP__S_IDLE, 
  TUnitP__S_SETUP_ONETIME, 

  TUnitP__S_SETUP, 
  TUnitP__S_RUN, 
  TUnitP__S_TEARDOWN, 

  TUnitP__S_TEARDOWN_ONETIME
};







static inline void TUnitP__setUpOneTimeDone(void );
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



static inline void TUnitP__SetUpOneTime__done(void );
#line 252
static inline void TUnitP__TearDownOneTime__done(void );
#line 267
static inline void TUnitP__setUpOneTimeDone(void );
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
#line 374
static inline void TUnitP__SetUp__default__run(void );



static inline void TUnitP__TestCase__default__run(uint8_t testId);



static inline void TUnitP__TearDown__default__run(void );
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

enum Msp430ClockP____nesc_unnamed4334 {

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
uint8_t arg_0x2b99bc98c690);
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
uint8_t arg_0x2b99bc98c690);
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 51 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 70
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
#line 115
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
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void );
#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 61
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(uint8_t l_cm);
#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
#line 99
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 119
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 164
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
#line 181
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void );
# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__CC2int(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__compareControl(void );
#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);







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
uint8_t arg_0x2b99bc7d7a18);
# 59 "/opt/tinyos-2.1.1/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 50 "/opt/tinyos-2.1.1/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4335 {

  SchedulerBasicP__NUM_TASKS = 31U, 
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
enum Link_TUnitProcessingP____nesc_unnamed4336 {
#line 85
  Link_TUnitProcessingP__sendEventMsg = 3U
};
#line 85
typedef int Link_TUnitProcessingP____nesc_sillytask_sendEventMsg[Link_TUnitProcessingP__sendEventMsg];
enum Link_TUnitProcessingP____nesc_unnamed4337 {
#line 86
  Link_TUnitProcessingP__allDone = 4U
};
#line 86
typedef int Link_TUnitProcessingP____nesc_sillytask_allDone[Link_TUnitProcessingP__allDone];
#line 55
message_t Link_TUnitProcessingP__eventMsg[5];


uint8_t Link_TUnitProcessingP__writingEventMsg;


uint8_t Link_TUnitProcessingP__sendingEventMsg;




enum Link_TUnitProcessingP____nesc_unnamed4338 {
  Link_TUnitProcessingP__S_OFF, 
  Link_TUnitProcessingP__S_ON
};




enum Link_TUnitProcessingP____nesc_unnamed4339 {
  Link_TUnitProcessingP__S_IDLE, 
  Link_TUnitProcessingP__S_BUSY
};

enum Link_TUnitProcessingP____nesc_unnamed4340 {
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
am_id_t arg_0x2b99bcb95340, 
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
am_id_t arg_0x2b99bcb95340, 
# 121 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
message_t * msg, 


uint8_t len);
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(
# 38 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2b99bcb961c8, 
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
enum /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4341 {
#line 118
  AMQueueImplP__0__CancelTask = 5U
};
#line 118
typedef int /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_CancelTask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask];
#line 161
enum /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4342 {
#line 161
  AMQueueImplP__0__errorTask = 6U
};
#line 161
typedef int /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_errorTask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask];
#line 49
#line 47
typedef struct /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4343 {
  message_t * msg;
} /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t;

uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 2;
/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[2];
uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[2 / 8 + 1];

static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );

static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void );
#line 82
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
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
am_id_t arg_0x2b99bcbeeba8, 
# 92 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(
# 37 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x2b99bcbebcd0, 
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









static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(message_t *amsg, am_addr_t addr);
#line 161
static inline am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(message_t *amsg);




static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(message_t *amsg, am_id_t type);
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
enum SerialP____nesc_unnamed4344 {
#line 189
  SerialP__RunTx = 7U
};
#line 189
typedef int SerialP____nesc_sillytask_RunTx[SerialP__RunTx];
#line 320
enum SerialP____nesc_unnamed4345 {
#line 320
  SerialP__startDoneTask = 8U
};
#line 320
typedef int SerialP____nesc_sillytask_startDoneTask[SerialP__startDoneTask];





enum SerialP____nesc_unnamed4346 {
#line 326
  SerialP__stopDoneTask = 9U
};
#line 326
typedef int SerialP____nesc_sillytask_stopDoneTask[SerialP__stopDoneTask];








enum SerialP____nesc_unnamed4347 {
#line 335
  SerialP__defaultSerialFlushTask = 10U
};
#line 335
typedef int SerialP____nesc_sillytask_defaultSerialFlushTask[SerialP__defaultSerialFlushTask];
#line 79
enum SerialP____nesc_unnamed4348 {
  SerialP__RX_DATA_BUFFER_SIZE = 2, 
  SerialP__TX_DATA_BUFFER_SIZE = 4, 
  SerialP__SERIAL_MTU = 255, 
  SerialP__SERIAL_VERSION = 1, 
  SerialP__ACK_QUEUE_SIZE = 5
};

enum SerialP____nesc_unnamed4349 {
  SerialP__RXSTATE_NOSYNC, 
  SerialP__RXSTATE_PROTO, 
  SerialP__RXSTATE_TOKEN, 
  SerialP__RXSTATE_INFO, 
  SerialP__RXSTATE_INACTIVE
};

enum SerialP____nesc_unnamed4350 {
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
typedef enum SerialP____nesc_unnamed4351 {
  SerialP__BUFFER_AVAILABLE, 
  SerialP__BUFFER_FILLING, 
  SerialP__BUFFER_COMPLETE
} SerialP__tx_data_buffer_states_t;

enum SerialP____nesc_unnamed4352 {
  SerialP__TX_ACK_INDEX = 0, 
  SerialP__TX_DATA_INDEX = 1, 
  SerialP__TX_BUFFER_COUNT = 2
};






#line 122
typedef struct SerialP____nesc_unnamed4353 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP__RX_DATA_BUFFER_SIZE + 1];
} SerialP__rx_buf_t;




#line 128
typedef struct SerialP____nesc_unnamed4354 {
  uint8_t state;
  uint8_t buf;
} SerialP__tx_buf_t;





#line 133
typedef struct SerialP____nesc_unnamed4355 {
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
uart_id_t arg_0x2b99bcd15238, 
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
uart_id_t arg_0x2b99bcd166e8, 
# 60 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(
# 43 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2b99bcd14570, 
# 31 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(
# 43 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2b99bcd14570);
# 23 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(
# 43 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x2b99bcd14570, 
# 23 "/opt/tinyos-2.1.1/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 60 "/opt/tinyos-2.1.1/tos/lib/serial/SendBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend(void );
#line 51
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(uint8_t first_byte);
# 147 "/opt/tinyos-2.1.1/tos/lib/serial/SerialDispatcherP.nc"
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4356 {
#line 147
  SerialDispatcherP__0__signalSendDone = 11U
};
#line 147
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_signalSendDone[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone];
#line 264
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4357 {
#line 264
  SerialDispatcherP__0__receiveTask = 12U
};
#line 264
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_receiveTask[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask];
#line 55
#line 51
typedef enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4358 {
  SerialDispatcherP__0__SEND_STATE_IDLE = 0, 
  SerialDispatcherP__0__SEND_STATE_BEGIN = 1, 
  SerialDispatcherP__0__SEND_STATE_DATA = 2
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__send_state_t;

enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4359 {
  SerialDispatcherP__0__RECV_STATE_IDLE = 0, 
  SerialDispatcherP__0__RECV_STATE_BEGIN = 1, 
  SerialDispatcherP__0__RECV_STATE_DATA = 2
};






#line 63
typedef struct /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4360 {
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
typedef struct HdlcTranslateC____nesc_unnamed4361 {
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
uint8_t arg_0x2b99bce09cd0);
# 79 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__receivedByte(
# 77 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0d7e0, 
# 79 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__receiveDone(
# 77 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0d7e0, 
# 95 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__sendDone(
# 77 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0d7e0, 
# 53 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__granted(
# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0f688);
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__release(
# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0aa18);
# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__immediateRequest(
# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0aa18);
# 118 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static bool /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__isOwner(
# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
uint8_t arg_0x2b99bce0aa18);
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
# 48 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__get(void );
#line 48
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
#line 48
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );
#line 45
static void /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__set(void );
static void /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__makeOutput(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 48
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 45
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void );





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
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 71 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 38 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









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
static void LedsP__Led2__clr(void );
# 45 "/opt/tinyos-2.1.1/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 93
static inline void LedsP__Leds__led2On(void );
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




static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );




static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__rxDone(
# 70 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x2b99bd185410, 
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
uint8_t data);
#line 80
static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__txDone(
# 70 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x2b99bd185410);
# 80 "/opt/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId(void );
# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
# 39 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1____nesc_unnamed4362 {
#line 39
  FcfsResourceQueueC__1__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[2U];
uint8_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
uint8_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

static inline error_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );




static inline bool /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );







static inline resource_client_id_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
# 51 "/opt/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(
# 55 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c0020);
# 55 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(
# 60 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1be340);
# 49 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1be340);
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
uint8_t arg_0x2b99bd1c2d40);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void );
# 75 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
enum /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4363 {
#line 75
  ArbiterP__0__grantedTask = 13U
};
#line 75
typedef int /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4364 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4365 {
#line 68
  ArbiterP__0__default_owner_id = 2U
};
#line 69
enum /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4366 {
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
uint8_t StateImplP__state[9U];

enum StateImplP____nesc_unnamed4367 {
  StateImplP__S_IDLE = 0
};


static inline error_t StateImplP__Init__init(void );
#line 96
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState);
#line 111
static inline void StateImplP__State__forceState(uint8_t id, uint8_t reqState);






static inline void StateImplP__State__toIdle(uint8_t id);







static inline bool StateImplP__State__isIdle(uint8_t id);






static bool StateImplP__State__isState(uint8_t id, uint8_t myState);









static uint8_t StateImplP__State__getState(uint8_t id);
# 51 "/opt/tinyos-2.1.1/tos/system/ActiveMessageAddressC.nc"
am_addr_t ActiveMessageAddressC__addr = TOS_AM_ADDRESS;


am_group_t ActiveMessageAddressC__group = TOS_AM_GROUP;






static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );
#line 82
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
#line 95
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static error_t StatisticsP__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static void StatisticsP__State__toIdle(void );
# 66 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/FifoQueue.nc"
static bool StatisticsP__FifoQueue__isEmpty(void );
#line 51
static error_t StatisticsP__FifoQueue__dequeue(void *location, uint8_t maxSize);
# 55 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunitstats/StatisticsP.nc"
message_t StatisticsP__myMsg;




enum StatisticsP____nesc_unnamed4368 {
  StatisticsP__S_IDLE, 
  StatisticsP__S_BUSY
};
#line 109
static void StatisticsP__AMSend__sendDone(message_t *msg, error_t error);
#line 128
static inline bool StatisticsP__StatsQuery__isIdle(void );
# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static void /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__send(
#line 56
message_t * msg, 







uint8_t len);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
static void /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 151
static void /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 45 "/opt/tinyos-2.1.1/tos/system/AMQueueEntryP.nc"
static inline error_t /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__sendDone(message_t *m, error_t err);
# 47 "/opt/tinyos-2.x-contrib/tunit/tos/lib/fifoqueue/FifoQueueP.nc"
/*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__q_element_t /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__myQueue[1U];


uint8_t /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__nextEnqueueLocation;


uint8_t /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__currentDequeueLocation;


uint8_t /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__currentSize;


static inline error_t /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__Init__init(void );
#line 92
static inline error_t /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__FifoQueue__dequeue(void *location, uint8_t maxSize);
#line 129
static inline bool /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__FifoQueue__isEmpty(void );
#line 143
static inline void /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__FifoQueue__reset(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static void CC2420CsmaP__SplitControl__startDone(error_t error);
#line 117
static void CC2420CsmaP__SplitControl__stopDone(error_t error);
# 95 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420CsmaP__RadioBackoff__requestCca(message_t * msg);
#line 81
static void CC2420CsmaP__RadioBackoff__requestInitialBackoff(message_t * msg);






static void CC2420CsmaP__RadioBackoff__requestCongestionBackoff(message_t * msg);
#line 66
static void CC2420CsmaP__SubBackoff__setCongestionBackoff(uint16_t backoffTime);
#line 60
static void CC2420CsmaP__SubBackoff__setInitialBackoff(uint16_t backoffTime);
# 51 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static error_t CC2420CsmaP__CC2420Transmit__send(message_t * p_msg, bool useCca);
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void CC2420CsmaP__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 41 "/opt/tinyos-2.1.1/tos/interfaces/Random.nc"
static uint16_t CC2420CsmaP__Random__rand16(void );
# 74 "/opt/tinyos-2.1.1/tos/interfaces/StdControl.nc"
static error_t CC2420CsmaP__SubControl__start(void );









static error_t CC2420CsmaP__SubControl__stop(void );
# 42 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420CsmaP__CC2420PacketBody__getHeader(message_t * msg);




static cc2420_metadata_t * CC2420CsmaP__CC2420PacketBody__getMetadata(message_t * msg);
# 71 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Power.nc"
static error_t CC2420CsmaP__CC2420Power__startOscillator(void );
#line 90
static error_t CC2420CsmaP__CC2420Power__rxOn(void );
#line 51
static error_t CC2420CsmaP__CC2420Power__startVReg(void );
#line 63
static error_t CC2420CsmaP__CC2420Power__stopVReg(void );
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t CC2420CsmaP__Resource__release(void );
#line 78
static error_t CC2420CsmaP__Resource__request(void );
# 66 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static bool CC2420CsmaP__SplitControlState__isState(uint8_t myState);
#line 45
static error_t CC2420CsmaP__SplitControlState__requestState(uint8_t reqState);





static void CC2420CsmaP__SplitControlState__forceState(uint8_t reqState);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t CC2420CsmaP__sendDone_task__postTask(void );
#line 56
static error_t CC2420CsmaP__stopDone_task__postTask(void );
#line 56
static error_t CC2420CsmaP__startDone_task__postTask(void );
# 74 "/opt/tinyos-2.1.1/tos/chips/cc2420/csma/CC2420CsmaP.nc"
enum CC2420CsmaP____nesc_unnamed4369 {
#line 74
  CC2420CsmaP__startDone_task = 14U
};
#line 74
typedef int CC2420CsmaP____nesc_sillytask_startDone_task[CC2420CsmaP__startDone_task];
enum CC2420CsmaP____nesc_unnamed4370 {
#line 75
  CC2420CsmaP__stopDone_task = 15U
};
#line 75
typedef int CC2420CsmaP____nesc_sillytask_stopDone_task[CC2420CsmaP__stopDone_task];
enum CC2420CsmaP____nesc_unnamed4371 {
#line 76
  CC2420CsmaP__sendDone_task = 16U
};
#line 76
typedef int CC2420CsmaP____nesc_sillytask_sendDone_task[CC2420CsmaP__sendDone_task];
#line 58
enum CC2420CsmaP____nesc_unnamed4372 {
  CC2420CsmaP__S_STOPPED, 
  CC2420CsmaP__S_STARTING, 
  CC2420CsmaP__S_STARTED, 
  CC2420CsmaP__S_STOPPING, 
  CC2420CsmaP__S_TRANSMITTING
};

message_t * CC2420CsmaP__m_msg;

error_t CC2420CsmaP__sendErr = SUCCESS;


bool CC2420CsmaP__ccaOn;






static inline void CC2420CsmaP__shutdown(void );


static error_t CC2420CsmaP__SplitControl__start(void );
#line 96
static inline error_t CC2420CsmaP__SplitControl__stop(void );
#line 122
static inline error_t CC2420CsmaP__Send__send(message_t *p_msg, uint8_t len);
#line 170
static inline uint8_t CC2420CsmaP__Send__maxPayloadLength(void );
#line 202
static inline void CC2420CsmaP__CC2420Transmit__sendDone(message_t *p_msg, error_t err);




static inline void CC2420CsmaP__CC2420Power__startVRegDone(void );



static inline void CC2420CsmaP__Resource__granted(void );



static inline void CC2420CsmaP__CC2420Power__startOscillatorDone(void );




static inline void CC2420CsmaP__SubBackoff__requestInitialBackoff(message_t *msg);






static inline void CC2420CsmaP__SubBackoff__requestCongestionBackoff(message_t *msg);
#line 241
static inline void CC2420CsmaP__sendDone_task__runTask(void );
#line 254
static inline void CC2420CsmaP__startDone_task__runTask(void );







static inline void CC2420CsmaP__stopDone_task__runTask(void );









static inline void CC2420CsmaP__shutdown(void );
# 53 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ControlP__CC2420Config__syncDone(error_t error);
# 55 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__RXCTRL1__write(uint16_t data);
# 55 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupTimer__start(CC2420ControlP__StartupTimer__size_type dt);
# 55 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__MDMCTRL0__write(uint16_t data);
# 35 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__RSTN__makeOutput(void );
#line 29
static void CC2420ControlP__RSTN__set(void );
static void CC2420ControlP__RSTN__clr(void );
# 63 "/opt/tinyos-2.1.1/tos/interfaces/Read.nc"
static void CC2420ControlP__ReadRssi__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t CC2420ControlP__syncDone__postTask(void );
# 47 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__RSSI__read(uint16_t *data);







static cc2420_status_t CC2420ControlP__IOCFG0__write(uint16_t data);
# 50 "/opt/tinyos-2.1.1/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t CC2420ControlP__ActiveMessageAddress__amAddress(void );




static am_group_t CC2420ControlP__ActiveMessageAddress__amGroup(void );
# 35 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__CSN__makeOutput(void );
#line 29
static void CC2420ControlP__CSN__set(void );
static void CC2420ControlP__CSN__clr(void );




static void CC2420ControlP__VREN__makeOutput(void );
#line 29
static void CC2420ControlP__VREN__set(void );
static void CC2420ControlP__VREN__clr(void );
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SXOSCON__strobe(void );
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__SpiResource__release(void );
#line 78
static error_t CC2420ControlP__SpiResource__request(void );
#line 110
static error_t CC2420ControlP__SyncResource__release(void );
#line 78
static error_t CC2420ControlP__SyncResource__request(void );
# 76 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Power.nc"
static void CC2420ControlP__CC2420Power__startOscillatorDone(void );
#line 56
static void CC2420ControlP__CC2420Power__startVRegDone(void );
# 55 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__IOCFG1__write(uint16_t data);
#line 55
static cc2420_status_t CC2420ControlP__FSCTRL__write(uint16_t data);
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRXON__strobe(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void CC2420ControlP__Resource__granted(void );
# 63 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420ControlP__PANID__write(uint8_t offset, uint8_t * data, uint8_t length);
# 50 "/opt/tinyos-2.1.1/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ControlP__InterruptCCA__disable(void );
#line 42
static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void );
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__RssiResource__release(void );
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRFOFF__strobe(void );
# 117 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
enum CC2420ControlP____nesc_unnamed4373 {
#line 117
  CC2420ControlP__sync = 17U
};
#line 117
typedef int CC2420ControlP____nesc_sillytask_sync[CC2420ControlP__sync];
enum CC2420ControlP____nesc_unnamed4374 {
#line 118
  CC2420ControlP__syncDone = 18U
};
#line 118
typedef int CC2420ControlP____nesc_sillytask_syncDone[CC2420ControlP__syncDone];
#line 85
#line 79
typedef enum CC2420ControlP____nesc_unnamed4375 {
  CC2420ControlP__S_VREG_STOPPED, 
  CC2420ControlP__S_VREG_STARTING, 
  CC2420ControlP__S_VREG_STARTED, 
  CC2420ControlP__S_XOSC_STARTING, 
  CC2420ControlP__S_XOSC_STARTED
} CC2420ControlP__cc2420_control_state_t;

uint8_t CC2420ControlP__m_channel;

uint8_t CC2420ControlP__m_tx_power;

uint16_t CC2420ControlP__m_pan;

uint16_t CC2420ControlP__m_short_addr;

bool CC2420ControlP__m_sync_busy;


bool CC2420ControlP__autoAckEnabled;


bool CC2420ControlP__hwAutoAckDefault;


bool CC2420ControlP__addressRecognition;


bool CC2420ControlP__hwAddressRecognition;

CC2420ControlP__cc2420_control_state_t CC2420ControlP__m_state = CC2420ControlP__S_VREG_STOPPED;



static void CC2420ControlP__writeFsctrl(void );
static void CC2420ControlP__writeMdmctrl0(void );
static void CC2420ControlP__writeId(void );





static inline error_t CC2420ControlP__Init__init(void );
#line 171
static inline error_t CC2420ControlP__Resource__request(void );







static inline error_t CC2420ControlP__Resource__release(void );







static inline error_t CC2420ControlP__CC2420Power__startVReg(void );
#line 199
static error_t CC2420ControlP__CC2420Power__stopVReg(void );







static inline error_t CC2420ControlP__CC2420Power__startOscillator(void );
#line 249
static inline error_t CC2420ControlP__CC2420Power__rxOn(void );
#line 279
static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void );







static inline uint16_t CC2420ControlP__CC2420Config__getPanAddr(void );
#line 300
static inline error_t CC2420ControlP__CC2420Config__sync(void );
#line 332
static inline bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void );
#line 359
static inline bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void );






static inline bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void );









static inline void CC2420ControlP__SyncResource__granted(void );
#line 390
static inline void CC2420ControlP__SpiResource__granted(void );




static inline void CC2420ControlP__RssiResource__granted(void );
#line 408
static inline void CC2420ControlP__StartupTimer__fired(void );









static inline void CC2420ControlP__InterruptCCA__fired(void );
#line 442
static inline void CC2420ControlP__sync__runTask(void );



static inline void CC2420ControlP__syncDone__runTask(void );









static void CC2420ControlP__writeFsctrl(void );
#line 473
static void CC2420ControlP__writeMdmctrl0(void );
#line 492
static void CC2420ControlP__writeId(void );
#line 509
static inline void CC2420ControlP__ReadRssi__default__readDone(error_t error, uint16_t data);
# 30 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 67 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 46 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 36
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 33
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 42 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 54
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 53 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 56 "/opt/tinyos-2.1.1/tos/lib/timer/TransformCounterC.nc"
/*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type /*Counter32khz32C.Transform*/TransformCounterC__0__m_upper;

enum /*Counter32khz32C.Transform*/TransformCounterC__0____nesc_unnamed4376 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 0, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type ) - /*Counter32khz32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type ) + 0, 



  TransformCounterC__0__OVERFLOW_MASK = /*Counter32khz32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*Counter32khz32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 122
static inline void /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 67 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 92
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 62
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 53 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 66 "/opt/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0;
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt;

enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0____nesc_unnamed4377 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 0, 
  TransformAlarmC__0__MAX_DELAY = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 91
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 136
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt);









static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt);




static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 166
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 64 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput(void );
#line 59
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__get(void );
# 40 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get(void );
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void );
# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
# 59 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get(void );
# 40 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void );
# 59 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get(void );
# 40 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void );
# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void );
# 64 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput(void );
#line 59
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__get(void );
# 40 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get(void );
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void );
# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__clr(void );
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
# 57 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void );
# 50 "/opt/tinyos-2.1.1/tos/interfaces/GpioCapture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(uint16_t time);
# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm);

static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void );
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void );
#line 33
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void );
#line 78
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void );
# 38 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(uint8_t mode);
#line 50
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void );



static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void );



static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void );






static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__fired(void );
#line 61
static void HplMsp430InterruptP__Port26__fired(void );
#line 61
static void HplMsp430InterruptP__Port17__fired(void );
#line 61
static void HplMsp430InterruptP__Port21__fired(void );
#line 61
static void HplMsp430InterruptP__Port12__fired(void );
#line 61
static void HplMsp430InterruptP__Port24__fired(void );
#line 61
static void HplMsp430InterruptP__Port15__fired(void );
#line 61
static void HplMsp430InterruptP__Port27__fired(void );
#line 61
static void HplMsp430InterruptP__Port10__fired(void );
#line 61
static void HplMsp430InterruptP__Port22__fired(void );
#line 61
static void HplMsp430InterruptP__Port13__fired(void );
#line 61
static void HplMsp430InterruptP__Port25__fired(void );
#line 61
static void HplMsp430InterruptP__Port16__fired(void );
#line 61
static void HplMsp430InterruptP__Port20__fired(void );
#line 61
static void HplMsp430InterruptP__Port11__fired(void );
#line 61
static void HplMsp430InterruptP__Port23__fired(void );
# 84 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
void sig_PORT1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(36)))  ;
#line 122
static inline void HplMsp430InterruptP__Port10__default__fired(void );
static inline void HplMsp430InterruptP__Port11__default__fired(void );

static inline void HplMsp430InterruptP__Port13__default__fired(void );

static inline void HplMsp430InterruptP__Port15__default__fired(void );
static inline void HplMsp430InterruptP__Port16__default__fired(void );
static inline void HplMsp430InterruptP__Port17__default__fired(void );


static inline void HplMsp430InterruptP__Port12__enable(void );

static inline void HplMsp430InterruptP__Port14__enable(void );





static inline void HplMsp430InterruptP__Port12__disable(void );

static inline void HplMsp430InterruptP__Port14__disable(void );



static inline void HplMsp430InterruptP__Port10__clear(void );
static inline void HplMsp430InterruptP__Port11__clear(void );
static inline void HplMsp430InterruptP__Port12__clear(void );
static inline void HplMsp430InterruptP__Port13__clear(void );
static inline void HplMsp430InterruptP__Port14__clear(void );
static inline void HplMsp430InterruptP__Port15__clear(void );
static inline void HplMsp430InterruptP__Port16__clear(void );
static inline void HplMsp430InterruptP__Port17__clear(void );
#line 174
static inline void HplMsp430InterruptP__Port12__edge(bool l2h);
#line 186
static inline void HplMsp430InterruptP__Port14__edge(bool l2h);
#line 213
void sig_PORT2_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(38)))  ;
#line 249
static inline void HplMsp430InterruptP__Port20__default__fired(void );
static inline void HplMsp430InterruptP__Port21__default__fired(void );
static inline void HplMsp430InterruptP__Port22__default__fired(void );
static inline void HplMsp430InterruptP__Port23__default__fired(void );
static inline void HplMsp430InterruptP__Port24__default__fired(void );
static inline void HplMsp430InterruptP__Port25__default__fired(void );
static inline void HplMsp430InterruptP__Port26__default__fired(void );
static inline void HplMsp430InterruptP__Port27__default__fired(void );
#line 273
static inline void HplMsp430InterruptP__Port20__clear(void );
static inline void HplMsp430InterruptP__Port21__clear(void );
static inline void HplMsp430InterruptP__Port22__clear(void );
static inline void HplMsp430InterruptP__Port23__clear(void );
static inline void HplMsp430InterruptP__Port24__clear(void );
static inline void HplMsp430InterruptP__Port25__clear(void );
static inline void HplMsp430InterruptP__Port26__clear(void );
static inline void HplMsp430InterruptP__Port27__clear(void );
# 41 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear(void );
#line 36
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable(void );
#line 56
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high);
#line 31
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable(void );
# 57 "/opt/tinyos-2.1.1/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired(void );
# 41 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(bool rising);








static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );







static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 41 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear(void );
#line 36
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable(void );
#line 56
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high);
#line 31
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable(void );
# 57 "/opt/tinyos-2.1.1/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired(void );
# 41 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(bool rising);
#line 54
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void );



static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 59 "/opt/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
static error_t CC2420SpiP__SpiPacket__send(
#line 48
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
# 91 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420SpiP__Fifo__writeDone(
# 46 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd778328, 
# 91 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420SpiP__Fifo__readDone(
# 46 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd778328, 
# 71 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 24 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420SpiP__ChipSpiResource__releasing(void );
# 34 "/opt/tinyos-2.1.1/tos/interfaces/SpiByte.nc"
static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static void CC2420SpiP__WorkingState__toIdle(void );




static bool CC2420SpiP__WorkingState__isIdle(void );
#line 45
static error_t CC2420SpiP__WorkingState__requestState(uint8_t reqState);
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__SpiResource__release(void );
#line 87
static error_t CC2420SpiP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420SpiP__SpiResource__request(void );
#line 118
static bool CC2420SpiP__SpiResource__isOwner(void );
#line 92
static void CC2420SpiP__Resource__granted(
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b99bd77a158);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t CC2420SpiP__grant__postTask(void );
# 88 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
enum CC2420SpiP____nesc_unnamed4378 {
#line 88
  CC2420SpiP__grant = 19U
};
#line 88
typedef int CC2420SpiP____nesc_sillytask_grant[CC2420SpiP__grant];
#line 63
enum CC2420SpiP____nesc_unnamed4379 {
  CC2420SpiP__RESOURCE_COUNT = 5U, 
  CC2420SpiP__NO_HOLDER = 0xFF
};


enum CC2420SpiP____nesc_unnamed4380 {
  CC2420SpiP__S_IDLE, 
  CC2420SpiP__S_BUSY
};


uint16_t CC2420SpiP__m_addr;


uint8_t CC2420SpiP__m_requests = 0;


uint8_t CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;


bool CC2420SpiP__release;


static error_t CC2420SpiP__attemptRelease(void );







static inline void CC2420SpiP__ChipSpiResource__abortRelease(void );






static inline error_t CC2420SpiP__ChipSpiResource__attemptRelease(void );




static error_t CC2420SpiP__Resource__request(uint8_t id);
#line 126
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id);
#line 149
static error_t CC2420SpiP__Resource__release(uint8_t id);
#line 178
static inline uint8_t CC2420SpiP__Resource__isOwner(uint8_t id);





static inline void CC2420SpiP__SpiResource__granted(void );




static cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 209
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len);



static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 260
static cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len);
#line 287
static inline cc2420_status_t CC2420SpiP__Reg__read(uint8_t addr, uint16_t *data);
#line 305
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data);
#line 318
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr);










static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error);








static error_t CC2420SpiP__attemptRelease(void );
#line 358
static inline void CC2420SpiP__grant__runTask(void );








static inline void CC2420SpiP__Resource__default__granted(uint8_t id);


static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error);


static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task__postTask(void );
# 71 "/opt/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__sendDone(
# 76 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7e7368, 
# 64 "/opt/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Msp430SpiConfigure__getConfig(
# 79 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7e42f0);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__granted(
# 73 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7ece00);
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__release(
# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7e6108);
# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__immediateRequest(
# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7e6108);
# 78 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__request(
# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7e6108);
# 118 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__isOwner(
# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x2b99bd7e6108);
# 100 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__enableRxIntr(void );
#line 88
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__resetUsci(bool reset);
#line 106
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__clrRxIntr(void );
#line 97
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__disableRxIntr(void );
#line 115
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__tx(uint8_t data);






static uint8_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__rx(void );
#line 148
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__setModeSpi(msp430_spi_union_config_t *config);
#line 105
static bool /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__isRxIntrPending(void );
#line 137
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__disableSpi(void );
# 99 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
enum /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0____nesc_unnamed4381 {
#line 99
  Msp430SpiNoDmaBP__0__signalDone_task = 20U
};
#line 99
typedef int /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0____nesc_sillytask_signalDone_task[/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task];
#line 88
enum /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0____nesc_unnamed4382 {
  Msp430SpiNoDmaBP__0__SPI_ATOMIC_SIZE = 2
};

uint16_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_len;
uint8_t * /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_tx_buf;
uint8_t * /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_rx_buf;
uint16_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos;
uint8_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_client;

static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone(void );


static inline error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__immediateRequest(uint8_t id);



static inline error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__request(uint8_t id);



static inline uint8_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__isOwner(uint8_t id);



static inline error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__release(uint8_t id);



static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__ResourceConfigure__configure(uint8_t id);



static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__ResourceConfigure__unconfigure(uint8_t id);





static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__granted(uint8_t id);



static uint8_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiByte__write(uint8_t tx);
#line 143
static inline error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__isOwner(uint8_t id);
static inline error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__request(uint8_t id);
static inline error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__release(uint8_t id);




static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__default__granted(uint8_t id);

static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__continueOp(void );
#line 176
static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len);
#line 198
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task__runTask(void );



static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__rxDone(uint8_t data);
#line 215
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone(void );




static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__txDone(void );

static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error);
# 7 "/opt/tinyos-2.1.1/tos/platforms/z1/chips/msp430X/usci/Z1UsciP.nc"
msp430_spi_union_config_t /*Msp430SpiNoDmaB0P.Z1UsciP*/Z1UsciP__0__msp430_spi_z1_config = { { 
.ubr = 2, 
.ucmode = 0, 
.ucmst = 1, 
.uc7bit = 0, 
.ucmsb = 1, 
.ucckpl = 1, 
.ucckph = 0, 
.ucssel = 2 } };


static inline msp430_spi_union_config_t */*Msp430SpiNoDmaB0P.Z1UsciP*/Z1UsciP__0__Msp430SpiConfigure__getConfig(uint8_t id);
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void HplMsp430UsciB0P__UCLK__selectIOFunc(void );
#line 78
static void HplMsp430UsciB0P__UCLK__selectModuleFunc(void );
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void HplMsp430UsciB0P__Interrupts__rxDone(uint8_t data);
#line 80
static void HplMsp430UsciB0P__Interrupts__txDone(void );
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
static void HplMsp430UsciB0P__SOMI__selectIOFunc(void );
#line 78
static void HplMsp430UsciB0P__SOMI__selectModuleFunc(void );






static void HplMsp430UsciB0P__SIMO__selectIOFunc(void );
#line 78
static void HplMsp430UsciB0P__SIMO__selectModuleFunc(void );
# 79 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB0P.nc"
static volatile uint8_t HplMsp430UsciB0P__IE2 __asm ("0x0001");
static volatile uint8_t HplMsp430UsciB0P__IFG2 __asm ("0x0003");
static volatile uint8_t HplMsp430UsciB0P__UCB0CTL0 __asm ("0x0068");
static volatile uint8_t HplMsp430UsciB0P__UCB0CTL1 __asm ("0x0069");
static volatile uint8_t HplMsp430UsciB0P__UCB0RXBUF __asm ("0x006E");
static volatile uint8_t HplMsp430UsciB0P__UCB0TXBUF __asm ("0x006F");



static inline void HplMsp430UsciB0P__UsciRawInterrupts__rxDone(uint8_t temp);



static inline void HplMsp430UsciB0P__UsciRawInterrupts__txDone(void );
#line 113
static inline void HplMsp430UsciB0P__Usci__setUbr(uint16_t control);
#line 133
static inline void HplMsp430UsciB0P__Usci__resetUsci(bool reset);
#line 166
static inline void HplMsp430UsciB0P__Usci__enableSpi(void );







static inline void HplMsp430UsciB0P__Usci__disableSpi(void );







static inline void HplMsp430UsciB0P__configSpi(msp430_spi_union_config_t *config);





static void HplMsp430UsciB0P__Usci__setModeSpi(msp430_spi_union_config_t *config);
#line 205
static inline bool HplMsp430UsciB0P__Usci__isRxIntrPending(void );









static inline void HplMsp430UsciB0P__Usci__clrRxIntr(void );



static inline void HplMsp430UsciB0P__Usci__clrIntr(void );



static inline void HplMsp430UsciB0P__Usci__disableRxIntr(void );







static inline void HplMsp430UsciB0P__Usci__disableIntr(void );



static inline void HplMsp430UsciB0P__Usci__enableRxIntr(void );
#line 256
static inline void HplMsp430UsciB0P__Usci__tx(uint8_t data);



static inline uint8_t HplMsp430UsciB0P__Usci__rx(void );
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__Interrupts__rxDone(
# 70 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x2b99bd185410, 
# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
uint8_t data);
#line 80
static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__Interrupts__txDone(
# 70 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
uint8_t arg_0x2b99bd185410);
# 80 "/opt/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__ArbiterInfo__userId(void );
# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__RawInterrupts__txDone(void );




static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__Interrupts__default__rxDone(uint8_t id, uint8_t data);
# 39 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2____nesc_unnamed4383 {
#line 39
  FcfsResourceQueueC__2__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ[2U];
uint8_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead = /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
uint8_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qTail = /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

static inline error_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__Init__init(void );




static inline bool /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty(void );



static inline bool /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue(void );
#line 72
static inline error_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id);
# 43 "/opt/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(
# 55 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c0020);
# 51 "/opt/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(
# 55 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c0020);
# 55 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(
# 60 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1be340);
# 49 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(
# 60 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1be340);
# 69 "/opt/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue(void );
# 73 "/opt/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void );
#line 46
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(
# 54 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
uint8_t arg_0x2b99bd1c2d40);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask(void );
# 75 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
enum /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4384 {
#line 75
  ArbiterP__1__grantedTask = 21U
};
#line 75
typedef int /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_sillytask_grantedTask[/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask];
#line 67
enum /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4385 {
#line 67
  ArbiterP__1__RES_CONTROLLED, ArbiterP__1__RES_GRANTING, ArbiterP__1__RES_IMM_GRANTING, ArbiterP__1__RES_BUSY
};
#line 68
enum /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4386 {
#line 68
  ArbiterP__1__default_owner_id = 2U
};
#line 69
enum /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4387 {
#line 69
  ArbiterP__1__NO_RES = 0xFF
};
uint8_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
uint8_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id;
uint8_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;



static inline error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(uint8_t id);
#line 90
static inline error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(uint8_t id);
#line 108
static inline error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(uint8_t id);
#line 130
static error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
#line 150
static bool /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void );
#line 163
static uint8_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void );










static uint8_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(uint8_t id);
#line 187
static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
#line 199
static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id);

static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id);

static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(uint8_t id);

static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted(void );

static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested(void );


static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested(void );


static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id);
# 81 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420TransmitP__RadioBackoff__requestInitialBackoff(message_t * msg);






static void CC2420TransmitP__RadioBackoff__requestCongestionBackoff(message_t * msg);
# 59 "/opt/tinyos-2.1.1/tos/interfaces/PacketTimeStamp.nc"
static void CC2420TransmitP__PacketTimeStamp__clear(
#line 55
message_t * msg);
#line 67
static void CC2420TransmitP__PacketTimeStamp__set(
#line 62
message_t * msg, 




CC2420TransmitP__PacketTimeStamp__size_type value);
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__STXONCCA__strobe(void );
# 43 "/opt/tinyos-2.1.1/tos/interfaces/GpioCapture.nc"
static error_t CC2420TransmitP__CaptureSFD__captureFallingEdge(void );
#line 55
static void CC2420TransmitP__CaptureSFD__disable(void );
#line 42
static error_t CC2420TransmitP__CaptureSFD__captureRisingEdge(void );
# 98 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static CC2420TransmitP__BackoffTimer__size_type CC2420TransmitP__BackoffTimer__getNow(void );
#line 55
static void CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__BackoffTimer__size_type dt);






static void CC2420TransmitP__BackoffTimer__stop(void );
# 63 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420TransmitP__TXFIFO_RAM__write(uint8_t offset, uint8_t * data, uint8_t length);
# 55 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420TransmitP__TXCTRL__write(uint16_t data);
# 55 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420TransmitP__CC2420Receive__sfd_dropped(void );
#line 49
static void CC2420TransmitP__CC2420Receive__sfd(uint32_t time);
# 73 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static void CC2420TransmitP__Send__sendDone(message_t * p_msg, error_t error);
# 31 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP__ChipSpiResource__abortRelease(void );







static error_t CC2420TransmitP__ChipSpiResource__attemptRelease(void );
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__SFLUSHTX__strobe(void );
# 35 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CSN__makeOutput(void );
#line 29
static void CC2420TransmitP__CSN__set(void );
static void CC2420TransmitP__CSN__clr(void );
# 42 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420TransmitP__CC2420PacketBody__getHeader(message_t * msg);




static cc2420_metadata_t * CC2420TransmitP__CC2420PacketBody__getMetadata(message_t * msg);
# 47 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
static uint8_t CC2420TransmitP__PacketTimeSyncOffset__get(
#line 42
message_t * msg);
#line 39
static bool CC2420TransmitP__PacketTimeSyncOffset__isSet(
#line 35
message_t * msg);
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t CC2420TransmitP__SpiResource__release(void );
#line 87
static error_t CC2420TransmitP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420TransmitP__SpiResource__request(void );
# 33 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CCA__makeInput(void );
#line 32
static bool CC2420TransmitP__CCA__get(void );
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__SNOP__strobe(void );
# 33 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__SFD__makeInput(void );
#line 32
static bool CC2420TransmitP__SFD__get(void );
# 82 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static cc2420_status_t CC2420TransmitP__TXFIFO__write(uint8_t * data, uint8_t length);
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__STXON__strobe(void );
# 99 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
#line 89
typedef enum CC2420TransmitP____nesc_unnamed4388 {
  CC2420TransmitP__S_STOPPED, 
  CC2420TransmitP__S_STARTED, 
  CC2420TransmitP__S_LOAD, 
  CC2420TransmitP__S_SAMPLE_CCA, 
  CC2420TransmitP__S_BEGIN_TRANSMIT, 
  CC2420TransmitP__S_SFD, 
  CC2420TransmitP__S_EFD, 
  CC2420TransmitP__S_ACK_WAIT, 
  CC2420TransmitP__S_CANCEL
} CC2420TransmitP__cc2420_transmit_state_t;





enum CC2420TransmitP____nesc_unnamed4389 {
  CC2420TransmitP__CC2420_ABORT_PERIOD = 320
};
#line 120
message_t * CC2420TransmitP__m_msg;

bool CC2420TransmitP__m_cca;

uint8_t CC2420TransmitP__m_tx_power;

CC2420TransmitP__cc2420_transmit_state_t CC2420TransmitP__m_state = CC2420TransmitP__S_STOPPED;

bool CC2420TransmitP__m_receiving = FALSE;

uint16_t CC2420TransmitP__m_prev_time;


bool CC2420TransmitP__sfdHigh;


bool CC2420TransmitP__abortSpiRelease;


int8_t CC2420TransmitP__totalCcaChecks;


uint16_t CC2420TransmitP__myInitialBackoff;


uint16_t CC2420TransmitP__myCongestionBackoff;



static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca);

static void CC2420TransmitP__loadTXFIFO(void );
static void CC2420TransmitP__attemptSend(void );
static void CC2420TransmitP__congestionBackoff(void );
static error_t CC2420TransmitP__acquireSpiResource(void );
static inline error_t CC2420TransmitP__releaseSpiResource(void );
static void CC2420TransmitP__signalDone(error_t err);



static inline error_t CC2420TransmitP__Init__init(void );







static inline error_t CC2420TransmitP__StdControl__start(void );










static error_t CC2420TransmitP__StdControl__stop(void );
#line 192
static inline error_t CC2420TransmitP__Send__send(message_t * p_msg, bool useCca);
#line 243
static inline void CC2420TransmitP__RadioBackoff__setInitialBackoff(uint16_t backoffTime);







static inline void CC2420TransmitP__RadioBackoff__setCongestionBackoff(uint16_t backoffTime);







static __inline uint32_t CC2420TransmitP__getTime32(uint16_t time);
#line 278
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time);
#line 375
static inline void CC2420TransmitP__ChipSpiResource__releasing(void );
#line 387
static inline void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t *ack_msg);
#line 414
static inline void CC2420TransmitP__SpiResource__granted(void );
#line 452
static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);
#line 484
static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);










static inline void CC2420TransmitP__BackoffTimer__fired(void );
#line 545
static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca);
#line 735
static void CC2420TransmitP__attemptSend(void );
#line 780
static void CC2420TransmitP__congestionBackoff(void );






static error_t CC2420TransmitP__acquireSpiResource(void );







static inline error_t CC2420TransmitP__releaseSpiResource(void );
#line 817
static void CC2420TransmitP__loadTXFIFO(void );
#line 842
static void CC2420TransmitP__signalDone(error_t err);
# 32 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP__FIFO__get(void );
# 86 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Config.nc"
static bool CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled(void );
#line 110
static bool CC2420ReceiveP__CC2420Config__isAutoAckEnabled(void );
#line 105
static bool CC2420ReceiveP__CC2420Config__isHwAutoAckDefault(void );
#line 64
static uint16_t CC2420ReceiveP__CC2420Config__getShortAddr(void );
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t CC2420ReceiveP__receiveDone_task__postTask(void );
# 59 "/opt/tinyos-2.1.1/tos/interfaces/PacketTimeStamp.nc"
static void CC2420ReceiveP__PacketTimeStamp__clear(
#line 55
message_t * msg);
#line 67
static void CC2420ReceiveP__PacketTimeStamp__set(
#line 62
message_t * msg, 




CC2420ReceiveP__PacketTimeStamp__size_type value);
# 32 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP__FIFOP__get(void );
# 63 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420ReceiveP__CC2420Receive__receive(uint8_t type, message_t * message);
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP__SACK__strobe(void );
# 29 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
static void CC2420ReceiveP__CSN__set(void );
static void CC2420ReceiveP__CSN__clr(void );
# 42 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420ReceiveP__CC2420PacketBody__getHeader(message_t * msg);




static cc2420_metadata_t * CC2420ReceiveP__CC2420PacketBody__getMetadata(message_t * msg);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420ReceiveP__Receive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t CC2420ReceiveP__SpiResource__release(void );
#line 87
static error_t CC2420ReceiveP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420ReceiveP__SpiResource__request(void );
#line 118
static bool CC2420ReceiveP__SpiResource__isOwner(void );
# 62 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420ReceiveP__RXFIFO__continueRead(uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420ReceiveP__RXFIFO__beginRead(uint8_t * data, uint8_t length);
# 50 "/opt/tinyos-2.1.1/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ReceiveP__InterruptFIFOP__disable(void );
#line 43
static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void );
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP__SFLUSHRX__strobe(void );
# 148 "/opt/tinyos-2.1.1/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
enum CC2420ReceiveP____nesc_unnamed4390 {
#line 148
  CC2420ReceiveP__receiveDone_task = 22U
};
#line 148
typedef int CC2420ReceiveP____nesc_sillytask_receiveDone_task[CC2420ReceiveP__receiveDone_task];
#line 89
#line 81
typedef enum CC2420ReceiveP____nesc_unnamed4391 {
  CC2420ReceiveP__S_STOPPED, 
  CC2420ReceiveP__S_STARTED, 
  CC2420ReceiveP__S_RX_LENGTH, 
  CC2420ReceiveP__S_RX_DEC, 
  CC2420ReceiveP__S_RX_DEC_WAIT, 
  CC2420ReceiveP__S_RX_FCF, 
  CC2420ReceiveP__S_RX_PAYLOAD
} CC2420ReceiveP__cc2420_receive_state_t;

enum CC2420ReceiveP____nesc_unnamed4392 {
  CC2420ReceiveP__RXFIFO_SIZE = 128, 
  CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE = 8, 
  CC2420ReceiveP__SACK_HEADER_LENGTH = 7
};

uint32_t CC2420ReceiveP__m_timestamp_queue[CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE];

uint8_t CC2420ReceiveP__m_timestamp_head;

uint8_t CC2420ReceiveP__m_timestamp_size;





uint8_t CC2420ReceiveP__m_missed_packets;



bool CC2420ReceiveP__receivingPacket;


uint8_t CC2420ReceiveP__rxFrameLength;

uint8_t CC2420ReceiveP__m_bytes_left;

message_t * CC2420ReceiveP__m_p_rx_buf;

message_t CC2420ReceiveP__m_rx_buf;
#line 137
CC2420ReceiveP__cc2420_receive_state_t CC2420ReceiveP__m_state;



static void CC2420ReceiveP__reset_state(void );
static void CC2420ReceiveP__beginReceive(void );
static void CC2420ReceiveP__receive(void );
static void CC2420ReceiveP__waitForNextPacket(void );
static void CC2420ReceiveP__flush(void );
static inline bool CC2420ReceiveP__passesAddressCheck(message_t * msg);




static inline error_t CC2420ReceiveP__Init__init(void );





static inline error_t CC2420ReceiveP__StdControl__start(void );
#line 171
static error_t CC2420ReceiveP__StdControl__stop(void );
#line 186
static inline void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time);








static inline void CC2420ReceiveP__CC2420Receive__sfd_dropped(void );
#line 212
static inline void CC2420ReceiveP__InterruptFIFOP__fired(void );
#line 508
static inline void CC2420ReceiveP__SpiResource__granted(void );
#line 525
static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error);
#line 663
static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error);







static inline void CC2420ReceiveP__receiveDone_task__runTask(void );
#line 704
static inline void CC2420ReceiveP__CC2420Config__syncDone(error_t error);






static void CC2420ReceiveP__beginReceive(void );
#line 728
static void CC2420ReceiveP__flush(void );
#line 754
static void CC2420ReceiveP__receive(void );









static void CC2420ReceiveP__waitForNextPacket(void );
#line 808
static void CC2420ReceiveP__reset_state(void );










static inline bool CC2420ReceiveP__passesAddressCheck(message_t *msg);
# 65 "/opt/tinyos-2.1.1/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline error_t CC2420PacketP__Acks__requestAck(message_t *p_msg);









static inline bool CC2420PacketP__Acks__wasAcked(message_t *p_msg);
#line 98
static uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t *p_msg);







static void CC2420PacketP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId);







static inline cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg);



static inline cc2420_metadata_t *CC2420PacketP__CC2420PacketBody__getMetadata(message_t *msg);
#line 137
static void CC2420PacketP__PacketTimeStamp32khz__clear(message_t *msg);





static inline void CC2420PacketP__PacketTimeStamp32khz__set(message_t *msg, uint32_t value);
#line 176
static inline bool CC2420PacketP__PacketTimeSyncOffset__isSet(message_t *msg);








static inline uint8_t CC2420PacketP__PacketTimeSyncOffset__get(message_t *msg);
# 47 "/opt/tinyos-2.1.1/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 30 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void );
# 67 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void );
# 46 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void );
#line 36
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void );
#line 33
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void );
# 42 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );
#line 54
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 53 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__overflow(void );
# 56 "/opt/tinyos-2.1.1/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__1__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__1____nesc_unnamed4393 {

  TransformCounterC__1__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__1__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT, 
  TransformCounterC__1__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type ) + 5, 



  TransformCounterC__1__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get(void );
#line 122
static inline void /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
# 67 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__fired(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__stop(void );
# 53 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get(void );
# 66 "/opt/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1____nesc_unnamed4394 {

  TransformAlarmC__1__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type ) - 1 - 5, 
  TransformAlarmC__1__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm(void );
#line 136
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type dt);
#line 151
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
#line 166
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 98 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 72 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 63 "/opt/tinyos-2.1.1/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4395 {
#line 63
  AlarmToTimerC__0__fired = 23U
};
#line 63
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired];
#line 44
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot);
#line 60
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 125 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 118
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 37 "/opt/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b99bdc492f8);
#line 60
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4396 {
#line 60
  VirtualizeTimerC__0__updateFromTimer = 24U
};
#line 60
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4397 {

  VirtualizeTimerC__0__NUM_TIMERS = 4U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4398 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now);
#line 89
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
#line 128
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num);




static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num);
#line 193
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 47 "/opt/tinyos-2.1.1/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 41 "/opt/tinyos-2.1.1/tos/system/RandomMlcgC.nc"
uint32_t RandomMlcgC__seed;


static inline error_t RandomMlcgC__Init__init(void );
#line 58
static uint32_t RandomMlcgC__Random__rand32(void );
#line 78
static inline uint16_t RandomMlcgC__Random__rand16(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t UniqueSendP__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
#line 101
static uint8_t UniqueSendP__SubSend__maxPayloadLength(void );
#line 89
static void UniqueSendP__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 41 "/opt/tinyos-2.1.1/tos/interfaces/Random.nc"
static uint16_t UniqueSendP__Random__rand16(void );
# 42 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * UniqueSendP__CC2420PacketBody__getHeader(message_t * msg);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static void UniqueSendP__State__toIdle(void );
#line 45
static error_t UniqueSendP__State__requestState(uint8_t reqState);
# 54 "/opt/tinyos-2.1.1/tos/chips/cc2420/unique/UniqueSendP.nc"
uint8_t UniqueSendP__localSendId;

enum UniqueSendP____nesc_unnamed4399 {
  UniqueSendP__S_IDLE, 
  UniqueSendP__S_SENDING
};


static inline error_t UniqueSendP__Init__init(void );
#line 75
static error_t UniqueSendP__Send__send(message_t *msg, uint8_t len);
#line 95
static inline uint8_t UniqueSendP__Send__maxPayloadLength(void );








static inline void UniqueSendP__SubSend__sendDone(message_t *msg, error_t error);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



UniqueReceiveP__Receive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 42 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * UniqueReceiveP__CC2420PacketBody__getHeader(message_t * msg);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



UniqueReceiveP__DuplicateReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 59 "/opt/tinyos-2.1.1/tos/chips/cc2420/unique/UniqueReceiveP.nc"
#line 56
struct UniqueReceiveP____nesc_unnamed4400 {
  am_addr_t source;
  uint8_t dsn;
} UniqueReceiveP__receivedMessages[4];

uint8_t UniqueReceiveP__writeIndex = 0;


uint8_t UniqueReceiveP__recycleSourceElement;

enum UniqueReceiveP____nesc_unnamed4401 {
  UniqueReceiveP__INVALID_ELEMENT = 0xFF
};


static inline error_t UniqueReceiveP__Init__init(void );









static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn);
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn);


static inline message_t *UniqueReceiveP__SubReceive__receive(message_t *msg, void *payload, 
uint8_t len);
#line 111
static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn);
#line 137
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn);
#line 158
static inline message_t *UniqueReceiveP__DuplicateReceive__default__receive(message_t *msg, void *payload, uint8_t len);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t CC2420TinyosNetworkP__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
#line 101
static uint8_t CC2420TinyosNetworkP__SubSend__maxPayloadLength(void );
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t CC2420TinyosNetworkP__grantTask__postTask(void );
# 77 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Packet.nc"
static void CC2420TinyosNetworkP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId);
#line 75
static uint8_t CC2420TinyosNetworkP__CC2420Packet__getNetwork(message_t *p_msg);
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__ActiveSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 69 "/opt/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
static error_t CC2420TinyosNetworkP__Queue__enqueue(resource_client_id_t id);
#line 43
static bool CC2420TinyosNetworkP__Queue__isEmpty(void );
#line 60
static resource_client_id_t CC2420TinyosNetworkP__Queue__dequeue(void );
# 42 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420TinyosNetworkP__CC2420PacketBody__getHeader(message_t * msg);




static cc2420_metadata_t * CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(message_t * msg);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420TinyosNetworkP__BareReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static void CC2420TinyosNetworkP__Resource__granted(
# 46 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x2b99bdd37550);
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__BareSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420TinyosNetworkP__ActiveReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 148 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
enum CC2420TinyosNetworkP____nesc_unnamed4402 {
#line 148
  CC2420TinyosNetworkP__grantTask = 25U
};
#line 148
typedef int CC2420TinyosNetworkP____nesc_sillytask_grantTask[CC2420TinyosNetworkP__grantTask];
#line 66
enum CC2420TinyosNetworkP____nesc_unnamed4403 {
  CC2420TinyosNetworkP__OWNER_NONE = 0xff, 
  CC2420TinyosNetworkP__TINYOS_N_NETWORKS = 1U
};

uint8_t CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__OWNER_NONE;
#line 71
uint8_t CC2420TinyosNetworkP__next_owner;

static inline error_t CC2420TinyosNetworkP__ActiveSend__send(message_t *msg, uint8_t len);








static inline uint8_t CC2420TinyosNetworkP__ActiveSend__maxPayloadLength(void );



static inline void *CC2420TinyosNetworkP__ActiveSend__getPayload(message_t *msg, uint8_t len);
#line 118
static inline void CC2420TinyosNetworkP__SubSend__sendDone(message_t *msg, error_t error);








static inline message_t *CC2420TinyosNetworkP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 148
static inline void CC2420TinyosNetworkP__grantTask__runTask(void );
#line 167
static inline error_t CC2420TinyosNetworkP__Resource__request(uint8_t id);
#line 184
static inline error_t CC2420TinyosNetworkP__Resource__immediateRequest(uint8_t id);
#line 198
static inline error_t CC2420TinyosNetworkP__Resource__release(uint8_t id);
#line 210
static inline message_t *CC2420TinyosNetworkP__BareReceive__default__receive(message_t *msg, void *payload, uint8_t len);


static inline void CC2420TinyosNetworkP__BareSend__default__sendDone(message_t *msg, error_t error);








static inline void CC2420TinyosNetworkP__Resource__default__granted(uint8_t client);
# 39 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
enum /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0____nesc_unnamed4404 {
#line 39
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[1];
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void );




static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );



static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
#line 72
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t CC2420ActiveMessageP__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
#line 114
static 
#line 112
void * 

CC2420ActiveMessageP__SubSend__getPayload(
#line 111
message_t * msg, 


uint8_t len);
#line 101
static uint8_t CC2420ActiveMessageP__SubSend__maxPayloadLength(void );
# 70 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Config.nc"
static uint16_t CC2420ActiveMessageP__CC2420Config__getPanAddr(void );
# 95 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420ActiveMessageP__RadioBackoff__requestCca(
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b99bddf1d60, 
# 95 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
#line 81
static void CC2420ActiveMessageP__RadioBackoff__requestInitialBackoff(
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b99bddf1d60, 
# 81 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);






static void CC2420ActiveMessageP__RadioBackoff__requestCongestionBackoff(
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b99bddf1d60, 
# 88 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
# 59 "/opt/tinyos-2.1.1/tos/interfaces/SendNotifier.nc"
static void CC2420ActiveMessageP__SendNotifier__aboutToSend(
# 44 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b99bddf1258, 
# 59 "/opt/tinyos-2.1.1/tos/interfaces/SendNotifier.nc"
am_addr_t dest, 
#line 57
message_t * msg);
# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static void CC2420ActiveMessageP__AMSend__sendDone(
# 39 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b99bddf66c0, 
# 92 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420ActiveMessageP__Snoop__receive(
# 41 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b99bddf4480, 
# 60 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 50 "/opt/tinyos-2.1.1/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t CC2420ActiveMessageP__ActiveMessageAddress__amAddress(void );
# 42 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420ActiveMessageP__CC2420PacketBody__getHeader(message_t * msg);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420ActiveMessageP__Receive__receive(
# 40 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b99bddf5908, 
# 60 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
static error_t CC2420ActiveMessageP__RadioResource__release(void );
#line 87
static error_t CC2420ActiveMessageP__RadioResource__immediateRequest(void );
#line 78
static error_t CC2420ActiveMessageP__RadioResource__request(void );
# 62 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
uint16_t CC2420ActiveMessageP__pending_length;
message_t *CC2420ActiveMessageP__pending_message = (void *)0;

static void CC2420ActiveMessageP__RadioResource__granted(void );
#line 78
static error_t CC2420ActiveMessageP__AMSend__send(am_id_t id, am_addr_t addr, 
message_t *msg, 
uint8_t len);
#line 113
static inline uint8_t CC2420ActiveMessageP__AMSend__maxPayloadLength(am_id_t id);








static inline am_addr_t CC2420ActiveMessageP__AMPacket__address(void );



static am_addr_t CC2420ActiveMessageP__AMPacket__destination(message_t *amsg);




static am_addr_t CC2420ActiveMessageP__AMPacket__source(message_t *amsg);




static inline void CC2420ActiveMessageP__AMPacket__setDestination(message_t *amsg, am_addr_t addr);









static inline bool CC2420ActiveMessageP__AMPacket__isForMe(message_t *amsg);




static am_id_t CC2420ActiveMessageP__AMPacket__type(message_t *amsg);




static void CC2420ActiveMessageP__AMPacket__setType(message_t *amsg, am_id_t type);
#line 181
static uint8_t CC2420ActiveMessageP__Packet__payloadLength(message_t *msg);



static void CC2420ActiveMessageP__Packet__setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t CC2420ActiveMessageP__Packet__maxPayloadLength(void );



static inline void *CC2420ActiveMessageP__Packet__getPayload(message_t *msg, uint8_t len);





static inline void CC2420ActiveMessageP__SubSend__sendDone(message_t *msg, error_t result);






static inline message_t *CC2420ActiveMessageP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 222
static inline void CC2420ActiveMessageP__CC2420Config__syncDone(error_t error);





static inline void CC2420ActiveMessageP__SubBackoff__requestInitialBackoff(message_t *msg);




static inline void CC2420ActiveMessageP__SubBackoff__requestCongestionBackoff(message_t *msg);



static inline void CC2420ActiveMessageP__SubBackoff__requestCca(message_t *msg);
#line 266
static inline message_t *CC2420ActiveMessageP__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);



static inline message_t *CC2420ActiveMessageP__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);







static inline void CC2420ActiveMessageP__SendNotifier__default__aboutToSend(am_id_t amId, am_addr_t addr, message_t *msg);

static inline void CC2420ActiveMessageP__RadioBackoff__default__requestInitialBackoff(am_id_t id, 
message_t *msg);


static inline void CC2420ActiveMessageP__RadioBackoff__default__requestCongestionBackoff(am_id_t id, 
message_t *msg);


static inline void CC2420ActiveMessageP__RadioBackoff__default__requestCca(am_id_t id, 
message_t *msg);
# 43 "/opt/tinyos-2.1.1/tos/lib/net/RootControl.nc"
static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__RootControl__isRoot(void );
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
#line 112
static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubSend__maxPayloadLength(void );
# 50 "/opt/tinyos-2.1.1/tos/lib/net/CollectionDebug.nc"
static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(uint8_t type);
#line 62
static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEventMsg(uint8_t type, uint16_t msg, am_addr_t origin, am_addr_t node);
# 51 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimator.nc"
static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__LinkEstimator__txAck(am_addr_t neighbor);



static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__LinkEstimator__txNoAck(am_addr_t neighbor);
# 40 "/opt/tinyos-2.1.1/tos/interfaces/Cache.nc"
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__SentCache__insert(/*CtpP.Forwarder*/CtpForwardingEngineP__0__SentCache__t item);







static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__SentCache__lookup(/*CtpP.Forwarder*/CtpForwardingEngineP__0__SentCache__t item);
# 31 "/opt/tinyos-2.1.1/tos/interfaces/Intercept.nc"
static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__Intercept__forward(
# 114 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
collection_id_t arg_0x2b99bde92da8, 
# 20 "/opt/tinyos-2.1.1/tos/interfaces/Intercept.nc"
message_t * msg, 

void * payload, 








uint8_t len);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*CtpP.Forwarder*/CtpForwardingEngineP__0__Snoop__receive(
# 113 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
collection_id_t arg_0x2b99bde92270, 
# 60 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 41 "/opt/tinyos-2.1.1/tos/interfaces/Random.nc"
static uint16_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Random__rand16(void );
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__sendDone(
# 111 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
uint8_t arg_0x2b99bde943f0, 
# 85 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 81 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__RetxmitTimer__isRunning(void );
#line 62
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__RetxmitTimer__startOneShot(uint32_t dt);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask__postTask(void );
# 73 "/opt/tinyos-2.1.1/tos/interfaces/Queue.nc"
static 
#line 71
/*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__t  

/*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__head(void );
#line 90
static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__enqueue(
#line 86
/*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__t  newVal);
#line 101
static 
#line 99
/*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__t  

/*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__element(uint8_t idx);
#line 81
static 
#line 79
/*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__t  

/*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__dequeue(void );
#line 50
static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__empty(void );







static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__size(void );
# 71 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpInfo.nc"
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__recomputeRoutes(void );
#line 59
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__triggerRouteUpdate(void );
#line 52
static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__getEtx(uint16_t *etx);
#line 66
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__triggerImmediateRouteUpdate(void );









static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__setNeighborCongested(am_addr_t n, bool congested);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubPacket__payloadLength(
#line 63
message_t * msg);
#line 115
static 
#line 112
void * 


/*CtpP.Forwarder*/CtpForwardingEngineP__0__SubPacket__getPayload(
#line 110
message_t * msg, 




uint8_t len);
#line 95
static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubPacket__maxPayloadLength(void );
#line 83
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubPacket__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*CtpP.Forwarder*/CtpForwardingEngineP__0__Receive__receive(
# 112 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
collection_id_t arg_0x2b99bde93648, 
# 60 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 49 "/opt/tinyos-2.1.1/tos/lib/net/UnicastNameFreeRouting.nc"
static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__UnicastNameFreeRouting__hasRoute(void );
#line 48
static am_addr_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__UnicastNameFreeRouting__nextHop(void );
# 48 "/opt/tinyos-2.1.1/tos/interfaces/PacketAcknowledgements.nc"
static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__PacketAcknowledgements__requestAck(
#line 42
message_t * msg);
#line 74
static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__PacketAcknowledgements__wasAcked(
#line 69
message_t * msg);
# 97 "/opt/tinyos-2.1.1/tos/interfaces/Pool.nc"
static 
#line 94
/*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__t * 


/*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__get(void );
#line 61
static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__empty(void );
#line 89
static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__put(
#line 85
/*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__t * newVal);
# 77 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
static am_addr_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__source(
#line 73
message_t * amsg);
#line 57
static am_addr_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__address(void );









static am_addr_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__destination(
#line 63
message_t * amsg);
# 97 "/opt/tinyos-2.1.1/tos/interfaces/Pool.nc"
static 
#line 94
/*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__t * 


/*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__get(void );
#line 61
static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__empty(void );
#line 89
static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__put(
#line 85
/*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__t * newVal);
# 46 "/opt/tinyos-2.1.1/tos/lib/net/CollectionId.nc"
static collection_id_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionId__fetch(
# 146 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
uint8_t arg_0x2b99bdecf968);
#line 240
enum /*CtpP.Forwarder*/CtpForwardingEngineP__0____nesc_unnamed4405 {
#line 240
  CtpForwardingEngineP__0__sendTask = 26U
};
#line 240
typedef int /*CtpP.Forwarder*/CtpForwardingEngineP__0____nesc_sillytask_sendTask[/*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask];
#line 169
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__startRetxmitTimer(uint16_t mask, uint16_t offset);
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__clearState(uint8_t state);
static inline bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__hasState(uint8_t state);
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__setState(uint8_t state);


enum /*CtpP.Forwarder*/CtpForwardingEngineP__0____nesc_unnamed4406 {
  CtpForwardingEngineP__0__QUEUE_CONGESTED = 0x1, 
  CtpForwardingEngineP__0__ROUTING_ON = 0x2, 
  CtpForwardingEngineP__0__RADIO_ON = 0x4, 
  CtpForwardingEngineP__0__ACK_PENDING = 0x8, 
  CtpForwardingEngineP__0__SENDING = 0x10
};


uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__forwardingState = 0;




am_addr_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__lastParent;



uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__seqno;

enum /*CtpP.Forwarder*/CtpForwardingEngineP__0____nesc_unnamed4407 {
  CtpForwardingEngineP__0__CLIENT_COUNT = 1U
};






fe_queue_entry_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__clientEntries[/*CtpP.Forwarder*/CtpForwardingEngineP__0__CLIENT_COUNT];
fe_queue_entry_t * /*CtpP.Forwarder*/CtpForwardingEngineP__0__clientPtrs[/*CtpP.Forwarder*/CtpForwardingEngineP__0__CLIENT_COUNT];







message_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__loopbackMsg;
message_t * /*CtpP.Forwarder*/CtpForwardingEngineP__0__loopbackMsgPtr;

static inline error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Init__init(void );
#line 228
static inline error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__StdControl__start(void );
#line 245
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__RadioControl__startDone(error_t err);









static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__startRetxmitTimer(uint16_t window, uint16_t offset);
#line 268
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__UnicastNameFreeRouting__routeFound(void );




static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__UnicastNameFreeRouting__noRoute(void );





static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__RadioControl__stopDone(error_t err);





static inline ctp_data_header_t */*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(message_t *m);
#line 300
static inline error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__send(uint8_t client, message_t *msg, uint8_t len);
#line 351
static inline uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__maxPayloadLength(uint8_t client);



static inline void */*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__getPayload(uint8_t client, message_t *msg, uint8_t len);
#line 377
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask__runTask(void );
#line 489
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__packetComplete(fe_queue_entry_t *qe, message_t *msg, bool success);
#line 533
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubSend__sendDone(message_t *msg, error_t error);
#line 584
static inline message_t * /*CtpP.Forwarder*/CtpForwardingEngineP__0__forward(message_t * m);
#line 674
static inline message_t *
/*CtpP.Forwarder*/CtpForwardingEngineP__0__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 737
static inline message_t *
/*CtpP.Forwarder*/CtpForwardingEngineP__0__SubSnoop__receive(message_t *msg, void *payload, uint8_t len);
#line 754
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__RetxmitTimer__fired(void );





static inline bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpCongestion__isCongested(void );








static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__LinkEstimator__evicted(am_addr_t neighbor);







static inline uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__payloadLength(message_t *msg);



static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__maxPayloadLength(void );



static void */*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__getPayload(message_t *msg, uint8_t len);








static am_addr_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getOrigin(message_t *msg);

static inline uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getSequenceNumber(message_t *msg);





static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getType(message_t *msg);
static am_addr_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getOrigin(message_t *msg);
static inline uint16_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getEtx(message_t *msg);
static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getSequenceNumber(message_t *msg);
static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getThl(message_t *msg);
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__setThl(message_t *msg, uint8_t thl);


static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__setEtx(message_t *msg, uint16_t e);

static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__option(message_t *msg, ctp_options_t opt);


static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__setOption(message_t *msg, ctp_options_t opt);


static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__clearOption(message_t *msg, ctp_options_t opt);






static inline bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__matchInstance(message_t *m1, message_t *m2);
#line 843
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__clearState(uint8_t state);


static inline bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__hasState(uint8_t state);


static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__setState(uint8_t state);






static inline 
#line 855
void 
/*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__default__sendDone(uint8_t client, message_t *msg, error_t error);



static inline 
#line 859
bool 
/*CtpP.Forwarder*/CtpForwardingEngineP__0__Intercept__default__forward(collection_id_t collectid, message_t *msg, void *payload, 
uint8_t len);



static inline message_t *
/*CtpP.Forwarder*/CtpForwardingEngineP__0__Receive__default__receive(collection_id_t collectid, message_t *msg, void *payload, 
uint8_t len);



static inline message_t *
/*CtpP.Forwarder*/CtpForwardingEngineP__0__Snoop__default__receive(collection_id_t collectid, message_t *msg, void *payload, 
uint8_t len);



static inline collection_id_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionId__default__fetch(uint8_t client);







static inline error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__default__logEvent(uint8_t type);








static inline error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__default__logEventMsg(uint8_t type, uint16_t msg, am_addr_t origin, am_addr_t node);
# 60 "/opt/tinyos-2.1.1/tos/system/PoolP.nc"
uint8_t /*CtpP.MessagePoolP.PoolP*/PoolP__0__free;
uint8_t /*CtpP.MessagePoolP.PoolP*/PoolP__0__index;
/*CtpP.MessagePoolP.PoolP*/PoolP__0__pool_t * /*CtpP.MessagePoolP.PoolP*/PoolP__0__queue[12];
/*CtpP.MessagePoolP.PoolP*/PoolP__0__pool_t /*CtpP.MessagePoolP.PoolP*/PoolP__0__pool[12];

static inline error_t /*CtpP.MessagePoolP.PoolP*/PoolP__0__Init__init(void );









static inline bool /*CtpP.MessagePoolP.PoolP*/PoolP__0__Pool__empty(void );
#line 88
static inline /*CtpP.MessagePoolP.PoolP*/PoolP__0__pool_t */*CtpP.MessagePoolP.PoolP*/PoolP__0__Pool__get(void );
#line 103
static error_t /*CtpP.MessagePoolP.PoolP*/PoolP__0__Pool__put(/*CtpP.MessagePoolP.PoolP*/PoolP__0__pool_t *newVal);
#line 60
uint8_t /*CtpP.QEntryPoolP.PoolP*/PoolP__1__free;
uint8_t /*CtpP.QEntryPoolP.PoolP*/PoolP__1__index;
/*CtpP.QEntryPoolP.PoolP*/PoolP__1__pool_t * /*CtpP.QEntryPoolP.PoolP*/PoolP__1__queue[12];
/*CtpP.QEntryPoolP.PoolP*/PoolP__1__pool_t /*CtpP.QEntryPoolP.PoolP*/PoolP__1__pool[12];

static inline error_t /*CtpP.QEntryPoolP.PoolP*/PoolP__1__Init__init(void );









static inline bool /*CtpP.QEntryPoolP.PoolP*/PoolP__1__Pool__empty(void );
#line 88
static inline /*CtpP.QEntryPoolP.PoolP*/PoolP__1__pool_t */*CtpP.QEntryPoolP.PoolP*/PoolP__1__Pool__get(void );
#line 103
static error_t /*CtpP.QEntryPoolP.PoolP*/PoolP__1__Pool__put(/*CtpP.QEntryPoolP.PoolP*/PoolP__1__pool_t *newVal);
# 48 "/opt/tinyos-2.1.1/tos/system/QueueC.nc"
/*CtpP.SendQueueP*/QueueC__0__queue_t  /*CtpP.SendQueueP*/QueueC__0__queue[13];
uint8_t /*CtpP.SendQueueP*/QueueC__0__head = 0;
uint8_t /*CtpP.SendQueueP*/QueueC__0__tail = 0;
uint8_t /*CtpP.SendQueueP*/QueueC__0__size = 0;

static inline bool /*CtpP.SendQueueP*/QueueC__0__Queue__empty(void );



static inline uint8_t /*CtpP.SendQueueP*/QueueC__0__Queue__size(void );



static inline uint8_t /*CtpP.SendQueueP*/QueueC__0__Queue__maxSize(void );



static inline /*CtpP.SendQueueP*/QueueC__0__queue_t /*CtpP.SendQueueP*/QueueC__0__Queue__head(void );



static inline void /*CtpP.SendQueueP*/QueueC__0__printQueue(void );
#line 85
static /*CtpP.SendQueueP*/QueueC__0__queue_t /*CtpP.SendQueueP*/QueueC__0__Queue__dequeue(void );
#line 97
static error_t /*CtpP.SendQueueP*/QueueC__0__Queue__enqueue(/*CtpP.SendQueueP*/QueueC__0__queue_t newVal);
#line 112
static inline /*CtpP.SendQueueP*/QueueC__0__queue_t /*CtpP.SendQueueP*/QueueC__0__Queue__element(uint8_t idx);
# 60 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpPacket.nc"
static am_addr_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getOrigin(message_t *msg);
#line 54
static uint8_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getThl(message_t *msg);








static uint8_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getSequenceNumber(message_t *msg);


static uint8_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getType(message_t *msg);
# 58 "/opt/tinyos-2.1.1/tos/lib/net/ctp/LruCtpMsgCacheP.nc"
#line 53
typedef struct /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0____nesc_unnamed4408 {
  am_addr_t origin;
  uint8_t seqno;
  collection_id_t type;
  uint8_t thl;
} /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__ctp_packet_sig_t;

/*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__ctp_packet_sig_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__cache[4];
uint8_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__first;
uint8_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__count;

static inline error_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__Init__init(void );
#line 84
static uint8_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__lookup(message_t *m);
#line 100
static inline void /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__remove(uint8_t i);
#line 116
static inline void /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__Cache__insert(message_t *m);
#line 135
static inline bool /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__Cache__lookup(message_t *m);
# 61 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimator.nc"
static void LinkEstimatorP__LinkEstimator__evicted(am_addr_t neighbor);
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static error_t LinkEstimatorP__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
#line 99
static void LinkEstimatorP__Send__sendDone(
#line 92
message_t * msg, 






error_t error);
# 41 "/opt/tinyos-2.1.1/tos/interfaces/Random.nc"
static uint16_t LinkEstimatorP__Random__rand16(void );
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
static uint8_t LinkEstimatorP__SubPacket__payloadLength(
#line 63
message_t * msg);
#line 115
static 
#line 112
void * 


LinkEstimatorP__SubPacket__getPayload(
#line 110
message_t * msg, 




uint8_t len);
#line 95
static uint8_t LinkEstimatorP__SubPacket__maxPayloadLength(void );
# 77 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
static am_addr_t LinkEstimatorP__SubAMPacket__source(
#line 73
message_t * amsg);
#line 67
static am_addr_t LinkEstimatorP__SubAMPacket__destination(
#line 63
message_t * amsg);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



LinkEstimatorP__Receive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 43 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CompareBit.nc"
static bool LinkEstimatorP__CompareBit__shouldInsert(message_t * msg, void * payload, uint8_t len);
# 47 "/opt/tinyos-2.1.1/tos/interfaces/LinkPacketMetadata.nc"
static bool LinkEstimatorP__LinkPacketMetadata__highChannelQuality(
#line 44
message_t * msg);
# 58 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
enum LinkEstimatorP____nesc_unnamed4409 {


  LinkEstimatorP__EVICT_ETX_THRESHOLD = 65, 


  LinkEstimatorP__MAX_PKT_GAP = 10, 
  LinkEstimatorP__BEST_ETX = 10, 
  LinkEstimatorP__INVALID_RVAL = 0xff, 
  LinkEstimatorP__INVALID_NEIGHBOR_ADDR = 0xff, 


  LinkEstimatorP__VERY_LARGE_ETX_VALUE = 0xffff, 


  LinkEstimatorP__ALPHA = 9, 


  LinkEstimatorP__DLQ_PKT_WINDOW = 5, 


  LinkEstimatorP__BLQ_PKT_WINDOW = 3, 



  LinkEstimatorP__LARGE_ETX_VALUE = 70
};


neighbor_table_entry_t LinkEstimatorP__NeighborTable[10];

uint8_t LinkEstimatorP__linkEstSeq = 0;



uint8_t LinkEstimatorP__prevSentIdx = 0;


static inline linkest_header_t *LinkEstimatorP__getHeader(message_t *m);




static inline linkest_footer_t *LinkEstimatorP__getFooter(message_t *m, uint8_t len);







static inline uint8_t LinkEstimatorP__addLinkEstHeaderAndFooter(message_t *msg, uint8_t len);
#line 163
static void LinkEstimatorP__initNeighborIdx(uint8_t i, am_addr_t ll_addr);
#line 176
static uint8_t LinkEstimatorP__findIdx(am_addr_t ll_addr);
#line 189
static uint8_t LinkEstimatorP__findEmptyNeighborIdx(void );
#line 202
static uint8_t LinkEstimatorP__findWorstNeighborIdx(uint8_t thresholdETX);
#line 237
static inline uint8_t LinkEstimatorP__findRandomNeighborIdx(void );
#line 275
static void LinkEstimatorP__updateETX(neighbor_table_entry_t *ne, uint16_t newEst);





static void LinkEstimatorP__updateDETX(neighbor_table_entry_t *ne);
#line 300
static inline uint16_t LinkEstimatorP__computeETX(uint8_t q1);
#line 315
static inline void LinkEstimatorP__updateNeighborTableEst(am_addr_t n);
#line 355
static void LinkEstimatorP__updateNeighborEntryIdx(uint8_t idx, uint8_t seq);
#line 391
static inline void LinkEstimatorP__print_neighbor_table(void );
#line 405
static void LinkEstimatorP__print_packet(message_t *msg, uint8_t len);










static inline void LinkEstimatorP__initNeighborTable(void );







static inline error_t LinkEstimatorP__StdControl__start(void );









static inline error_t LinkEstimatorP__Init__init(void );






static uint16_t LinkEstimatorP__LinkEstimator__getLinkQuality(am_addr_t neighbor);
#line 457
static inline error_t LinkEstimatorP__LinkEstimator__insertNeighbor(am_addr_t neighbor);
#line 485
static error_t LinkEstimatorP__LinkEstimator__pinNeighbor(am_addr_t neighbor);









static inline error_t LinkEstimatorP__LinkEstimator__unpinNeighbor(am_addr_t neighbor);
#line 507
static inline error_t LinkEstimatorP__LinkEstimator__txAck(am_addr_t neighbor);
#line 524
static inline error_t LinkEstimatorP__LinkEstimator__txNoAck(am_addr_t neighbor);
#line 540
static inline error_t LinkEstimatorP__LinkEstimator__clearDLQ(am_addr_t neighbor);
#line 555
static inline error_t LinkEstimatorP__Send__send(am_addr_t addr, message_t *msg, uint8_t len);










static inline void LinkEstimatorP__AMSend__sendDone(message_t *msg, error_t error);








static inline uint8_t LinkEstimatorP__Send__maxPayloadLength(void );



static inline void *LinkEstimatorP__Send__getPayload(message_t *msg, uint8_t len);






static inline void LinkEstimatorP__processReceivedMessage(message_t * msg, void * payload, uint8_t len);
#line 663
static inline message_t *LinkEstimatorP__SubReceive__receive(message_t *msg, 
void *payload, 
uint8_t len);
#line 679
static uint8_t LinkEstimatorP__Packet__payloadLength(message_t *msg);
#line 698
static uint8_t LinkEstimatorP__Packet__maxPayloadLength(void );




static void *LinkEstimatorP__Packet__getPayload(message_t *msg, uint8_t len);
# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static void /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__send(
#line 56
message_t * msg, 







uint8_t len);
#line 101
static uint8_t /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__maxPayloadLength(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
static void /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 151
static void /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 45 "/opt/tinyos-2.1.1/tos/system/AMQueueEntryP.nc"
static inline error_t /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__sendDone(message_t *m, error_t err);



static inline uint8_t /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__maxPayloadLength(void );
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__send(
# 40 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
am_id_t arg_0x2b99bcb95340, 
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
#line 112
static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__maxPayloadLength(
# 40 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
am_id_t arg_0x2b99bcb95340);
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__sendDone(
# 38 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2b99bcb961c8, 
# 85 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Packet__payloadLength(
#line 63
message_t * msg);
#line 83
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask__postTask(void );
# 67 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__destination(
#line 63
message_t * amsg);
#line 136
static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__type(
#line 132
message_t * amsg);
# 118 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__1____nesc_unnamed4410 {
#line 118
  AMQueueImplP__1__CancelTask = 27U
};
#line 118
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__1____nesc_sillytask_CancelTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__CancelTask];
#line 161
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__1____nesc_unnamed4411 {
#line 161
  AMQueueImplP__1__errorTask = 28U
};
#line 161
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__1____nesc_sillytask_errorTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask];
#line 49
#line 47
typedef struct /*AMQueueP.AMQueueImplP*/AMQueueImplP__1____nesc_unnamed4412 {
  message_t * msg;
} /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue_entry_t;

uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current = 2;
/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue_entry_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[2];
uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__cancelMask[2 / 8 + 1];

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__tryToSend(void );

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__nextPacket(void );
#line 82
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len);
#line 118
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__CancelTask__runTask(void );
#line 155
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__sendDone(uint8_t last, message_t * msg, error_t err);





static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask__runTask(void );




static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__tryToSend(void );
#line 181
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__sendDone(am_id_t id, message_t *msg, error_t err);
#line 199
static inline uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__maxPayloadLength(uint8_t id);







static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__default__sendDone(uint8_t id, message_t *msg, error_t err);
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__updateRouteTask__postTask(void );
# 68 "/opt/tinyos-2.1.1/tos/lib/net/CollectionDebug.nc"
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__CollectionDebug__logEventRoute(uint8_t type, am_addr_t parent, uint8_t hopcount, uint16_t metric);
#line 56
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__CollectionDebug__logEventDbg(uint8_t type, uint16_t arg1, uint16_t arg2, uint16_t arg3);
# 44 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimator.nc"
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__pinNeighbor(am_addr_t neighbor);
#line 41
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__insertNeighbor(am_addr_t neighbor);
#line 58
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__clearDLQ(am_addr_t neighbor);
#line 47
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__unpinNeighbor(am_addr_t neighbor);
#line 38
static uint16_t /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__getLinkQuality(uint16_t neighbor);
# 41 "/opt/tinyos-2.1.1/tos/interfaces/Random.nc"
static uint16_t /*CtpP.Router*/CtpRoutingEngineP__0__Random__rand16(void );
#line 35
static uint32_t /*CtpP.Router*/CtpRoutingEngineP__0__Random__rand32(void );
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__sendBeaconTask__postTask(void );
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
#line 124
static 
#line 122
void * 

/*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__getPayload(
#line 121
message_t * msg, 


uint8_t len);
#line 112
static uint8_t /*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__maxPayloadLength(void );
# 62 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void /*CtpP.Router*/CtpRoutingEngineP__0__BeaconTimer__startOneShot(uint32_t dt);




static void /*CtpP.Router*/CtpRoutingEngineP__0__BeaconTimer__stop(void );
#line 53
static void /*CtpP.Router*/CtpRoutingEngineP__0__RouteTimer__startPeriodic(uint32_t dt);
# 7 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpCongestion.nc"
static bool /*CtpP.Router*/CtpRoutingEngineP__0__CtpCongestion__isCongested(void );
# 77 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
static am_addr_t /*CtpP.Router*/CtpRoutingEngineP__0__AMPacket__source(
#line 73
message_t * amsg);
#line 57
static am_addr_t /*CtpP.Router*/CtpRoutingEngineP__0__AMPacket__address(void );
#line 136
static am_id_t /*CtpP.Router*/CtpRoutingEngineP__0__AMPacket__type(
#line 132
message_t * amsg);
# 51 "/opt/tinyos-2.1.1/tos/lib/net/UnicastNameFreeRouting.nc"
static void /*CtpP.Router*/CtpRoutingEngineP__0__Routing__routeFound(void );
static void /*CtpP.Router*/CtpRoutingEngineP__0__Routing__noRoute(void );
# 252 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
enum /*CtpP.Router*/CtpRoutingEngineP__0____nesc_unnamed4413 {
#line 252
  CtpRoutingEngineP__0__updateRouteTask = 29U
};
#line 252
typedef int /*CtpP.Router*/CtpRoutingEngineP__0____nesc_sillytask_updateRouteTask[/*CtpP.Router*/CtpRoutingEngineP__0__updateRouteTask];
#line 375
enum /*CtpP.Router*/CtpRoutingEngineP__0____nesc_unnamed4414 {
#line 375
  CtpRoutingEngineP__0__sendBeaconTask = 30U
};
#line 375
typedef int /*CtpP.Router*/CtpRoutingEngineP__0____nesc_sillytask_sendBeaconTask[/*CtpP.Router*/CtpRoutingEngineP__0__sendBeaconTask];
#line 125
bool /*CtpP.Router*/CtpRoutingEngineP__0__ECNOff = TRUE;



bool /*CtpP.Router*/CtpRoutingEngineP__0__radioOn = FALSE;


bool /*CtpP.Router*/CtpRoutingEngineP__0__running = FALSE;

bool /*CtpP.Router*/CtpRoutingEngineP__0__sending = FALSE;


bool /*CtpP.Router*/CtpRoutingEngineP__0__justEvicted = FALSE;

route_info_t /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo;
bool /*CtpP.Router*/CtpRoutingEngineP__0__state_is_root;
am_addr_t /*CtpP.Router*/CtpRoutingEngineP__0__my_ll_addr;

message_t /*CtpP.Router*/CtpRoutingEngineP__0__beaconMsgBuffer;
ctp_routing_header_t */*CtpP.Router*/CtpRoutingEngineP__0__beaconMsg;


routing_table_entry /*CtpP.Router*/CtpRoutingEngineP__0__routingTable[10];
uint8_t /*CtpP.Router*/CtpRoutingEngineP__0__routingTableActive;


uint32_t /*CtpP.Router*/CtpRoutingEngineP__0__parentChanges;



static inline void /*CtpP.Router*/CtpRoutingEngineP__0__routingTableInit(void );
static uint8_t /*CtpP.Router*/CtpRoutingEngineP__0__routingTableFind(am_addr_t arg_0x2b99be150cb8);
static inline error_t /*CtpP.Router*/CtpRoutingEngineP__0__routingTableUpdateEntry(am_addr_t arg_0x2b99be1c86b8, am_addr_t arg_0x2b99be1c8980, uint16_t arg_0x2b99be1c8c40);
static inline error_t /*CtpP.Router*/CtpRoutingEngineP__0__routingTableEvict(am_addr_t neighbor);

uint32_t /*CtpP.Router*/CtpRoutingEngineP__0__currentInterval = 128;
uint32_t /*CtpP.Router*/CtpRoutingEngineP__0__t;
bool /*CtpP.Router*/CtpRoutingEngineP__0__tHasPassed;

static void /*CtpP.Router*/CtpRoutingEngineP__0__chooseAdvertiseTime(void );








static inline void /*CtpP.Router*/CtpRoutingEngineP__0__resetInterval(void );




static inline void /*CtpP.Router*/CtpRoutingEngineP__0__decayInterval(void );







static inline void /*CtpP.Router*/CtpRoutingEngineP__0__remainingInterval(void );






static inline error_t /*CtpP.Router*/CtpRoutingEngineP__0__Init__init(void );
#line 209
static inline error_t /*CtpP.Router*/CtpRoutingEngineP__0__StdControl__start(void );
#line 226
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__RadioControl__startDone(error_t error);










static inline void /*CtpP.Router*/CtpRoutingEngineP__0__RadioControl__stopDone(error_t error);






static inline bool /*CtpP.Router*/CtpRoutingEngineP__0__passLinkEtxThreshold(uint16_t etx);







static inline void /*CtpP.Router*/CtpRoutingEngineP__0__updateRouteTask__runTask(void );
#line 375
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__sendBeaconTask__runTask(void );
#line 416
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__sendDone(message_t *msg, error_t error);







static inline void /*CtpP.Router*/CtpRoutingEngineP__0__RouteTimer__fired(void );





static inline void /*CtpP.Router*/CtpRoutingEngineP__0__BeaconTimer__fired(void );
#line 445
static inline ctp_routing_header_t */*CtpP.Router*/CtpRoutingEngineP__0__getHeader(message_t * m);






static inline message_t */*CtpP.Router*/CtpRoutingEngineP__0__BeaconReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 502
static void /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__evicted(am_addr_t neighbor);
#line 514
static inline am_addr_t /*CtpP.Router*/CtpRoutingEngineP__0__Routing__nextHop(void );


static inline bool /*CtpP.Router*/CtpRoutingEngineP__0__Routing__hasRoute(void );
#line 531
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__getEtx(uint16_t *etx);
#line 544
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__recomputeRoutes(void );



static inline void /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__triggerRouteUpdate(void );



static inline void /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__triggerImmediateRouteUpdate(void );



static void /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__setNeighborCongested(am_addr_t n, bool congested);
#line 586
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__RootControl__setRoot(void );
#line 611
static inline bool /*CtpP.Router*/CtpRoutingEngineP__0__RootControl__isRoot(void );
#line 632
static inline bool /*CtpP.Router*/CtpRoutingEngineP__0__CompareBit__shouldInsert(message_t *msg, void *payload, uint8_t len);
#line 681
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__routingTableInit(void );





static uint8_t /*CtpP.Router*/CtpRoutingEngineP__0__routingTableFind(am_addr_t neighbor);
#line 699
static inline error_t /*CtpP.Router*/CtpRoutingEngineP__0__routingTableUpdateEntry(am_addr_t from, am_addr_t parent, uint16_t etx);
#line 742
static inline error_t /*CtpP.Router*/CtpRoutingEngineP__0__routingTableEvict(am_addr_t neighbor);
#line 765
static inline error_t /*CtpP.Router*/CtpRoutingEngineP__0__CollectionDebug__default__logEventDbg(uint8_t type, uint16_t arg1, uint16_t arg2, uint16_t arg3);





static inline error_t /*CtpP.Router*/CtpRoutingEngineP__0__CollectionDebug__default__logEventRoute(uint8_t type, am_addr_t parent, uint8_t hopcount, uint16_t etx);



static bool /*CtpP.Router*/CtpRoutingEngineP__0__CtpRoutingPacket__getOption(message_t *msg, ctp_options_t opt);
# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
static void /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__Send__send(
#line 56
message_t * msg, 







uint8_t len);
# 92 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
static void /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 151
static void /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 45 "/opt/tinyos-2.1.1/tos/system/AMQueueEntryP.nc"
static inline error_t /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__Send__sendDone(message_t *m, error_t err);
# 8 "/opt/tinyos-2.1.1/tos/lib/net/ctp/DummyActiveMessageP.nc"
static inline bool DummyActiveMessageP__LinkPacketMetadata__highChannelQuality(message_t *msg);
# 50 "/opt/tinyos-2.1.1/tos/lib/net/CollectionIdP.nc"
static inline collection_id_t /*HomeCommManagerC.CollectionSenderC.CollectionSenderP.CollectionIdP*/CollectionIdP__0__CollectionId__fetch(void );
# 92 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static void HomeCommManagerP__SplitControl__startDone(error_t error);
#line 117
static void HomeCommManagerP__SplitControl__stopDone(error_t error);
#line 83
static error_t HomeCommManagerP__RadioControl__start(void );
#line 109
static error_t HomeCommManagerP__RadioControl__stop(void );
# 74 "/opt/tinyos-2.1.1/tos/interfaces/StdControl.nc"
static error_t HomeCommManagerP__RoutingControl__start(void );
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
static error_t HomeCommManagerP__RadioSend__send(
#line 56
message_t * msg, 







uint8_t len);
#line 114
static 
#line 112
void * 

HomeCommManagerP__RadioSend__getPayload(
#line 111
message_t * msg, 


uint8_t len);
# 8 "/home/chuka/projects/puppet-os/interfaces/PuppetAPI.nc"
static void HomeCommManagerP__PuppetAPI__registerRequestDone(message_t *msg, error_t err);
# 26 "/home/chuka/projects/puppet-os/system/HomeCommManagerP.nc"
static inline error_t HomeCommManagerP__validateRegisterRequest(register_request_t *reg);
static inline error_t HomeCommManagerP__sendPacket(register_request_t *req);


int HomeCommManagerP__ready = 0;
message_t HomeCommManagerP__message_buf;

static inline error_t HomeCommManagerP__SplitControl__start(void );





static inline void HomeCommManagerP__RadioControl__startDone(error_t err);










static inline error_t HomeCommManagerP__SplitControl__stop(void );





static inline void HomeCommManagerP__RadioControl__stopDone(error_t err);






static inline error_t HomeCommManagerP__PuppetAPI__registerDeviceRequest(register_request_t *req);
#line 79
static inline void HomeCommManagerP__RadioSend__sendDone(message_t *msg, error_t e);




static inline error_t HomeCommManagerP__sendPacket(register_request_t *req);
#line 96
static inline error_t HomeCommManagerP__validateRegisterRequest(register_request_t *reg);
# 41 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/TestCase.nc"
static void TestAPIComplexP__TestRegister__done(void );
# 40 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
static void TestAPIComplexP__TearDownOneTime__done(void );
# 83 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
static error_t TestAPIComplexP__SplitControl__start(void );
#line 109
static error_t TestAPIComplexP__SplitControl__stop(void );
# 41 "/opt/tinyos-2.1.1/tos/lib/net/RootControl.nc"
static error_t TestAPIComplexP__RootControl__setRoot(void );
# 78 "/opt/tinyos-2.1.1/tos/interfaces/Leds.nc"
static void TestAPIComplexP__Leds__led2On(void );
# 40 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
static void TestAPIComplexP__SetUpOneTime__done(void );
# 6 "/home/chuka/projects/puppet-os/interfaces/PuppetAPI.nc"
static error_t TestAPIComplexP__PuppetAPI__registerDeviceRequest(register_request_t *reg);
# 22 "TestAPIComplexP.nc"
register_request_t *TestAPIComplexP__reg;

enum TestAPIComplexP____nesc_unnamed4415 {
  TestAPIComplexP__NUM_OF_PACKETS = 1000
};


static inline void TestAPIComplexP__SetUpOneTime__run(void );






static inline void TestAPIComplexP__TearDownOneTime__run(void );




static inline void TestAPIComplexP__SplitControl__startDone(error_t err);




static inline void TestAPIComplexP__SplitControl__stopDone(error_t err);




static inline void TestAPIComplexP__TestRegister__run(void );
#line 63
static inline void TestAPIComplexP__PuppetAPI__registerRequestDone(message_t *msg, error_t err);










static inline message_t *TestAPIComplexP__Snoop__receive(message_t *msg, void *payload, uint8_t len);





static message_t *TestAPIComplexP__Receive__receive(message_t *msg, void *payload, uint8_t len);
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
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x2b99bc98c690){
#line 28
  switch (arg_0x2b99bc98c690) {
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
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x2b99bc98c690);
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
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4416 {
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
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4417 {
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
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4418 {
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

# 166 "/opt/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

# 47 "/opt/tinyos-2.1.1/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 71 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
inline static void /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 71
  /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 71
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 71
}
#line 71
# 122 "/opt/tinyos-2.1.1/tos/lib/timer/TransformCounterC.nc"
static inline void /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*Counter32khz32C.Transform*/TransformCounterC__0__m_upper++;
    if ((/*Counter32khz32C.Transform*/TransformCounterC__0__m_upper & /*Counter32khz32C.Transform*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
      /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__overflow();
      }
  }
}

# 47 "/opt/tinyos-2.1.1/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void )
{
}

# 166 "/opt/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow(void )
{
}

# 71 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__overflow(void ){
#line 71
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow();
#line 71
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow();
#line 71
}
#line 71
# 122 "/opt/tinyos-2.1.1/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC__1__m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC__1__m_upper & /*CounterMilli32C.Transform*/TransformCounterC__1__OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__overflow();
      }
  }
}

# 71 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 71
  /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow();
#line 71
  /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
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

# 103 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void )
{
}

#line 103
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 37
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 37
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow();
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

# 78 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 171 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Resource__request(void )
#line 171
{
  return CC2420ControlP__SpiResource__request();
}

# 78 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420CsmaP__Resource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420ControlP__Resource__request();
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 207 "/opt/tinyos-2.1.1/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Power__startVRegDone(void )
#line 207
{
  CC2420CsmaP__Resource__request();
}

# 56 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static void CC2420ControlP__CC2420Power__startVRegDone(void ){
#line 56
  CC2420CsmaP__CC2420Power__startVRegDone();
#line 56
}
#line 56
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set();
}

# 29 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__set(void ){
#line 29
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set();
#line 29
}
#line 29
# 39 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr();
}

# 30 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__clr(void ){
#line 30
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr();
#line 30
}
#line 30
# 408 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__StartupTimer__fired(void )
#line 408
{
  if (CC2420ControlP__m_state == CC2420ControlP__S_VREG_STARTING) {
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTED;
      CC2420ControlP__RSTN__clr();
      CC2420ControlP__RSTN__set();
      CC2420ControlP__CC2420Power__startVRegDone();
    }
}

# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 795 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__releaseSpiResource(void )
#line 795
{
  CC2420TransmitP__SpiResource__release();
  return SUCCESS;
}

# 50 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void )
#line 50
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_RISING);
}

# 42 "/opt/tinyos-2.1.1/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP__CaptureSFD__captureRisingEdge(void ){
#line 42
  unsigned char __nesc_result;
#line 42

#line 42
  __nesc_result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge();
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP__SFLUSHTX__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SFLUSHTX);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 53 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
inline static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/opt/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get();
}

#line 146
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(), dt);
}

# 55 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__BackoffTimer__size_type dt){
#line 55
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(dt);
#line 55
}
#line 55
# 48 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )32U & (0x01 << 4);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw() != 0;
}

# 59 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__get(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__get();
}

# 32 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static bool CC2420TransmitP__CCA__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 496 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__BackoffTimer__fired(void )
#line 496
{
  /* atomic removed: atomic calls only */
#line 497
  {
    switch (CC2420TransmitP__m_state) {

        case CC2420TransmitP__S_SAMPLE_CCA: 


          if (CC2420TransmitP__CCA__get()) {
              CC2420TransmitP__m_state = CC2420TransmitP__S_BEGIN_TRANSMIT;
              CC2420TransmitP__BackoffTimer__start(CC2420_TIME_ACK_TURNAROUND);
            }
          else {
              CC2420TransmitP__congestionBackoff();
            }
        break;

        case CC2420TransmitP__S_BEGIN_TRANSMIT: 
          case CC2420TransmitP__S_CANCEL: 
            if (CC2420TransmitP__acquireSpiResource() == SUCCESS) {
                CC2420TransmitP__attemptSend();
              }
        break;

        case CC2420TransmitP__S_ACK_WAIT: 
          CC2420TransmitP__signalDone(SUCCESS);
        break;

        case CC2420TransmitP__S_SFD: 


          CC2420TransmitP__SFLUSHTX__strobe();
        CC2420TransmitP__CaptureSFD__captureRisingEdge();
        CC2420TransmitP__releaseSpiResource();
        CC2420TransmitP__signalDone(ERETRY);
        break;

        default: 
          break;
      }
  }
}

# 67 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 67
  CC2420TransmitP__BackoffTimer__fired();
#line 67
  CC2420ControlP__StartupTimer__fired();
#line 67
}
#line 67
# 151 "/opt/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt == 0) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__fired();
      }
    else 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm();
      }
  }
}

# 67 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 67
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 47 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 47
}
#line 47
# 59 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 34
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
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
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4419 {
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

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 38 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 53 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
inline static /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 53
  unsigned int __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 70 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 35 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending();
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 43 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 60 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
inline static bool /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 119 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 46 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 46
}
#line 46
# 84 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 33 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 30 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 154 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 32 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 70 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 92 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 92
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 246 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_leuint8(const void * source)
#line 246
{
  const uint8_t *base = source;

#line 248
  return base[0];
}

# 284 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__RadioBackoff__default__requestCongestionBackoff(am_id_t id, 
message_t *msg)
#line 285
{
}

# 88 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420ActiveMessageP__RadioBackoff__requestCongestionBackoff(am_id_t arg_0x2b99bddf1d60, message_t * msg){
#line 88
    CC2420ActiveMessageP__RadioBackoff__default__requestCongestionBackoff(arg_0x2b99bddf1d60, msg);
#line 88
}
#line 88
# 233 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SubBackoff__requestCongestionBackoff(message_t *msg)
#line 233
{
  CC2420ActiveMessageP__RadioBackoff__requestCongestionBackoff(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.data), msg);
}

# 88 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__RadioBackoff__requestCongestionBackoff(message_t * msg){
#line 88
  CC2420ActiveMessageP__SubBackoff__requestCongestionBackoff(msg);
#line 88
}
#line 88
# 78 "/opt/tinyos-2.1.1/tos/system/RandomMlcgC.nc"
static inline uint16_t RandomMlcgC__Random__rand16(void )
#line 78
{
  return (uint16_t )RandomMlcgC__Random__rand32();
}

# 41 "/opt/tinyos-2.1.1/tos/interfaces/Random.nc"
inline static uint16_t CC2420CsmaP__Random__rand16(void ){
#line 41
  unsigned int __nesc_result;
#line 41

#line 41
  __nesc_result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 251 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__RadioBackoff__setCongestionBackoff(uint16_t backoffTime)
#line 251
{
  CC2420TransmitP__myCongestionBackoff = backoffTime + 1;
}

# 66 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__SubBackoff__setCongestionBackoff(uint16_t backoffTime){
#line 66
  CC2420TransmitP__RadioBackoff__setCongestionBackoff(backoffTime);
#line 66
}
#line 66
# 227 "/opt/tinyos-2.1.1/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__SubBackoff__requestCongestionBackoff(message_t *msg)
#line 227
{
  CC2420CsmaP__SubBackoff__setCongestionBackoff(CC2420CsmaP__Random__rand16()
   % (0x7 * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP__RadioBackoff__requestCongestionBackoff(msg);
}

# 88 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420TransmitP__RadioBackoff__requestCongestionBackoff(message_t * msg){
#line 88
  CC2420CsmaP__SubBackoff__requestCongestionBackoff(msg);
#line 88
}
#line 88
# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 45 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static error_t CC2420SpiP__WorkingState__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(5U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 143 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__isOwner(uint8_t id)
#line 143
{
#line 143
  return 0x0080;
}

# 118 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static bool /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__isOwner(uint8_t arg_0x2b99bd7e6108){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  switch (arg_0x2b99bd7e6108) {
#line 118
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID:
#line 118
      __nesc_result = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsciC*/Msp430UsciB0C__0__CLIENT_ID);
#line 118
      break;
#line 118
    default:
#line 118
      __nesc_result = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__isOwner(arg_0x2b99bd7e6108);
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
# 109 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline uint8_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__isOwner(uint8_t id)
#line 109
{
  return /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__isOwner(id);
}

# 118 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static bool CC2420SpiP__SpiResource__isOwner(void ){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  __nesc_result = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID);
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 18 "/opt/tinyos-2.1.1/tos/platforms/z1/chips/msp430X/usci/Z1UsciP.nc"
static inline msp430_spi_union_config_t */*Msp430SpiNoDmaB0P.Z1UsciP*/Z1UsciP__0__Msp430SpiConfigure__getConfig(uint8_t id)
#line 18
{
  return (msp430_spi_union_config_t *)&/*Msp430SpiNoDmaB0P.Z1UsciP*/Z1UsciP__0__msp430_spi_z1_config;
}

# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiConfigure.nc"
inline static msp430_spi_union_config_t */*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Msp430SpiConfigure__getConfig(uint8_t arg_0x2b99bd7e42f0){
#line 71
  union __nesc_unnamed4282 *__nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430SpiNoDmaB0P.Z1UsciP*/Z1UsciP__0__Msp430SpiConfigure__getConfig(arg_0x2b99bd7e42f0);
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 148 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB.nc"
inline static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__setModeSpi(msp430_spi_union_config_t *config){
#line 148
  HplMsp430UsciB0P__Usci__setModeSpi(config);
#line 148
}
#line 148
# 117 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__ResourceConfigure__configure(uint8_t id)
#line 117
{
  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__setModeSpi(/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Msp430SpiConfigure__getConfig(id));
}

# 213 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id)
#line 213
{
}

# 49 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(uint8_t arg_0x2b99bd1be340){
#line 49
  switch (arg_0x2b99bd1be340) {
#line 49
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsciC*/Msp430UsciB0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__ResourceConfigure__configure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(arg_0x2b99bd1be340);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 210 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested(void )
#line 210
{
  /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release();
}

# 81 "/opt/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__immediateRequested();
#line 81
}
#line 81
# 203 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(uint8_t id)
#line 203
{
}

# 51 "/opt/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(uint8_t arg_0x2b99bd1c0020){
#line 51
    /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(arg_0x2b99bd1c0020);
#line 51
}
#line 51
# 90 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(uint8_t id)
#line 90
{
  /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /* atomic removed: atomic calls only */
#line 92
  {
    if (/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) {
        /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_IMM_GRANTING;
        /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 97
        0x0080;

#line 97
        return __nesc_temp;
      }
  }
#line 99
  /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__resId == id) {
      /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
      return SUCCESS;
    }
  /* atomic removed: atomic calls only */
#line 104
  /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
  return 0x0080;
}

# 145 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__immediateRequest(uint8_t id)
#line 145
{
#line 145
  return 0x0080;
}

# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__immediateRequest(uint8_t arg_0x2b99bd7e6108){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  switch (arg_0x2b99bd7e6108) {
#line 87
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID:
#line 87
      __nesc_result = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsciC*/Msp430UsciB0C__0__CLIENT_ID);
#line 87
      break;
#line 87
    default:
#line 87
      __nesc_result = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__immediateRequest(arg_0x2b99bd7e6108);
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
# 101 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__immediateRequest(uint8_t id)
#line 101
{
  return /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__immediateRequest(id);
}

# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 231 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__Usci__disableIntr(void )
#line 231
{
  HplMsp430UsciB0P__IE2 &= ~((1 << 3) | (1 << 2));
}

#line 219
static inline void HplMsp430UsciB0P__Usci__clrIntr(void )
#line 219
{
  HplMsp430UsciB0P__IFG2 &= ~((1 << 3) | (1 << 2));
}

# 54 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 3;
}

# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciB0P__UCLK__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc();
#line 78
}
#line 78
# 54 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 2;
}

# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciB0P__SOMI__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc();
#line 78
}
#line 78
# 54 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 1;
}

# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciB0P__SIMO__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc();
#line 78
}
#line 78
# 166 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__Usci__enableSpi(void )
#line 166
{
  /* atomic removed: atomic calls only */
#line 167
  {
    HplMsp430UsciB0P__SIMO__selectModuleFunc();
    HplMsp430UsciB0P__SOMI__selectModuleFunc();
    HplMsp430UsciB0P__UCLK__selectModuleFunc();
  }
}

#line 113
static inline void HplMsp430UsciB0P__Usci__setUbr(uint16_t control)
#line 113
{
  /* atomic removed: atomic calls only */
#line 114
  {
    UCB0BR0 = control & 0x00FF;
    UCB0BR1 = (control >> 8) & 0x00FF;
  }
}

#line 182
static inline void HplMsp430UsciB0P__configSpi(msp430_spi_union_config_t *config)
#line 182
{
  HplMsp430UsciB0P__UCB0CTL1 = config->spiRegisters.uctl1 | 0x01;
  HplMsp430UsciB0P__UCB0CTL0 = config->spiRegisters.uctl0 | 0x01;
  HplMsp430UsciB0P__Usci__setUbr(config->spiRegisters.ubr);
}

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
inline static void CC2420SpiP__WorkingState__toIdle(void ){
#line 56
  StateImplP__State__toIdle(5U);
#line 56
}
#line 56
# 78 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 207 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested(void )
#line 207
{
  /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release();
}

# 73 "/opt/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void ){
#line 73
  /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__requested();
#line 73
}
#line 73
# 54 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 54
{
  /* atomic removed: atomic calls only */
#line 55
  {
    unsigned char __nesc_temp = 
#line 55
    /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ[id] != /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY || /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qTail == id;

#line 55
    return __nesc_temp;
  }
}

#line 72
static inline error_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id)
#line 72
{
  /* atomic removed: atomic calls only */
#line 73
  {
    if (!/*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(id)) {
        if (/*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead == /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
          /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead = id;
          }
        else {
#line 78
          /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ[/*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qTail] = id;
          }
#line 79
        /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qTail = id;
        {
          unsigned char __nesc_temp = 
#line 80
          SUCCESS;

#line 80
          return __nesc_temp;
        }
      }
#line 82
    {
      unsigned char __nesc_temp = 
#line 82
      EBUSY;

#line 82
      return __nesc_temp;
    }
  }
}

# 69 "/opt/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(id);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 201 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id)
#line 201
{
}

# 43 "/opt/tinyos-2.1.1/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(uint8_t arg_0x2b99bd1c0020){
#line 43
    /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(arg_0x2b99bd1c0020);
#line 43
}
#line 43
# 77 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(uint8_t id)
#line 77
{
  /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__requested(/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /* atomic removed: atomic calls only */
#line 79
  {
    if (/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) {
        /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING;
        /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 84
        /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__enqueue(id);

#line 84
        return __nesc_temp;
      }
  }
#line 86
  /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 144 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__request(uint8_t id)
#line 144
{
#line 144
  return 0x0080;
}

# 78 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__request(uint8_t arg_0x2b99bd7e6108){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  switch (arg_0x2b99bd7e6108) {
#line 78
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID:
#line 78
      __nesc_result = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsciC*/Msp430UsciB0C__0__CLIENT_ID);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__request(arg_0x2b99bd7e6108);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 105 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__request(uint8_t id)
#line 105
{
  return /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__request(id);
}

# 78 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 256 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__Usci__tx(uint8_t data)
#line 256
{
  HplMsp430UsciB0P__UCB0TXBUF = data;
}

# 115 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB.nc"
inline static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__tx(uint8_t data){
#line 115
  HplMsp430UsciB0P__Usci__tx(data);
#line 115
}
#line 115
# 205 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB0P.nc"
static inline bool HplMsp430UsciB0P__Usci__isRxIntrPending(void )
#line 205
{
  if (HplMsp430UsciB0P__IFG2 & (1 << 2)) {
    return TRUE;
    }
#line 208
  return FALSE;
}

# 105 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB.nc"
inline static bool /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__isRxIntrPending(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = HplMsp430UsciB0P__Usci__isRxIntrPending();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 215 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__Usci__clrRxIntr(void )
#line 215
{
  HplMsp430UsciB0P__IFG2 &= ~(1 << 2);
}

# 106 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB.nc"
inline static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__clrRxIntr(void ){
#line 106
  HplMsp430UsciB0P__Usci__clrRxIntr();
#line 106
}
#line 106
# 260 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB0P.nc"
static inline uint8_t HplMsp430UsciB0P__Usci__rx(void )
#line 260
{
  return HplMsp430UsciB0P__UCB0RXBUF;
}

# 122 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB.nc"
inline static uint8_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__rx(void ){
#line 122
  unsigned char __nesc_result;
#line 122

#line 122
  __nesc_result = HplMsp430UsciB0P__Usci__rx();
#line 122

#line 122
  return __nesc_result;
#line 122
}
#line 122
# 95 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__ChipSpiResource__abortRelease(void )
#line 95
{
  /* atomic removed: atomic calls only */
#line 96
  CC2420SpiP__release = FALSE;
}

# 31 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static void CC2420TransmitP__ChipSpiResource__abortRelease(void ){
#line 31
  CC2420SpiP__ChipSpiResource__abortRelease();
#line 31
}
#line 31
# 375 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__ChipSpiResource__releasing(void )
#line 375
{
  if (CC2420TransmitP__abortSpiRelease) {
      CC2420TransmitP__ChipSpiResource__abortRelease();
    }
}

# 24 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static void CC2420SpiP__ChipSpiResource__releasing(void ){
#line 24
  CC2420TransmitP__ChipSpiResource__releasing();
#line 24
}
#line 24
# 205 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted(void )
#line 205
{
}

# 46 "/opt/tinyos-2.1.1/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__default__granted();
#line 46
}
#line 46
# 133 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__Usci__resetUsci(bool reset)
#line 133
{
  if (reset) {
    HplMsp430UsciB0P__UCB0CTL1 |= 0x01;
    }
  else {
#line 137
    HplMsp430UsciB0P__UCB0CTL1 &= ~0x01;
    }
}

# 88 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB.nc"
inline static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__resetUsci(bool reset){
#line 88
  HplMsp430UsciB0P__Usci__resetUsci(reset);
#line 88
}
#line 88
# 56 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 3);
}

# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciB0P__UCLK__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 2);
}

# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciB0P__SOMI__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 1);
}

# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciB0P__SIMO__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc();
#line 85
}
#line 85
# 174 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__Usci__disableSpi(void )
#line 174
{
  /* atomic removed: atomic calls only */
#line 175
  {
    HplMsp430UsciB0P__SIMO__selectIOFunc();
    HplMsp430UsciB0P__SOMI__selectIOFunc();
    HplMsp430UsciB0P__UCLK__selectIOFunc();
  }
}

# 137 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB.nc"
inline static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__disableSpi(void ){
#line 137
  HplMsp430UsciB0P__Usci__disableSpi();
#line 137
}
#line 137
# 121 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 121
{
  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__resetUsci(TRUE);
  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__disableSpi();
  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__resetUsci(FALSE);
}

# 215 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id)
#line 215
{
}

# 55 "/opt/tinyos-2.1.1/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(uint8_t arg_0x2b99bd1be340){
#line 55
  switch (arg_0x2b99bd1be340) {
#line 55
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsciC*/Msp430UsciB0C__0__CLIENT_ID:
#line 55
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__ResourceConfigure__unconfigure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID);
#line 55
      break;
#line 55
    default:
#line 55
      /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(arg_0x2b99bd1be340);
#line 55
      break;
#line 55
    }
#line 55
}
#line 55
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 58 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead != /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
        uint8_t id = /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead;

#line 62
        /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead = /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ[/*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead];
        if (/*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead == /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
          /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qTail = /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
          }
#line 65
        /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ[id] = /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
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
      /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/opt/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 50 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 51
  {
    unsigned char __nesc_temp = 
#line 51
    /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__qHead == /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

#line 51
    return __nesc_temp;
  }
}

# 43 "/opt/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 108 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(uint8_t id)
#line 108
{
  /* atomic removed: atomic calls only */
#line 109
  {
    if (/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY && /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__resId == id) {
        if (/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty() == FALSE) {
            /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue();
            /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__NO_RES;
            /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING;
            /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask();
            /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
          }
        else {
            /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id;
            /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
            /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
            /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted();
          }
        {
          unsigned char __nesc_temp = 
#line 124
          SUCCESS;

#line 124
          return __nesc_temp;
        }
      }
  }
#line 127
  return 0x0080;
}

# 146 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__release(uint8_t id)
#line 146
{
#line 146
  return 0x0080;
}

# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__release(uint8_t arg_0x2b99bd7e6108){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0x2b99bd7e6108) {
#line 110
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID:
#line 110
      __nesc_result = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsciC*/Msp430UsciB0C__0__CLIENT_ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__default__release(arg_0x2b99bd7e6108);
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
# 113 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__release(uint8_t id)
#line 113
{
  return /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__release(id);
}

# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP__STXONCCA__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_STXONCCA);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
inline static cc2420_status_t CC2420TransmitP__STXON__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_STXON);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
inline static cc2420_status_t CC2420TransmitP__SNOP__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SNOP);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 102 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP__ChipSpiResource__attemptRelease(void )
#line 102
{
  return CC2420SpiP__attemptRelease();
}

# 39 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static error_t CC2420TransmitP__ChipSpiResource__attemptRelease(void ){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = CC2420SpiP__ChipSpiResource__attemptRelease();
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 54 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )31U |= 0x01 << 1;
}

# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc();
#line 78
}
#line 78
# 46 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4420 {
#line 46
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 61
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(uint8_t l_cm)
{
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x = { 
  .cm = l_cm & 0x03, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 1, 
  .scs = 1, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(x);
}

#line 99
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm)
{
  * (volatile uint16_t * )388U = /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(cm);
}

# 44 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm){
#line 44
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(cm);
#line 44
}
#line 44
# 119 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void )
{
  * (volatile uint16_t * )388U |= 0x0010;
}

# 46 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents();
#line 46
}
#line 46
# 181 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
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

# 276 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_leuint16(const void * source)
#line 276
{
  const uint8_t *base = source;

#line 278
  return ((uint16_t )base[1] << 8) | base[0];
}

#line 301
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

#line 294
static __inline  uint32_t __nesc_ntoh_uint32(const void * source)
#line 294
{
  const uint8_t *base = source;

#line 296
  return ((((uint32_t )base[0] << 24) | (
  (uint32_t )base[1] << 16)) | (
  (uint32_t )base[2] << 8)) | base[3];
}

# 59 "/opt/tinyos-2.1.1/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420TransmitP__PacketTimeStamp__clear(message_t * msg){
#line 59
  CC2420PacketP__PacketTimeStamp32khz__clear(msg);
#line 59
}
#line 59
# 195 "/opt/tinyos-2.1.1/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Receive__sfd_dropped(void )
#line 195
{
  if (CC2420ReceiveP__m_timestamp_size) {
      CC2420ReceiveP__m_timestamp_size--;
    }
}

# 55 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420TransmitP__CC2420Receive__sfd_dropped(void ){
#line 55
  CC2420ReceiveP__CC2420Receive__sfd_dropped();
#line 55
}
#line 55
# 48 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )28U & (0x01 << 1);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw() != 0;
}

# 59 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__get(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__get();
}

# 32 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static bool CC2420TransmitP__SFD__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 186 "/opt/tinyos-2.1.1/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time)
#line 186
{
  if (CC2420ReceiveP__m_timestamp_size < CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE) {
      uint8_t tail = (CC2420ReceiveP__m_timestamp_head + CC2420ReceiveP__m_timestamp_size) % 
      CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE;

#line 190
      CC2420ReceiveP__m_timestamp_queue[tail] = time;
      CC2420ReceiveP__m_timestamp_size++;
    }
}

# 49 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420TransmitP__CC2420Receive__sfd(uint32_t time){
#line 49
  CC2420ReceiveP__CC2420Receive__sfd(time);
#line 49
}
#line 49
# 54 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void )
#line 54
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_FALLING);
}

# 43 "/opt/tinyos-2.1.1/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP__CaptureSFD__captureFallingEdge(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 114 "/opt/tinyos-2.1.1/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg)
#line 114
{
  return (cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
}

# 42 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420TransmitP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 54 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 62 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 62
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 62
}
#line 62
# 91 "/opt/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 62 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP__BackoffTimer__stop(void ){
#line 62
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 62
}
#line 62
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__set();
}

# 29 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__set(void ){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 29
}
#line 29
# 63 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420TransmitP__TXFIFO_RAM__write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Ram__write(CC2420_RAM_TXFIFO, offset, data, length);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 39 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr();
}

# 30 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__clr(void ){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 30
}
#line 30
# 185 "/opt/tinyos-2.1.1/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline uint8_t CC2420PacketP__PacketTimeSyncOffset__get(message_t *msg)
{
  return __nesc_ntoh_leuint8(CC2420PacketP__CC2420PacketBody__getHeader(msg)->length.data)
   + (sizeof(cc2420_header_t ) - MAC_HEADER_SIZE)
   - MAC_FOOTER_SIZE
   - sizeof(timesync_radio_t );
}

# 47 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
inline static uint8_t CC2420TransmitP__PacketTimeSyncOffset__get(message_t * msg){
#line 47
  unsigned char __nesc_result;
#line 47

#line 47
  __nesc_result = CC2420PacketP__PacketTimeSyncOffset__get(msg);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 235 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 235
{
  const uint8_t *base = source;

#line 237
  return base[0];
}

#line 257
static __inline  int8_t __nesc_ntoh_int8(const void * source)
#line 257
{
#line 257
  return __nesc_ntoh_uint8(source);
}

# 118 "/opt/tinyos-2.1.1/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline cc2420_metadata_t *CC2420PacketP__CC2420PacketBody__getMetadata(message_t *msg)
#line 118
{
  return (cc2420_metadata_t *)msg->metadata;
}

#line 176
static inline bool CC2420PacketP__PacketTimeSyncOffset__isSet(message_t *msg)
{
  return __nesc_ntoh_int8(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timesync.data);
}

# 39 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
inline static bool CC2420TransmitP__PacketTimeSyncOffset__isSet(message_t * msg){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = CC2420PacketP__PacketTimeSyncOffset__isSet(msg);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 143 "/opt/tinyos-2.1.1/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline void CC2420PacketP__PacketTimeStamp32khz__set(message_t *msg, uint32_t value)
{
  __nesc_hton_uint32(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timestamp.data, value);
}

# 67 "/opt/tinyos-2.1.1/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420TransmitP__PacketTimeStamp__set(message_t * msg, CC2420TransmitP__PacketTimeStamp__size_type value){
#line 67
  CC2420PacketP__PacketTimeStamp32khz__set(msg, value);
#line 67
}
#line 67
# 98 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static CC2420TransmitP__BackoffTimer__size_type CC2420TransmitP__BackoffTimer__getNow(void ){
#line 98
  unsigned long __nesc_result;
#line 98

#line 98
  __nesc_result = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 259 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static __inline uint32_t CC2420TransmitP__getTime32(uint16_t time)
{
  uint32_t recent_time = CC2420TransmitP__BackoffTimer__getNow();

#line 262
  return recent_time + (int16_t )(time - recent_time);
}

#line 278
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time)
#line 278
{
  unsigned char *__nesc_temp49;
  unsigned char *__nesc_temp48;
#line 279
  uint32_t time32;
  uint8_t sfd_state = 0;

  /* atomic removed: atomic calls only */
#line 281
  {
    time32 = CC2420TransmitP__getTime32(time);
    switch (CC2420TransmitP__m_state) {

        case CC2420TransmitP__S_SFD: 
          CC2420TransmitP__m_state = CC2420TransmitP__S_EFD;
        CC2420TransmitP__sfdHigh = TRUE;


        CC2420TransmitP__m_receiving = FALSE;
        CC2420TransmitP__CaptureSFD__captureFallingEdge();
        CC2420TransmitP__PacketTimeStamp__set(CC2420TransmitP__m_msg, time32);
        if (CC2420TransmitP__PacketTimeSyncOffset__isSet(CC2420TransmitP__m_msg)) {
            uint8_t absOffset = sizeof(message_header_t ) - sizeof(cc2420_header_t ) + CC2420TransmitP__PacketTimeSyncOffset__get(CC2420TransmitP__m_msg);
            timesync_radio_t *timesync = (timesync_radio_t *)((nx_uint8_t *)CC2420TransmitP__m_msg + absOffset);

            (__nesc_temp48 = (*timesync).data, __nesc_hton_uint32(__nesc_temp48, __nesc_ntoh_uint32(__nesc_temp48) - time32));
            CC2420TransmitP__CSN__clr();
            CC2420TransmitP__TXFIFO_RAM__write(absOffset, (uint8_t *)timesync, sizeof(timesync_radio_t ));
            CC2420TransmitP__CSN__set();

            (__nesc_temp49 = (*timesync).data, __nesc_hton_uint32(__nesc_temp49, __nesc_ntoh_uint32(__nesc_temp49) + time32));
          }

        if (__nesc_ntoh_leuint16(CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg)->fcf.data) & (1 << IEEE154_FCF_ACK_REQ)) {

            CC2420TransmitP__abortSpiRelease = TRUE;
          }
        CC2420TransmitP__releaseSpiResource();
        CC2420TransmitP__BackoffTimer__stop();

        if (CC2420TransmitP__SFD__get()) {
            break;
          }


        case CC2420TransmitP__S_EFD: 
          CC2420TransmitP__sfdHigh = FALSE;
        CC2420TransmitP__CaptureSFD__captureRisingEdge();

        if (__nesc_ntoh_leuint16(CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg)->fcf.data) & (1 << IEEE154_FCF_ACK_REQ)) {
            CC2420TransmitP__m_state = CC2420TransmitP__S_ACK_WAIT;
            CC2420TransmitP__BackoffTimer__start(CC2420_ACK_WAIT_DELAY);
          }
        else 
#line 324
          {
            CC2420TransmitP__signalDone(SUCCESS);
          }

        if (!CC2420TransmitP__SFD__get()) {
            break;
          }


        default: 

          if (!CC2420TransmitP__m_receiving && CC2420TransmitP__sfdHigh == FALSE) {
              CC2420TransmitP__sfdHigh = TRUE;
              CC2420TransmitP__CaptureSFD__captureFallingEdge();

              sfd_state = CC2420TransmitP__SFD__get();
              CC2420TransmitP__CC2420Receive__sfd(time32);
              CC2420TransmitP__m_receiving = TRUE;
              CC2420TransmitP__m_prev_time = time;
              if (CC2420TransmitP__SFD__get()) {

                  return;
                }
            }



        if (CC2420TransmitP__sfdHigh == TRUE) {
            CC2420TransmitP__sfdHigh = FALSE;
            CC2420TransmitP__CaptureSFD__captureRisingEdge();
            CC2420TransmitP__m_receiving = FALSE;








            if (sfd_state == 0 && time - CC2420TransmitP__m_prev_time < 10) {
                CC2420TransmitP__CC2420Receive__sfd_dropped();
                if (CC2420TransmitP__m_msg) {
                  CC2420TransmitP__PacketTimeStamp__clear(CC2420TransmitP__m_msg);
                  }
              }
#line 368
            break;
          }
      }
  }
}

# 50 "/opt/tinyos-2.1.1/tos/interfaces/GpioCapture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(uint16_t time){
#line 50
  CC2420TransmitP__CaptureSFD__captured(time);
#line 50
}
#line 50
# 164 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void )
{
  * (volatile uint16_t * )388U &= ~0x0002;
}

# 57 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow();
#line 57
}
#line 57
# 84 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )388U &= ~0x0001;
}

# 33 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 65 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/GpioCaptureC.nc"
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time)
#line 65
{
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(time);
}

# 75 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 75
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4421 {
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

# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 70 "/opt/tinyos-2.1.1/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 71
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 67 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 67
}
#line 67
# 151 "/opt/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm();
      }
  }
}

# 67 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void )
{
  * (volatile uint16_t * )390U &= ~0x0010;
}

# 47 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents();
#line 47
}
#line 47
# 59 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired();
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 34
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired();
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
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4422 {
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

# 53 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get(void ){
#line 53
  unsigned int __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53







inline static bool /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 119 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void )
{
  * (volatile uint16_t * )390U |= 0x0010;
}

# 46 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents();
#line 46
}
#line 46
# 84 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )390U &= ~0x0001;
}

# 33 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )406U = x;
}

# 30 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 154 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )406U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get() + x;
}

# 32 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 70 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents();
  }
}

# 92 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 181 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
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
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4423 {
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
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4424 {
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
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4425 {
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
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4426 {
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
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 268 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__UnicastNameFreeRouting__routeFound(void )
#line 268
{
  ;
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask__postTask();
}

# 51 "/opt/tinyos-2.1.1/tos/lib/net/UnicastNameFreeRouting.nc"
inline static void /*CtpP.Router*/CtpRoutingEngineP__0__Routing__routeFound(void ){
#line 51
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__UnicastNameFreeRouting__routeFound();
#line 51
}
#line 51
# 273 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__UnicastNameFreeRouting__noRoute(void )
#line 273
{
}

# 52 "/opt/tinyos-2.1.1/tos/lib/net/UnicastNameFreeRouting.nc"
inline static void /*CtpP.Router*/CtpRoutingEngineP__0__Routing__noRoute(void ){
#line 52
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__UnicastNameFreeRouting__noRoute();
#line 52
}
#line 52
# 173 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__resetInterval(void )
#line 173
{
  /*CtpP.Router*/CtpRoutingEngineP__0__currentInterval = 128;
  /*CtpP.Router*/CtpRoutingEngineP__0__chooseAdvertiseTime();
}

#line 548
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__triggerRouteUpdate(void )
#line 548
{
  /*CtpP.Router*/CtpRoutingEngineP__0__resetInterval();
}

# 540 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline error_t LinkEstimatorP__LinkEstimator__clearDLQ(am_addr_t neighbor)
#line 540
{
  neighbor_table_entry_t *ne;
  uint8_t nidx = LinkEstimatorP__findIdx(neighbor);

#line 543
  if (nidx == LinkEstimatorP__INVALID_RVAL) {
      return 0x0080;
    }
  ne = &LinkEstimatorP__NeighborTable[nidx];
  ne->data_total = 0;
  ne->data_success = 0;
  return SUCCESS;
}

# 58 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimator.nc"
inline static error_t /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__clearDLQ(am_addr_t neighbor){
#line 58
  unsigned char __nesc_result;
#line 58

#line 58
  __nesc_result = LinkEstimatorP__LinkEstimator__clearDLQ(neighbor);
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
#line 44
inline static error_t /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__pinNeighbor(am_addr_t neighbor){
#line 44
  unsigned char __nesc_result;
#line 44

#line 44
  __nesc_result = LinkEstimatorP__LinkEstimator__pinNeighbor(neighbor);
#line 44

#line 44
  return __nesc_result;
#line 44
}
#line 44
# 495 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline error_t LinkEstimatorP__LinkEstimator__unpinNeighbor(am_addr_t neighbor)
#line 495
{
  uint8_t nidx = LinkEstimatorP__findIdx(neighbor);

#line 497
  if (nidx == LinkEstimatorP__INVALID_RVAL) {
      return 0x0080;
    }
  LinkEstimatorP__NeighborTable[nidx].flags &= ~PINNED_ENTRY;
  return SUCCESS;
}

# 47 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimator.nc"
inline static error_t /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__unpinNeighbor(am_addr_t neighbor){
#line 47
  unsigned char __nesc_result;
#line 47

#line 47
  __nesc_result = LinkEstimatorP__LinkEstimator__unpinNeighbor(neighbor);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 765 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline error_t /*CtpP.Router*/CtpRoutingEngineP__0__CollectionDebug__default__logEventDbg(uint8_t type, uint16_t arg1, uint16_t arg2, uint16_t arg3)
#line 765
{
  return SUCCESS;
}

# 56 "/opt/tinyos-2.1.1/tos/lib/net/CollectionDebug.nc"
inline static error_t /*CtpP.Router*/CtpRoutingEngineP__0__CollectionDebug__logEventDbg(uint8_t type, uint16_t arg1, uint16_t arg2, uint16_t arg3){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = /*CtpP.Router*/CtpRoutingEngineP__0__CollectionDebug__default__logEventDbg(type, arg1, arg2, arg3);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 244 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline bool /*CtpP.Router*/CtpRoutingEngineP__0__passLinkEtxThreshold(uint16_t etx)
#line 244
{
  return etx < ETX_THRESHOLD;
}

# 38 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimator.nc"
inline static uint16_t /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__getLinkQuality(uint16_t neighbor){
#line 38
  unsigned int __nesc_result;
#line 38

#line 38
  __nesc_result = LinkEstimatorP__LinkEstimator__getLinkQuality(neighbor);
#line 38

#line 38
  return __nesc_result;
#line 38
}
#line 38
# 252 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__updateRouteTask__runTask(void )
#line 252
{
  uint8_t i;
  routing_table_entry *entry;
  routing_table_entry *best;
  uint16_t minEtx;
  uint16_t currentEtx;
  uint16_t linkEtx;
#line 258
  uint16_t pathEtx;

  if (/*CtpP.Router*/CtpRoutingEngineP__0__state_is_root) {
    return;
    }
  best = (void *)0;

  minEtx = MAX_METRIC;

  currentEtx = MAX_METRIC;

  ;


  for (i = 0; i < /*CtpP.Router*/CtpRoutingEngineP__0__routingTableActive; i++) {
      entry = &/*CtpP.Router*/CtpRoutingEngineP__0__routingTable[i];


      if (entry->info.parent == INVALID_ADDR || entry->info.parent == /*CtpP.Router*/CtpRoutingEngineP__0__my_ll_addr) {
          ;


          continue;
        }

      linkEtx = /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__getLinkQuality(entry->neighbor);
      ;


      pathEtx = linkEtx + entry->info.etx;

      if (entry->neighbor == /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent) {
          ;
          currentEtx = pathEtx;

          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 293
            {
              /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.etx = entry->info.etx;
              /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.congested = entry->info.congested;
            }
#line 296
            __nesc_atomic_end(__nesc_atomic); }
          continue;
        }

      if (entry->info.congested) {
        continue;
        }
      if (!/*CtpP.Router*/CtpRoutingEngineP__0__passLinkEtxThreshold(linkEtx)) {
          ;
          continue;
        }

      if (pathEtx < minEtx) {
          ;
          minEtx = pathEtx;
          best = entry;
        }
    }
#line 328
  if (minEtx != MAX_METRIC) {

      if ((
#line 329
      currentEtx == MAX_METRIC || (
      /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.congested && minEtx < /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.etx + 10)) || 
      minEtx + PARENT_SWITCH_THRESHOLD < currentEtx) {




          /*CtpP.Router*/CtpRoutingEngineP__0__parentChanges++;

          ;
          /*CtpP.Router*/CtpRoutingEngineP__0__CollectionDebug__logEventDbg(NET_C_TREE_NEW_PARENT, best->neighbor, best->info.etx, minEtx);
          /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__unpinNeighbor(/*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent);
          /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__pinNeighbor(best->neighbor);
          /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__clearDLQ(best->neighbor);

          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 344
            {
              /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent = best->neighbor;
              /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.etx = best->info.etx;
              /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.congested = best->info.congested;
            }
#line 348
            __nesc_atomic_end(__nesc_atomic); }
          if (currentEtx - minEtx > 20) {
              /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__triggerRouteUpdate();
            }
        }
    }




  if (/*CtpP.Router*/CtpRoutingEngineP__0__justEvicted && /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent == INVALID_ADDR) {
    /*CtpP.Router*/CtpRoutingEngineP__0__Routing__noRoute();
    }
  else {



    if (
#line 364
    !/*CtpP.Router*/CtpRoutingEngineP__0__justEvicted && 
    currentEtx == MAX_METRIC && 
    minEtx != MAX_METRIC) {
      /*CtpP.Router*/CtpRoutingEngineP__0__Routing__routeFound();
      }
    }
#line 368
  /*CtpP.Router*/CtpRoutingEngineP__0__justEvicted = FALSE;
}

# 35 "/opt/tinyos-2.1.1/tos/interfaces/Random.nc"
inline static uint32_t /*CtpP.Router*/CtpRoutingEngineP__0__Random__rand32(void ){
#line 35
  unsigned long __nesc_result;
#line 35

#line 35
  __nesc_result = RandomMlcgC__Random__rand32();
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 153 "/opt/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning = FALSE;
}

# 67 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void /*CtpP.Router*/CtpRoutingEngineP__0__BeaconTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(1U);
#line 67
}
#line 67
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 264 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_uint16(const void * source)
#line 264
{
  const uint8_t *base = source;

#line 266
  return ((uint16_t )base[0] << 8) | base[1];
}

static __inline  uint16_t __nesc_hton_uint16(void * target, uint16_t value)
#line 269
{
  uint8_t *base = target;

#line 271
  base[1] = value;
  base[0] = value >> 8;
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

# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static error_t /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__Send__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__send(1U, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 151 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static void /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  CC2420ActiveMessageP__AMPacket__setType(amsg, t);
#line 151
}
#line 151
# 281 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_leuint16(void * target, uint16_t value)
#line 281
{
  uint8_t *base = target;

#line 283
  base[0] = value;
  base[1] = value >> 8;
  return value;
}

# 42 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420ActiveMessageP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 136 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__AMPacket__setDestination(message_t *amsg, am_addr_t addr)
#line 136
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 138
  __nesc_hton_leuint16(header->dest.data, addr);
}

# 92 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static void /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  CC2420ActiveMessageP__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
# 45 "/opt/tinyos-2.1.1/tos/system/AMQueueEntryP.nc"
static inline error_t /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 47
{
  /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMPacket__setDestination(msg, dest);
  /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMPacket__setType(msg, 112);
  return /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__Send__send(msg, len);
}

# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static error_t LinkEstimatorP__AMSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMSend__send(addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 170 "/opt/tinyos-2.1.1/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline uint8_t CC2420CsmaP__Send__maxPayloadLength(void )
#line 170
{
  return 28;
}

# 101 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static uint8_t UniqueSendP__SubSend__maxPayloadLength(void ){
#line 101
  unsigned char __nesc_result;
#line 101

#line 101
  __nesc_result = CC2420CsmaP__Send__maxPayloadLength();
#line 101

#line 101
  return __nesc_result;
#line 101
}
#line 101
# 95 "/opt/tinyos-2.1.1/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline uint8_t UniqueSendP__Send__maxPayloadLength(void )
#line 95
{
  return UniqueSendP__SubSend__maxPayloadLength();
}

# 101 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static uint8_t CC2420TinyosNetworkP__SubSend__maxPayloadLength(void ){
#line 101
  unsigned char __nesc_result;
#line 101

#line 101
  __nesc_result = UniqueSendP__Send__maxPayloadLength();
#line 101

#line 101
  return __nesc_result;
#line 101
}
#line 101
# 82 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline uint8_t CC2420TinyosNetworkP__ActiveSend__maxPayloadLength(void )
#line 82
{
  return CC2420TinyosNetworkP__SubSend__maxPayloadLength();
}

# 101 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static uint8_t CC2420ActiveMessageP__SubSend__maxPayloadLength(void ){
#line 101
  unsigned char __nesc_result;
#line 101

#line 101
  __nesc_result = CC2420TinyosNetworkP__ActiveSend__maxPayloadLength();
#line 101

#line 101
  return __nesc_result;
#line 101
}
#line 101
# 189 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline uint8_t CC2420ActiveMessageP__Packet__maxPayloadLength(void )
#line 189
{
  return CC2420ActiveMessageP__SubSend__maxPayloadLength();
}

# 95 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
inline static uint8_t LinkEstimatorP__SubPacket__maxPayloadLength(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = CC2420ActiveMessageP__Packet__maxPayloadLength();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 101 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline linkest_footer_t *LinkEstimatorP__getFooter(message_t *m, uint8_t len)
#line 101
{

  return (linkest_footer_t *)(len + (uint8_t *)LinkEstimatorP__Packet__getPayload(m, len + sizeof(linkest_footer_t )));
}

# 86 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void *CC2420TinyosNetworkP__ActiveSend__getPayload(message_t *msg, uint8_t len)
#line 86
{
  if (len <= CC2420TinyosNetworkP__ActiveSend__maxPayloadLength()) {
      return msg->data;
    }
  else 
#line 89
    {
      return (void *)0;
    }
}

# 114 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static void * CC2420ActiveMessageP__SubSend__getPayload(message_t * msg, uint8_t len){
#line 114
  void *__nesc_result;
#line 114

#line 114
  __nesc_result = CC2420TinyosNetworkP__ActiveSend__getPayload(msg, len);
#line 114

#line 114
  return __nesc_result;
#line 114
}
#line 114
# 193 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void *CC2420ActiveMessageP__Packet__getPayload(message_t *msg, uint8_t len)
#line 193
{
  return CC2420ActiveMessageP__SubSend__getPayload(msg, len);
}

# 115 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
inline static void * LinkEstimatorP__SubPacket__getPayload(message_t * msg, uint8_t len){
#line 115
  void *__nesc_result;
#line 115

#line 115
  __nesc_result = CC2420ActiveMessageP__Packet__getPayload(msg, len);
#line 115

#line 115
  return __nesc_result;
#line 115
}
#line 115
# 96 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline linkest_header_t *LinkEstimatorP__getHeader(message_t *m)
#line 96
{
  return (linkest_header_t *)LinkEstimatorP__SubPacket__getPayload(m, sizeof(linkest_header_t ));
}










static inline uint8_t LinkEstimatorP__addLinkEstHeaderAndFooter(message_t *msg, uint8_t len)
#line 109
{
  unsigned char *__nesc_temp54;
#line 110
  uint8_t newlen;
  linkest_header_t * hdr;
  linkest_footer_t * footer;
  uint8_t i;
#line 113
  uint8_t j;
#line 113
  uint8_t k;
  uint8_t maxEntries;
#line 114
  uint8_t newPrevSentIdx;

#line 115
  ;
  hdr = LinkEstimatorP__getHeader(msg);
  footer = LinkEstimatorP__getFooter(msg, len);

  maxEntries = (LinkEstimatorP__SubPacket__maxPayloadLength() - len - sizeof(linkest_header_t ))
   / sizeof(linkest_footer_t );



  if (maxEntries > NUM_ENTRIES_FLAG) {
      maxEntries = NUM_ENTRIES_FLAG;
    }
  ;

  j = 0;
  newPrevSentIdx = 0;
  for (i = 0; i < 10 && j < maxEntries; i++) {
      uint8_t neighborCount;
      neighbor_stat_entry_t * neighborLists;

#line 134
      if (maxEntries <= 10) {
        neighborCount = maxEntries;
        }
      else {
#line 137
        neighborCount = 10;
        }
      neighborLists = (neighbor_stat_entry_t * )footer->neighborList;
      k = (LinkEstimatorP__prevSentIdx + i + 1) % 10;
      if (LinkEstimatorP__NeighborTable[k].flags & VALID_ENTRY && 
      LinkEstimatorP__NeighborTable[k].flags & MATURE_ENTRY) {
          __nesc_hton_uint16(neighborLists[j].ll_addr.data, LinkEstimatorP__NeighborTable[k].ll_addr);
          __nesc_hton_uint8(neighborLists[j].inquality.data, LinkEstimatorP__NeighborTable[k].inquality);
          newPrevSentIdx = k;
          ;

          j++;
        }
    }
  LinkEstimatorP__prevSentIdx = newPrevSentIdx;

  __nesc_hton_uint8(hdr->seq.data, LinkEstimatorP__linkEstSeq++);
  __nesc_hton_uint8(hdr->flags.data, 0);
  (__nesc_temp54 = hdr->flags.data, __nesc_hton_uint8(__nesc_temp54, __nesc_ntoh_uint8(__nesc_temp54) | (NUM_ENTRIES_FLAG & j)));
  newlen = sizeof(linkest_header_t ) + len + j * sizeof(linkest_footer_t );
  ;
  return newlen;
}

#line 555
static inline error_t LinkEstimatorP__Send__send(am_addr_t addr, message_t *msg, uint8_t len)
#line 555
{
  uint8_t newlen;

#line 557
  newlen = LinkEstimatorP__addLinkEstHeaderAndFooter(msg, len);
  ;
  ;
  LinkEstimatorP__print_packet(msg, newlen);
  return LinkEstimatorP__AMSend__send(addr, msg, newlen);
}

# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static error_t /*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = LinkEstimatorP__Send__send(addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 771 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline error_t /*CtpP.Router*/CtpRoutingEngineP__0__CollectionDebug__default__logEventRoute(uint8_t type, am_addr_t parent, uint8_t hopcount, uint16_t etx)
#line 771
{
  return SUCCESS;
}

# 68 "/opt/tinyos-2.1.1/tos/lib/net/CollectionDebug.nc"
inline static error_t /*CtpP.Router*/CtpRoutingEngineP__0__CollectionDebug__logEventRoute(uint8_t type, am_addr_t parent, uint8_t hopcount, uint16_t metric){
#line 68
  unsigned char __nesc_result;
#line 68

#line 68
  __nesc_result = /*CtpP.Router*/CtpRoutingEngineP__0__CollectionDebug__default__logEventRoute(type, parent, hopcount, metric);
#line 68

#line 68
  return __nesc_result;
#line 68
}
#line 68
# 760 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpCongestion__isCongested(void )
#line 760
{
  return FALSE;
}

# 7 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpCongestion.nc"
inline static bool /*CtpP.Router*/CtpRoutingEngineP__0__CtpCongestion__isCongested(void ){
#line 7
  unsigned char __nesc_result;
#line 7

#line 7
  __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpCongestion__isCongested();
#line 7

#line 7
  return __nesc_result;
#line 7
}
#line 7
# 375 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__sendBeaconTask__runTask(void )
#line 375
{
  unsigned char *__nesc_temp56;
  unsigned char *__nesc_temp55;
#line 376
  error_t eval;

#line 377
  if (/*CtpP.Router*/CtpRoutingEngineP__0__sending) {
      return;
    }

  __nesc_hton_uint8(/*CtpP.Router*/CtpRoutingEngineP__0__beaconMsg->options.data, 0);


  if (/*CtpP.Router*/CtpRoutingEngineP__0__CtpCongestion__isCongested()) {
      (__nesc_temp55 = /*CtpP.Router*/CtpRoutingEngineP__0__beaconMsg->options.data, __nesc_hton_uint8(__nesc_temp55, __nesc_ntoh_uint8(__nesc_temp55) | CTP_OPT_ECN));
    }

  __nesc_hton_uint16(/*CtpP.Router*/CtpRoutingEngineP__0__beaconMsg->parent.data, /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent);
  if (/*CtpP.Router*/CtpRoutingEngineP__0__state_is_root) {
      __nesc_hton_uint16(/*CtpP.Router*/CtpRoutingEngineP__0__beaconMsg->etx.data, /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.etx);
    }
  else {
#line 392
    if (/*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent == INVALID_ADDR) {
        __nesc_hton_uint16(/*CtpP.Router*/CtpRoutingEngineP__0__beaconMsg->etx.data, /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.etx);
        (__nesc_temp56 = /*CtpP.Router*/CtpRoutingEngineP__0__beaconMsg->options.data, __nesc_hton_uint8(__nesc_temp56, __nesc_ntoh_uint8(__nesc_temp56) | CTP_OPT_PULL));
      }
    else 
#line 395
      {
        __nesc_hton_uint16(/*CtpP.Router*/CtpRoutingEngineP__0__beaconMsg->etx.data, /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.etx + /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__getLinkQuality(/*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent));
      }
    }
  ;



  /*CtpP.Router*/CtpRoutingEngineP__0__CollectionDebug__logEventRoute(NET_C_TREE_SENT_BEACON, __nesc_ntoh_uint16(/*CtpP.Router*/CtpRoutingEngineP__0__beaconMsg->parent.data), 0, __nesc_ntoh_uint16(/*CtpP.Router*/CtpRoutingEngineP__0__beaconMsg->etx.data));

  eval = /*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__send(AM_BROADCAST_ADDR, 
  &/*CtpP.Router*/CtpRoutingEngineP__0__beaconMsgBuffer, 
  sizeof(ctp_routing_header_t ));
  if (eval == SUCCESS) {
      /*CtpP.Router*/CtpRoutingEngineP__0__sending = TRUE;
    }
  else {
#line 410
    if (eval == EOFF) {
        /*CtpP.Router*/CtpRoutingEngineP__0__radioOn = FALSE;
        ;
      }
    }
}

# 83 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Packet__setPayloadLength(message_t * msg, uint8_t len){
#line 83
  CC2420ActiveMessageP__Packet__setPayloadLength(msg, len);
#line 83
}
#line 83
# 287 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline uint16_t CC2420ControlP__CC2420Config__getPanAddr(void )
#line 287
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 288
    {
      unsigned int __nesc_temp = 
#line 288
      CC2420ControlP__m_pan;

      {
#line 288
        __nesc_atomic_end(__nesc_atomic); 
#line 288
        return __nesc_temp;
      }
    }
#line 290
    __nesc_atomic_end(__nesc_atomic); }
}

# 70 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static uint16_t CC2420ActiveMessageP__CC2420Config__getPanAddr(void ){
#line 70
  unsigned int __nesc_result;
#line 70

#line 70
  __nesc_result = CC2420ControlP__CC2420Config__getPanAddr();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 43 "/opt/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static bool CC2420TinyosNetworkP__Queue__isEmpty(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 184 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__Resource__immediateRequest(uint8_t id)
#line 184
{
  if (CC2420TinyosNetworkP__resource_owner == id) {
#line 185
    return EALREADY;
    }
  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      if (CC2420TinyosNetworkP__resource_owner == CC2420TinyosNetworkP__OWNER_NONE && CC2420TinyosNetworkP__Queue__isEmpty()) {
          CC2420TinyosNetworkP__resource_owner = id;
          return SUCCESS;
        }
      return 0x0080;
    }
  else 
#line 193
    {
      CC2420TinyosNetworkP__resource_owner = id;
      return SUCCESS;
    }
}

# 87 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420ActiveMessageP__RadioResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = CC2420TinyosNetworkP__Resource__immediateRequest(CC2420ActiveMessageC__CC2420_AM_SEND_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 278 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SendNotifier__default__aboutToSend(am_id_t amId, am_addr_t addr, message_t *msg)
#line 278
{
}

# 59 "/opt/tinyos-2.1.1/tos/interfaces/SendNotifier.nc"
inline static void CC2420ActiveMessageP__SendNotifier__aboutToSend(am_id_t arg_0x2b99bddf1258, am_addr_t dest, message_t * msg){
#line 59
    CC2420ActiveMessageP__SendNotifier__default__aboutToSend(arg_0x2b99bddf1258, dest, msg);
#line 59
}
#line 59
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static error_t CC2420TinyosNetworkP__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = UniqueSendP__Send__send(msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 77 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Packet.nc"
inline static void CC2420TinyosNetworkP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId){
#line 77
  CC2420PacketP__CC2420Packet__setNetwork(p_msg, networkId);
#line 77
}
#line 77
# 73 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__ActiveSend__send(message_t *msg, uint8_t len)
#line 73
{
  CC2420TinyosNetworkP__CC2420Packet__setNetwork(msg, 0x3f);
  return CC2420TinyosNetworkP__SubSend__send(msg, len);
}

# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static error_t CC2420ActiveMessageP__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = CC2420TinyosNetworkP__ActiveSend__send(msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 45 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static error_t UniqueSendP__State__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(7U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 42 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * UniqueSendP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 257 "/usr/local/lib/ncc/nesc_nx.h"
static __inline  int8_t __nesc_hton_int8(void * target, int8_t value)
#line 257
{
#line 257
  __nesc_hton_uint8(target, value);
#line 257
  return value;
}

#line 251
static __inline  uint8_t __nesc_hton_leuint8(void * target, uint8_t value)
#line 251
{
  uint8_t *base = target;

#line 253
  base[0] = value;
  return value;
}

# 545 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca)
#line 545
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 546
    {
      if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
          {
            unsigned char __nesc_temp = 
#line 548
            ECANCEL;

            {
#line 548
              __nesc_atomic_end(__nesc_atomic); 
#line 548
              return __nesc_temp;
            }
          }
        }
#line 551
      if (CC2420TransmitP__m_state != CC2420TransmitP__S_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 552
            0x0080;

            {
#line 552
              __nesc_atomic_end(__nesc_atomic); 
#line 552
              return __nesc_temp;
            }
          }
        }


      CC2420TransmitP__m_state = CC2420TransmitP__S_LOAD;
      CC2420TransmitP__m_cca = cca;
      CC2420TransmitP__m_msg = p_msg;
      CC2420TransmitP__totalCcaChecks = 0;
    }
#line 562
    __nesc_atomic_end(__nesc_atomic); }

  if (CC2420TransmitP__acquireSpiResource() == SUCCESS) {
      CC2420TransmitP__loadTXFIFO();
    }

  return SUCCESS;
}

#line 192
static inline error_t CC2420TransmitP__Send__send(message_t * p_msg, bool useCca)
#line 192
{
  return CC2420TransmitP__send(p_msg, useCca);
}

# 51 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
inline static error_t CC2420CsmaP__CC2420Transmit__send(message_t * p_msg, bool useCca){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420TransmitP__Send__send(p_msg, useCca);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 288 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__RadioBackoff__default__requestCca(am_id_t id, 
message_t *msg)
#line 289
{
}

# 95 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420ActiveMessageP__RadioBackoff__requestCca(am_id_t arg_0x2b99bddf1d60, message_t * msg){
#line 95
    CC2420ActiveMessageP__RadioBackoff__default__requestCca(arg_0x2b99bddf1d60, msg);
#line 95
}
#line 95
# 237 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SubBackoff__requestCca(message_t *msg)
#line 237
{

  CC2420ActiveMessageP__RadioBackoff__requestCca(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.data), msg);
}

# 95 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__RadioBackoff__requestCca(message_t * msg){
#line 95
  CC2420ActiveMessageP__SubBackoff__requestCca(msg);
#line 95
}
#line 95
# 51 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static void CC2420CsmaP__SplitControlState__forceState(uint8_t reqState){
#line 51
  StateImplP__State__forceState(6U, reqState);
#line 51
}
#line 51
#line 66
inline static bool CC2420CsmaP__SplitControlState__isState(uint8_t myState){
#line 66
  unsigned char __nesc_result;
#line 66

#line 66
  __nesc_result = StateImplP__State__isState(6U, myState);
#line 66

#line 66
  return __nesc_result;
#line 66
}
#line 66
# 47 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420CsmaP__CC2420PacketBody__getMetadata(message_t * msg){
#line 47
  nx_struct cc2420_metadata_t *__nesc_result;
#line 47

#line 47
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
#line 42
inline static cc2420_header_t * CC2420CsmaP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 122 "/opt/tinyos-2.1.1/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline error_t CC2420CsmaP__Send__send(message_t *p_msg, uint8_t len)
#line 122
{
  unsigned char *__nesc_temp47;
  unsigned char *__nesc_temp46;
#line 124
  cc2420_header_t *header = CC2420CsmaP__CC2420PacketBody__getHeader(p_msg);
  cc2420_metadata_t *metadata = CC2420CsmaP__CC2420PacketBody__getMetadata(p_msg);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 127
    {
      if (!CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTED)) {
          {
            unsigned char __nesc_temp = 
#line 129
            0x0080;

            {
#line 129
              __nesc_atomic_end(__nesc_atomic); 
#line 129
              return __nesc_temp;
            }
          }
        }
#line 132
      CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_TRANSMITTING);
      CC2420CsmaP__m_msg = p_msg;
    }
#line 134
    __nesc_atomic_end(__nesc_atomic); }

  __nesc_hton_leuint8(header->length.data, len + CC2420_SIZE);



  (__nesc_temp46 = header->fcf.data, __nesc_hton_leuint16(__nesc_temp46, __nesc_ntoh_leuint16(__nesc_temp46) & (1 << IEEE154_FCF_ACK_REQ)));

  (__nesc_temp47 = header->fcf.data, __nesc_hton_leuint16(__nesc_temp47, __nesc_ntoh_leuint16(__nesc_temp47) | ((((IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE) | (
  1 << IEEE154_FCF_INTRAPAN)) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE)) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE))));

  __nesc_hton_int8(metadata->ack.data, FALSE);
  __nesc_hton_uint8(metadata->rssi.data, 0);
  __nesc_hton_uint8(metadata->lqi.data, 0);

  __nesc_hton_uint32(metadata->timestamp.data, CC2420_INVALID_TIMESTAMP);

  CC2420CsmaP__ccaOn = TRUE;
  CC2420CsmaP__RadioBackoff__requestCca(CC2420CsmaP__m_msg);

  CC2420CsmaP__CC2420Transmit__send(CC2420CsmaP__m_msg, CC2420CsmaP__ccaOn);
  return SUCCESS;
}

# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static error_t UniqueSendP__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = CC2420CsmaP__Send__send(msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 55 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420TransmitP__TXCTRL__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_TXCTRL, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 59 "/opt/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
inline static error_t CC2420SpiP__SpiPacket__send(uint8_t * txBuf, uint8_t * rxBuf, uint16_t len){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__send(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID, txBuf, rxBuf, len);
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 34 "/opt/tinyos-2.1.1/tos/interfaces/SpiByte.nc"
inline static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx){
#line 34
  unsigned char __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiByte__write(tx);
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 61 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static bool CC2420SpiP__WorkingState__isIdle(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = StateImplP__State__isIdle(5U);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 214 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 215
{

  uint8_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 219
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 221
            status;

            {
#line 221
              __nesc_atomic_end(__nesc_atomic); 
#line 221
              return __nesc_temp;
            }
          }
        }
    }
#line 225
    __nesc_atomic_end(__nesc_atomic); }
#line 225
  CC2420SpiP__m_addr = addr;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__SpiPacket__send(data, (void *)0, len);

  return status;
}

# 82 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static cc2420_status_t CC2420TransmitP__TXFIFO__write(uint8_t * data, uint8_t length){
#line 82
  unsigned char __nesc_result;
#line 82

#line 82
  __nesc_result = CC2420SpiP__Fifo__write(CC2420_TXFIFO, data, length);
#line 82

#line 82
  return __nesc_result;
#line 82
}
#line 82
# 235 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__Usci__enableRxIntr(void )
#line 235
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 236
    {
      HplMsp430UsciB0P__IFG2 &= ~(1 << 2);
      HplMsp430UsciB0P__IE2 |= 1 << 2;
    }
#line 239
    __nesc_atomic_end(__nesc_atomic); }
}

# 100 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB.nc"
inline static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__enableRxIntr(void ){
#line 100
  HplMsp430UsciB0P__Usci__enableRxIntr();
#line 100
}
#line 100
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 54 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 54
{
  /* atomic removed: atomic calls only */
#line 55
  {
    unsigned char __nesc_temp = 
#line 55
    /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[id] != /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY || /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail == id;

#line 55
    return __nesc_temp;
  }
}

#line 72
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id)
#line 72
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 73
    {
      if (!/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(id)) {
          if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = id;
            }
          else {
#line 78
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail] = id;
            }
#line 79
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = id;
          {
            unsigned char __nesc_temp = 
#line 80
            SUCCESS;

            {
#line 80
              __nesc_atomic_end(__nesc_atomic); 
#line 80
              return __nesc_temp;
            }
          }
        }
#line 82
      {
        unsigned char __nesc_temp = 
#line 82
        EBUSY;

        {
#line 82
          __nesc_atomic_end(__nesc_atomic); 
#line 82
          return __nesc_temp;
        }
      }
    }
#line 85
    __nesc_atomic_end(__nesc_atomic); }
}

# 69 "/opt/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static error_t CC2420TinyosNetworkP__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(id);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420TinyosNetworkP__grantTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420TinyosNetworkP__grantTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 167 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__Resource__request(uint8_t id)
#line 167
{

  CC2420TinyosNetworkP__grantTask__postTask();

  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {

      return CC2420TinyosNetworkP__Queue__enqueue(id);
    }
  else 
#line 174
    {
      if (id == CC2420TinyosNetworkP__resource_owner) {
          return EALREADY;
        }
      else 
#line 177
        {
          CC2420TinyosNetworkP__next_owner = id;
          return SUCCESS;
        }
    }
}

# 78 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420ActiveMessageP__RadioResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420TinyosNetworkP__Resource__request(CC2420ActiveMessageC__CC2420_AM_SEND_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static void /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__sendDone(message_t * msg, error_t error){
#line 99
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubSend__sendDone(msg, error);
#line 99
}
#line 99
# 57 "/opt/tinyos-2.1.1/tos/system/AMQueueEntryP.nc"
static inline void /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__sendDone(message_t *m, error_t err)
#line 57
{
  /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__sendDone(m, err);
}

# 416 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__sendDone(message_t *msg, error_t error)
#line 416
{
  if (msg != &/*CtpP.Router*/CtpRoutingEngineP__0__beaconMsgBuffer || !/*CtpP.Router*/CtpRoutingEngineP__0__sending) {

      return;
    }
  /*CtpP.Router*/CtpRoutingEngineP__0__sending = FALSE;
}

# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static void LinkEstimatorP__Send__sendDone(message_t * msg, error_t error){
#line 99
  /*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__sendDone(msg, error);
#line 99
}
#line 99
# 566 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline void LinkEstimatorP__AMSend__sendDone(message_t *msg, error_t error)
#line 566
{
  LinkEstimatorP__Send__sendDone(msg, error);
}

# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static void /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMSend__sendDone(message_t * msg, error_t error){
#line 99
  LinkEstimatorP__AMSend__sendDone(msg, error);
#line 99
}
#line 99
# 57 "/opt/tinyos-2.1.1/tos/system/AMQueueEntryP.nc"
static inline void /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__Send__sendDone(message_t *m, error_t err)
#line 57
{
  /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__AMSend__sendDone(m, err);
}

# 207 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__default__sendDone(uint8_t id, message_t *msg, error_t err)
#line 207
{
}

# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__sendDone(uint8_t arg_0x2b99bcb961c8, message_t * msg, error_t error){
#line 89
  switch (arg_0x2b99bcb961c8) {
#line 89
    case 0U:
#line 89
      /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__sendDone(msg, error);
#line 89
      break;
#line 89
    case 1U:
#line 89
      /*CtpP.SendControl.SenderC.AMQueueEntryP*/AMQueueEntryP__3__Send__sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__default__sendDone(arg_0x2b99bcb961c8, msg, error);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
# 118 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__CancelTask__runTask(void )
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
  for (i = 0; i < 2 / 8 + 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__cancelMask[i]) {
          for (mask = 1, j = 0; j < 8; j++) {
              if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__cancelMask[i] & mask) {
                  last = i * 8 + j;
                  msg = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[last].msg;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[last].msg = (void *)0;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__cancelMask[i] &= ~mask;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__sendDone(last, msg, ECANCEL);
                }
              mask <<= 1;
            }
        }
    }
}

# 41 "/opt/tinyos-2.1.1/tos/interfaces/Random.nc"
inline static uint16_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Random__rand16(void ){
#line 41
  unsigned int __nesc_result;
#line 41

#line 41
  __nesc_result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 75 "/opt/tinyos-2.1.1/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline bool CC2420PacketP__Acks__wasAcked(message_t *p_msg)
#line 75
{
  return __nesc_ntoh_int8(CC2420PacketP__CC2420PacketBody__getMetadata(p_msg)->ack.data);
}

# 74 "/opt/tinyos-2.1.1/tos/interfaces/PacketAcknowledgements.nc"
inline static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__PacketAcknowledgements__wasAcked(message_t * msg){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = CC2420PacketP__Acks__wasAcked(msg);
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 524 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline error_t LinkEstimatorP__LinkEstimator__txNoAck(am_addr_t neighbor)
#line 524
{
  neighbor_table_entry_t *ne;
  uint8_t nidx = LinkEstimatorP__findIdx(neighbor);

#line 527
  if (nidx == LinkEstimatorP__INVALID_RVAL) {
      return 0x0080;
    }

  ne = &LinkEstimatorP__NeighborTable[nidx];
  ne->data_total++;
  if (ne->data_total >= LinkEstimatorP__DLQ_PKT_WINDOW) {
      LinkEstimatorP__updateDETX(ne);
    }
  return SUCCESS;
}

# 55 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimator.nc"
inline static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__LinkEstimator__txNoAck(am_addr_t neighbor){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = LinkEstimatorP__LinkEstimator__txNoAck(neighbor);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*CtpP.Router*/CtpRoutingEngineP__0__updateRouteTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*CtpP.Router*/CtpRoutingEngineP__0__updateRouteTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 544 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__recomputeRoutes(void )
#line 544
{
  /*CtpP.Router*/CtpRoutingEngineP__0__updateRouteTask__postTask();
}

# 71 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpInfo.nc"
inline static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__recomputeRoutes(void ){
#line 71
  /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__recomputeRoutes();
#line 71
}
#line 71
# 69 "/opt/tinyos-2.1.1/tos/system/QueueC.nc"
static inline void /*CtpP.SendQueueP*/QueueC__0__printQueue(void )
#line 69
{
}

# 63 "TestAPIComplexP.nc"
static inline void TestAPIComplexP__PuppetAPI__registerRequestDone(message_t *msg, error_t err)
{
  if (!(err == SUCCESS)) {
#line 65
      assertTunitFail("Message was not sent", 20U);
#line 65
      ;
    }
  else 
#line 65
    {
#line 65
      assertTunitSuccess(21U);
#line 65
      ;
    }
#line 65
  ;
  if (!(msg != (void *)0)) {
#line 66
      assertTunitFail("Message was NULL", 22U);
#line 66
      ;
    }
  else 
#line 66
    {
#line 66
      assertTunitSuccess(23U);
#line 66
      ;
    }
#line 66
  ;
}

# 8 "/home/chuka/projects/puppet-os/interfaces/PuppetAPI.nc"
inline static void HomeCommManagerP__PuppetAPI__registerRequestDone(message_t *msg, error_t err){
#line 8
  TestAPIComplexP__PuppetAPI__registerRequestDone(msg, err);
#line 8
}
#line 8
# 79 "/home/chuka/projects/puppet-os/system/HomeCommManagerP.nc"
static inline void HomeCommManagerP__RadioSend__sendDone(message_t *msg, error_t e)
{
  HomeCommManagerP__PuppetAPI__registerRequestDone(msg, e);
}

# 856 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline 
#line 855
void 
/*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__default__sendDone(uint8_t client, message_t *msg, error_t error)
#line 856
{
}

# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__sendDone(uint8_t arg_0x2b99bde943f0, message_t * msg, error_t error){
#line 89
  switch (arg_0x2b99bde943f0) {
#line 89
    case 0U:
#line 89
      HomeCommManagerP__RadioSend__sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__default__sendDone(arg_0x2b99bde943f0, msg, error);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
# 66 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpPacket.nc"
inline static uint8_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getType(message_t *msg){
#line 66
  unsigned char __nesc_result;
#line 66

#line 66
  __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getType(msg);
#line 66

#line 66
  return __nesc_result;
#line 66
}
#line 66
#line 54
inline static uint8_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getThl(message_t *msg){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getThl(msg);
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54









inline static uint8_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getSequenceNumber(message_t *msg){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getSequenceNumber(msg);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
#line 60
inline static am_addr_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getOrigin(message_t *msg){
#line 60
  unsigned int __nesc_result;
#line 60

#line 60
  __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getOrigin(msg);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 100 "/opt/tinyos-2.1.1/tos/lib/net/ctp/LruCtpMsgCacheP.nc"
static inline void /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__remove(uint8_t i)
#line 100
{
  uint8_t j;

#line 102
  if (i >= /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__count) {
    return;
    }
#line 104
  if (i == 0) {

      /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__first = (/*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__first + 1) % 4;
    }
  else 
#line 107
    {

      for (j = i; j < /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__count; j++) {
          memcpy(&/*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__cache[(j + /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__first) % 4], &/*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__cache[(j + /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__first + 1) % 4], sizeof(/*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__ctp_packet_sig_t ));
        }
    }
  /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__count--;
}

static inline void /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__Cache__insert(message_t *m)
#line 116
{
  uint8_t i;

#line 118
  if (/*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__count == 4) {





      i = /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__lookup(m);
      /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__remove(i % /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__count);
    }

  /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__cache[(/*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__first + /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__count) % 4].origin = /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getOrigin(m);
  /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__cache[(/*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__first + /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__count) % 4].seqno = /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getSequenceNumber(m);
  /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__cache[(/*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__first + /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__count) % 4].thl = /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getThl(m);
  /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__cache[(/*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__first + /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__count) % 4].type = /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getType(m);
  /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__count++;
}

# 40 "/opt/tinyos-2.1.1/tos/interfaces/Cache.nc"
inline static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__SentCache__insert(/*CtpP.Forwarder*/CtpForwardingEngineP__0__SentCache__t item){
#line 40
  /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__Cache__insert(item);
#line 40
}
#line 40
# 507 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline error_t LinkEstimatorP__LinkEstimator__txAck(am_addr_t neighbor)
#line 507
{
  neighbor_table_entry_t *ne;
  uint8_t nidx = LinkEstimatorP__findIdx(neighbor);

#line 510
  if (nidx == LinkEstimatorP__INVALID_RVAL) {
      return 0x0080;
    }
  ne = &LinkEstimatorP__NeighborTable[nidx];
  ne->data_success++;
  ne->data_total++;
  if (ne->data_total >= LinkEstimatorP__DLQ_PKT_WINDOW) {
      LinkEstimatorP__updateDETX(ne);
    }
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimator.nc"
inline static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__LinkEstimator__txAck(am_addr_t neighbor){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = LinkEstimatorP__LinkEstimator__txAck(neighbor);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 161 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask__runTask(void )
#line 161
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current, /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current].msg, 0x0080);
}

# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__send(am_id_t arg_0x2b99bcb95340, am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = CC2420ActiveMessageP__AMSend__send(arg_0x2b99bcb95340, addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
inline static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Packet__payloadLength(message_t * msg){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = CC2420ActiveMessageP__Packet__payloadLength(msg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 67 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__destination(message_t * amsg){
#line 67
  unsigned int __nesc_result;
#line 67

#line 67
  __nesc_result = CC2420ActiveMessageP__AMPacket__destination(amsg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
#line 136
inline static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__type(message_t * amsg){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = CC2420ActiveMessageP__AMPacket__type(amsg);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 57 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__nextPacket(void )
#line 57
{
  uint8_t i;

#line 59
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current + 1) % 2;
  for (i = 0; i < 2; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current].msg == (void *)0 || 
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__cancelMask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current / 8] & (1 << /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current % 8)) 
        {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current + 1) % 2;
        }
      else {
          break;
        }
    }
  if (i >= 2) {
#line 70
    /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current = 2;
    }
}

#line 166
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__tryToSend(void )
#line 166
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__nextPacket();
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current < 2) {
      error_t nextErr;
      message_t *nextMsg = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current].msg;
      am_id_t nextId = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__type(nextMsg);
      am_addr_t nextDest = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__destination(nextMsg);
      uint8_t len = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Packet__payloadLength(nextMsg);

#line 174
      nextErr = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__send(nextId, nextDest, nextMsg, len);
      if (nextErr != SUCCESS) {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask__postTask();
        }
    }
}

# 885 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__default__logEvent(uint8_t type)
#line 885
{
  return SUCCESS;
}

# 50 "/opt/tinyos-2.1.1/tos/lib/net/CollectionDebug.nc"
inline static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(uint8_t type){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__default__logEvent(type);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 785 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__maxPayloadLength(void )
#line 785
{
  return /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubPacket__maxPayloadLength() - sizeof(ctp_data_header_t );
}

# 83 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
inline static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubPacket__setPayloadLength(message_t * msg, uint8_t len){
#line 83
  CC2420ActiveMessageP__Packet__setPayloadLength(msg, len);
#line 83
}
#line 83
# 781 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__setPayloadLength(message_t *msg, uint8_t len)
#line 781
{
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubPacket__setPayloadLength(msg, len + sizeof(ctp_data_header_t ));
}

#line 849
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__setState(uint8_t state)
#line 849
{
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__forwardingState = /*CtpP.Forwarder*/CtpForwardingEngineP__0__forwardingState | state;
}

# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static error_t /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__send(0U, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 151 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static void /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  CC2420ActiveMessageP__AMPacket__setType(amsg, t);
#line 151
}
#line 151
#line 92
inline static void /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  CC2420ActiveMessageP__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
# 45 "/opt/tinyos-2.1.1/tos/system/AMQueueEntryP.nc"
static inline error_t /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 47
{
  /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMPacket__setDestination(msg, dest);
  /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMPacket__setType(msg, 113);
  return /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__send(msg, len);
}

# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__send(addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 843 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__clearState(uint8_t state)
#line 843
{
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__forwardingState = /*CtpP.Forwarder*/CtpForwardingEngineP__0__forwardingState & ~state;
}

#line 285
static inline ctp_data_header_t */*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(message_t *m)
#line 285
{
  return (ctp_data_header_t *)/*CtpP.Forwarder*/CtpForwardingEngineP__0__SubPacket__getPayload(m, sizeof(ctp_data_header_t ));
}

#line 819
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__setOption(message_t *msg, ctp_options_t opt)
#line 819
{
  unsigned char *__nesc_temp52;

#line 820
  (__nesc_temp52 = /*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(msg)->options.data, __nesc_hton_uint8(__nesc_temp52, __nesc_ntoh_uint8(__nesc_temp52) | opt));
}

#line 846
static inline bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__hasState(uint8_t state)
#line 846
{
  return /*CtpP.Forwarder*/CtpForwardingEngineP__0__forwardingState & state;
}

# 65 "/opt/tinyos-2.1.1/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline error_t CC2420PacketP__Acks__requestAck(message_t *p_msg)
#line 65
{
  unsigned char *__nesc_temp50;

#line 66
  (__nesc_temp50 = CC2420PacketP__CC2420PacketBody__getHeader(p_msg)->fcf.data, __nesc_hton_leuint16(__nesc_temp50, __nesc_ntoh_leuint16(__nesc_temp50) | (1 << IEEE154_FCF_ACK_REQ)));
  return SUCCESS;
}

# 48 "/opt/tinyos-2.1.1/tos/interfaces/PacketAcknowledgements.nc"
inline static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__PacketAcknowledgements__requestAck(message_t * msg){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = CC2420PacketP__Acks__requestAck(msg);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 822 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__clearOption(message_t *msg, ctp_options_t opt)
#line 822
{
  unsigned char *__nesc_temp53;

#line 823
  (__nesc_temp53 = /*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(msg)->options.data, __nesc_hton_uint8(__nesc_temp53, __nesc_ntoh_uint8(__nesc_temp53) & ~opt));
}

#line 814
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__setEtx(message_t *msg, uint16_t e)
#line 814
{
#line 814
  __nesc_hton_uint16(/*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(msg)->etx.data, e);
}

#line 865
static inline message_t *
/*CtpP.Forwarder*/CtpForwardingEngineP__0__Receive__default__receive(collection_id_t collectid, message_t *msg, void *payload, 
uint8_t len)
#line 867
{
  return msg;
}

# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
inline static message_t * /*CtpP.Forwarder*/CtpForwardingEngineP__0__Receive__receive(collection_id_t arg_0x2b99bde93648, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  switch (arg_0x2b99bde93648) {
#line 67
    case AM_REGISTER_REQUEST:
#line 67
      __nesc_result = TestAPIComplexP__Receive__receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__Receive__default__receive(arg_0x2b99bde93648, msg, payload, len);
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
# 67 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
inline static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubPacket__payloadLength(message_t * msg){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = CC2420ActiveMessageP__Packet__payloadLength(msg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 777 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__payloadLength(message_t *msg)
#line 777
{
  return /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubPacket__payloadLength(msg) - sizeof(ctp_data_header_t );
}

# 611 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline bool /*CtpP.Router*/CtpRoutingEngineP__0__RootControl__isRoot(void )
#line 611
{
  return /*CtpP.Router*/CtpRoutingEngineP__0__state_is_root;
}

# 43 "/opt/tinyos-2.1.1/tos/lib/net/RootControl.nc"
inline static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__RootControl__isRoot(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*CtpP.Router*/CtpRoutingEngineP__0__RootControl__isRoot();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 89 "/opt/tinyos-2.1.1/tos/interfaces/Pool.nc"
inline static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__put(/*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__t * newVal){
#line 89
  unsigned char __nesc_result;
#line 89

#line 89
  __nesc_result = /*CtpP.QEntryPoolP.PoolP*/PoolP__1__Pool__put(newVal);
#line 89

#line 89
  return __nesc_result;
#line 89
}
#line 89
inline static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__put(/*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__t * newVal){
#line 89
  unsigned char __nesc_result;
#line 89

#line 89
  __nesc_result = /*CtpP.MessagePoolP.PoolP*/PoolP__0__Pool__put(newVal);
#line 89

#line 89
  return __nesc_result;
#line 89
}
#line 89
# 81 "/opt/tinyos-2.1.1/tos/interfaces/Queue.nc"
inline static /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__t  /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__dequeue(void ){
#line 81
  struct __nesc_unnamed4300 *__nesc_result;
#line 81

#line 81
  __nesc_result = /*CtpP.SendQueueP*/QueueC__0__Queue__dequeue();
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 135 "/opt/tinyos-2.1.1/tos/lib/net/ctp/LruCtpMsgCacheP.nc"
static inline bool /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__Cache__lookup(message_t *m)
#line 135
{
  return /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__lookup(m) < /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__count;
}

# 48 "/opt/tinyos-2.1.1/tos/interfaces/Cache.nc"
inline static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__SentCache__lookup(/*CtpP.Forwarder*/CtpForwardingEngineP__0__SentCache__t item){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__Cache__lookup(item);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 514 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline am_addr_t /*CtpP.Router*/CtpRoutingEngineP__0__Routing__nextHop(void )
#line 514
{
  return /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent;
}

# 48 "/opt/tinyos-2.1.1/tos/lib/net/UnicastNameFreeRouting.nc"
inline static am_addr_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__UnicastNameFreeRouting__nextHop(void ){
#line 48
  unsigned int __nesc_result;
#line 48

#line 48
  __nesc_result = /*CtpP.Router*/CtpRoutingEngineP__0__Routing__nextHop();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 65 "/opt/tinyos-2.1.1/tos/system/QueueC.nc"
static inline /*CtpP.SendQueueP*/QueueC__0__queue_t /*CtpP.SendQueueP*/QueueC__0__Queue__head(void )
#line 65
{
  return /*CtpP.SendQueueP*/QueueC__0__queue[/*CtpP.SendQueueP*/QueueC__0__head];
}

# 73 "/opt/tinyos-2.1.1/tos/interfaces/Queue.nc"
inline static /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__t  /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__head(void ){
#line 73
  struct __nesc_unnamed4300 *__nesc_result;
#line 73

#line 73
  __nesc_result = /*CtpP.SendQueueP*/QueueC__0__Queue__head();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 52 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpInfo.nc"
inline static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__getEtx(uint16_t *etx){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__getEtx(etx);
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 517 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline bool /*CtpP.Router*/CtpRoutingEngineP__0__Routing__hasRoute(void )
#line 517
{
  return /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent != INVALID_ADDR;
}

# 49 "/opt/tinyos-2.1.1/tos/lib/net/UnicastNameFreeRouting.nc"
inline static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__UnicastNameFreeRouting__hasRoute(void ){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = /*CtpP.Router*/CtpRoutingEngineP__0__Routing__hasRoute();
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 53 "/opt/tinyos-2.1.1/tos/system/QueueC.nc"
static inline bool /*CtpP.SendQueueP*/QueueC__0__Queue__empty(void )
#line 53
{
  return /*CtpP.SendQueueP*/QueueC__0__size == 0;
}

# 50 "/opt/tinyos-2.1.1/tos/interfaces/Queue.nc"
inline static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__empty(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*CtpP.SendQueueP*/QueueC__0__Queue__empty();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 377 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask__runTask(void )
#line 377
{
  uint16_t gradient;

#line 379
  ;
  if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__hasState(/*CtpP.Forwarder*/CtpForwardingEngineP__0__SENDING) || /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__empty()) {
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_SENDQUEUE_EMPTY);
      return;
    }
  else {
    if ((
#line 384
    !/*CtpP.Forwarder*/CtpForwardingEngineP__0__RootControl__isRoot() && 
    !/*CtpP.Forwarder*/CtpForwardingEngineP__0__UnicastNameFreeRouting__hasRoute()) || 
    /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__getEtx(&gradient) != SUCCESS) {








        ;
        /*CtpP.Forwarder*/CtpForwardingEngineP__0__RetxmitTimer__startOneShot(NO_ROUTE_RETRY);
        /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_NO_ROUTE);
        return;
      }
    else {



        error_t subsendResult;
        fe_queue_entry_t *qe = /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__head();
        uint8_t payloadLen = /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubPacket__payloadLength(qe->msg);
        am_addr_t dest = /*CtpP.Forwarder*/CtpForwardingEngineP__0__UnicastNameFreeRouting__nextHop();

        if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__SentCache__lookup(qe->msg)) {




            /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_DUPLICATE_CACHE_AT_SEND);
            /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__dequeue();
            if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__put(qe->msg) != SUCCESS) {
              /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_PUT_MSGPOOL_ERR);
              }
#line 418
            if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__put(qe) != SUCCESS) {
              /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_PUT_QEPOOL_ERR);
              }
            /*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask__postTask();
            return;
          }


        ;

        if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__RootControl__isRoot()) {

            collection_id_t collectid = __nesc_ntoh_uint8(/*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(qe->msg)->type.data);
            uint8_t *payload;
            uint8_t payloadLength;

            memcpy(/*CtpP.Forwarder*/CtpForwardingEngineP__0__loopbackMsgPtr, qe->msg, sizeof(message_t ));

            payload = /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__getPayload(/*CtpP.Forwarder*/CtpForwardingEngineP__0__loopbackMsgPtr, /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__payloadLength(/*CtpP.Forwarder*/CtpForwardingEngineP__0__loopbackMsgPtr));
            payloadLength = /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__payloadLength(/*CtpP.Forwarder*/CtpForwardingEngineP__0__loopbackMsgPtr);
            ;
            /*CtpP.Forwarder*/CtpForwardingEngineP__0__loopbackMsgPtr = /*CtpP.Forwarder*/CtpForwardingEngineP__0__Receive__receive(collectid, /*CtpP.Forwarder*/CtpForwardingEngineP__0__loopbackMsgPtr, 
            payload, 
            payloadLength);
            /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubSend__sendDone(qe->msg, SUCCESS);
          }
        else {

            /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__setEtx(qe->msg, gradient);
            /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__clearOption(qe->msg, CTP_OPT_ECN | CTP_OPT_PULL);
            if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__PacketAcknowledgements__requestAck(qe->msg) == SUCCESS) {
                /*CtpP.Forwarder*/CtpForwardingEngineP__0__setState(/*CtpP.Forwarder*/CtpForwardingEngineP__0__ACK_PENDING);
              }
            if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__hasState(/*CtpP.Forwarder*/CtpForwardingEngineP__0__QUEUE_CONGESTED)) {
                /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__setOption(qe->msg, CTP_OPT_ECN);
                /*CtpP.Forwarder*/CtpForwardingEngineP__0__clearState(/*CtpP.Forwarder*/CtpForwardingEngineP__0__QUEUE_CONGESTED);
              }

            subsendResult = /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubSend__send(dest, qe->msg, payloadLen);
            if (subsendResult == SUCCESS) {

                /*CtpP.Forwarder*/CtpForwardingEngineP__0__setState(/*CtpP.Forwarder*/CtpForwardingEngineP__0__SENDING);
                ;
                return;
              }
            else {
              if (subsendResult == ESIZE) {
                  ;
                  /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__setPayloadLength(qe->msg, /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__maxPayloadLength());
                  /*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask__postTask();
                  /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_SUBSEND_SIZE);
                }
              else {
                  ;
                }
              }
          }
      }
    }
}

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
inline static void TestAPIComplexP__TestRegister__done(void ){
#line 41
  TUnitP__TestCase__done(/*TestAPIComplexC.TestRegisterC*/TestCaseC__0__TUNIT_TEST_ID);
#line 41
}
#line 41
# 198 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__Resource__release(uint8_t id)
#line 198
{
  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      CC2420TinyosNetworkP__grantTask__postTask();
    }
  CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__OWNER_NONE;
  return SUCCESS;
}

#line 222
static inline void CC2420TinyosNetworkP__Resource__default__granted(uint8_t client)
#line 222
{
  CC2420TinyosNetworkP__Resource__release(client);
}

# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static void CC2420TinyosNetworkP__Resource__granted(uint8_t arg_0x2b99bdd37550){
#line 92
  switch (arg_0x2b99bdd37550) {
#line 92
    case CC2420ActiveMessageC__CC2420_AM_SEND_ID:
#line 92
      CC2420ActiveMessageP__RadioResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      CC2420TinyosNetworkP__Resource__default__granted(arg_0x2b99bdd37550);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 58 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void )
#line 58
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 59
    {
      if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead != /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
          uint8_t id = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead;

#line 62
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead];
          if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
            }
#line 65
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[id] = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
          {
            unsigned char __nesc_temp = 
#line 66
            id;

            {
#line 66
              __nesc_atomic_end(__nesc_atomic); 
#line 66
              return __nesc_temp;
            }
          }
        }
#line 68
      {
        unsigned char __nesc_temp = 
#line 68
        /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

        {
#line 68
          __nesc_atomic_end(__nesc_atomic); 
#line 68
          return __nesc_temp;
        }
      }
    }
#line 71
    __nesc_atomic_end(__nesc_atomic); }
}

# 60 "/opt/tinyos-2.1.1/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t CC2420TinyosNetworkP__Queue__dequeue(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 148 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__grantTask__runTask(void )
#line 148
{


  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      if (CC2420TinyosNetworkP__resource_owner == CC2420TinyosNetworkP__OWNER_NONE && !CC2420TinyosNetworkP__Queue__isEmpty()) {
          CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__Queue__dequeue();

          if (CC2420TinyosNetworkP__resource_owner != CC2420TinyosNetworkP__OWNER_NONE) {
              CC2420TinyosNetworkP__Resource__granted(CC2420TinyosNetworkP__resource_owner);
            }
        }
    }
  else 
#line 159
    {
      if (CC2420TinyosNetworkP__next_owner != CC2420TinyosNetworkP__resource_owner) {
          CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__next_owner;
          CC2420TinyosNetworkP__Resource__granted(CC2420TinyosNetworkP__resource_owner);
        }
    }
}

# 92 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 47 "/opt/tinyos-2.1.1/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(t0, dt);
}

#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 83
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(t0, dt, TRUE);
}

# 118 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 118
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 118
}
#line 118
# 54 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
}

# 62 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop();
#line 62
}
#line 62
# 91 "/opt/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__stop();
}

# 62 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop();
#line 62
}
#line 62
# 60 "/opt/tinyos-2.1.1/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 61
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 67 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 67
}
#line 67
# 53 "/opt/tinyos-2.1.1/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/opt/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get();
}

# 98 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 98
  unsigned long __nesc_result;
#line 98

#line 98
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 85 "/opt/tinyos-2.1.1/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 86
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 125 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void ){
#line 125
  unsigned long __nesc_result;
#line 125

#line 125
  __nesc_result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow();
#line 125

#line 125
  return __nesc_result;
#line 125
}
#line 125
# 89 "/opt/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(now);
        }
      else {
#line 124
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(now, min_remaining);
        }
    }
}

# 178 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__decayInterval(void )
#line 178
{
  /*CtpP.Router*/CtpRoutingEngineP__0__currentInterval *= 2;
  if (/*CtpP.Router*/CtpRoutingEngineP__0__currentInterval > 512000L) {
      /*CtpP.Router*/CtpRoutingEngineP__0__currentInterval = 512000L;
    }
  /*CtpP.Router*/CtpRoutingEngineP__0__chooseAdvertiseTime();
}

# 148 "/opt/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, TRUE);
}

# 62 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void /*CtpP.Router*/CtpRoutingEngineP__0__BeaconTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(1U, dt);
#line 62
}
#line 62
# 186 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__remainingInterval(void )
#line 186
{
  uint32_t remaining = /*CtpP.Router*/CtpRoutingEngineP__0__currentInterval;

#line 188
  remaining -= /*CtpP.Router*/CtpRoutingEngineP__0__t;
  /*CtpP.Router*/CtpRoutingEngineP__0__tHasPassed = TRUE;
  /*CtpP.Router*/CtpRoutingEngineP__0__BeaconTimer__startOneShot(remaining);
}

# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t /*CtpP.Router*/CtpRoutingEngineP__0__sendBeaconTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*CtpP.Router*/CtpRoutingEngineP__0__sendBeaconTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 430 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__BeaconTimer__fired(void )
#line 430
{
  if (/*CtpP.Router*/CtpRoutingEngineP__0__radioOn && /*CtpP.Router*/CtpRoutingEngineP__0__running) {
      if (!/*CtpP.Router*/CtpRoutingEngineP__0__tHasPassed) {
          /*CtpP.Router*/CtpRoutingEngineP__0__updateRouteTask__postTask();
          /*CtpP.Router*/CtpRoutingEngineP__0__sendBeaconTask__postTask();
          ;
          /*CtpP.Router*/CtpRoutingEngineP__0__remainingInterval();
        }
      else {
          /*CtpP.Router*/CtpRoutingEngineP__0__decayInterval();
        }
    }
}

#line 424
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__RouteTimer__fired(void )
#line 424
{
  if (/*CtpP.Router*/CtpRoutingEngineP__0__radioOn && /*CtpP.Router*/CtpRoutingEngineP__0__running) {
      /*CtpP.Router*/CtpRoutingEngineP__0__updateRouteTask__postTask();
    }
}

# 754 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__RetxmitTimer__fired(void )
#line 754
{
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__clearState(/*CtpP.Forwarder*/CtpForwardingEngineP__0__SENDING);
  ;
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask__postTask();
}

# 193 "/opt/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 72 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x2b99bdc492f8){
#line 72
  switch (arg_0x2b99bdc492f8) {
#line 72
    case 1U:
#line 72
      /*CtpP.Router*/CtpRoutingEngineP__0__BeaconTimer__fired();
#line 72
      break;
#line 72
    case 2U:
#line 72
      /*CtpP.Router*/CtpRoutingEngineP__0__RouteTimer__fired();
#line 72
      break;
#line 72
    case 3U:
#line 72
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__RetxmitTimer__fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x2b99bdc492f8);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 128 "/opt/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 72 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 72
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 72
}
#line 72
# 80 "/opt/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 82
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type __nesc_temp = 
#line 82
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;

      {
#line 82
        __nesc_atomic_end(__nesc_atomic); 
#line 82
        return __nesc_temp;
      }
    }
#line 84
    __nesc_atomic_end(__nesc_atomic); }
}

# 105 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 105
  unsigned long __nesc_result;
#line 105

#line 105
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 63 "/opt/tinyos-2.1.1/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 210 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline message_t *CC2420TinyosNetworkP__BareReceive__default__receive(message_t *msg, void *payload, uint8_t len)
#line 210
{
  return msg;
}

# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
inline static message_t * CC2420TinyosNetworkP__BareReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = CC2420TinyosNetworkP__BareReceive__default__receive(msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 42 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420TinyosNetworkP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 74 "TestAPIComplexP.nc"
static inline message_t *TestAPIComplexP__Snoop__receive(message_t *msg, void *payload, uint8_t len)
{
  if (msg == (void *)0) {
#line 76
      assertTunitFail("msg"" was null", 24U);
#line 76
      ;
    }
  else 
#line 76
    {
#line 76
      assertTunitSuccess(25U);
#line 76
      ;
    }
#line 76
  ;
  return msg;
}

# 871 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline message_t *
/*CtpP.Forwarder*/CtpForwardingEngineP__0__Snoop__default__receive(collection_id_t collectid, message_t *msg, void *payload, 
uint8_t len)
#line 873
{
  return msg;
}

# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
inline static message_t * /*CtpP.Forwarder*/CtpForwardingEngineP__0__Snoop__receive(collection_id_t arg_0x2b99bde92270, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  switch (arg_0x2b99bde92270) {
#line 67
    case AM_REGISTER_REQUEST:
#line 67
      __nesc_result = TestAPIComplexP__Snoop__receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__Snoop__default__receive(arg_0x2b99bde92270, msg, payload, len);
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
# 76 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpInfo.nc"
inline static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__setNeighborCongested(am_addr_t n, bool congested){
#line 76
  /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__setNeighborCongested(n, congested);
#line 76
}
#line 76
#line 59
inline static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__triggerRouteUpdate(void ){
#line 59
  /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__triggerRouteUpdate();
#line 59
}
#line 59
# 77 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__source(message_t * amsg){
#line 77
  unsigned int __nesc_result;
#line 77

#line 77
  __nesc_result = CC2420ActiveMessageP__AMPacket__source(amsg);
#line 77

#line 77
  return __nesc_result;
#line 77
}
#line 77
# 737 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline message_t *
/*CtpP.Forwarder*/CtpForwardingEngineP__0__SubSnoop__receive(message_t *msg, void *payload, uint8_t len)
#line 738
{

  am_addr_t proximalSrc = /*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__source(msg);



  if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__option(msg, CTP_OPT_PULL)) {
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__triggerRouteUpdate();
    }

  /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__setNeighborCongested(proximalSrc, /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__option(msg, CTP_OPT_ECN));
  return /*CtpP.Forwarder*/CtpForwardingEngineP__0__Snoop__receive(/*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getType(msg), 
  msg, payload + sizeof(ctp_data_header_t ), 
  len - sizeof(ctp_data_header_t ));
}

# 270 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline message_t *CC2420ActiveMessageP__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 270
{
  return msg;
}

# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
inline static message_t * CC2420ActiveMessageP__Snoop__receive(am_id_t arg_0x2b99bddf4480, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  switch (arg_0x2b99bddf4480) {
#line 67
    case 113:
#line 67
      __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubSnoop__receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      __nesc_result = CC2420ActiveMessageP__Snoop__default__receive(arg_0x2b99bddf4480, msg, payload, len);
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
# 158 "/opt/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num)
{
  return /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning;
}

# 81 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__RetxmitTimer__isRunning(void ){
#line 81
  unsigned char __nesc_result;
#line 81

#line 81
  __nesc_result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(3U);
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 67 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__destination(message_t * amsg){
#line 67
  unsigned int __nesc_result;
#line 67

#line 67
  __nesc_result = CC2420ActiveMessageP__AMPacket__destination(amsg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 800 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getSequenceNumber(message_t *msg)
#line 800
{
#line 800
  return __nesc_ntoh_uint8(/*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(msg)->originSeqNo.data);
}

#line 894
static inline error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__default__logEventMsg(uint8_t type, uint16_t msg, am_addr_t origin, am_addr_t node)
#line 894
{
  return SUCCESS;
}

# 62 "/opt/tinyos-2.1.1/tos/lib/net/CollectionDebug.nc"
inline static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEventMsg(uint8_t type, uint16_t msg, am_addr_t origin, am_addr_t node){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__default__logEventMsg(type, msg, origin, node);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 552 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__triggerImmediateRouteUpdate(void )
#line 552
{
  /*CtpP.Router*/CtpRoutingEngineP__0__resetInterval();
}

# 66 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpInfo.nc"
inline static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__triggerImmediateRouteUpdate(void ){
#line 66
  /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__triggerImmediateRouteUpdate();
#line 66
}
#line 66
# 808 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline uint16_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getEtx(message_t *msg)
#line 808
{
#line 808
  return __nesc_ntoh_uint16(/*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(msg)->etx.data);
}

# 90 "/opt/tinyos-2.1.1/tos/interfaces/Queue.nc"
inline static error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__enqueue(/*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__t  newVal){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*CtpP.SendQueueP*/QueueC__0__Queue__enqueue(newVal);
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 88 "/opt/tinyos-2.1.1/tos/system/PoolP.nc"
static inline /*CtpP.MessagePoolP.PoolP*/PoolP__0__pool_t */*CtpP.MessagePoolP.PoolP*/PoolP__0__Pool__get(void )
#line 88
{
  if (/*CtpP.MessagePoolP.PoolP*/PoolP__0__free) {
      /*CtpP.MessagePoolP.PoolP*/PoolP__0__pool_t *rval = /*CtpP.MessagePoolP.PoolP*/PoolP__0__queue[/*CtpP.MessagePoolP.PoolP*/PoolP__0__index];

#line 91
      /*CtpP.MessagePoolP.PoolP*/PoolP__0__queue[/*CtpP.MessagePoolP.PoolP*/PoolP__0__index] = (void *)0;
      /*CtpP.MessagePoolP.PoolP*/PoolP__0__free--;
      /*CtpP.MessagePoolP.PoolP*/PoolP__0__index++;
      if (/*CtpP.MessagePoolP.PoolP*/PoolP__0__index == 12) {
          /*CtpP.MessagePoolP.PoolP*/PoolP__0__index = 0;
        }
      ;
      return rval;
    }
  return (void *)0;
}

# 97 "/opt/tinyos-2.1.1/tos/interfaces/Pool.nc"
inline static /*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__t * /*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__get(void ){
#line 97
  nx_struct message_t *__nesc_result;
#line 97

#line 97
  __nesc_result = /*CtpP.MessagePoolP.PoolP*/PoolP__0__Pool__get();
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 88 "/opt/tinyos-2.1.1/tos/system/PoolP.nc"
static inline /*CtpP.QEntryPoolP.PoolP*/PoolP__1__pool_t */*CtpP.QEntryPoolP.PoolP*/PoolP__1__Pool__get(void )
#line 88
{
  if (/*CtpP.QEntryPoolP.PoolP*/PoolP__1__free) {
      /*CtpP.QEntryPoolP.PoolP*/PoolP__1__pool_t *rval = /*CtpP.QEntryPoolP.PoolP*/PoolP__1__queue[/*CtpP.QEntryPoolP.PoolP*/PoolP__1__index];

#line 91
      /*CtpP.QEntryPoolP.PoolP*/PoolP__1__queue[/*CtpP.QEntryPoolP.PoolP*/PoolP__1__index] = (void *)0;
      /*CtpP.QEntryPoolP.PoolP*/PoolP__1__free--;
      /*CtpP.QEntryPoolP.PoolP*/PoolP__1__index++;
      if (/*CtpP.QEntryPoolP.PoolP*/PoolP__1__index == 12) {
          /*CtpP.QEntryPoolP.PoolP*/PoolP__1__index = 0;
        }
      ;
      return rval;
    }
  return (void *)0;
}

# 97 "/opt/tinyos-2.1.1/tos/interfaces/Pool.nc"
inline static /*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__t * /*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__get(void ){
#line 97
  struct __nesc_unnamed4300 *__nesc_result;
#line 97

#line 97
  __nesc_result = /*CtpP.QEntryPoolP.PoolP*/PoolP__1__Pool__get();
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 75 "/opt/tinyos-2.1.1/tos/system/PoolP.nc"
static inline bool /*CtpP.QEntryPoolP.PoolP*/PoolP__1__Pool__empty(void )
#line 75
{
  ;
  return /*CtpP.QEntryPoolP.PoolP*/PoolP__1__free == 0;
}

# 61 "/opt/tinyos-2.1.1/tos/interfaces/Pool.nc"
inline static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__empty(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = /*CtpP.QEntryPoolP.PoolP*/PoolP__1__Pool__empty();
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 75 "/opt/tinyos-2.1.1/tos/system/PoolP.nc"
static inline bool /*CtpP.MessagePoolP.PoolP*/PoolP__0__Pool__empty(void )
#line 75
{
  ;
  return /*CtpP.MessagePoolP.PoolP*/PoolP__0__free == 0;
}

# 61 "/opt/tinyos-2.1.1/tos/interfaces/Pool.nc"
inline static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__empty(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = /*CtpP.MessagePoolP.PoolP*/PoolP__0__Pool__empty();
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 584 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline message_t * /*CtpP.Forwarder*/CtpForwardingEngineP__0__forward(message_t * m)
#line 584
{
  if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__empty()) {
      ;

      /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_MSG_POOL_EMPTY);
    }
  else {
#line 590
    if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__empty()) {
        ;


        /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_QENTRY_POOL_EMPTY);
      }
    else {
        message_t *newMsg;
        fe_queue_entry_t *qe;
        uint16_t gradient;

        qe = /*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__get();
        if (qe == (void *)0) {
            /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_GET_MSGPOOL_ERR);
            return m;
          }

        newMsg = /*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__get();
        if (newMsg == (void *)0) {
            /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_GET_QEPOOL_ERR);
            return m;
          }

        memset(newMsg, 0, sizeof(message_t ));
        memset(m->metadata, 0, sizeof(message_metadata_t ));

        qe->msg = m;
        qe->client = 0xff;
        qe->retries = MAX_RETRIES;


        if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__enqueue(qe) == SUCCESS) {
            ;

            if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__getEtx(&gradient) == SUCCESS) {

                if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getEtx(m) <= gradient) {




                    /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__triggerImmediateRouteUpdate();
                    /*CtpP.Forwarder*/CtpForwardingEngineP__0__startRetxmitTimer(LOOPY_WINDOW, LOOPY_OFFSET);
                    /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEventMsg(NET_C_FE_LOOP_DETECTED, 
                    /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getSequenceNumber(m), 
                    /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getOrigin(m), 
                    /*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__destination(m));
                  }
              }

            if (!/*CtpP.Forwarder*/CtpForwardingEngineP__0__RetxmitTimer__isRunning()) {


                ;
                /*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask__postTask();
              }


            return newMsg;
          }
        else 
#line 649
          {

            if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__put(newMsg) != SUCCESS) {
              /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_PUT_MSGPOOL_ERR);
              }
#line 653
            if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__put(qe) != SUCCESS) {
              /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_PUT_QEPOOL_ERR);
              }
          }
      }
    }



  /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_SEND_QUEUE_FULL);
  return m;
}

#line 860
static inline 
#line 859
bool 
/*CtpP.Forwarder*/CtpForwardingEngineP__0__Intercept__default__forward(collection_id_t collectid, message_t *msg, void *payload, 
uint8_t len)
#line 861
{
  return TRUE;
}

# 31 "/opt/tinyos-2.1.1/tos/interfaces/Intercept.nc"
inline static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__Intercept__forward(collection_id_t arg_0x2b99bde92da8, message_t * msg, void * payload, uint8_t len){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
    __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__Intercept__default__forward(arg_0x2b99bde92da8, msg, payload, len);
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 829 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__matchInstance(message_t *m1, message_t *m2)
#line 829
{
  return /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getOrigin(m1) == /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getOrigin(m2) && 
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getSequenceNumber(m1) == /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getSequenceNumber(m2) && 
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getThl(m1) == /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getThl(m2) && 
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getType(m1) == /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getType(m2);
}

# 112 "/opt/tinyos-2.1.1/tos/system/QueueC.nc"
static inline /*CtpP.SendQueueP*/QueueC__0__queue_t /*CtpP.SendQueueP*/QueueC__0__Queue__element(uint8_t idx)
#line 112
{
  idx += /*CtpP.SendQueueP*/QueueC__0__head;
  if (idx >= 13) {
      idx -= 13;
    }
  return /*CtpP.SendQueueP*/QueueC__0__queue[idx];
}

# 101 "/opt/tinyos-2.1.1/tos/interfaces/Queue.nc"
inline static /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__t  /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__element(uint8_t idx){
#line 101
  struct __nesc_unnamed4300 *__nesc_result;
#line 101

#line 101
  __nesc_result = /*CtpP.SendQueueP*/QueueC__0__Queue__element(idx);
#line 101

#line 101
  return __nesc_result;
#line 101
}
#line 101
# 57 "/opt/tinyos-2.1.1/tos/system/QueueC.nc"
static inline uint8_t /*CtpP.SendQueueP*/QueueC__0__Queue__size(void )
#line 57
{
  return /*CtpP.SendQueueP*/QueueC__0__size;
}

# 58 "/opt/tinyos-2.1.1/tos/interfaces/Queue.nc"
inline static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__size(void ){
#line 58
  unsigned char __nesc_result;
#line 58

#line 58
  __nesc_result = /*CtpP.SendQueueP*/QueueC__0__Queue__size();
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 113 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline uint8_t CC2420ActiveMessageP__AMSend__maxPayloadLength(am_id_t id)
#line 113
{
  return CC2420ActiveMessageP__Packet__maxPayloadLength();
}

# 112 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__maxPayloadLength(am_id_t arg_0x2b99bcb95340){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = CC2420ActiveMessageP__AMSend__maxPayloadLength(arg_0x2b99bcb95340);
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 199 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static inline uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__maxPayloadLength(uint8_t id)
#line 199
{
  return /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__maxPayloadLength(0);
}

# 101 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static uint8_t /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__maxPayloadLength(void ){
#line 101
  unsigned char __nesc_result;
#line 101

#line 101
  __nesc_result = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__maxPayloadLength(0U);
#line 101

#line 101
  return __nesc_result;
#line 101
}
#line 101
# 61 "/opt/tinyos-2.1.1/tos/system/AMQueueEntryP.nc"
static inline uint8_t /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__maxPayloadLength(void )
#line 61
{
  return /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__Send__maxPayloadLength();
}

# 112 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubSend__maxPayloadLength(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = /*CtpP.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__2__AMSend__maxPayloadLength();
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 811 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__setThl(message_t *msg, uint8_t thl)
#line 811
{
#line 811
  __nesc_hton_uint8(/*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(msg)->thl.data, thl);
}

#line 674
static inline message_t *
/*CtpP.Forwarder*/CtpForwardingEngineP__0__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 675
{
  collection_id_t collectid;
  bool duplicate = FALSE;
  fe_queue_entry_t *qe;
  uint8_t i;
#line 679
  uint8_t thl;


  collectid = /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getType(msg);



  thl = /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getThl(msg);
  thl++;
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__setThl(msg, thl);

  /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEventMsg(NET_C_FE_RCV_MSG, 
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getSequenceNumber(msg), 
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getOrigin(msg), 
  thl--);
  if (len > /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubSend__maxPayloadLength()) {
      return msg;
    }



  if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__SentCache__lookup(msg)) {
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_DUPLICATE_CACHE);
      return msg;
    }

  if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__size() > 0) {
      for (i = /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__size(); --i; ) {
          qe = /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__element(i);
          if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__matchInstance(qe->msg, msg)) {
              duplicate = TRUE;
              break;
            }
        }
    }

  if (duplicate) {
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_DUPLICATE_QUEUE);
      return msg;
    }
  else {

    if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__RootControl__isRoot()) {
      return /*CtpP.Forwarder*/CtpForwardingEngineP__0__Receive__receive(collectid, msg, 
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__getPayload(msg, /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__payloadLength(msg)), 
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__payloadLength(msg));
      }
    else {
      if (!/*CtpP.Forwarder*/CtpForwardingEngineP__0__Intercept__forward(collectid, msg, 
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__getPayload(msg, /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__payloadLength(msg)), 
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__payloadLength(msg))) {
        return msg;
        }
      else 
#line 731
        {
          ;
          return /*CtpP.Forwarder*/CtpForwardingEngineP__0__forward(msg);
        }
      }
    }
}

# 699 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline error_t /*CtpP.Router*/CtpRoutingEngineP__0__routingTableUpdateEntry(am_addr_t from, am_addr_t parent, uint16_t etx)
#line 699
{
  uint8_t idx;
  uint16_t linkEtx;

#line 702
  linkEtx = /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__getLinkQuality(from);

  idx = /*CtpP.Router*/CtpRoutingEngineP__0__routingTableFind(from);
  if (idx == 10) {




      ;
      return 0x0080;
    }
  else {
#line 713
    if (idx == /*CtpP.Router*/CtpRoutingEngineP__0__routingTableActive) {

        if (/*CtpP.Router*/CtpRoutingEngineP__0__passLinkEtxThreshold(linkEtx)) {
            { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 716
              {
                /*CtpP.Router*/CtpRoutingEngineP__0__routingTable[idx].neighbor = from;
                /*CtpP.Router*/CtpRoutingEngineP__0__routingTable[idx].info.parent = parent;
                /*CtpP.Router*/CtpRoutingEngineP__0__routingTable[idx].info.etx = etx;
                /*CtpP.Router*/CtpRoutingEngineP__0__routingTable[idx].info.haveHeard = 1;
                /*CtpP.Router*/CtpRoutingEngineP__0__routingTable[idx].info.congested = FALSE;
                /*CtpP.Router*/CtpRoutingEngineP__0__routingTableActive++;
              }
#line 723
              __nesc_atomic_end(__nesc_atomic); }
            ;
          }
        else 
#line 725
          {
            ;
          }
      }
    else 
#line 728
      {

        { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 730
          {
            /*CtpP.Router*/CtpRoutingEngineP__0__routingTable[idx].neighbor = from;
            /*CtpP.Router*/CtpRoutingEngineP__0__routingTable[idx].info.parent = parent;
            /*CtpP.Router*/CtpRoutingEngineP__0__routingTable[idx].info.etx = etx;
            /*CtpP.Router*/CtpRoutingEngineP__0__routingTable[idx].info.haveHeard = 1;
          }
#line 735
          __nesc_atomic_end(__nesc_atomic); }
        ;
      }
    }
#line 738
  return SUCCESS;
}

# 769 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__LinkEstimator__evicted(am_addr_t neighbor)
#line 769
{
}

# 61 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimator.nc"
inline static void LinkEstimatorP__LinkEstimator__evicted(am_addr_t neighbor){
#line 61
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__LinkEstimator__evicted(neighbor);
#line 61
  /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__evicted(neighbor);
#line 61
}
#line 61
# 457 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline error_t LinkEstimatorP__LinkEstimator__insertNeighbor(am_addr_t neighbor)
#line 457
{
  uint8_t nidx;

  nidx = LinkEstimatorP__findIdx(neighbor);
  if (nidx != LinkEstimatorP__INVALID_RVAL) {
      ;
      return SUCCESS;
    }

  nidx = LinkEstimatorP__findEmptyNeighborIdx();
  if (nidx != LinkEstimatorP__INVALID_RVAL) {
      ;
      LinkEstimatorP__initNeighborIdx(nidx, neighbor);
      return SUCCESS;
    }
  else 
#line 471
    {
      nidx = LinkEstimatorP__findWorstNeighborIdx(LinkEstimatorP__BEST_ETX);
      if (nidx != LinkEstimatorP__INVALID_RVAL) {
          ;

          LinkEstimatorP__LinkEstimator__evicted(LinkEstimatorP__NeighborTable[nidx].ll_addr);
          LinkEstimatorP__initNeighborIdx(nidx, neighbor);
          return SUCCESS;
        }
    }
  return 0x0080;
}

# 41 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimator.nc"
inline static error_t /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__insertNeighbor(am_addr_t neighbor){
#line 41
  unsigned char __nesc_result;
#line 41

#line 41
  __nesc_result = LinkEstimatorP__LinkEstimator__insertNeighbor(neighbor);
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 77 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*CtpP.Router*/CtpRoutingEngineP__0__AMPacket__source(message_t * amsg){
#line 77
  unsigned int __nesc_result;
#line 77

#line 77
  __nesc_result = CC2420ActiveMessageP__AMPacket__source(amsg);
#line 77

#line 77
  return __nesc_result;
#line 77
}
#line 77
# 452 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline message_t */*CtpP.Router*/CtpRoutingEngineP__0__BeaconReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 452
{
  am_addr_t from;
  ctp_routing_header_t *rcvBeacon;
  bool congested;


  if (len != sizeof(ctp_routing_header_t )) {
      ;




      return msg;
    }


  from = /*CtpP.Router*/CtpRoutingEngineP__0__AMPacket__source(msg);
  rcvBeacon = (ctp_routing_header_t *)payload;

  congested = /*CtpP.Router*/CtpRoutingEngineP__0__CtpRoutingPacket__getOption(msg, CTP_OPT_ECN);

  ;




  if (__nesc_ntoh_uint16(rcvBeacon->parent.data) != INVALID_ADDR) {



      if (__nesc_ntoh_uint16(rcvBeacon->etx.data) == 0) {
          ;
          /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__insertNeighbor(from);
          /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__pinNeighbor(from);
        }


      /*CtpP.Router*/CtpRoutingEngineP__0__routingTableUpdateEntry(from, __nesc_ntoh_uint16(rcvBeacon->parent.data), __nesc_ntoh_uint16(rcvBeacon->etx.data));
      /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__setNeighborCongested(from, congested);
    }

  if (/*CtpP.Router*/CtpRoutingEngineP__0__CtpRoutingPacket__getOption(msg, CTP_OPT_PULL)) {
      /*CtpP.Router*/CtpRoutingEngineP__0__resetInterval();
    }
  return msg;
}

# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
inline static message_t * LinkEstimatorP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = /*CtpP.Router*/CtpRoutingEngineP__0__BeaconReceive__receive(msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 41 "/opt/tinyos-2.1.1/tos/interfaces/Random.nc"
inline static uint16_t LinkEstimatorP__Random__rand16(void ){
#line 41
  unsigned int __nesc_result;
#line 41

#line 41
  __nesc_result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 237 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline uint8_t LinkEstimatorP__findRandomNeighborIdx(void )
#line 237
{
  uint8_t i;
  uint8_t cnt;
  uint8_t num_eligible_eviction;

  num_eligible_eviction = 0;
  for (i = 0; i < 10; i++) {
      if (LinkEstimatorP__NeighborTable[i].flags & VALID_ENTRY) {
          if (LinkEstimatorP__NeighborTable[i].flags & PINNED_ENTRY || 
          LinkEstimatorP__NeighborTable[i].flags & MATURE_ENTRY) {
            }
          else 
#line 247
            {
              num_eligible_eviction++;
            }
        }
    }

  if (num_eligible_eviction == 0) {
      return LinkEstimatorP__INVALID_RVAL;
    }

  cnt = LinkEstimatorP__Random__rand16() % num_eligible_eviction;

  for (i = 0; i < 10; i++) {
      if (! LinkEstimatorP__NeighborTable[i].flags & VALID_ENTRY) {
        continue;
        }
#line 262
      if (LinkEstimatorP__NeighborTable[i].flags & PINNED_ENTRY || 
      LinkEstimatorP__NeighborTable[i].flags & MATURE_ENTRY) {
        continue;
        }
#line 265
      if (cnt-- == 0) {
        return i;
        }
    }
#line 268
  return LinkEstimatorP__INVALID_RVAL;
}

# 136 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static am_id_t /*CtpP.Router*/CtpRoutingEngineP__0__AMPacket__type(message_t * amsg){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = CC2420ActiveMessageP__AMPacket__type(amsg);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 632 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline bool /*CtpP.Router*/CtpRoutingEngineP__0__CompareBit__shouldInsert(message_t *msg, void *payload, uint8_t len)
#line 632
{

  bool found = FALSE;
  uint16_t pathEtx;
  uint16_t neighEtx;
  int i;
  routing_table_entry *entry;
  ctp_routing_header_t *rcvBeacon;

  if (/*CtpP.Router*/CtpRoutingEngineP__0__AMPacket__type(msg) != AM_CTP_ROUTING || 
  len != sizeof(ctp_routing_header_t )) {
    return FALSE;
    }

  rcvBeacon = (ctp_routing_header_t *)payload;

  if (__nesc_ntoh_uint16(rcvBeacon->parent.data) == INVALID_ADDR) {
    return FALSE;
    }
  if (__nesc_ntoh_uint16(rcvBeacon->etx.data) == 0) {
      return TRUE;
    }

  pathEtx = __nesc_ntoh_uint16(rcvBeacon->etx.data);


  for (i = 0; i < /*CtpP.Router*/CtpRoutingEngineP__0__routingTableActive && !found; i++) {
      entry = &/*CtpP.Router*/CtpRoutingEngineP__0__routingTable[i];

      if (entry->neighbor == /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent) {
        continue;
        }
#line 663
      neighEtx = entry->info.etx;
      found |= pathEtx < neighEtx;
    }
  return found;
}

# 43 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CompareBit.nc"
inline static bool LinkEstimatorP__CompareBit__shouldInsert(message_t * msg, void * payload, uint8_t len){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*CtpP.Router*/CtpRoutingEngineP__0__CompareBit__shouldInsert(msg, payload, len);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 8 "/opt/tinyos-2.1.1/tos/lib/net/ctp/DummyActiveMessageP.nc"
static inline bool DummyActiveMessageP__LinkPacketMetadata__highChannelQuality(message_t *msg)
#line 8
{
  return 0;
}

# 47 "/opt/tinyos-2.1.1/tos/interfaces/LinkPacketMetadata.nc"
inline static bool LinkEstimatorP__LinkPacketMetadata__highChannelQuality(message_t * msg){
#line 47
  unsigned char __nesc_result;
#line 47

#line 47
  __nesc_result = DummyActiveMessageP__LinkPacketMetadata__highChannelQuality(msg);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 391 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline void LinkEstimatorP__print_neighbor_table(void )
#line 391
{
  uint8_t i;
  neighbor_table_entry_t *ne;

#line 394
  for (i = 0; i < 10; i++) {
      ne = &LinkEstimatorP__NeighborTable[i];
      if (ne->flags & VALID_ENTRY) {
          ;
        }
    }
}

# 77 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static am_addr_t LinkEstimatorP__SubAMPacket__source(message_t * amsg){
#line 77
  unsigned int __nesc_result;
#line 77

#line 77
  __nesc_result = CC2420ActiveMessageP__AMPacket__source(amsg);
#line 77

#line 77
  return __nesc_result;
#line 77
}
#line 77
#line 67
inline static am_addr_t LinkEstimatorP__SubAMPacket__destination(message_t * amsg){
#line 67
  unsigned int __nesc_result;
#line 67

#line 67
  __nesc_result = CC2420ActiveMessageP__AMPacket__destination(amsg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 586 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline void LinkEstimatorP__processReceivedMessage(message_t * msg, void * payload, uint8_t len)
#line 586
{
  uint8_t nidx;
  uint8_t num_entries;

  ;
  LinkEstimatorP__print_packet(msg, len);

  if (LinkEstimatorP__SubAMPacket__destination(msg) == AM_BROADCAST_ADDR) {
      linkest_header_t *hdr = LinkEstimatorP__getHeader(msg);
      am_addr_t ll_addr;

      ll_addr = LinkEstimatorP__SubAMPacket__source(msg);

      ;

      num_entries = __nesc_ntoh_uint8(hdr->flags.data) & NUM_ENTRIES_FLAG;
      LinkEstimatorP__print_neighbor_table();
#line 618
      nidx = LinkEstimatorP__findIdx(ll_addr);
      if (nidx != LinkEstimatorP__INVALID_RVAL) {
          ;
          LinkEstimatorP__updateNeighborEntryIdx(nidx, __nesc_ntoh_uint8(hdr->seq.data));
        }
      else 
#line 622
        {
          nidx = LinkEstimatorP__findEmptyNeighborIdx();
          if (nidx != LinkEstimatorP__INVALID_RVAL) {
              ;
              LinkEstimatorP__initNeighborIdx(nidx, ll_addr);
              LinkEstimatorP__updateNeighborEntryIdx(nidx, __nesc_ntoh_uint8(hdr->seq.data));
            }
          else 
#line 628
            {
              nidx = LinkEstimatorP__findWorstNeighborIdx(LinkEstimatorP__EVICT_ETX_THRESHOLD);
              if (nidx != LinkEstimatorP__INVALID_RVAL) {
                  ;

                  LinkEstimatorP__LinkEstimator__evicted(LinkEstimatorP__NeighborTable[nidx].ll_addr);
                  LinkEstimatorP__initNeighborIdx(nidx, ll_addr);
                }
              else 
#line 635
                {
                  ;





                  if (LinkEstimatorP__LinkPacketMetadata__highChannelQuality(msg)) {
                      if (LinkEstimatorP__CompareBit__shouldInsert(msg, 
                      LinkEstimatorP__Packet__getPayload(msg, LinkEstimatorP__Packet__payloadLength(msg)), 
                      LinkEstimatorP__Packet__payloadLength(msg))) {
                          nidx = LinkEstimatorP__findRandomNeighborIdx();
                          if (nidx != LinkEstimatorP__INVALID_RVAL) {
                              LinkEstimatorP__LinkEstimator__evicted(LinkEstimatorP__NeighborTable[nidx].ll_addr);
                              LinkEstimatorP__initNeighborIdx(nidx, ll_addr);
                            }
                        }
                    }
                }
            }
        }
    }
}





static inline message_t *LinkEstimatorP__SubReceive__receive(message_t *msg, 
void *payload, 
uint8_t len)
#line 665
{
  ;
  LinkEstimatorP__processReceivedMessage(msg, payload, len);
  return LinkEstimatorP__Receive__receive(msg, 
  LinkEstimatorP__Packet__getPayload(msg, LinkEstimatorP__Packet__payloadLength(msg)), 
  LinkEstimatorP__Packet__payloadLength(msg));
}

# 266 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline message_t *CC2420ActiveMessageP__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 266
{
  return msg;
}

# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
inline static message_t * CC2420ActiveMessageP__Receive__receive(am_id_t arg_0x2b99bddf5908, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  switch (arg_0x2b99bddf5908) {
#line 67
    case 112:
#line 67
      __nesc_result = LinkEstimatorP__SubReceive__receive(msg, payload, len);
#line 67
      break;
#line 67
    case 113:
#line 67
      __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubReceive__receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      __nesc_result = CC2420ActiveMessageP__Receive__default__receive(arg_0x2b99bddf5908, msg, payload, len);
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
# 61 "/opt/tinyos-2.1.1/tos/system/ActiveMessageAddressC.nc"
static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void )
#line 61
{
  return ActiveMessageAddressC__amAddress();
}

# 50 "/opt/tinyos-2.1.1/tos/interfaces/ActiveMessageAddress.nc"
inline static am_addr_t CC2420ActiveMessageP__ActiveMessageAddress__amAddress(void ){
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
# 122 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline am_addr_t CC2420ActiveMessageP__AMPacket__address(void )
#line 122
{
  return CC2420ActiveMessageP__ActiveMessageAddress__amAddress();
}

#line 146
static inline bool CC2420ActiveMessageP__AMPacket__isForMe(message_t *amsg)
#line 146
{
  return CC2420ActiveMessageP__AMPacket__destination(amsg) == CC2420ActiveMessageP__AMPacket__address() || 
  CC2420ActiveMessageP__AMPacket__destination(amsg) == AM_BROADCAST_ADDR;
}

#line 206
static inline message_t *CC2420ActiveMessageP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 206
{

  if (CC2420ActiveMessageP__AMPacket__isForMe(msg)) {
      return CC2420ActiveMessageP__Receive__receive(CC2420ActiveMessageP__AMPacket__type(msg), msg, payload, len);
    }
  else {
      return CC2420ActiveMessageP__Snoop__receive(CC2420ActiveMessageP__AMPacket__type(msg), msg, payload, len);
    }
}

# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
inline static message_t * CC2420TinyosNetworkP__ActiveReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = CC2420ActiveMessageP__SubReceive__receive(msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 75 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Packet.nc"
inline static uint8_t CC2420TinyosNetworkP__CC2420Packet__getNetwork(message_t *p_msg){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420PacketP__CC2420Packet__getNetwork(p_msg);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 47 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(message_t * msg){
#line 47
  nx_struct cc2420_metadata_t *__nesc_result;
#line 47

#line 47
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 127 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline message_t *CC2420TinyosNetworkP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 127
{

  if (! __nesc_ntoh_int8(CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(msg)->crc.data)) {
      return msg;
    }

  if (CC2420TinyosNetworkP__CC2420Packet__getNetwork(msg) == 0x3f) {
      return CC2420TinyosNetworkP__ActiveReceive__receive(msg, payload, len);
    }
  else 
#line 135
    {
      cc2420_header_t *hdr = CC2420TinyosNetworkP__CC2420PacketBody__getHeader(msg);

#line 137
      return CC2420TinyosNetworkP__BareReceive__receive(msg, & hdr->network, len + AM_OVERHEAD);
    }
}

# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
inline static message_t * UniqueReceiveP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = CC2420TinyosNetworkP__SubReceive__receive(msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 137 "/opt/tinyos-2.1.1/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn)
#line 137
{
  uint8_t element = UniqueReceiveP__recycleSourceElement;
  bool increment = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 141
    {
      if (element == UniqueReceiveP__INVALID_ELEMENT || UniqueReceiveP__writeIndex == element) {

          element = UniqueReceiveP__writeIndex;
          increment = TRUE;
        }

      UniqueReceiveP__receivedMessages[element].source = msgSource;
      UniqueReceiveP__receivedMessages[element].dsn = msgDsn;
      if (increment) {
          UniqueReceiveP__writeIndex++;
          UniqueReceiveP__writeIndex %= 4;
        }
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}


static inline message_t *UniqueReceiveP__DuplicateReceive__default__receive(message_t *msg, void *payload, uint8_t len)
#line 158
{
  return msg;
}

# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
inline static message_t * UniqueReceiveP__DuplicateReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = UniqueReceiveP__DuplicateReceive__default__receive(msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 111 "/opt/tinyos-2.1.1/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn)
#line 111
{
  int i;

#line 113
  UniqueReceiveP__recycleSourceElement = UniqueReceiveP__INVALID_ELEMENT;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 115
    {
      for (i = 0; i < 4; i++) {
          if (UniqueReceiveP__receivedMessages[i].source == msgSource) {
              if (UniqueReceiveP__receivedMessages[i].dsn == msgDsn) {

                  {
                    unsigned char __nesc_temp = 
#line 120
                    TRUE;

                    {
#line 120
                      __nesc_atomic_end(__nesc_atomic); 
#line 120
                      return __nesc_temp;
                    }
                  }
                }
#line 123
              UniqueReceiveP__recycleSourceElement = i;
            }
        }
    }
#line 126
    __nesc_atomic_end(__nesc_atomic); }

  return FALSE;
}

# 42 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * UniqueReceiveP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 85 "/opt/tinyos-2.1.1/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline message_t *UniqueReceiveP__SubReceive__receive(message_t *msg, void *payload, 
uint8_t len)
#line 86
{
  uint16_t msgSource = __nesc_ntoh_leuint16(UniqueReceiveP__CC2420PacketBody__getHeader(msg)->src.data);
  uint8_t msgDsn = __nesc_ntoh_leuint8(UniqueReceiveP__CC2420PacketBody__getHeader(msg)->dsn.data);

  if (UniqueReceiveP__hasSeen(msgSource, msgDsn)) {
      return UniqueReceiveP__DuplicateReceive__receive(msg, payload, len);
    }
  else {
      UniqueReceiveP__insert(msgSource, msgDsn);
      return UniqueReceiveP__Receive__receive(msg, payload, len);
    }
}

# 67 "/opt/tinyos-2.1.1/tos/interfaces/Receive.nc"
inline static message_t * CC2420ReceiveP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = UniqueReceiveP__SubReceive__receive(msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 64 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static uint16_t CC2420ReceiveP__CC2420Config__getShortAddr(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = CC2420ControlP__CC2420Config__getShortAddr();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 332 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void )
#line 332
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 333
    {
      unsigned char __nesc_temp = 
#line 333
      CC2420ControlP__addressRecognition;

      {
#line 333
        __nesc_atomic_end(__nesc_atomic); 
#line 333
        return __nesc_temp;
      }
    }
#line 335
    __nesc_atomic_end(__nesc_atomic); }
}

# 86 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled(void ){
#line 86
  unsigned char __nesc_result;
#line 86

#line 86
  __nesc_result = CC2420ControlP__CC2420Config__isAddressRecognitionEnabled();
#line 86

#line 86
  return __nesc_result;
#line 86
}
#line 86
# 42 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420ReceiveP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 819 "/opt/tinyos-2.1.1/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline bool CC2420ReceiveP__passesAddressCheck(message_t *msg)
#line 819
{
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(msg);

  if (!CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled()) {
      return TRUE;
    }

  return __nesc_ntoh_leuint16(header->dest.data) == CC2420ReceiveP__CC2420Config__getShortAddr()
   || __nesc_ntoh_leuint16(header->dest.data) == AM_BROADCAST_ADDR;
}

# 47 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420ReceiveP__CC2420PacketBody__getMetadata(message_t * msg){
#line 47
  nx_struct cc2420_metadata_t *__nesc_result;
#line 47

#line 47
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 671 "/opt/tinyos-2.1.1/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__receiveDone_task__runTask(void )
#line 671
{
  cc2420_metadata_t *metadata = CC2420ReceiveP__CC2420PacketBody__getMetadata(CC2420ReceiveP__m_p_rx_buf);
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf);
  uint8_t length = __nesc_ntoh_leuint8(header->length.data);
  uint8_t tmpLen __attribute((unused))  = sizeof(message_t ) - ((size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

  __nesc_hton_int8(metadata->crc.data, buf[length] >> 7);
  __nesc_hton_uint8(metadata->lqi.data, buf[length] & 0x7f);
  __nesc_hton_uint8(metadata->rssi.data, buf[length - 1]);

  if (CC2420ReceiveP__passesAddressCheck(CC2420ReceiveP__m_p_rx_buf) && length >= CC2420_SIZE) {
#line 696
      CC2420ReceiveP__m_p_rx_buf = CC2420ReceiveP__Receive__receive(CC2420ReceiveP__m_p_rx_buf, CC2420ReceiveP__m_p_rx_buf->data, 
      length - CC2420_SIZE);
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 699
    CC2420ReceiveP__receivingPacket = FALSE;
#line 699
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ReceiveP__waitForNextPacket();
}

# 300 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline uint16_t LinkEstimatorP__computeETX(uint8_t q1)
#line 300
{
  uint16_t q;

#line 302
  if (q1 > 0) {
      q = 2500 / q1;
      if (q > 250) {
          q = LinkEstimatorP__VERY_LARGE_ETX_VALUE;
        }
      return q;
    }
  else 
#line 308
    {
      return LinkEstimatorP__VERY_LARGE_ETX_VALUE;
    }
}



static inline void LinkEstimatorP__updateNeighborTableEst(am_addr_t n)
#line 315
{
  uint8_t i;
#line 316
  uint8_t totalPkt;
  neighbor_table_entry_t *ne;
  uint8_t newEst;
  uint8_t minPkt;

  minPkt = LinkEstimatorP__BLQ_PKT_WINDOW;
  ;
  for (i = 0; i < 10; i++) {
      ne = &LinkEstimatorP__NeighborTable[i];
      if (ne->ll_addr == n) {
          if (ne->flags & VALID_ENTRY) {
              ;
              ne->flags |= MATURE_ENTRY;
              totalPkt = ne->rcvcnt + ne->failcnt;
              ;
              if (totalPkt < minPkt) {
                  totalPkt = minPkt;
                }
              if (totalPkt == 0) {
                  ne->inquality = LinkEstimatorP__ALPHA * ne->inquality / 10;
                }
              else 
#line 336
                {
                  newEst = 250UL * ne->rcvcnt / totalPkt;
                  ;
                  ne->inquality = (LinkEstimatorP__ALPHA * ne->inquality + (10 - LinkEstimatorP__ALPHA) * newEst) / 10;
                }
              ne->rcvcnt = 0;
              ne->failcnt = 0;
              LinkEstimatorP__updateETX(ne, LinkEstimatorP__computeETX(ne->inquality));
            }
          else 
#line 344
            {
              ;
            }
        }
    }
}

# 742 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline error_t /*CtpP.Router*/CtpRoutingEngineP__0__routingTableEvict(am_addr_t neighbor)
#line 742
{
  uint8_t idx;
#line 743
  uint8_t i;

#line 744
  idx = /*CtpP.Router*/CtpRoutingEngineP__0__routingTableFind(neighbor);
  if (idx == /*CtpP.Router*/CtpRoutingEngineP__0__routingTableActive) {
    return 0x0080;
    }
#line 747
  /*CtpP.Router*/CtpRoutingEngineP__0__routingTableActive--;
  for (i = idx; i < /*CtpP.Router*/CtpRoutingEngineP__0__routingTableActive; i++) {
      /*CtpP.Router*/CtpRoutingEngineP__0__routingTable[i] = /*CtpP.Router*/CtpRoutingEngineP__0__routingTable[i + 1];
    }
  return SUCCESS;
}

# 67 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
inline static uint8_t LinkEstimatorP__SubPacket__payloadLength(message_t * msg){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = CC2420ActiveMessageP__Packet__payloadLength(msg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 575 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline uint8_t LinkEstimatorP__Send__maxPayloadLength(void )
#line 575
{
  return LinkEstimatorP__Packet__maxPayloadLength();
}

# 112 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static uint8_t /*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__maxPayloadLength(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = LinkEstimatorP__Send__maxPayloadLength();
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 579 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline void *LinkEstimatorP__Send__getPayload(message_t *msg, uint8_t len)
#line 579
{
  return LinkEstimatorP__Packet__getPayload(msg, len);
}

# 124 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static void * /*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__getPayload(message_t * msg, uint8_t len){
#line 124
  void *__nesc_result;
#line 124

#line 124
  __nesc_result = LinkEstimatorP__Send__getPayload(msg, len);
#line 124

#line 124
  return __nesc_result;
#line 124
}
#line 124
# 445 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline ctp_routing_header_t */*CtpP.Router*/CtpRoutingEngineP__0__getHeader(message_t * m)
#line 445
{
  return (ctp_routing_header_t *)/*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__getPayload(m, /*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__maxPayloadLength());
}

# 61 "/opt/tinyos-2.1.1/tos/system/QueueC.nc"
static inline uint8_t /*CtpP.SendQueueP*/QueueC__0__Queue__maxSize(void )
#line 61
{
  return 13;
}

# 178 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline uint8_t CC2420SpiP__Resource__isOwner(uint8_t id)
#line 178
{
  /* atomic removed: atomic calls only */
#line 179
  {
    unsigned char __nesc_temp = 
#line 179
    CC2420SpiP__m_holder == id;

#line 179
    return __nesc_temp;
  }
}

# 118 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static bool CC2420ReceiveP__SpiResource__isOwner(void ){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  __nesc_result = CC2420SpiP__Resource__isOwner(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
#line 87
inline static error_t CC2420ReceiveP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
#line 78
inline static error_t CC2420ReceiveP__SpiResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420SpiP__grant__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420SpiP__grant);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 184 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__SpiResource__granted(void )
#line 184
{
  CC2420SpiP__grant__postTask();
}

# 151 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__default__granted(uint8_t id)
#line 151
{
}

# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__granted(uint8_t arg_0x2b99bd7ece00){
#line 92
  switch (arg_0x2b99bd7ece00) {
#line 92
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID:
#line 92
      CC2420SpiP__SpiResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__default__granted(arg_0x2b99bd7ece00);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 127 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__granted(uint8_t id)
#line 127
{
  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__granted(id);
}

# 199 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id)
#line 199
{
}

# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(uint8_t arg_0x2b99bd1c2d40){
#line 92
  switch (arg_0x2b99bd1c2d40) {
#line 92
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsciC*/Msp430UsciB0C__0__CLIENT_ID:
#line 92
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__granted(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(arg_0x2b99bd1c2d40);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 187 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void )
#line 187
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;
      /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY;
    }
#line 191
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__resId);
}

# 222 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error)
#line 222
{
}

# 71 "/opt/tinyos-2.1.1/tos/interfaces/SpiPacket.nc"
inline static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__sendDone(uint8_t arg_0x2b99bd7e7368, uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
#line 71
  switch (arg_0x2b99bd7e7368) {
#line 71
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID:
#line 71
      CC2420SpiP__SpiPacket__sendDone(txBuf, rxBuf, len, error);
#line 71
      break;
#line 71
    default:
#line 71
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__default__sendDone(arg_0x2b99bd7e7368, txBuf, rxBuf, len, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 215 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone(void )
#line 215
{
  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__sendDone(/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_client, /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_tx_buf, /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_rx_buf, /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_len, 
  SUCCESS);
}

#line 198
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task__runTask(void )
#line 198
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 199
    /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone();
#line 199
    __nesc_atomic_end(__nesc_atomic); }
}

# 484 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 485
{
}

# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420ReceiveP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 29 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__set(void ){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 29
}
#line 29
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420ReceiveP__receiveDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420ReceiveP__receiveDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 47 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420TransmitP__CC2420PacketBody__getMetadata(message_t * msg){
#line 47
  nx_struct cc2420_metadata_t *__nesc_result;
#line 47

#line 47
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 387 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t *ack_msg)
#line 387
{
  cc2420_header_t *ack_header;
  cc2420_header_t *msg_header;
  cc2420_metadata_t *msg_metadata;
  uint8_t *ack_buf;
  uint8_t length;

  if (type == IEEE154_TYPE_ACK && CC2420TransmitP__m_msg) {
      ack_header = CC2420TransmitP__CC2420PacketBody__getHeader(ack_msg);
      msg_header = CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg);

      if (CC2420TransmitP__m_state == CC2420TransmitP__S_ACK_WAIT && __nesc_ntoh_leuint8(msg_header->dsn.data) == __nesc_ntoh_leuint8(ack_header->dsn.data)) {
          CC2420TransmitP__BackoffTimer__stop();

          msg_metadata = CC2420TransmitP__CC2420PacketBody__getMetadata(CC2420TransmitP__m_msg);
          ack_buf = (uint8_t *)ack_header;
          length = __nesc_ntoh_leuint8(ack_header->length.data);

          __nesc_hton_int8(msg_metadata->ack.data, TRUE);
          __nesc_hton_uint8(msg_metadata->rssi.data, ack_buf[length - 1]);
          __nesc_hton_uint8(msg_metadata->lqi.data, ack_buf[length] & 0x7f);
          CC2420TransmitP__signalDone(SUCCESS);
        }
    }
}

# 63 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420ReceiveP__CC2420Receive__receive(uint8_t type, message_t * message){
#line 63
  CC2420TransmitP__CC2420Receive__receive(type, message);
#line 63
}
#line 63
# 59 "/opt/tinyos-2.1.1/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420ReceiveP__PacketTimeStamp__clear(message_t * msg){
#line 59
  CC2420PacketP__PacketTimeStamp32khz__clear(msg);
#line 59
}
#line 59








inline static void CC2420ReceiveP__PacketTimeStamp__set(message_t * msg, CC2420ReceiveP__PacketTimeStamp__size_type value){
#line 67
  CC2420PacketP__PacketTimeStamp32khz__set(msg, value);
#line 67
}
#line 67
# 48 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )32U & (0x01 << 2);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__getRaw() != 0;
}

# 59 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*HplMsp430GeneralIOC.P12*/HplMsp430GeneralIOP__2__IO__get();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get();
}

# 32 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFOP__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 48 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )32U & (0x01 << 3);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw() != 0;
}

# 59 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 40 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get();
}

# 32 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFO__get(void ){
#line 32
  unsigned char __nesc_result;
#line 32

#line 32
  __nesc_result = /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get();
#line 32

#line 32
  return __nesc_result;
#line 32
}
#line 32
# 209 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 210
{
  return CC2420SpiP__SpiPacket__send((void *)0, data, len);
}

# 62 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static error_t CC2420ReceiveP__RXFIFO__continueRead(uint8_t * data, uint8_t length){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = CC2420SpiP__Fifo__continueRead(CC2420_RXFIFO, data, length);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
#line 51
inline static cc2420_status_t CC2420ReceiveP__RXFIFO__beginRead(uint8_t * data, uint8_t length){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420SpiP__Fifo__beginRead(CC2420_RXFIFO, data, length);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 30 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__clr(void ){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 30
}
#line 30
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ReceiveP__SACK__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SACK);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 359 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void )
#line 359
{
  /* atomic removed: atomic calls only */
#line 360
  {
    unsigned char __nesc_temp = 
#line 360
    CC2420ControlP__hwAutoAckDefault;

#line 360
    return __nesc_temp;
  }
}

# 105 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isHwAutoAckDefault(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = CC2420ControlP__CC2420Config__isHwAutoAckDefault();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 366 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void )
#line 366
{
  /* atomic removed: atomic calls only */
#line 367
  {
    unsigned char __nesc_temp = 
#line 367
    CC2420ControlP__autoAckEnabled;

#line 367
    return __nesc_temp;
  }
}

# 110 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isAutoAckEnabled(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420ControlP__CC2420Config__isAutoAckEnabled();
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 525 "/opt/tinyos-2.1.1/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error)
#line 526
{
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf);
  uint8_t tmpLen __attribute((unused))  = sizeof(message_t ) - ((size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

#line 530
  CC2420ReceiveP__rxFrameLength = buf[0];

  switch (CC2420ReceiveP__m_state) {

      case CC2420ReceiveP__S_RX_LENGTH: 
        CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_FCF;



      if (CC2420ReceiveP__rxFrameLength + 1 > CC2420ReceiveP__m_bytes_left) 



        {

          CC2420ReceiveP__flush();
        }
      else {
          if (!CC2420ReceiveP__FIFO__get() && !CC2420ReceiveP__FIFOP__get()) {
              CC2420ReceiveP__m_bytes_left -= CC2420ReceiveP__rxFrameLength + 1;
            }

          if (CC2420ReceiveP__rxFrameLength <= MAC_PACKET_SIZE) {
              if (CC2420ReceiveP__rxFrameLength > 0) {
                  if (CC2420ReceiveP__rxFrameLength > CC2420ReceiveP__SACK_HEADER_LENGTH) {

                      CC2420ReceiveP__RXFIFO__continueRead(buf + 1, CC2420ReceiveP__SACK_HEADER_LENGTH);
                    }
                  else {

                      CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;
                      CC2420ReceiveP__RXFIFO__continueRead(buf + 1, CC2420ReceiveP__rxFrameLength);
                    }
                }
              else {
                  /* atomic removed: atomic calls only */
                  CC2420ReceiveP__receivingPacket = FALSE;
                  CC2420ReceiveP__CSN__set();
                  CC2420ReceiveP__SpiResource__release();
                  CC2420ReceiveP__waitForNextPacket();
                }
            }
          else {

              CC2420ReceiveP__flush();
            }
        }
      break;

      case CC2420ReceiveP__S_RX_FCF: 
        CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;










      if (CC2420ReceiveP__CC2420Config__isAutoAckEnabled() && !CC2420ReceiveP__CC2420Config__isHwAutoAckDefault()) {



          if (((__nesc_ntoh_leuint16(
#line 592
          header->fcf.data) >> IEEE154_FCF_ACK_REQ) & 0x01) == 1
           && (__nesc_ntoh_leuint16(header->dest.data) == CC2420ReceiveP__CC2420Config__getShortAddr()
           || __nesc_ntoh_leuint16(header->dest.data) == AM_BROADCAST_ADDR)
           && ((__nesc_ntoh_leuint16(header->fcf.data) >> IEEE154_FCF_FRAME_TYPE) & 7) == IEEE154_TYPE_DATA) {

              CC2420ReceiveP__CSN__set();
              CC2420ReceiveP__CSN__clr();
              CC2420ReceiveP__SACK__strobe();
              CC2420ReceiveP__CSN__set();
              CC2420ReceiveP__CSN__clr();
              CC2420ReceiveP__RXFIFO__beginRead(buf + 1 + CC2420ReceiveP__SACK_HEADER_LENGTH, 
              CC2420ReceiveP__rxFrameLength - CC2420ReceiveP__SACK_HEADER_LENGTH);
              return;
            }
        }

      CC2420ReceiveP__RXFIFO__continueRead(buf + 1 + CC2420ReceiveP__SACK_HEADER_LENGTH, 
      CC2420ReceiveP__rxFrameLength - CC2420ReceiveP__SACK_HEADER_LENGTH);
      break;

      case CC2420ReceiveP__S_RX_PAYLOAD: 

        CC2420ReceiveP__CSN__set();
      if (!CC2420ReceiveP__m_missed_packets) {

          CC2420ReceiveP__SpiResource__release();
        }




      if ((((
#line 621
      CC2420ReceiveP__m_missed_packets && CC2420ReceiveP__FIFO__get()) || !CC2420ReceiveP__FIFOP__get())
       || !CC2420ReceiveP__m_timestamp_size)
       || CC2420ReceiveP__rxFrameLength <= 10) {
          CC2420ReceiveP__PacketTimeStamp__clear(CC2420ReceiveP__m_p_rx_buf);
        }
      else {
          if (CC2420ReceiveP__m_timestamp_size == 1) {
            CC2420ReceiveP__PacketTimeStamp__set(CC2420ReceiveP__m_p_rx_buf, CC2420ReceiveP__m_timestamp_queue[CC2420ReceiveP__m_timestamp_head]);
            }
#line 629
          CC2420ReceiveP__m_timestamp_head = (CC2420ReceiveP__m_timestamp_head + 1) % CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE;
          CC2420ReceiveP__m_timestamp_size--;

          if (CC2420ReceiveP__m_timestamp_size > 0) {
              CC2420ReceiveP__PacketTimeStamp__clear(CC2420ReceiveP__m_p_rx_buf);
              CC2420ReceiveP__m_timestamp_head = 0;
              CC2420ReceiveP__m_timestamp_size = 0;
            }
        }



      if (buf[CC2420ReceiveP__rxFrameLength] >> 7 && rx_buf) {
          uint8_t type = (__nesc_ntoh_leuint16(header->fcf.data) >> IEEE154_FCF_FRAME_TYPE) & 7;

#line 643
          CC2420ReceiveP__CC2420Receive__receive(type, CC2420ReceiveP__m_p_rx_buf);
          if (type == IEEE154_TYPE_DATA) {
              CC2420ReceiveP__receiveDone_task__postTask();
              return;
            }
        }

      CC2420ReceiveP__waitForNextPacket();
      break;

      default: /* atomic removed: atomic calls only */
        CC2420ReceiveP__receivingPacket = FALSE;
      CC2420ReceiveP__CSN__set();
      CC2420ReceiveP__SpiResource__release();
      break;
    }
}

# 370 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error)
#line 370
{
}

# 71 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__readDone(uint8_t arg_0x2b99bd778328, uint8_t * data, uint8_t length, error_t error){
#line 71
  switch (arg_0x2b99bd778328) {
#line 71
    case CC2420_TXFIFO:
#line 71
      CC2420TransmitP__TXFIFO__readDone(data, length, error);
#line 71
      break;
#line 71
    case CC2420_RXFIFO:
#line 71
      CC2420ReceiveP__RXFIFO__readDone(data, length, error);
#line 71
      break;
#line 71
    default:
#line 71
      CC2420SpiP__Fifo__default__readDone(arg_0x2b99bd778328, data, length, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ReceiveP__SFLUSHRX__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SFLUSHRX);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 280 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__RadioBackoff__default__requestInitialBackoff(am_id_t id, 
message_t *msg)
#line 281
{
}

# 81 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420ActiveMessageP__RadioBackoff__requestInitialBackoff(am_id_t arg_0x2b99bddf1d60, message_t * msg){
#line 81
    CC2420ActiveMessageP__RadioBackoff__default__requestInitialBackoff(arg_0x2b99bddf1d60, msg);
#line 81
}
#line 81
# 228 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SubBackoff__requestInitialBackoff(message_t *msg)
#line 228
{
  CC2420ActiveMessageP__RadioBackoff__requestInitialBackoff(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.data), msg);
}

# 81 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__RadioBackoff__requestInitialBackoff(message_t * msg){
#line 81
  CC2420ActiveMessageP__SubBackoff__requestInitialBackoff(msg);
#line 81
}
#line 81
# 243 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__RadioBackoff__setInitialBackoff(uint16_t backoffTime)
#line 243
{
  CC2420TransmitP__myInitialBackoff = backoffTime + 1;
}

# 60 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__SubBackoff__setInitialBackoff(uint16_t backoffTime){
#line 60
  CC2420TransmitP__RadioBackoff__setInitialBackoff(backoffTime);
#line 60
}
#line 60
# 220 "/opt/tinyos-2.1.1/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__SubBackoff__requestInitialBackoff(message_t *msg)
#line 220
{
  CC2420CsmaP__SubBackoff__setInitialBackoff(CC2420CsmaP__Random__rand16()
   % (0x1F * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP__RadioBackoff__requestInitialBackoff(msg);
}

# 81 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420TransmitP__RadioBackoff__requestInitialBackoff(message_t * msg){
#line 81
  CC2420CsmaP__SubBackoff__requestInitialBackoff(msg);
#line 81
}
#line 81
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__sendDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__sendDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 202 "/opt/tinyos-2.1.1/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Transmit__sendDone(message_t *p_msg, error_t err)
#line 202
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 203
    CC2420CsmaP__sendErr = err;
#line 203
    __nesc_atomic_end(__nesc_atomic); }
  CC2420CsmaP__sendDone_task__postTask();
}

# 73 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
inline static void CC2420TransmitP__Send__sendDone(message_t * p_msg, error_t error){
#line 73
  CC2420CsmaP__CC2420Transmit__sendDone(p_msg, error);
#line 73
}
#line 73
# 452 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 453
{

  CC2420TransmitP__CSN__set();
  if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
      /* atomic removed: atomic calls only */
#line 457
      {
        CC2420TransmitP__CSN__clr();
        CC2420TransmitP__SFLUSHTX__strobe();
        CC2420TransmitP__CSN__set();
      }
      CC2420TransmitP__releaseSpiResource();
      CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
      CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
    }
  else {
#line 466
    if (!CC2420TransmitP__m_cca) {
        /* atomic removed: atomic calls only */
#line 467
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_BEGIN_TRANSMIT;
        }
        CC2420TransmitP__attemptSend();
      }
    else {
        CC2420TransmitP__releaseSpiResource();
        /* atomic removed: atomic calls only */
#line 474
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_SAMPLE_CCA;
        }

        CC2420TransmitP__RadioBackoff__requestInitialBackoff(CC2420TransmitP__m_msg);
        CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__myInitialBackoff);
      }
    }
}

# 663 "/opt/tinyos-2.1.1/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 663
{
}

# 373 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 373
{
}

# 91 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__writeDone(uint8_t arg_0x2b99bd778328, uint8_t * data, uint8_t length, error_t error){
#line 91
  switch (arg_0x2b99bd778328) {
#line 91
    case CC2420_TXFIFO:
#line 91
      CC2420TransmitP__TXFIFO__writeDone(data, length, error);
#line 91
      break;
#line 91
    case CC2420_RXFIFO:
#line 91
      CC2420ReceiveP__RXFIFO__writeDone(data, length, error);
#line 91
      break;
#line 91
    default:
#line 91
      CC2420SpiP__Fifo__default__writeDone(arg_0x2b99bd778328, data, length, error);
#line 91
      break;
#line 91
    }
#line 91
}
#line 91
# 55 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__RXCTRL1__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_RXCTRL1, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
inline static cc2420_status_t CC2420ControlP__IOCFG0__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_IOCFG0, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SXOSCON__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SXOSCON);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 134 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__enable(void )
#line 134
{
#line 134
  P1IE |= 1 << 4;
}

# 31 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable(void ){
#line 31
  HplMsp430InterruptP__Port14__enable();
#line 31
}
#line 31
# 186 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__edge(bool l2h)
#line 186
{
  /* atomic removed: atomic calls only */
#line 187
  {
    if (l2h) {
#line 188
      P1IES &= ~(1 << 4);
      }
    else {
#line 189
      P1IES |= 1 << 4;
      }
  }
}

# 56 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high){
#line 56
  HplMsp430InterruptP__Port14__edge(low_to_high);
#line 56
}
#line 56
# 150 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__clear(void )
#line 150
{
#line 150
  P1IFG &= ~(1 << 4);
}

# 41 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port14__clear();
#line 41
}
#line 41
# 142 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__disable(void )
#line 142
{
#line 142
  P1IE &= ~(1 << 4);
}

# 36 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable(void ){
#line 36
  HplMsp430InterruptP__Port14__disable();
#line 36
}
#line 36
# 58 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear();
  }
  return SUCCESS;
}

#line 41
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(bool rising)
#line 41
{
  /* atomic removed: atomic calls only */
#line 42
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(rising);
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable();
  }
  return SUCCESS;
}

static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void )
#line 50
{
  return /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(TRUE);
}

# 42 "/opt/tinyos-2.1.1/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void ){
#line 42
  unsigned char __nesc_result;
#line 42

#line 42
  __nesc_result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge();
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 55 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__IOCFG1__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_IOCFG1, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 207 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__startOscillator(void )
#line 207
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 208
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_VREG_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 210
            0x0080;

            {
#line 210
              __nesc_atomic_end(__nesc_atomic); 
#line 210
              return __nesc_temp;
            }
          }
        }
#line 213
      CC2420ControlP__m_state = CC2420ControlP__S_XOSC_STARTING;
      CC2420ControlP__IOCFG1__write(CC2420_SFDMUX_XOSC16M_STABLE << 
      CC2420_IOCFG1_CCAMUX);

      CC2420ControlP__InterruptCCA__enableRisingEdge();
      CC2420ControlP__SXOSCON__strobe();

      CC2420ControlP__IOCFG0__write((1 << CC2420_IOCFG0_FIFOP_POLARITY) | (
      127 << CC2420_IOCFG0_FIFOP_THR));

      CC2420ControlP__writeFsctrl();
      CC2420ControlP__writeMdmctrl0();

      CC2420ControlP__RXCTRL1__write(((((((1 << CC2420_RXCTRL1_RXBPF_LOCUR) | (
      1 << CC2420_RXCTRL1_LOW_LOWGAIN)) | (
      1 << CC2420_RXCTRL1_HIGH_HGM)) | (
      1 << CC2420_RXCTRL1_LNA_CAP_ARRAY)) | (
      1 << CC2420_RXCTRL1_RXMIX_TAIL)) | (
      1 << CC2420_RXCTRL1_RXMIX_VCM)) | (
      2 << CC2420_RXCTRL1_RXMIX_CURRENT));
    }
#line 233
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 71 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__startOscillator(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = CC2420ControlP__CC2420Power__startOscillator();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 211 "/opt/tinyos-2.1.1/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__Resource__granted(void )
#line 211
{
  CC2420CsmaP__CC2420Power__startOscillator();
}

# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static void CC2420ControlP__Resource__granted(void ){
#line 92
  CC2420CsmaP__Resource__granted();
#line 92
}
#line 92
# 30 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__clr(void ){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 30
}
#line 30
# 390 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__SpiResource__granted(void )
#line 390
{
  CC2420ControlP__CSN__clr();
  CC2420ControlP__Resource__granted();
}

# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420ControlP__syncDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420ControlP__syncDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 29 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__set(void ){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 29
}
#line 29
# 45 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SRXON__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SRXON);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
inline static cc2420_status_t CC2420ControlP__SRFOFF__strobe(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SRFOFF);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 376 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__SyncResource__granted(void )
#line 376
{
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRFOFF__strobe();
  CC2420ControlP__writeFsctrl();
  CC2420ControlP__writeMdmctrl0();
  CC2420ControlP__writeId();
  CC2420ControlP__CSN__set();
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRXON__strobe();
  CC2420ControlP__CSN__set();
  CC2420ControlP__SyncResource__release();
  CC2420ControlP__syncDone__postTask();
}

#line 509
static inline void CC2420ControlP__ReadRssi__default__readDone(error_t error, uint16_t data)
#line 509
{
}

# 63 "/opt/tinyos-2.1.1/tos/interfaces/Read.nc"
inline static void CC2420ControlP__ReadRssi__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val){
#line 63
  CC2420ControlP__ReadRssi__default__readDone(result, val);
#line 63
}
#line 63
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__RssiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.RssiResource*/CC2420SpiC__2__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 287 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Reg__read(uint8_t addr, uint16_t *data)
#line 287
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 291
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 293
            status;

            {
#line 293
              __nesc_atomic_end(__nesc_atomic); 
#line 293
              return __nesc_temp;
            }
          }
        }
    }
#line 297
    __nesc_atomic_end(__nesc_atomic); }
#line 297
  status = CC2420SpiP__SpiByte__write(addr | 0x40);
  *data = (uint16_t )CC2420SpiP__SpiByte__write(0) << 8;
  *data |= CC2420SpiP__SpiByte__write(0);

  return status;
}

# 47 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__RSSI__read(uint16_t *data){
#line 47
  unsigned char __nesc_result;
#line 47

#line 47
  __nesc_result = CC2420SpiP__Reg__read(CC2420_RSSI, data);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 395 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__RssiResource__granted(void )
#line 395
{
  uint16_t data = 0;

#line 397
  CC2420ControlP__CSN__clr();
  CC2420ControlP__RSSI__read(&data);
  CC2420ControlP__CSN__set();

  CC2420ControlP__RssiResource__release();
  data += 0x7f;
  data &= 0x00ff;
  CC2420ControlP__ReadRssi__readDone(SUCCESS, data);
}

# 414 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__SpiResource__granted(void )
#line 414
{
  uint8_t cur_state;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 417
    {
      cur_state = CC2420TransmitP__m_state;
    }
#line 419
    __nesc_atomic_end(__nesc_atomic); }

  switch (cur_state) {
      case CC2420TransmitP__S_LOAD: 
        CC2420TransmitP__loadTXFIFO();
      break;

      case CC2420TransmitP__S_BEGIN_TRANSMIT: 
        CC2420TransmitP__attemptSend();
      break;

      case CC2420TransmitP__S_CANCEL: 
        CC2420TransmitP__CSN__clr();
      CC2420TransmitP__SFLUSHTX__strobe();
      CC2420TransmitP__CSN__set();
      CC2420TransmitP__releaseSpiResource();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 435
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
        }
#line 437
        __nesc_atomic_end(__nesc_atomic); }
      CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
      break;

      default: 
        CC2420TransmitP__releaseSpiResource();
      break;
    }
}

# 508 "/opt/tinyos-2.1.1/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__SpiResource__granted(void )
#line 508
{







  CC2420ReceiveP__receive();
}

# 367 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Resource__default__granted(uint8_t id)
#line 367
{
}

# 92 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static void CC2420SpiP__Resource__granted(uint8_t arg_0x2b99bd77a158){
#line 92
  switch (arg_0x2b99bd77a158) {
#line 92
    case /*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID:
#line 92
      CC2420ControlP__SpiResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID:
#line 92
      CC2420ControlP__SyncResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ControlC.RssiResource*/CC2420SpiC__2__CLIENT_ID:
#line 92
      CC2420ControlP__RssiResource__granted();
#line 92
      break;
#line 92
    case /*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID:
#line 92
      CC2420TransmitP__SpiResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID:
#line 92
      CC2420ReceiveP__SpiResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      CC2420SpiP__Resource__default__granted(arg_0x2b99bd77a158);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 358 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__grant__runTask(void )
#line 358
{
  uint8_t holder;

#line 360
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
    {
      holder = CC2420SpiP__m_holder;
    }
#line 362
    __nesc_atomic_end(__nesc_atomic); }
  CC2420SpiP__Resource__granted(holder);
}

# 55 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__FSCTRL__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_FSCTRL, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
inline static cc2420_status_t CC2420ControlP__MDMCTRL0__write(uint16_t data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__write(CC2420_MDMCTRL0, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 63 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420ControlP__PANID__write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Ram__write(CC2420_RAM_PANID, offset, data, length);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 222 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__CC2420Config__syncDone(error_t error)
#line 222
{
}

# 704 "/opt/tinyos-2.1.1/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Config__syncDone(error_t error)
#line 704
{
}

# 53 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static void CC2420ControlP__CC2420Config__syncDone(error_t error){
#line 53
  CC2420ReceiveP__CC2420Config__syncDone(error);
#line 53
  CC2420ActiveMessageP__CC2420Config__syncDone(error);
#line 53
}
#line 53
# 446 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__syncDone__runTask(void )
#line 446
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 447
    CC2420ControlP__m_sync_busy = FALSE;
#line 447
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__CC2420Config__syncDone(SUCCESS);
}

# 78 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 300 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Config__sync(void )
#line 300
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 301
    {
      if (CC2420ControlP__m_sync_busy) {
          {
            unsigned char __nesc_temp = 
#line 303
            0x0080;

            {
#line 303
              __nesc_atomic_end(__nesc_atomic); 
#line 303
              return __nesc_temp;
            }
          }
        }
#line 306
      CC2420ControlP__m_sync_busy = TRUE;
      if (CC2420ControlP__m_state == CC2420ControlP__S_XOSC_STARTED) {
          CC2420ControlP__SyncResource__request();
        }
      else 
#line 309
        {
          CC2420ControlP__syncDone__postTask();
        }
    }
#line 312
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 442
static inline void CC2420ControlP__sync__runTask(void )
#line 442
{
  CC2420ControlP__CC2420Config__sync();
}

# 213 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__BareSend__default__sendDone(message_t *msg, error_t error)
#line 213
{
}

# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static void CC2420TinyosNetworkP__BareSend__sendDone(message_t * msg, error_t error){
#line 89
  CC2420TinyosNetworkP__BareSend__default__sendDone(msg, error);
#line 89
}
#line 89
# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static void CC2420ActiveMessageP__AMSend__sendDone(am_id_t arg_0x2b99bddf66c0, message_t * msg, error_t error){
#line 99
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__sendDone(arg_0x2b99bddf66c0, msg, error);
#line 99
}
#line 99
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420ActiveMessageP__RadioResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420TinyosNetworkP__Resource__release(CC2420ActiveMessageC__CC2420_AM_SEND_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 199 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP__SubSend__sendDone(message_t *msg, error_t result)
#line 199
{
  CC2420ActiveMessageP__RadioResource__release();
  CC2420ActiveMessageP__AMSend__sendDone(CC2420ActiveMessageP__AMPacket__type(msg), msg, result);
}

# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static void CC2420TinyosNetworkP__ActiveSend__sendDone(message_t * msg, error_t error){
#line 89
  CC2420ActiveMessageP__SubSend__sendDone(msg, error);
#line 89
}
#line 89
# 118 "/opt/tinyos-2.1.1/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__SubSend__sendDone(message_t *msg, error_t error)
#line 118
{
  if (CC2420TinyosNetworkP__CC2420Packet__getNetwork(msg) == 0x3f) {
      CC2420TinyosNetworkP__ActiveSend__sendDone(msg, error);
    }
  else 
#line 121
    {
      CC2420TinyosNetworkP__BareSend__sendDone(msg, error);
    }
}

# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static void UniqueSendP__Send__sendDone(message_t * msg, error_t error){
#line 89
  CC2420TinyosNetworkP__SubSend__sendDone(msg, error);
#line 89
}
#line 89
# 104 "/opt/tinyos-2.1.1/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline void UniqueSendP__SubSend__sendDone(message_t *msg, error_t error)
#line 104
{
  UniqueSendP__State__toIdle();
  UniqueSendP__Send__sendDone(msg, error);
}

# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static void CC2420CsmaP__Send__sendDone(message_t * msg, error_t error){
#line 89
  UniqueSendP__SubSend__sendDone(msg, error);
#line 89
}
#line 89
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__stopDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__stopDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 63 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__stopVReg(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420ControlP__CC2420Power__stopVReg();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 58 "/opt/tinyos-2.1.1/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 84 "/opt/tinyos-2.1.1/tos/interfaces/StdControl.nc"
inline static error_t CC2420CsmaP__SubControl__stop(void ){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  __nesc_result = CC2420TransmitP__StdControl__stop();
#line 84
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__StdControl__stop());
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 272 "/opt/tinyos-2.1.1/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__shutdown(void )
#line 272
{
  CC2420CsmaP__SubControl__stop();
  CC2420CsmaP__CC2420Power__stopVReg();
  CC2420CsmaP__stopDone_task__postTask();
}

#line 241
static inline void CC2420CsmaP__sendDone_task__runTask(void )
#line 241
{
  error_t packetErr;

#line 243
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 243
    packetErr = CC2420CsmaP__sendErr;
#line 243
    __nesc_atomic_end(__nesc_atomic); }
  if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STOPPING)) {
      CC2420CsmaP__shutdown();
    }
  else {
      CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STARTED);
    }

  CC2420CsmaP__Send__sendDone(CC2420CsmaP__m_msg, packetErr);
}

# 56 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )31U &= ~(0x01 << 1);
}

# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc();
#line 85
}
#line 85
# 124 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void )
{
  * (volatile uint16_t * )388U &= ~0x0010;
}

# 47 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents();
#line 47
}
#line 47
# 58 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/GpioCaptureC.nc"
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc();
  }
}

# 55 "/opt/tinyos-2.1.1/tos/interfaces/GpioCapture.nc"
inline static void CC2420TransmitP__CaptureSFD__disable(void ){
#line 55
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable();
#line 55
}
#line 55
# 148 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port12__clear(void )
#line 148
{
#line 148
  P1IFG &= ~(1 << 2);
}

# 41 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port12__clear();
#line 41
}
#line 41
# 140 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port12__disable(void )
#line 140
{
#line 140
  P1IE &= ~(1 << 2);
}

# 36 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable(void ){
#line 36
  HplMsp430InterruptP__Port12__disable();
#line 36
}
#line 36
# 58 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear();
  }
  return SUCCESS;
}

# 50 "/opt/tinyos-2.1.1/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP__InterruptFIFOP__disable(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 46 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )29U &= ~(0x01 << 5);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 39 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__clr();
}

# 30 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__clr(void ){
#line 30
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr();
#line 30
}
#line 30
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

#line 252
static inline void TUnitP__TearDownOneTime__done(void )
#line 252
{
  TUnitP__tearDownOneTimeDone();
}

# 40 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
inline static void TestAPIComplexP__TearDownOneTime__done(void ){
#line 40
  TUnitP__TearDownOneTime__done();
#line 40
}
#line 40
# 46 "TestAPIComplexP.nc"
static inline void TestAPIComplexP__SplitControl__stopDone(error_t err)
{
  TestAPIComplexP__TearDownOneTime__done();
}

# 117 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static void HomeCommManagerP__SplitControl__stopDone(error_t error){
#line 117
  TestAPIComplexP__SplitControl__stopDone(error);
#line 117
}
#line 117
# 56 "/home/chuka/projects/puppet-os/system/HomeCommManagerP.nc"
static inline void HomeCommManagerP__RadioControl__stopDone(error_t err)
{
  if (err == SUCCESS) {
    HomeCommManagerP__ready = 0;
    }
#line 60
  HomeCommManagerP__SplitControl__stopDone(err);
}

# 279 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__RadioControl__stopDone(error_t err)
#line 279
{
  if (err == SUCCESS) {
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__clearState(/*CtpP.Forwarder*/CtpForwardingEngineP__0__RADIO_ON);
    }
}

# 237 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__RadioControl__stopDone(error_t error)
#line 237
{
  /*CtpP.Router*/CtpRoutingEngineP__0__radioOn = FALSE;
  ;
}

# 117 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static void CC2420CsmaP__SplitControl__stopDone(error_t error){
#line 117
  /*CtpP.Router*/CtpRoutingEngineP__0__RadioControl__stopDone(error);
#line 117
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__RadioControl__stopDone(error);
#line 117
  HomeCommManagerP__RadioControl__stopDone(error);
#line 117
}
#line 117
# 262 "/opt/tinyos-2.1.1/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__stopDone_task__runTask(void )
#line 262
{
  CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STOPPED);
  CC2420CsmaP__SplitControl__stopDone(SUCCESS);
}

# 51 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static void TUnitP__TestState__forceState(uint8_t reqState){
#line 51
  StateImplP__State__forceState(3U, reqState);
#line 51
}
#line 51
#line 71
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
# 267 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__setUpOneTimeDone(void )
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

#line 240
static inline void TUnitP__SetUpOneTime__done(void )
#line 240
{
  TUnitP__setUpOneTimeDone();
}

# 40 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
inline static void TestAPIComplexP__SetUpOneTime__done(void ){
#line 40
  TUnitP__SetUpOneTime__done();
#line 40
}
#line 40
# 41 "TestAPIComplexP.nc"
static inline void TestAPIComplexP__SplitControl__startDone(error_t err)
{
  TestAPIComplexP__SetUpOneTime__done();
}

# 92 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static void HomeCommManagerP__SplitControl__startDone(error_t error){
#line 92
  TestAPIComplexP__SplitControl__startDone(error);
#line 92
}
#line 92
# 424 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline error_t LinkEstimatorP__StdControl__start(void )
#line 424
{
  ;
  return SUCCESS;
}

# 143 "/opt/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, FALSE);
}

# 53 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
inline static void /*CtpP.Router*/CtpRoutingEngineP__0__RouteTimer__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(2U, dt);
#line 53
}
#line 53
# 209 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline error_t /*CtpP.Router*/CtpRoutingEngineP__0__StdControl__start(void )
#line 209
{

  if (!/*CtpP.Router*/CtpRoutingEngineP__0__running) {
      /*CtpP.Router*/CtpRoutingEngineP__0__running = TRUE;
      /*CtpP.Router*/CtpRoutingEngineP__0__resetInterval();
      /*CtpP.Router*/CtpRoutingEngineP__0__RouteTimer__startPeriodic(BEACON_INTERVAL);
      ;
    }
  return SUCCESS;
}

# 228 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__StdControl__start(void )
#line 228
{
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__setState(/*CtpP.Forwarder*/CtpForwardingEngineP__0__ROUTING_ON);
  return SUCCESS;
}

# 74 "/opt/tinyos-2.1.1/tos/interfaces/StdControl.nc"
inline static error_t HomeCommManagerP__RoutingControl__start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__StdControl__start();
#line 74
  __nesc_result = ecombine(__nesc_result, /*CtpP.Router*/CtpRoutingEngineP__0__StdControl__start());
#line 74
  __nesc_result = ecombine(__nesc_result, LinkEstimatorP__StdControl__start());
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 39 "/home/chuka/projects/puppet-os/system/HomeCommManagerP.nc"
static inline void HomeCommManagerP__RadioControl__startDone(error_t err)
{

  if (err == SUCCESS) 
    {
      HomeCommManagerP__RoutingControl__start();
      HomeCommManagerP__ready = 1;
    }
  HomeCommManagerP__SplitControl__startDone(err);
}

# 245 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline void /*CtpP.Forwarder*/CtpForwardingEngineP__0__RadioControl__startDone(error_t err)
#line 245
{
  if (err == SUCCESS) {
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__setState(/*CtpP.Forwarder*/CtpForwardingEngineP__0__RADIO_ON);
      if (!/*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__empty()) {
          ;
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask__postTask();
        }
    }
}

# 41 "/opt/tinyos-2.1.1/tos/interfaces/Random.nc"
inline static uint16_t /*CtpP.Router*/CtpRoutingEngineP__0__Random__rand16(void ){
#line 41
  unsigned int __nesc_result;
#line 41

#line 41
  __nesc_result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 226 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__RadioControl__startDone(error_t error)
#line 226
{
  /*CtpP.Router*/CtpRoutingEngineP__0__radioOn = TRUE;
  ;
  if (/*CtpP.Router*/CtpRoutingEngineP__0__running) {
      uint16_t nextInt;

#line 231
      nextInt = /*CtpP.Router*/CtpRoutingEngineP__0__Random__rand16() % BEACON_INTERVAL;
      nextInt += BEACON_INTERVAL >> 1;
      /*CtpP.Router*/CtpRoutingEngineP__0__BeaconTimer__startOneShot(nextInt);
    }
}

# 92 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static void CC2420CsmaP__SplitControl__startDone(error_t error){
#line 92
  /*CtpP.Router*/CtpRoutingEngineP__0__RadioControl__startDone(error);
#line 92
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__RadioControl__startDone(error);
#line 92
  HomeCommManagerP__RadioControl__startDone(error);
#line 92
}
#line 92
# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 179 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Resource__release(void )
#line 179
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 180
    {
      CC2420ControlP__CSN__set();
      {
        unsigned char __nesc_temp = 
#line 182
        CC2420ControlP__SpiResource__release();

        {
#line 182
          __nesc_atomic_end(__nesc_atomic); 
#line 182
          return __nesc_temp;
        }
      }
    }
#line 185
    __nesc_atomic_end(__nesc_atomic); }
}

# 110 "/opt/tinyos-2.1.1/tos/interfaces/Resource.nc"
inline static error_t CC2420CsmaP__Resource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = CC2420ControlP__Resource__release();
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 249 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__rxOn(void )
#line 249
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 250
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_XOSC_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 252
            0x0080;

            {
#line 252
              __nesc_atomic_end(__nesc_atomic); 
#line 252
              return __nesc_temp;
            }
          }
        }
#line 254
      CC2420ControlP__SRXON__strobe();
    }
#line 255
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 90 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__rxOn(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = CC2420ControlP__CC2420Power__rxOn();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 132 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port12__enable(void )
#line 132
{
#line 132
  P1IE |= 1 << 2;
}

# 31 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable(void ){
#line 31
  HplMsp430InterruptP__Port12__enable();
#line 31
}
#line 31
# 174 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port12__edge(bool l2h)
#line 174
{
  /* atomic removed: atomic calls only */
#line 175
  {
    if (l2h) {
#line 176
      P1IES &= ~(1 << 2);
      }
    else {
#line 177
      P1IES |= 1 << 2;
      }
  }
}

# 56 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high){
#line 56
  HplMsp430InterruptP__Port12__edge(low_to_high);
#line 56
}
#line 56
# 41 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(bool rising)
#line 41
{
  /* atomic removed: atomic calls only */
#line 42
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(rising);
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable();
  }
  return SUCCESS;
}





static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void )
#line 54
{
  return /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(FALSE);
}

# 43 "/opt/tinyos-2.1.1/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 157 "/opt/tinyos-2.1.1/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__StdControl__start(void )
#line 157
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 158
    {
      CC2420ReceiveP__reset_state();
      CC2420ReceiveP__m_state = CC2420ReceiveP__S_STARTED;
      CC2420ReceiveP__receivingPacket = FALSE;




      CC2420ReceiveP__InterruptFIFOP__enableFallingEdge();
    }
#line 167
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 168 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__StdControl__start(void )
#line 168
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 169
    {
      CC2420TransmitP__CaptureSFD__captureRisingEdge();
      CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
      CC2420TransmitP__m_receiving = FALSE;
      CC2420TransmitP__abortSpiRelease = FALSE;
      CC2420TransmitP__m_tx_power = 0;
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 74 "/opt/tinyos-2.1.1/tos/interfaces/StdControl.nc"
inline static error_t CC2420CsmaP__SubControl__start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = CC2420TransmitP__StdControl__start();
#line 74
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__StdControl__start());
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 254 "/opt/tinyos-2.1.1/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__startDone_task__runTask(void )
#line 254
{
  CC2420CsmaP__SubControl__start();
  CC2420CsmaP__CC2420Power__rxOn();
  CC2420CsmaP__Resource__release();
  CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STARTED);
  CC2420CsmaP__SplitControl__startDone(SUCCESS);
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
# 50 "/opt/tinyos-2.1.1/tos/lib/net/CollectionIdP.nc"
static inline collection_id_t /*HomeCommManagerC.CollectionSenderC.CollectionSenderP.CollectionIdP*/CollectionIdP__0__CollectionId__fetch(void )
#line 50
{
  return 16;
}

# 877 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline collection_id_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionId__default__fetch(uint8_t client)
#line 877
{
  return 0;
}

# 46 "/opt/tinyos-2.1.1/tos/lib/net/CollectionId.nc"
inline static collection_id_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionId__fetch(uint8_t arg_0x2b99bdecf968){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  switch (arg_0x2b99bdecf968) {
#line 46
    case 0U:
#line 46
      __nesc_result = /*HomeCommManagerC.CollectionSenderC.CollectionSenderP.CollectionIdP*/CollectionIdP__0__CollectionId__fetch();
#line 46
      break;
#line 46
    default:
#line 46
      __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionId__default__fetch(arg_0x2b99bdecf968);
#line 46
      break;
#line 46
    }
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 351 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__maxPayloadLength(uint8_t client)
#line 351
{
  return /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__maxPayloadLength();
}

#line 300
static inline error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__send(uint8_t client, message_t *msg, uint8_t len)
#line 300
{
  ctp_data_header_t *hdr;
  fe_queue_entry_t *qe;

#line 303
  ;
  if (!/*CtpP.Forwarder*/CtpForwardingEngineP__0__hasState(/*CtpP.Forwarder*/CtpForwardingEngineP__0__ROUTING_ON)) {
#line 304
      return EOFF;
    }
#line 305
  if (len > /*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__maxPayloadLength(client)) {
#line 305
      return ESIZE;
    }
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__setPayloadLength(msg, len);
  hdr = /*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(msg);
  __nesc_hton_uint16(hdr->origin.data, TOS_NODE_ID);
  __nesc_hton_uint8(hdr->originSeqNo.data, /*CtpP.Forwarder*/CtpForwardingEngineP__0__seqno++);
  __nesc_hton_uint8(hdr->type.data, /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionId__fetch(client));
  __nesc_hton_uint8(hdr->thl.data, 0);

  if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__clientPtrs[client] == (void *)0) {
      ;
      return EBUSY;
    }

  qe = /*CtpP.Forwarder*/CtpForwardingEngineP__0__clientPtrs[client];
  qe->msg = msg;
  qe->client = client;
  qe->retries = MAX_RETRIES;
  ;
  if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__enqueue(qe) == SUCCESS) {
      if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__hasState(/*CtpP.Forwarder*/CtpForwardingEngineP__0__RADIO_ON) && !/*CtpP.Forwarder*/CtpForwardingEngineP__0__hasState(/*CtpP.Forwarder*/CtpForwardingEngineP__0__SENDING)) {
          ;
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask__postTask();
        }
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__clientPtrs[client] = (void *)0;
      return SUCCESS;
    }
  else {
      ;




      /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_SEND_QUEUE_FULL);


      return 0x0080;
    }
}

# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static error_t HomeCommManagerP__RadioSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__send(0U, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 355 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline void */*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__getPayload(uint8_t client, message_t *msg, uint8_t len)
#line 355
{
  return /*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__getPayload(msg, len);
}

# 114 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static void * HomeCommManagerP__RadioSend__getPayload(message_t * msg, uint8_t len){
#line 114
  void *__nesc_result;
#line 114

#line 114
  __nesc_result = /*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__getPayload(0U, msg, len);
#line 114

#line 114
  return __nesc_result;
#line 114
}
#line 114
# 84 "/home/chuka/projects/puppet-os/system/HomeCommManagerP.nc"
static inline error_t HomeCommManagerP__sendPacket(register_request_t *req)
{
  error_t err = EOFF;

#line 87
  if (HomeCommManagerP__ready) 
    {
      register_request_t *ptr = (register_request_t *)HomeCommManagerP__RadioSend__getPayload(&HomeCommManagerP__message_buf, sizeof(register_request_t ));

#line 90
      memcpy(ptr, req, sizeof(register_request_t ));
      return HomeCommManagerP__RadioSend__send(&HomeCommManagerP__message_buf, sizeof(register_request_t ));
    }
  return err;
}

static inline error_t HomeCommManagerP__validateRegisterRequest(register_request_t *reg)
{

  if (__nesc_ntoh_uint32(
#line 98
  reg->device_type_id.data) != 0 && 
  reg->sensor_ids != 0 && __nesc_ntoh_uint32(
  reg->man_id.data) != 0) {
    return SUCCESS;
    }
  else {
#line 103
    return EINVAL;
    }
}

#line 63
static inline error_t HomeCommManagerP__PuppetAPI__registerDeviceRequest(register_request_t *req)
{
  error_t err;

#line 66
  if (req == (void *)0) {
    return 0x0080;
    }
  err = HomeCommManagerP__validateRegisterRequest(req);
  if (err != SUCCESS) {
    return err;
    }
  else {
      err = HomeCommManagerP__sendPacket(req);
      return err;
    }
}

# 6 "/home/chuka/projects/puppet-os/interfaces/PuppetAPI.nc"
inline static error_t TestAPIComplexP__PuppetAPI__registerDeviceRequest(register_request_t *reg){
#line 6
  unsigned char __nesc_result;
#line 6

#line 6
  __nesc_result = HomeCommManagerP__PuppetAPI__registerDeviceRequest(reg);
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6
# 51 "TestAPIComplexP.nc"
static inline void TestAPIComplexP__TestRegister__run(void )
{
  TestAPIComplexP__reg = (register_request_t *)malloc(sizeof(register_request_t ));

  __nesc_hton_uint32(TestAPIComplexP__reg->device_type_id.data, 1);
  __nesc_hton_uint16(TestAPIComplexP__reg->sensor_ids[0].data, 30);
  __nesc_hton_uint32(TestAPIComplexP__reg->man_id.data, 1678902);

  if (!(TestAPIComplexP__PuppetAPI__registerDeviceRequest(TestAPIComplexP__reg) == SUCCESS)) {
#line 59
      assertTunitFail("Could not send message", 18U);
#line 59
      ;
    }
  else 
#line 59
    {
#line 59
      assertTunitSuccess(19U);
#line 59
      ;
    }
#line 59
  ;
}

# 378 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
static inline void TUnitP__TestCase__default__run(uint8_t testId)
#line 378
{
  TUnitP__runDone__postTask();
}

# 39 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/TestCase.nc"
inline static void TUnitP__TestCase__run(uint8_t arg_0x2b99bc837e00){
#line 39
  switch (arg_0x2b99bc837e00) {
#line 39
    case /*TestAPIComplexC.TestRegisterC*/TestCaseC__0__TUNIT_TEST_ID:
#line 39
      TestAPIComplexP__TestRegister__run();
#line 39
      break;
#line 39
    default:
#line 39
      TUnitP__TestCase__default__run(arg_0x2b99bc837e00);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
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
inline static void /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__granted(uint8_t arg_0x2b99bce0f688){
#line 92
  switch (arg_0x2b99bce0f688) {
#line 92
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 92
      Z1SerialP__Resource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430Uart0P.UartP*/Msp430UartP__0__Resource__default__granted(arg_0x2b99bce0f688);
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
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x2b99bd1c2d40){
#line 92
  switch (arg_0x2b99bd1c2d40) {
#line 92
    case /*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID:
#line 92
      /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__granted(/*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0x2b99bd1c2d40);
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
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x2b99bd1be340){
#line 49
  switch (arg_0x2b99bd1be340) {
#line 49
    case /*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID:
#line 49
      /*Msp430Uart0P.UartP*/Msp430UartP__0__ResourceConfigure__configure(/*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x2b99bd1be340);
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
inline static msp430_uart_union_config_t */*Msp430Uart0P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(uint8_t arg_0x2b99bce09cd0){
#line 71
  union __nesc_unnamed4279 *__nesc_result;
#line 71

#line 71
  switch (arg_0x2b99bce09cd0) {
#line 71
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 71
      __nesc_result = Z1SerialP__Msp430UartConfigure__getConfig();
#line 71
      break;
#line 71
    default:
#line 71
      __nesc_result = /*Msp430Uart0P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(arg_0x2b99bce09cd0);
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

# 99 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static void /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__sendDone(message_t * msg, error_t error){
#line 99
  StatisticsP__AMSend__sendDone(msg, error);
#line 99
}
#line 99
# 57 "/opt/tinyos-2.1.1/tos/system/AMQueueEntryP.nc"
static inline void /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__sendDone(message_t *m, error_t err)
#line 57
{
  /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__sendDone(m, err);
}

# 207 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err)
#line 207
{
}

# 89 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(uint8_t arg_0x2b99bcb961c8, message_t * msg, error_t error){
#line 89
  switch (arg_0x2b99bcb961c8) {
#line 89
    case 0U:
#line 89
      /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(msg, error);
#line 89
      break;
#line 89
    case 1U:
#line 89
      /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(arg_0x2b99bcb961c8, msg, error);
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





  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 2) {
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
inline static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(am_id_t arg_0x2b99bcbeeba8, message_t * msg, error_t error){
#line 99
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(arg_0x2b99bcbeeba8, msg, error);
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
inline static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(uart_id_t arg_0x2b99bcd15238, message_t * msg, error_t error){
#line 89
  switch (arg_0x2b99bcd15238) {
#line 89
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 89
      /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(arg_0x2b99bcd15238, msg, error);
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
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 2;
  for (i = 0; i < 2; i++) {
      if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == (void *)0 || 
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current / 8] & (1 << /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current % 8)) 
        {
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 2;
        }
      else {
          break;
        }
    }
  if (i >= 2) {
#line 70
    /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 2;
    }
}

# 136 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
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
#line 67
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
# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(am_id_t arg_0x2b99bcb95340, am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(arg_0x2b99bcb95340, addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
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
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(uart_id_t arg_0x2b99bcd14570, message_t *msg, uint8_t upperLen){
#line 23
  unsigned char __nesc_result;
#line 23

#line 23
  switch (arg_0x2b99bcd14570) {
#line 23
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 23
      __nesc_result = SerialPacketInfoActiveMessageP__Info__dataLinkLength(msg, upperLen);
#line 23
      break;
#line 23
    default:
#line 23
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(arg_0x2b99bcd14570, msg, upperLen);
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
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(uart_id_t arg_0x2b99bcd14570){
#line 15
  unsigned char __nesc_result;
#line 15

#line 15
  switch (arg_0x2b99bcd14570) {
#line 15
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 15
      __nesc_result = SerialPacketInfoActiveMessageP__Info__offset();
#line 15
      break;
#line 15
    default:
#line 15
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(arg_0x2b99bcd14570);
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
# 129 "/opt/tinyos-2.x-contrib/tunit/tos/lib/fifoqueue/FifoQueueP.nc"
static inline bool /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__FifoQueue__isEmpty(void )
#line 129
{
  return /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__currentSize == 0;
}

#line 92
static inline error_t /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__FifoQueue__dequeue(void *location, uint8_t maxSize)
#line 92
{
  uint8_t nextDequeue = (/*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__currentDequeueLocation + 1) % 1U;

  if (maxSize < sizeof(/*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__q_element_t )) {

      return ESIZE;
    }

  if (/*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__FifoQueue__isEmpty()) {

      return 0x0080;
    }

  /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__currentDequeueLocation = nextDequeue;
  memcpy(location, &/*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__myQueue[/*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__currentDequeueLocation], sizeof(/*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__q_element_t ));
  /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__currentSize--;

  return SUCCESS;
}

# 51 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/FifoQueue.nc"
inline static error_t StatisticsP__FifoQueue__dequeue(void *location, uint8_t maxSize){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__FifoQueue__dequeue(location, maxSize);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 64 "/opt/tinyos-2.1.1/tos/interfaces/Send.nc"
inline static error_t /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(1U, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 151 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static void /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(amsg, t);
#line 151
}
#line 151
#line 92
inline static void /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
# 45 "/opt/tinyos-2.1.1/tos/system/AMQueueEntryP.nc"
static inline error_t /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 47
{
  /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setDestination(msg, dest);
  /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__AMPacket__setType(msg, 254);
  return /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__Send__send(msg, len);
}

# 69 "/opt/tinyos-2.1.1/tos/interfaces/AMSend.nc"
inline static error_t StatisticsP__AMSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*WireStatisticsC.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP__1__AMSend__send(addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
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
# 56 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static void StatisticsP__State__toIdle(void ){
#line 56
  StateImplP__State__toIdle(4U);
#line 56
}
#line 56
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

# 96 "/opt/tinyos-2.1.1/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline error_t CC2420CsmaP__SplitControl__stop(void )
#line 96
{
  if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTED)) {
      CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STOPPING);
      CC2420CsmaP__shutdown();
      return SUCCESS;
    }
  else {
#line 102
    if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STOPPED)) {
        return EALREADY;
      }
    else {
#line 105
      if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_TRANSMITTING)) {
          CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STOPPING);

          return SUCCESS;
        }
      else {
#line 110
        if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STOPPING)) {
            return SUCCESS;
          }
        }
      }
    }
#line 114
  return EBUSY;
}

# 109 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static error_t HomeCommManagerP__RadioControl__stop(void ){
#line 109
  unsigned char __nesc_result;
#line 109

#line 109
  __nesc_result = CC2420CsmaP__SplitControl__stop();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 50 "/home/chuka/projects/puppet-os/system/HomeCommManagerP.nc"
static inline error_t HomeCommManagerP__SplitControl__stop(void )
{

  return HomeCommManagerP__RadioControl__stop();
}

# 109 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static error_t TestAPIComplexP__SplitControl__stop(void ){
#line 109
  unsigned char __nesc_result;
#line 109

#line 109
  __nesc_result = HomeCommManagerP__SplitControl__stop();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 36 "TestAPIComplexP.nc"
static inline void TestAPIComplexP__TearDownOneTime__run(void )
{
  TestAPIComplexP__SplitControl__stop();
}

# 38 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
inline static void TUnitP__TearDownOneTime__run(void ){
#line 38
  TestAPIComplexP__TearDownOneTime__run();
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
inline static message_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(am_id_t arg_0x2b99bcbebcd0, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  switch (arg_0x2b99bcbebcd0) {
#line 67
    case 255:
#line 67
      __nesc_result = Link_TUnitProcessingP__SerialReceive__receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(arg_0x2b99bcbebcd0, msg, payload, len);
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
inline static message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(uart_id_t arg_0x2b99bcd166e8, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  switch (arg_0x2b99bcd166e8) {
#line 67
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 67
      __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(arg_0x2b99bcd166e8, msg, payload, len);
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
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(uart_id_t arg_0x2b99bcd14570, message_t *msg, uint8_t dataLinkLen){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  switch (arg_0x2b99bcd14570) {
#line 31
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 31
      __nesc_result = SerialPacketInfoActiveMessageP__Info__upperLength(msg, dataLinkLen);
#line 31
      break;
#line 31
    default:
#line 31
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(arg_0x2b99bcd14570, msg, dataLinkLen);
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
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x2b99bd1be340){
#line 55
  switch (arg_0x2b99bd1be340) {
#line 55
    case /*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID:
#line 55
      /*Msp430Uart0P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(/*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID);
#line 55
      break;
#line 55
    default:
#line 55
      /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x2b99bd1be340);
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
static inline resource_client_id_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead != /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
        uint8_t id = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead;

#line 62
        /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[/*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead];
        if (/*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
          }
#line 65
        /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[id] = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
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
      /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

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
  __nesc_result = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 50 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 51
  {
    unsigned char __nesc_temp = 
#line 51
    /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

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
  __nesc_result = /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty();
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
inline static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__release(uint8_t arg_0x2b99bce0aa18){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  switch (arg_0x2b99bce0aa18) {
#line 110
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 110
      __nesc_result = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(/*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID);
#line 110
      break;
#line 110
    default:
#line 110
      __nesc_result = /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__release(arg_0x2b99bce0aa18);
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
inline static bool /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__isOwner(uint8_t arg_0x2b99bce0aa18){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  switch (arg_0x2b99bce0aa18) {
#line 118
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 118
      __nesc_result = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(/*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID);
#line 118
      break;
#line 118
    default:
#line 118
      __nesc_result = /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__isOwner(arg_0x2b99bce0aa18);
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

# 41 "/opt/tinyos-2.1.1/tos/lib/net/RootControl.nc"
inline static error_t TestAPIComplexP__RootControl__setRoot(void ){
#line 41
  unsigned char __nesc_result;
#line 41

#line 41
  __nesc_result = /*CtpP.Router*/CtpRoutingEngineP__0__RootControl__setRoot();
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 39 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr();
#line 39
}
#line 39
# 38 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void )
#line 38
{
#line 38
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr();
}

# 30 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__clr(void ){
#line 30
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr();
#line 30
}
#line 30
# 93 "/opt/tinyos-2.1.1/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2On(void )
#line 93
{
  LedsP__Led2__clr();
  ;
#line 95
  ;
}

# 78 "/opt/tinyos-2.1.1/tos/interfaces/Leds.nc"
inline static void TestAPIComplexP__Leds__led2On(void ){
#line 78
  LedsP__Leds__led2On();
#line 78
}
#line 78
# 83 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static error_t HomeCommManagerP__RadioControl__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = CC2420CsmaP__SplitControl__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 33 "/home/chuka/projects/puppet-os/system/HomeCommManagerP.nc"
static inline error_t HomeCommManagerP__SplitControl__start(void )
{

  return HomeCommManagerP__RadioControl__start();
}

# 83 "/opt/tinyos-2.1.1/tos/interfaces/SplitControl.nc"
inline static error_t TestAPIComplexP__SplitControl__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = HomeCommManagerP__SplitControl__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 29 "TestAPIComplexP.nc"
static inline void TestAPIComplexP__SetUpOneTime__run(void )
{
  TestAPIComplexP__SplitControl__start();
  TestAPIComplexP__Leds__led2On();
  TestAPIComplexP__RootControl__setRoot();
}

# 38 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TestControl.nc"
inline static void TUnitP__SetUpOneTime__run(void ){
#line 38
  TestAPIComplexP__SetUpOneTime__run();
#line 38
}
#line 38
# 71 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
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
inline static void /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0x2b99bd1c0020){
#line 51
    /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(arg_0x2b99bd1c0020);
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
inline static error_t /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__immediateRequest(uint8_t arg_0x2b99bce0aa18){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  switch (arg_0x2b99bce0aa18) {
#line 87
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 87
      __nesc_result = /*Msp430UsciShareA0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(/*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID);
#line 87
      break;
#line 87
    default:
#line 87
      __nesc_result = /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciResource__default__immediateRequest(arg_0x2b99bce0aa18);
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

# 45 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
inline static error_t CC2420CsmaP__SplitControlState__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(6U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 55 "/opt/tinyos-2.1.1/tos/lib/timer/Alarm.nc"
inline static void CC2420ControlP__StartupTimer__start(CC2420ControlP__StartupTimer__size_type dt){
#line 55
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(dt);
#line 55
}
#line 55
# 45 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )29U |= 0x01 << 5;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 34 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__set();
}

# 29 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__set(void ){
#line 29
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set();
#line 29
}
#line 29
# 187 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__startVReg(void )
#line 187
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_VREG_STOPPED) {
          {
            unsigned char __nesc_temp = 
#line 190
            0x0080;

            {
#line 190
              __nesc_atomic_end(__nesc_atomic); 
#line 190
              return __nesc_temp;
            }
          }
        }
#line 192
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTING;
    }
#line 193
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__VREN__set();
  CC2420ControlP__StartupTimer__start(CC2420_TIME_VREN);
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__startVReg(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420ControlP__CC2420Power__startVReg();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
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
  for (i = 0; i < 2 / 8 + 1; i++) {
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
# 151 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static void /*Link_TUnitProcessingC.SerialEventSendC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(amsg, t);
#line 151
}
#line 151
#line 92
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

# 66 "/opt/tinyos-2.x-contrib/tunit/tos/interfaces/FifoQueue.nc"
inline static bool StatisticsP__FifoQueue__isEmpty(void ){
#line 66
  unsigned char __nesc_result;
#line 66

#line 66
  __nesc_result = /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__FifoQueue__isEmpty();
#line 66

#line 66
  return __nesc_result;
#line 66
}
#line 66
# 128 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunitstats/StatisticsP.nc"
static inline bool StatisticsP__StatsQuery__isIdle(void )
#line 128
{
  return StatisticsP__FifoQueue__isEmpty();
}

# 46 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/StatsQuery.nc"
inline static bool TUnitP__StatsQuery__isIdle(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = StatisticsP__StatsQuery__isIdle();
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
static inline error_t /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void )
#line 45
{
  memset(/*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ, /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY, sizeof /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ);
  return SUCCESS;
}

# 81 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
static inline error_t StateImplP__Init__init(void )
#line 81
{
  int i;

#line 83
  for (i = 0; i < 9U; i++) {
      StateImplP__state[i] = StateImplP__S_IDLE;
    }
  return SUCCESS;
}

# 143 "/opt/tinyos-2.x-contrib/tunit/tos/lib/fifoqueue/FifoQueueP.nc"
static inline void /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__FifoQueue__reset(void )
#line 143
{
  /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__nextEnqueueLocation = 0;
  /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__currentDequeueLocation = 1U - 1;
  /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__currentSize = 0;
}

#line 59
static inline error_t /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__Init__init(void )
#line 59
{
  /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__FifoQueue__reset();
  return SUCCESS;
}

# 416 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static inline void LinkEstimatorP__initNeighborTable(void )
#line 416
{
  uint8_t i;

  for (i = 0; i < 10; i++) {
      LinkEstimatorP__NeighborTable[i].flags = 0;
    }
}











static inline error_t LinkEstimatorP__Init__init(void )
#line 434
{
  ;
  LinkEstimatorP__initNeighborTable();
  return SUCCESS;
}

# 57 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__address(void ){
#line 57
  unsigned int __nesc_result;
#line 57

#line 57
  __nesc_result = CC2420ActiveMessageP__AMPacket__address();
#line 57

#line 57
  return __nesc_result;
#line 57
}
#line 57
# 216 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static inline error_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__Init__init(void )
#line 216
{
  int i;

#line 218
  for (i = 0; i < /*CtpP.Forwarder*/CtpForwardingEngineP__0__CLIENT_COUNT; i++) {
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__clientPtrs[i] = /*CtpP.Forwarder*/CtpForwardingEngineP__0__clientEntries + i;
      ;
    }
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__loopbackMsgPtr = &/*CtpP.Forwarder*/CtpForwardingEngineP__0__loopbackMsg;
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__lastParent = /*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__address();
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__seqno = 0;
  return SUCCESS;
}

# 57 "/opt/tinyos-2.1.1/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*CtpP.Router*/CtpRoutingEngineP__0__AMPacket__address(void ){
#line 57
  unsigned int __nesc_result;
#line 57

#line 57
  __nesc_result = CC2420ActiveMessageP__AMPacket__address();
#line 57

#line 57
  return __nesc_result;
#line 57
}
#line 57
# 681 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline void /*CtpP.Router*/CtpRoutingEngineP__0__routingTableInit(void )
#line 681
{
  /*CtpP.Router*/CtpRoutingEngineP__0__routingTableActive = 0;
}

# 26 "/opt/tinyos-2.1.1/tos/lib/net/ctp/TreeRouting.h"
static __inline void routeInfoInit(route_info_t *ri)
#line 26
{
  ri->parent = INVALID_ADDR;
  ri->etx = 0;
  ri->haveHeard = 0;
  ri->congested = FALSE;
}

# 193 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static inline error_t /*CtpP.Router*/CtpRoutingEngineP__0__Init__init(void )
#line 193
{
  uint8_t maxLength;

#line 195
  /*CtpP.Router*/CtpRoutingEngineP__0__radioOn = FALSE;
  /*CtpP.Router*/CtpRoutingEngineP__0__running = FALSE;
  /*CtpP.Router*/CtpRoutingEngineP__0__parentChanges = 0;
  /*CtpP.Router*/CtpRoutingEngineP__0__state_is_root = 0;
  routeInfoInit(&/*CtpP.Router*/CtpRoutingEngineP__0__routeInfo);
  /*CtpP.Router*/CtpRoutingEngineP__0__routingTableInit();
  /*CtpP.Router*/CtpRoutingEngineP__0__my_ll_addr = /*CtpP.Router*/CtpRoutingEngineP__0__AMPacket__address();
  /*CtpP.Router*/CtpRoutingEngineP__0__beaconMsg = /*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__getPayload(&/*CtpP.Router*/CtpRoutingEngineP__0__beaconMsgBuffer, /*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__maxPayloadLength());
  maxLength = /*CtpP.Router*/CtpRoutingEngineP__0__BeaconSend__maxPayloadLength();
  ;

  return SUCCESS;
}

# 82 "/opt/tinyos-2.1.1/tos/system/ActiveMessageAddressC.nc"
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void )
#line 82
{
  am_group_t myGroup;

  /* atomic removed: atomic calls only */
#line 84
  myGroup = ActiveMessageAddressC__group;
  return myGroup;
}

# 55 "/opt/tinyos-2.1.1/tos/interfaces/ActiveMessageAddress.nc"
inline static am_group_t CC2420ControlP__ActiveMessageAddress__amGroup(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amGroup();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
#line 50
inline static am_addr_t CC2420ControlP__ActiveMessageAddress__amAddress(void ){
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
# 52 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 5;
}

# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 6;
}

# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )26U |= 0x01 << 0;
}

# 71 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput();
}

# 35 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 35
}
#line 35
# 121 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Init__init(void )
#line 121
{
  CC2420ControlP__CSN__makeOutput();
  CC2420ControlP__RSTN__makeOutput();
  CC2420ControlP__VREN__makeOutput();

  CC2420ControlP__m_short_addr = CC2420ControlP__ActiveMessageAddress__amAddress();
  CC2420ControlP__m_pan = CC2420ControlP__ActiveMessageAddress__amGroup();
  CC2420ControlP__m_tx_power = 31;
  CC2420ControlP__m_channel = 26;





  CC2420ControlP__addressRecognition = TRUE;





  CC2420ControlP__hwAddressRecognition = FALSE;






  CC2420ControlP__autoAckEnabled = TRUE;






  CC2420ControlP__hwAutoAckDefault = FALSE;



  return SUCCESS;
}

# 45 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__Init__init(void )
#line 45
{
  memset(/*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ, /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__NO_ENTRY, sizeof /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__resQ);
  return SUCCESS;
}

# 46 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4427 {
#line 46
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(x);
}

#line 94
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl();
}

# 36 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 36
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 36
}
#line 36
# 42 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 50 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )30U &= ~(0x01 << 1);
}

# 64 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput();
#line 64
}
#line 64
# 41 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void )
#line 41
{
#line 41
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput();
}

# 33 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__SFD__makeInput(void ){
#line 33
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput();
#line 33
}
#line 33


inline static void CC2420TransmitP__CSN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 35
}
#line 35
# 50 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )34U &= ~(0x01 << 4);
}

# 64 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput();
#line 64
}
#line 64
# 41 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void )
#line 41
{
#line 41
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput();
}

# 33 "/opt/tinyos-2.1.1/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CCA__makeInput(void ){
#line 33
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput();
#line 33
}
#line 33
# 160 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__Init__init(void )
#line 160
{
  CC2420TransmitP__CCA__makeInput();
  CC2420TransmitP__CSN__makeOutput();
  CC2420TransmitP__SFD__makeInput();
  return SUCCESS;
}

# 151 "/opt/tinyos-2.1.1/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__Init__init(void )
#line 151
{
  CC2420ReceiveP__m_p_rx_buf = &CC2420ReceiveP__m_rx_buf;
  return SUCCESS;
}

# 46 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__CC2int(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4428 {
#line 46
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__CC2int(x);
}

#line 94
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )390U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__compareControl();
}

# 36 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void ){
#line 36
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare();
#line 36
}
#line 36
# 42 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 44 "/opt/tinyos-2.1.1/tos/system/RandomMlcgC.nc"
static inline error_t RandomMlcgC__Init__init(void )
#line 44
{
  /* atomic removed: atomic calls only */
#line 45
  RandomMlcgC__seed = (uint32_t )(TOS_NODE_ID + 1);

  return SUCCESS;
}

# 41 "/opt/tinyos-2.1.1/tos/interfaces/Random.nc"
inline static uint16_t UniqueSendP__Random__rand16(void ){
#line 41
  unsigned int __nesc_result;
#line 41

#line 41
  __nesc_result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 62 "/opt/tinyos-2.1.1/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline error_t UniqueSendP__Init__init(void )
#line 62
{
  UniqueSendP__localSendId = UniqueSendP__Random__rand16();
  return SUCCESS;
}

# 71 "/opt/tinyos-2.1.1/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline error_t UniqueReceiveP__Init__init(void )
#line 71
{
  int i;

#line 73
  for (i = 0; i < 4; i++) {
      UniqueReceiveP__receivedMessages[i].source = (am_addr_t )0xFFFF;
      UniqueReceiveP__receivedMessages[i].dsn = 0;
    }
  return SUCCESS;
}

# 45 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void )
#line 45
{
  memset(/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ, /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 65 "/opt/tinyos-2.1.1/tos/system/PoolP.nc"
static inline error_t /*CtpP.MessagePoolP.PoolP*/PoolP__0__Init__init(void )
#line 65
{
  int i;

#line 67
  for (i = 0; i < 12; i++) {
      /*CtpP.MessagePoolP.PoolP*/PoolP__0__queue[i] = &/*CtpP.MessagePoolP.PoolP*/PoolP__0__pool[i];
    }
  /*CtpP.MessagePoolP.PoolP*/PoolP__0__free = 12;
  /*CtpP.MessagePoolP.PoolP*/PoolP__0__index = 0;
  return SUCCESS;
}

#line 65
static inline error_t /*CtpP.QEntryPoolP.PoolP*/PoolP__1__Init__init(void )
#line 65
{
  int i;

#line 67
  for (i = 0; i < 12; i++) {
      /*CtpP.QEntryPoolP.PoolP*/PoolP__1__queue[i] = &/*CtpP.QEntryPoolP.PoolP*/PoolP__1__pool[i];
    }
  /*CtpP.QEntryPoolP.PoolP*/PoolP__1__free = 12;
  /*CtpP.QEntryPoolP.PoolP*/PoolP__1__index = 0;
  return SUCCESS;
}

# 64 "/opt/tinyos-2.1.1/tos/lib/net/ctp/LruCtpMsgCacheP.nc"
static inline error_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__Init__init(void )
#line 64
{
  /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__first = 0;
  /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__count = 0;
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.1/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__Init__init();
#line 51
  __nesc_result = ecombine(__nesc_result, /*CtpP.QEntryPoolP.PoolP*/PoolP__1__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*CtpP.MessagePoolP.PoolP*/PoolP__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, UniqueReceiveP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, UniqueSendP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, RandomMlcgC__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, CC2420TransmitP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__2__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, CC2420ControlP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*CtpP.Router*/CtpRoutingEngineP__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*CtpP.Forwarder*/CtpForwardingEngineP__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, LinkEstimatorP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*WireStatisticsC.FifoQueueC.FifoQueueP*/FifoQueueP__0__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, StateImplP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*Msp430UsciShareA0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init());
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
inline static void * /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__getPayload(am_id_t arg_0x2b99bcb95340, message_t * msg, uint8_t len){
#line 124
  void *__nesc_result;
#line 124

#line 124
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__getPayload(arg_0x2b99bcb95340, msg, len);
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
inline static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__receivedByte(uint8_t arg_0x2b99bce0d7e0, uint8_t byte){
#line 79
  switch (arg_0x2b99bce0d7e0) {
#line 79
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 79
      HdlcTranslateC__UartStream__receivedByte(byte);
#line 79
      break;
#line 79
    default:
#line 79
      /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(arg_0x2b99bce0d7e0, byte);
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
inline static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__receiveDone(uint8_t arg_0x2b99bce0d7e0, uint8_t * buf, uint16_t len, error_t error){
#line 99
  switch (arg_0x2b99bce0d7e0) {
#line 99
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 99
      HdlcTranslateC__UartStream__receiveDone(buf, len, error);
#line 99
      break;
#line 99
    default:
#line 99
      /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(arg_0x2b99bce0d7e0, buf, len, error);
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
inline static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__rxDone(uint8_t arg_0x2b99bd185410, uint8_t data){
#line 85
  switch (arg_0x2b99bd185410) {
#line 85
    case /*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID:
#line 85
      /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciInterrupts__rxDone(/*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID, data);
#line 85
      break;
#line 85
    default:
#line 85
      /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(arg_0x2b99bd185410, data);
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
# 88 "/opt/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__ArbiterInfo__userId(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId();
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 223 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__Usci__disableRxIntr(void )
#line 223
{
  HplMsp430UsciB0P__IE2 &= ~(1 << 2);
}

# 97 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB.nc"
inline static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__disableRxIntr(void ){
#line 97
  HplMsp430UsciB0P__Usci__disableRxIntr();
#line 97
}
#line 97
# 202 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__rxDone(uint8_t data)
#line 202
{

  if (/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_rx_buf) {
    /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_rx_buf[/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos - 1] = data;
    }
  if (/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos < /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_len) {
    /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__continueOp();
    }
  else 
#line 209
    {
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__disableRxIntr();
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone();
    }
}

# 89 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 89
{
}

# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
inline static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__Interrupts__rxDone(uint8_t arg_0x2b99bd185410, uint8_t data){
#line 85
  switch (arg_0x2b99bd185410) {
#line 85
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsciC*/Msp430UsciB0C__0__CLIENT_ID:
#line 85
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__rxDone(data);
#line 85
      break;
#line 85
    default:
#line 85
      /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__Interrupts__default__rxDone(arg_0x2b99bd185410, data);
#line 85
      break;
#line 85
    }
#line 85
}
#line 85
# 80 "/opt/tinyos-2.1.1/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__ArbiterInfo__inUse(void ){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse();
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 83 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__RawInterrupts__rxDone(uint8_t data)
#line 83
{
  if (/*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__ArbiterInfo__inUse()) {
    /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__Interrupts__rxDone(/*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__ArbiterInfo__userId(), data);
    }
}

# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
inline static void HplMsp430UsciB0P__Interrupts__rxDone(uint8_t data){
#line 85
  /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__RawInterrupts__rxDone(data);
#line 85
}
#line 85
# 88 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__UsciRawInterrupts__rxDone(uint8_t temp)
#line 88
{
  HplMsp430UsciB0P__Interrupts__rxDone(temp);
}

# 85 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB0RawInterruptsP__UsciB__rxDone(uint8_t data){
#line 85
  HplMsp430UsciB0P__UsciRawInterrupts__rxDone(data);
#line 85
}
#line 85
# 251 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UartP.nc"
static inline void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__sendDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error)
#line 251
{
}

# 57 "/opt/tinyos-2.1.1/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__sendDone(uint8_t arg_0x2b99bce0d7e0, uint8_t * buf, uint16_t len, error_t error){
#line 57
  switch (arg_0x2b99bce0d7e0) {
#line 57
    case /*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID:
#line 57
      HdlcTranslateC__UartStream__sendDone(buf, len, error);
#line 57
      break;
#line 57
    default:
#line 57
      /*Msp430Uart0P.UartP*/Msp430UartP__0__UartStream__default__sendDone(arg_0x2b99bce0d7e0, buf, len, error);
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
inline static void /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__txDone(uint8_t arg_0x2b99bd185410){
#line 80
  switch (arg_0x2b99bd185410) {
#line 80
    case /*PlatformSerialC.UartC.UsciC*/Msp430UsciA0C__0__CLIENT_ID:
#line 80
      /*Msp430Uart0P.UartP*/Msp430UartP__0__UsciInterrupts__txDone(/*PlatformSerialC.UartC*/Msp430Uart0C__0__CLIENT_ID);
#line 80
      break;
#line 80
    default:
#line 80
      /*Msp430UsciShareA0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(arg_0x2b99bd185410);
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
# 220 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__txDone(void )
#line 220
{
}

# 88 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__Interrupts__default__txDone(uint8_t id)
#line 88
{
}

# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
inline static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__Interrupts__txDone(uint8_t arg_0x2b99bd185410){
#line 80
  switch (arg_0x2b99bd185410) {
#line 80
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsciC*/Msp430UsciB0C__0__CLIENT_ID:
#line 80
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__txDone();
#line 80
      break;
#line 80
    default:
#line 80
      /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__Interrupts__default__txDone(arg_0x2b99bd185410);
#line 80
      break;
#line 80
    }
#line 80
}
#line 80
# 78 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__RawInterrupts__txDone(void )
#line 78
{
  if (/*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__ArbiterInfo__inUse()) {
    /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__Interrupts__txDone(/*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__ArbiterInfo__userId());
    }
}

# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciInterrupts.nc"
inline static void HplMsp430UsciB0P__Interrupts__txDone(void ){
#line 80
  /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__1__RawInterrupts__txDone();
#line 80
}
#line 80
# 92 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__UsciRawInterrupts__txDone(void )
#line 92
{
  HplMsp430UsciB0P__Interrupts__txDone();
}

# 80 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB0RawInterruptsP__UsciB__txDone(void ){
#line 80
  HplMsp430UsciB0P__UsciRawInterrupts__txDone();
#line 80
}
#line 80
# 146 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__clear(void )
#line 146
{
#line 146
  P1IFG &= ~(1 << 0);
}

#line 122
static inline void HplMsp430InterruptP__Port10__default__fired(void )
#line 122
{
#line 122
  HplMsp430InterruptP__Port10__clear();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port10__fired(void ){
#line 61
  HplMsp430InterruptP__Port10__default__fired();
#line 61
}
#line 61
# 147 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port11__clear(void )
#line 147
{
#line 147
  P1IFG &= ~(1 << 1);
}

#line 123
static inline void HplMsp430InterruptP__Port11__default__fired(void )
#line 123
{
#line 123
  HplMsp430InterruptP__Port11__clear();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port11__fired(void ){
#line 61
  HplMsp430InterruptP__Port11__default__fired();
#line 61
}
#line 61
# 212 "/opt/tinyos-2.1.1/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__InterruptFIFOP__fired(void )
#line 212
{
  if (CC2420ReceiveP__m_state == CC2420ReceiveP__S_STARTED) {

      CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_LENGTH;
      CC2420ReceiveP__beginReceive();
    }
  else 



    {
      CC2420ReceiveP__m_missed_packets++;
    }
}

# 57 "/opt/tinyos-2.1.1/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired(void ){
#line 57
  CC2420ReceiveP__InterruptFIFOP__fired();
#line 57
}
#line 57
# 66 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void )
#line 66
{
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port12__fired(void ){
#line 61
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired();
#line 61
}
#line 61
# 149 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port13__clear(void )
#line 149
{
#line 149
  P1IFG &= ~(1 << 3);
}

#line 125
static inline void HplMsp430InterruptP__Port13__default__fired(void )
#line 125
{
#line 125
  HplMsp430InterruptP__Port13__clear();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port13__fired(void ){
#line 61
  HplMsp430InterruptP__Port13__default__fired();
#line 61
}
#line 61
# 56 "/opt/tinyos-2.1.1/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__startDone_task__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__startDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 215 "/opt/tinyos-2.1.1/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Power__startOscillatorDone(void )
#line 215
{
  CC2420CsmaP__startDone_task__postTask();
}

# 76 "/opt/tinyos-2.1.1/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static void CC2420ControlP__CC2420Power__startOscillatorDone(void ){
#line 76
  CC2420CsmaP__CC2420Power__startOscillatorDone();
#line 76
}
#line 76
# 50 "/opt/tinyos-2.1.1/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__disable(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 418 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__InterruptCCA__fired(void )
#line 418
{
  CC2420ControlP__m_state = CC2420ControlP__S_XOSC_STARTED;
  CC2420ControlP__InterruptCCA__disable();
  CC2420ControlP__IOCFG1__write(0);
  CC2420ControlP__writeId();
  CC2420ControlP__CSN__set();
  CC2420ControlP__CSN__clr();
  CC2420ControlP__CC2420Power__startOscillatorDone();
}

# 57 "/opt/tinyos-2.1.1/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired(void ){
#line 57
  CC2420ControlP__InterruptCCA__fired();
#line 57
}
#line 57
# 66 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void )
#line 66
{
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port14__fired(void ){
#line 61
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired();
#line 61
}
#line 61
# 151 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__clear(void )
#line 151
{
#line 151
  P1IFG &= ~(1 << 5);
}

#line 127
static inline void HplMsp430InterruptP__Port15__default__fired(void )
#line 127
{
#line 127
  HplMsp430InterruptP__Port15__clear();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port15__fired(void ){
#line 61
  HplMsp430InterruptP__Port15__default__fired();
#line 61
}
#line 61
# 152 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port16__clear(void )
#line 152
{
#line 152
  P1IFG &= ~(1 << 6);
}

#line 128
static inline void HplMsp430InterruptP__Port16__default__fired(void )
#line 128
{
#line 128
  HplMsp430InterruptP__Port16__clear();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port16__fired(void ){
#line 61
  HplMsp430InterruptP__Port16__default__fired();
#line 61
}
#line 61
# 153 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port17__clear(void )
#line 153
{
#line 153
  P1IFG &= ~(1 << 7);
}

#line 129
static inline void HplMsp430InterruptP__Port17__default__fired(void )
#line 129
{
#line 129
  HplMsp430InterruptP__Port17__clear();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port17__fired(void ){
#line 61
  HplMsp430InterruptP__Port17__default__fired();
#line 61
}
#line 61
# 273 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port20__clear(void )
#line 273
{
#line 273
  P2IFG &= ~(1 << 0);
}

#line 249
static inline void HplMsp430InterruptP__Port20__default__fired(void )
#line 249
{
#line 249
  HplMsp430InterruptP__Port20__clear();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port20__fired(void ){
#line 61
  HplMsp430InterruptP__Port20__default__fired();
#line 61
}
#line 61
# 274 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port21__clear(void )
#line 274
{
#line 274
  P2IFG &= ~(1 << 1);
}

#line 250
static inline void HplMsp430InterruptP__Port21__default__fired(void )
#line 250
{
#line 250
  HplMsp430InterruptP__Port21__clear();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port21__fired(void ){
#line 61
  HplMsp430InterruptP__Port21__default__fired();
#line 61
}
#line 61
# 275 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port22__clear(void )
#line 275
{
#line 275
  P2IFG &= ~(1 << 2);
}

#line 251
static inline void HplMsp430InterruptP__Port22__default__fired(void )
#line 251
{
#line 251
  HplMsp430InterruptP__Port22__clear();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port22__fired(void ){
#line 61
  HplMsp430InterruptP__Port22__default__fired();
#line 61
}
#line 61
# 276 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port23__clear(void )
#line 276
{
#line 276
  P2IFG &= ~(1 << 3);
}

#line 252
static inline void HplMsp430InterruptP__Port23__default__fired(void )
#line 252
{
#line 252
  HplMsp430InterruptP__Port23__clear();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port23__fired(void ){
#line 61
  HplMsp430InterruptP__Port23__default__fired();
#line 61
}
#line 61
# 277 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port24__clear(void )
#line 277
{
#line 277
  P2IFG &= ~(1 << 4);
}

#line 253
static inline void HplMsp430InterruptP__Port24__default__fired(void )
#line 253
{
#line 253
  HplMsp430InterruptP__Port24__clear();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port24__fired(void ){
#line 61
  HplMsp430InterruptP__Port24__default__fired();
#line 61
}
#line 61
# 278 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port25__clear(void )
#line 278
{
#line 278
  P2IFG &= ~(1 << 5);
}

#line 254
static inline void HplMsp430InterruptP__Port25__default__fired(void )
#line 254
{
#line 254
  HplMsp430InterruptP__Port25__clear();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port25__fired(void ){
#line 61
  HplMsp430InterruptP__Port25__default__fired();
#line 61
}
#line 61
# 279 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port26__clear(void )
#line 279
{
#line 279
  P2IFG &= ~(1 << 6);
}

#line 255
static inline void HplMsp430InterruptP__Port26__default__fired(void )
#line 255
{
#line 255
  HplMsp430InterruptP__Port26__clear();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port26__fired(void ){
#line 61
  HplMsp430InterruptP__Port26__default__fired();
#line 61
}
#line 61
# 280 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__clear(void )
#line 280
{
#line 280
  P2IFG &= ~(1 << 7);
}

#line 256
static inline void HplMsp430InterruptP__Port27__default__fired(void )
#line 256
{
#line 256
  HplMsp430InterruptP__Port27__clear();
}

# 61 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port27__fired(void ){
#line 61
  HplMsp430InterruptP__Port27__default__fired();
#line 61
}
#line 61
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
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x2b99bc98c690){
#line 28
  switch (arg_0x2b99bc98c690) {
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
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x2b99bc98c690);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 136 "/opt/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

#line 96
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get();
#line 98
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 98
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type remaining;




  expires = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt;


  remaining = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type )(expires - now);


  if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 <= now) 
    {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY) 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 = now + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt = remaining - /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      remaining = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY;
    }
  else 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 += /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt = 0;
    }
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt((/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type )now << 0, 
  (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type )remaining << 0);
}

# 69 "/opt/tinyos-2.1.1/tos/lib/timer/TransformCounterC.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type high = /*Counter32khz32C.Transform*/TransformCounterC__0__m_upper;
      /*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type low = /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 76
      if (/*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*Counter32khz32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*Counter32khz32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 51 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 58
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )400U;

#line 61
        do {
#line 61
            t0 = t1;
#line 61
            t1 = * (volatile uint16_t * )400U;
          }
        while (
#line 61
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 62
          t1;

#line 62
          return __nesc_temp;
        }
      }
    }
  else 
#line 65
    {
      return * (volatile uint16_t * )400U;
    }
}

# 780 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__congestionBackoff(void )
#line 780
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 781
    {
      CC2420TransmitP__RadioBackoff__requestCongestionBackoff(CC2420TransmitP__m_msg);
      CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__myCongestionBackoff);
    }
#line 784
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/opt/tinyos-2.1.1/tos/system/RandomMlcgC.nc"
static uint32_t RandomMlcgC__Random__rand32(void )
#line 58
{
  uint32_t mlcg;
#line 59
  uint32_t p;
#line 59
  uint32_t q;
  uint64_t tmpseed;

#line 61
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      tmpseed = (uint64_t )33614U * (uint64_t )RandomMlcgC__seed;
      q = tmpseed;
      q = q >> 1;
      p = tmpseed >> 32;
      mlcg = p + q;
      if (mlcg & 0x80000000) {
          mlcg = mlcg & 0x7FFFFFFF;
          mlcg++;
        }
      RandomMlcgC__seed = mlcg;
    }
#line 73
    __nesc_atomic_end(__nesc_atomic); }
  return mlcg;
}

# 787 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static error_t CC2420TransmitP__acquireSpiResource(void )
#line 787
{
  error_t error = CC2420TransmitP__SpiResource__immediateRequest();

#line 789
  if (error != SUCCESS) {
      CC2420TransmitP__SpiResource__request();
    }
  return error;
}

# 126 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id)
#line 126
{
  error_t error;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    {
      if (CC2420SpiP__WorkingState__requestState(CC2420SpiP__S_BUSY) != SUCCESS) {
          {
            unsigned char __nesc_temp = 
#line 131
            EBUSY;

            {
#line 131
              __nesc_atomic_end(__nesc_atomic); 
#line 131
              return __nesc_temp;
            }
          }
        }
      if (CC2420SpiP__SpiResource__isOwner()) {
          CC2420SpiP__m_holder = id;
          error = SUCCESS;
        }
      else {
#line 139
        if ((error = CC2420SpiP__SpiResource__immediateRequest()) == SUCCESS) {
            CC2420SpiP__m_holder = id;
          }
        else {
            CC2420SpiP__WorkingState__toIdle();
          }
        }
    }
#line 146
    __nesc_atomic_end(__nesc_atomic); }
#line 146
  return error;
}

# 96 "/opt/tinyos-2.1.1/tos/system/StateImplP.nc"
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState)
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

# 174 "/opt/tinyos-2.1.1/tos/system/ArbiterP.nc"
static uint8_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(uint8_t id)
#line 174
{
  /* atomic removed: atomic calls only */
#line 175
  {
    if (/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__resId == id && /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY) {
        unsigned char __nesc_temp = 
#line 176
        TRUE;

#line 176
        return __nesc_temp;
      }
    else 
#line 177
      {
        unsigned char __nesc_temp = 
#line 177
        FALSE;

#line 177
        return __nesc_temp;
      }
  }
}

#line 130
static error_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void )
#line 130
{
  /* atomic removed: atomic calls only */
#line 131
  {
    if (/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__resId == /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id) {
        if (/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING) {
            /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask();
            {
              unsigned char __nesc_temp = 
#line 135
              SUCCESS;

#line 135
              return __nesc_temp;
            }
          }
        else {
#line 137
          if (/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_IMM_GRANTING) {
              /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;
              /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY;
              {
                unsigned char __nesc_temp = 
#line 140
                SUCCESS;

#line 140
                return __nesc_temp;
              }
            }
          }
      }
  }
#line 144
  return 0x0080;
}

# 188 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/HplMsp430UsciB0P.nc"
static void HplMsp430UsciB0P__Usci__setModeSpi(msp430_spi_union_config_t *config)
#line 188
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 189
    {
      HplMsp430UsciB0P__Usci__disableIntr();
      HplMsp430UsciB0P__Usci__clrIntr();
      HplMsp430UsciB0P__Usci__resetUsci(TRUE);
      HplMsp430UsciB0P__Usci__enableSpi();
      HplMsp430UsciB0P__configSpi(config);
      HplMsp430UsciB0P__Usci__resetUsci(FALSE);
    }
#line 196
    __nesc_atomic_end(__nesc_atomic); }
}

# 107 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__request(uint8_t id)
#line 107
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (CC2420SpiP__WorkingState__requestState(CC2420SpiP__S_BUSY) == SUCCESS) {
          CC2420SpiP__m_holder = id;
          if (CC2420SpiP__SpiResource__isOwner()) {
              CC2420SpiP__grant__postTask();
            }
          else {
              CC2420SpiP__SpiResource__request();
            }
        }
      else {
          CC2420SpiP__m_requests |= 1 << id;
        }
    }
#line 122
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 735 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__attemptSend(void )
#line 735
{
  uint8_t status;
  bool congestion = TRUE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 739
    {
      if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
          CC2420TransmitP__SFLUSHTX__strobe();
          CC2420TransmitP__releaseSpiResource();
          CC2420TransmitP__CSN__set();
          CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
          CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
          {
#line 746
            __nesc_atomic_end(__nesc_atomic); 
#line 746
            return;
          }
        }





      CC2420TransmitP__CSN__clr();
      status = CC2420TransmitP__m_cca ? CC2420TransmitP__STXONCCA__strobe() : CC2420TransmitP__STXON__strobe();
      if (!(status & CC2420_STATUS_TX_ACTIVE)) {
          status = CC2420TransmitP__SNOP__strobe();
          if (status & CC2420_STATUS_TX_ACTIVE) {
              congestion = FALSE;
            }
        }

      CC2420TransmitP__m_state = congestion ? CC2420TransmitP__S_SAMPLE_CCA : CC2420TransmitP__S_SFD;
      CC2420TransmitP__CSN__set();
    }
#line 765
    __nesc_atomic_end(__nesc_atomic); }

  if (congestion) {
      CC2420TransmitP__totalCcaChecks = 0;
      CC2420TransmitP__releaseSpiResource();
      CC2420TransmitP__congestionBackoff();
    }
  else 
#line 771
    {
      CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__CC2420_ABORT_PERIOD);
    }
}

# 318 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr)
#line 318
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 319
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 321
            0;

            {
#line 321
              __nesc_atomic_end(__nesc_atomic); 
#line 321
              return __nesc_temp;
            }
          }
        }
    }
#line 325
    __nesc_atomic_end(__nesc_atomic); }
#line 325
  return CC2420SpiP__SpiByte__write(addr);
}

# 131 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static uint8_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiByte__write(uint8_t tx)
#line 131
{
  uint8_t byte;


  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__tx(tx);
  while (!/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__isRxIntrPending()) ;
  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__clrRxIntr();
  byte = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__rx();

  return byte;
}

# 149 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__release(uint8_t id)
#line 149
{
  uint8_t i;

#line 151
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
    {
      if (CC2420SpiP__m_holder != id) {
          {
            unsigned char __nesc_temp = 
#line 153
            0x0080;

            {
#line 153
              __nesc_atomic_end(__nesc_atomic); 
#line 153
              return __nesc_temp;
            }
          }
        }
#line 156
      CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;
      if (!CC2420SpiP__m_requests) {
          CC2420SpiP__WorkingState__toIdle();
          CC2420SpiP__attemptRelease();
        }
      else {
          for (i = CC2420SpiP__m_holder + 1; ; i++) {
              i %= CC2420SpiP__RESOURCE_COUNT;

              if (CC2420SpiP__m_requests & (1 << i)) {
                  CC2420SpiP__m_holder = i;
                  CC2420SpiP__m_requests &= ~(1 << i);
                  CC2420SpiP__grant__postTask();
                  {
                    unsigned char __nesc_temp = 
#line 169
                    SUCCESS;

                    {
#line 169
                      __nesc_atomic_end(__nesc_atomic); 
#line 169
                      return __nesc_temp;
                    }
                  }
                }
            }
        }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
#line 175
  return SUCCESS;
}

#line 339
static error_t CC2420SpiP__attemptRelease(void )
#line 339
{


  if ((
#line 340
  CC2420SpiP__m_requests > 0
   || CC2420SpiP__m_holder != CC2420SpiP__NO_HOLDER)
   || !CC2420SpiP__WorkingState__isIdle()) {
      return 0x0080;
    }
  /* atomic removed: atomic calls only */
  CC2420SpiP__release = TRUE;
  CC2420SpiP__ChipSpiResource__releasing();
  /* atomic removed: atomic calls only */
#line 348
  {
    if (CC2420SpiP__release) {
        CC2420SpiP__SpiResource__release();
        {
          unsigned char __nesc_temp = 
#line 351
          SUCCESS;

#line 351
          return __nesc_temp;
        }
      }
  }
  return EBUSY;
}

# 45 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )25U |= 0x01 << 0;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

#line 46
static void /*HplMsp430GeneralIOC.P30*/HplMsp430GeneralIOP__16__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )25U &= ~(0x01 << 0);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 842 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__signalDone(error_t err)
#line 842
{
  /* atomic removed: atomic calls only */
#line 843
  CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
  CC2420TransmitP__abortSpiRelease = FALSE;
  CC2420TransmitP__ChipSpiResource__attemptRelease();
  CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, err);
}

# 38 "/opt/tinyos-2.1.1/tos/chips/msp430X/timer/GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(uint8_t mode)
#line 38
{
  /* atomic removed: atomic calls only */
#line 39
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(mode);
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents();
  }
  return SUCCESS;
}

# 46 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )29U &= ~(0x01 << 6);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

#line 45
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )29U |= 0x01 << 6;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 260 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len)
#line 262
{

  cc2420_status_t status = 0;
  uint8_t tmpLen = len;
  uint8_t * tmpData = (uint8_t * )data;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 268
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 270
            status;

            {
#line 270
              __nesc_atomic_end(__nesc_atomic); 
#line 270
              return __nesc_temp;
            }
          }
        }
    }
#line 274
    __nesc_atomic_end(__nesc_atomic); }
#line 274
  addr += offset;

  status = CC2420SpiP__SpiByte__write(addr | 0x80);
  CC2420SpiP__SpiByte__write((addr >> 1) & 0xc0);
  for (; len; len--) {
      CC2420SpiP__SpiByte__write(tmpData[tmpLen - len]);
    }

  return status;
}

# 137 "/opt/tinyos-2.1.1/tos/chips/cc2420/packet/CC2420PacketP.nc"
static void CC2420PacketP__PacketTimeStamp32khz__clear(message_t *msg)
{
  __nesc_hton_int8(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timesync.data, FALSE);
  __nesc_hton_uint32(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timestamp.data, CC2420_INVALID_TIMESTAMP);
}

# 96 "/opt/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get();
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type expires;
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type )remaining << 5);
}

# 69 "/opt/tinyos-2.1.1/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__1__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get();

#line 76
      if (/*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__1__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

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
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x2b99bc7d7a18){
#line 64
  switch (arg_0x2b99bc7d7a18) {
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
    case CC2420CsmaP__startDone_task:
#line 64
      CC2420CsmaP__startDone_task__runTask();
#line 64
      break;
#line 64
    case CC2420CsmaP__stopDone_task:
#line 64
      CC2420CsmaP__stopDone_task__runTask();
#line 64
      break;
#line 64
    case CC2420CsmaP__sendDone_task:
#line 64
      CC2420CsmaP__sendDone_task__runTask();
#line 64
      break;
#line 64
    case CC2420ControlP__sync:
#line 64
      CC2420ControlP__sync__runTask();
#line 64
      break;
#line 64
    case CC2420ControlP__syncDone:
#line 64
      CC2420ControlP__syncDone__runTask();
#line 64
      break;
#line 64
    case CC2420SpiP__grant:
#line 64
      CC2420SpiP__grant__runTask();
#line 64
      break;
#line 64
    case /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task:
#line 64
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task__runTask();
#line 64
      break;
#line 64
    case /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask:
#line 64
      /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask();
#line 64
      break;
#line 64
    case CC2420ReceiveP__receiveDone_task:
#line 64
      CC2420ReceiveP__receiveDone_task__runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired:
#line 64
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer:
#line 64
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask();
#line 64
      break;
#line 64
    case CC2420TinyosNetworkP__grantTask:
#line 64
      CC2420TinyosNetworkP__grantTask__runTask();
#line 64
      break;
#line 64
    case /*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask:
#line 64
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__sendTask__runTask();
#line 64
      break;
#line 64
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__CancelTask:
#line 64
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__CancelTask__runTask();
#line 64
      break;
#line 64
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask:
#line 64
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__errorTask__runTask();
#line 64
      break;
#line 64
    case /*CtpP.Router*/CtpRoutingEngineP__0__updateRouteTask:
#line 64
      /*CtpP.Router*/CtpRoutingEngineP__0__updateRouteTask__runTask();
#line 64
      break;
#line 64
    case /*CtpP.Router*/CtpRoutingEngineP__0__sendBeaconTask:
#line 64
      /*CtpP.Router*/CtpRoutingEngineP__0__sendBeaconTask__runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x2b99bc7d7a18);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 441 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static uint16_t LinkEstimatorP__LinkEstimator__getLinkQuality(am_addr_t neighbor)
#line 441
{
  uint8_t idx;

#line 443
  idx = LinkEstimatorP__findIdx(neighbor);
  if (idx == LinkEstimatorP__INVALID_RVAL) {
      return LinkEstimatorP__VERY_LARGE_ETX_VALUE;
    }
  else 
#line 446
    {
      if (LinkEstimatorP__NeighborTable[idx].flags & MATURE_ENTRY) {
          return LinkEstimatorP__NeighborTable[idx].etx;
        }
      else 
#line 449
        {
          return LinkEstimatorP__VERY_LARGE_ETX_VALUE;
        }
    }
}

#line 176
static uint8_t LinkEstimatorP__findIdx(am_addr_t ll_addr)
#line 176
{
  uint8_t i;

#line 178
  for (i = 0; i < 10; i++) {
      if (LinkEstimatorP__NeighborTable[i].flags & VALID_ENTRY) {
          if (LinkEstimatorP__NeighborTable[i].ll_addr == ll_addr) {
              return i;
            }
        }
    }
  return LinkEstimatorP__INVALID_RVAL;
}

#line 485
static error_t LinkEstimatorP__LinkEstimator__pinNeighbor(am_addr_t neighbor)
#line 485
{
  uint8_t nidx = LinkEstimatorP__findIdx(neighbor);

#line 487
  if (nidx == LinkEstimatorP__INVALID_RVAL) {
      return 0x0080;
    }
  LinkEstimatorP__NeighborTable[nidx].flags |= PINNED_ENTRY;
  return SUCCESS;
}

# 164 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static void /*CtpP.Router*/CtpRoutingEngineP__0__chooseAdvertiseTime(void )
#line 164
{
  /*CtpP.Router*/CtpRoutingEngineP__0__t = /*CtpP.Router*/CtpRoutingEngineP__0__currentInterval;
  /*CtpP.Router*/CtpRoutingEngineP__0__t /= 2;
  /*CtpP.Router*/CtpRoutingEngineP__0__t += /*CtpP.Router*/CtpRoutingEngineP__0__Random__rand32() % /*CtpP.Router*/CtpRoutingEngineP__0__t;
  /*CtpP.Router*/CtpRoutingEngineP__0__tHasPassed = FALSE;
  /*CtpP.Router*/CtpRoutingEngineP__0__BeaconTimer__stop();
  /*CtpP.Router*/CtpRoutingEngineP__0__BeaconTimer__startOneShot(/*CtpP.Router*/CtpRoutingEngineP__0__t);
}

# 133 "/opt/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 136
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 703 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static void *LinkEstimatorP__Packet__getPayload(message_t *msg, uint8_t len)
#line 703
{
  void *payload = LinkEstimatorP__SubPacket__getPayload(msg, len + sizeof(linkest_header_t ));

#line 705
  if (payload != (void *)0) {
      payload += sizeof(linkest_header_t );
    }
  return payload;
}

#line 405
static void LinkEstimatorP__print_packet(message_t *msg, uint8_t len)
#line 405
{
  uint8_t i;
  uint8_t *b;

  b = (uint8_t *)msg->data;
  for (i = 0; i < len; i++) 
    ;
  ;
}

# 156 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static void CC2420ActiveMessageP__AMPacket__setType(message_t *amsg, am_id_t type)
#line 156
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 158
  __nesc_hton_leuint8(header->type.data, type);
}

# 82 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len)
#line 83
{
  if (clientId >= 2) {
      return 0x0080;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[clientId].msg != (void *)0) {
      return EBUSY;
    }
  ;

  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[clientId].msg = msg;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Packet__setPayloadLength(msg, len);

  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current >= 2) {
      error_t err;
      am_id_t amId = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__type(msg);
      am_addr_t dest = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMPacket__destination(msg);

      ;
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current = clientId;

      err = /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__send(amId, dest, msg, len);
      if (err != SUCCESS) {
          ;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current = 2;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[clientId].msg = (void *)0;
        }

      return err;
    }
  else {
      ;
    }
  return SUCCESS;
}

# 185 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static void CC2420ActiveMessageP__Packet__setPayloadLength(message_t *msg, uint8_t len)
#line 185
{
  __nesc_hton_leuint8(CC2420ActiveMessageP__CC2420PacketBody__getHeader(msg)->length.data, len + CC2420_SIZE);
}

#line 151
static am_id_t CC2420ActiveMessageP__AMPacket__type(message_t *amsg)
#line 151
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 153
  return __nesc_ntoh_leuint8(header->type.data);
}

#line 126
static am_addr_t CC2420ActiveMessageP__AMPacket__destination(message_t *amsg)
#line 126
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 128
  return __nesc_ntoh_leuint16(header->dest.data);
}

#line 78
static error_t CC2420ActiveMessageP__AMSend__send(am_id_t id, am_addr_t addr, 
message_t *msg, 
uint8_t len)
#line 80
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(msg);

  if (len > CC2420ActiveMessageP__Packet__maxPayloadLength()) {
      return ESIZE;
    }

  __nesc_hton_leuint8(header->type.data, id);
  __nesc_hton_leuint16(header->dest.data, addr);
  __nesc_hton_leuint16(header->destpan.data, CC2420ActiveMessageP__CC2420Config__getPanAddr());
  __nesc_hton_leuint16(header->src.data, CC2420ActiveMessageP__AMPacket__address());

  if (CC2420ActiveMessageP__RadioResource__immediateRequest() == SUCCESS) {
      error_t rc;

#line 94
      CC2420ActiveMessageP__SendNotifier__aboutToSend(id, addr, msg);

      rc = CC2420ActiveMessageP__SubSend__send(msg, len);
      if (rc != SUCCESS) {
          CC2420ActiveMessageP__RadioResource__release();
        }

      return rc;
    }
  else 
#line 102
    {
      CC2420ActiveMessageP__pending_length = len;
      CC2420ActiveMessageP__pending_message = msg;
      return CC2420ActiveMessageP__RadioResource__request();
    }
}

# 95 "/opt/tinyos-2.1.1/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__amAddress(void )
#line 95
{
  am_addr_t myAddr;

#line 97
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 97
    myAddr = ActiveMessageAddressC__addr;
#line 97
    __nesc_atomic_end(__nesc_atomic); }
  return myAddr;
}

# 50 "/opt/tinyos-2.1.1/tos/system/FcfsResourceQueueC.nc"
static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void )
#line 50
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 51
    {
      unsigned char __nesc_temp = 
#line 51
      /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

      {
#line 51
        __nesc_atomic_end(__nesc_atomic); 
#line 51
        return __nesc_temp;
      }
    }
#line 53
    __nesc_atomic_end(__nesc_atomic); }
}

# 106 "/opt/tinyos-2.1.1/tos/chips/cc2420/packet/CC2420PacketP.nc"
static void CC2420PacketP__CC2420Packet__setNetwork(message_t *p_msg, uint8_t networkId)
#line 106
{

  __nesc_hton_leuint8(CC2420PacketP__CC2420PacketBody__getHeader(p_msg)->network.data, networkId);
}

# 75 "/opt/tinyos-2.1.1/tos/chips/cc2420/unique/UniqueSendP.nc"
static error_t UniqueSendP__Send__send(message_t *msg, uint8_t len)
#line 75
{
  error_t error;

#line 77
  if (UniqueSendP__State__requestState(UniqueSendP__S_SENDING) == SUCCESS) {
      __nesc_hton_leuint8(UniqueSendP__CC2420PacketBody__getHeader(msg)->dsn.data, UniqueSendP__localSendId++);

      if ((error = UniqueSendP__SubSend__send(msg, len)) != SUCCESS) {
          UniqueSendP__State__toIdle();
        }

      return error;
    }

  return EBUSY;
}

# 817 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__loadTXFIFO(void )
#line 817
{
  cc2420_header_t *header = CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg);
  uint8_t tx_power = __nesc_ntoh_uint8(CC2420TransmitP__CC2420PacketBody__getMetadata(CC2420TransmitP__m_msg)->tx_power.data);

  if (!tx_power) {
      tx_power = 31;
    }

  CC2420TransmitP__CSN__clr();

  if (CC2420TransmitP__m_tx_power != tx_power) {
      CC2420TransmitP__TXCTRL__write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
      3 << CC2420_TXCTRL_PA_CURRENT)) | (
      1 << CC2420_TXCTRL_RESERVED)) | ((
      tx_power & 0x1F) << CC2420_TXCTRL_PA_LEVEL));
    }

  CC2420TransmitP__m_tx_power = tx_power;

  {
    uint8_t tmpLen __attribute((unused))  = __nesc_ntoh_leuint8(header->length.data) - 1;

#line 838
    CC2420TransmitP__TXFIFO__write((uint8_t * )header, __nesc_ntoh_leuint8(header->length.data) - 1);
  }
}

# 305 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data)
#line 305
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 306
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 308
            0;

            {
#line 308
              __nesc_atomic_end(__nesc_atomic); 
#line 308
              return __nesc_temp;
            }
          }
        }
    }
#line 312
    __nesc_atomic_end(__nesc_atomic); }
#line 311
  CC2420SpiP__SpiByte__write(addr);
  CC2420SpiP__SpiByte__write(data >> 8);
  return CC2420SpiP__SpiByte__write(data & 0xff);
}

# 176 "/opt/tinyos-2.1.1/tos/chips/msp430X/usci/Msp430SpiNoDmaBP.nc"
static error_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len)
#line 178
{

  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_client = id;
  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_tx_buf = tx_buf;
  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_rx_buf = rx_buf;
  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_len = len;
  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos = 0;

  if (len) {
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__enableRxIntr();
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__continueOp();
    }
  else {
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task__postTask();
    }

  return SUCCESS;
}

#line 153
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__continueOp(void )
#line 153
{

  uint8_t end;
  uint8_t tmp;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 158
    {
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__tx(/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_tx_buf ? /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_tx_buf[/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos] : 0);

      end = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos + /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SPI_ATOMIC_SIZE;
      if (end > /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_len) {
        end = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_len;
        }
      while (++/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos < end) {
          while (!/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__isRxIntrPending()) ;
          tmp = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__rx();
          if (/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_rx_buf) {
            /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_rx_buf[/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos - 1] = tmp;
            }
#line 170
          /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__tx(/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_tx_buf ? /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_tx_buf[/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos] : 0);
        }
    }
#line 172
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static void UniqueSendP__State__toIdle(void ){
#line 56
  StateImplP__State__toIdle(7U);
#line 56
}
#line 56
# 533 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubSend__sendDone(message_t *msg, error_t error)
#line 533
{
  fe_queue_entry_t *qe = /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__head();

#line 535
  ;

  if (error != SUCCESS) {

      ;
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEventMsg(NET_C_FE_SENDDONE_FAIL, 
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getSequenceNumber(msg), 
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getOrigin(msg), 
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__destination(msg));
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__startRetxmitTimer(SENDDONE_FAIL_WINDOW, SENDDONE_FAIL_OFFSET);
    }
  else {
#line 546
    if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__hasState(/*CtpP.Forwarder*/CtpForwardingEngineP__0__ACK_PENDING) && !/*CtpP.Forwarder*/CtpForwardingEngineP__0__PacketAcknowledgements__wasAcked(msg)) {

        /*CtpP.Forwarder*/CtpForwardingEngineP__0__LinkEstimator__txNoAck(/*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__destination(msg));
        /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpInfo__recomputeRoutes();
        if (-- qe->retries) {
            ;
            /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEventMsg(NET_C_FE_SENDDONE_WAITACK, 
            /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getSequenceNumber(msg), 
            /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getOrigin(msg), 
            /*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__destination(msg));
            /*CtpP.Forwarder*/CtpForwardingEngineP__0__startRetxmitTimer(SENDDONE_NOACK_WINDOW, SENDDONE_NOACK_OFFSET);
          }
        else 
#line 557
          {

            /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__dequeue();
            /*CtpP.Forwarder*/CtpForwardingEngineP__0__clearState(/*CtpP.Forwarder*/CtpForwardingEngineP__0__SENDING);
            /*CtpP.Forwarder*/CtpForwardingEngineP__0__startRetxmitTimer(SENDDONE_OK_WINDOW, SENDDONE_OK_OFFSET);

            /*CtpP.Forwarder*/CtpForwardingEngineP__0__packetComplete(qe, msg, FALSE);
          }
      }
    else {



        /*CtpP.Forwarder*/CtpForwardingEngineP__0__SendQueue__dequeue();
        /*CtpP.Forwarder*/CtpForwardingEngineP__0__clearState(/*CtpP.Forwarder*/CtpForwardingEngineP__0__SENDING);
        /*CtpP.Forwarder*/CtpForwardingEngineP__0__startRetxmitTimer(SENDDONE_OK_WINDOW, SENDDONE_OK_OFFSET);
        /*CtpP.Forwarder*/CtpForwardingEngineP__0__LinkEstimator__txAck(/*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__destination(msg));
        /*CtpP.Forwarder*/CtpForwardingEngineP__0__packetComplete(qe, msg, TRUE);
      }
    }
}

# 115 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
static void * /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubPacket__getPayload(message_t * msg, uint8_t len){
#line 115
  void *__nesc_result;
#line 115

#line 115
  __nesc_result = CC2420ActiveMessageP__Packet__getPayload(msg, len);
#line 115

#line 115
  return __nesc_result;
#line 115
}
#line 115
# 798 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static am_addr_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getOrigin(message_t *msg)
#line 798
{
#line 798
  return __nesc_ntoh_uint16(/*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(msg)->origin.data);
}

#line 255
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__startRetxmitTimer(uint16_t window, uint16_t offset)
#line 255
{
  uint16_t r = /*CtpP.Forwarder*/CtpForwardingEngineP__0__Random__rand16();

#line 257
  r %= window;
  r += offset;
  /*CtpP.Forwarder*/CtpForwardingEngineP__0__RetxmitTimer__startOneShot(r);
  ;
}

# 62 "/opt/tinyos-2.1.1/tos/lib/timer/Timer.nc"
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__RetxmitTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(3U, dt);
#line 62
}
#line 62
# 281 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static void LinkEstimatorP__updateDETX(neighbor_table_entry_t *ne)
#line 281
{
  uint16_t estETX;

  if (ne->data_success == 0) {



      estETX = ne->data_total * 10;
    }
  else 
#line 289
    {
      estETX = 10 * ne->data_total / ne->data_success;
      ne->data_success = 0;
      ne->data_total = 0;
    }
  LinkEstimatorP__updateETX(ne, estETX);
}

#line 275
static void LinkEstimatorP__updateETX(neighbor_table_entry_t *ne, uint16_t newEst)
#line 275
{
  ne->etx = (LinkEstimatorP__ALPHA * ne->etx + (10 - LinkEstimatorP__ALPHA) * newEst) / 10;
}

# 85 "/opt/tinyos-2.1.1/tos/system/QueueC.nc"
static /*CtpP.SendQueueP*/QueueC__0__queue_t /*CtpP.SendQueueP*/QueueC__0__Queue__dequeue(void )
#line 85
{
  /*CtpP.SendQueueP*/QueueC__0__queue_t t = /*CtpP.SendQueueP*/QueueC__0__Queue__head();

#line 87
  ;
  if (!/*CtpP.SendQueueP*/QueueC__0__Queue__empty()) {
      /*CtpP.SendQueueP*/QueueC__0__head++;
      if (/*CtpP.SendQueueP*/QueueC__0__head == 13) {
#line 90
        /*CtpP.SendQueueP*/QueueC__0__head = 0;
        }
#line 91
      /*CtpP.SendQueueP*/QueueC__0__size--;
      /*CtpP.SendQueueP*/QueueC__0__printQueue();
    }
  return t;
}

# 489 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static void /*CtpP.Forwarder*/CtpForwardingEngineP__0__packetComplete(fe_queue_entry_t *qe, message_t *msg, bool success)
#line 489
{



  if (qe->client < /*CtpP.Forwarder*/CtpForwardingEngineP__0__CLIENT_COUNT) {
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__clientPtrs[qe->client] = qe;
      /*CtpP.Forwarder*/CtpForwardingEngineP__0__Send__sendDone(qe->client, msg, SUCCESS);
      if (success) {
          ;
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEventMsg(NET_C_FE_SENT_MSG, 
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getSequenceNumber(msg), 
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getOrigin(msg), 
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__destination(msg));
        }
      else 
#line 502
        {
          ;
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEventMsg(NET_C_FE_SENDDONE_FAIL_ACK_SEND, 
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getSequenceNumber(msg), 
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getOrigin(msg), 
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__destination(msg));
        }
    }
  else {
      if (success) {
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__SentCache__insert(qe->msg);
          ;
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEventMsg(NET_C_FE_FWD_MSG, 
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getSequenceNumber(msg), 
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getOrigin(msg), 
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__destination(msg));
        }
      else {
          ;
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEventMsg(NET_C_FE_SENDDONE_FAIL_ACK_FWD, 
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getSequenceNumber(msg), 
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionPacket__getOrigin(msg), 
          /*CtpP.Forwarder*/CtpForwardingEngineP__0__AMPacket__destination(msg));
        }
      if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__MessagePool__put(qe->msg) != SUCCESS) {
        /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_PUT_MSGPOOL_ERR);
        }
#line 528
      if (/*CtpP.Forwarder*/CtpForwardingEngineP__0__QEntryPool__put(qe) != SUCCESS) {
        /*CtpP.Forwarder*/CtpForwardingEngineP__0__CollectionDebug__logEvent(NET_C_FE_PUT_QEPOOL_ERR);
        }
    }
}

# 84 "/opt/tinyos-2.1.1/tos/lib/net/ctp/LruCtpMsgCacheP.nc"
static uint8_t /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__lookup(message_t *m)
#line 84
{
  uint8_t i;
  uint8_t idx;

#line 87
  for (i = 0; i < /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__count; i++) {
      idx = (i + /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__first) % 4;


      if (
#line 89
      /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getOrigin(m) == /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__cache[idx].origin && 
      /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getSequenceNumber(m) == /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__cache[idx].seqno && 
      /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getThl(m) == /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__cache[idx].thl && 
      /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__CtpPacket__getType(m) == /*CtpP.SentCacheP.CacheP*/LruCtpMsgCacheP__0__cache[idx].type) {
          break;
        }
    }
  return i;
}

# 807 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static am_addr_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getOrigin(message_t *msg)
#line 807
{
#line 807
  return __nesc_ntoh_uint16(/*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(msg)->origin.data);
}

#line 809
static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getSequenceNumber(message_t *msg)
#line 809
{
#line 809
  return __nesc_ntoh_uint8(/*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(msg)->originSeqNo.data);
}

#line 810
static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getThl(message_t *msg)
#line 810
{
#line 810
  return __nesc_ntoh_uint8(/*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(msg)->thl.data);
}

#line 806
static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__getType(message_t *msg)
#line 806
{
#line 806
  return __nesc_ntoh_uint8(/*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(msg)->type.data);
}

# 103 "/opt/tinyos-2.1.1/tos/system/PoolP.nc"
static error_t /*CtpP.MessagePoolP.PoolP*/PoolP__0__Pool__put(/*CtpP.MessagePoolP.PoolP*/PoolP__0__pool_t *newVal)
#line 103
{
  if (/*CtpP.MessagePoolP.PoolP*/PoolP__0__free >= 12) {
      return 0x0080;
    }
  else {
      uint16_t emptyIndex = /*CtpP.MessagePoolP.PoolP*/PoolP__0__index + /*CtpP.MessagePoolP.PoolP*/PoolP__0__free;

#line 109
      if (emptyIndex >= 12) {
          emptyIndex -= 12;
        }
      /*CtpP.MessagePoolP.PoolP*/PoolP__0__queue[emptyIndex] = newVal;
      /*CtpP.MessagePoolP.PoolP*/PoolP__0__free++;
      ;
      return SUCCESS;
    }
}

#line 103
static error_t /*CtpP.QEntryPoolP.PoolP*/PoolP__1__Pool__put(/*CtpP.QEntryPoolP.PoolP*/PoolP__1__pool_t *newVal)
#line 103
{
  if (/*CtpP.QEntryPoolP.PoolP*/PoolP__1__free >= 12) {
      return 0x0080;
    }
  else {
      uint16_t emptyIndex = /*CtpP.QEntryPoolP.PoolP*/PoolP__1__index + /*CtpP.QEntryPoolP.PoolP*/PoolP__1__free;

#line 109
      if (emptyIndex >= 12) {
          emptyIndex -= 12;
        }
      /*CtpP.QEntryPoolP.PoolP*/PoolP__1__queue[emptyIndex] = newVal;
      /*CtpP.QEntryPoolP.PoolP*/PoolP__1__free++;
      ;
      return SUCCESS;
    }
}

# 155 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__sendDone(uint8_t last, message_t * msg, error_t err)
#line 155
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[last].msg = (void *)0;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__tryToSend();
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__Send__sendDone(last, msg, err);
}

# 181 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static uint8_t CC2420ActiveMessageP__Packet__payloadLength(message_t *msg)
#line 181
{
  return __nesc_ntoh_leuint8(CC2420ActiveMessageP__CC2420PacketBody__getHeader(msg)->length.data) - CC2420_SIZE;
}

# 531 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__getEtx(uint16_t *etx)
#line 531
{
  if (etx == (void *)0) {
    return 0x0080;
    }
#line 534
  if (/*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent == INVALID_ADDR) {
    return 0x0080;
    }
#line 536
  if (/*CtpP.Router*/CtpRoutingEngineP__0__state_is_root == 1) {
      *etx = 0;
    }
  else 
#line 538
    {
      *etx = /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.etx + /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__getLinkQuality(/*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent);
    }
  return SUCCESS;
}

# 789 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static void */*CtpP.Forwarder*/CtpForwardingEngineP__0__Packet__getPayload(message_t *msg, uint8_t len)
#line 789
{
  uint8_t *payload = /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubPacket__getPayload(msg, len + sizeof(ctp_data_header_t ));

#line 791
  if (payload != (void *)0) {
      payload += sizeof(ctp_data_header_t );
    }
  return payload;
}

# 80 "TestAPIComplexP.nc"
static message_t *TestAPIComplexP__Receive__receive(message_t *msg, void *payload, uint8_t len)
{
  if (msg == (void *)0) {
#line 82
      assertTunitFail("msg"" was null", 26U);
#line 82
      ;
    }
  else 
#line 82
    {
#line 82
      assertTunitSuccess(27U);
#line 82
      ;
    }
#line 82
  ;
  TestAPIComplexP__TestRegister__done();
  return msg;
}

# 95 "/opt/tinyos-2.1.1/tos/interfaces/Packet.nc"
static uint8_t /*CtpP.Forwarder*/CtpForwardingEngineP__0__SubPacket__maxPayloadLength(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = CC2420ActiveMessageP__Packet__maxPayloadLength();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 65 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static void CC2420ActiveMessageP__RadioResource__granted(void )
#line 65
{
  uint8_t rc;
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(CC2420ActiveMessageP__pending_message);

  CC2420ActiveMessageP__SendNotifier__aboutToSend(__nesc_ntoh_leuint8(header->type.data), __nesc_ntoh_leuint16(header->dest.data), CC2420ActiveMessageP__pending_message);
  rc = CC2420ActiveMessageP__SubSend__send(CC2420ActiveMessageP__pending_message, CC2420ActiveMessageP__pending_length);
  if (rc != SUCCESS) {
      CC2420ActiveMessageP__RadioResource__release();
      CC2420ActiveMessageP__AMSend__sendDone(__nesc_ntoh_leuint8(header->type.data), CC2420ActiveMessageP__pending_message, rc);
    }
}

# 181 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__AMSend__sendDone(am_id_t id, message_t *msg, error_t err)
#line 181
{





  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current >= 2) {
      return;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current].msg == msg) {
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__1__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__1__current, msg, err);
    }
  else {
      ;
    }
}

# 62 "/opt/tinyos-2.1.1/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now)
{
  uint8_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 79
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 136 "/opt/tinyos-2.1.1/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

# 279 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void )
#line 279
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 280
    {
      unsigned int __nesc_temp = 
#line 280
      CC2420ControlP__m_short_addr;

      {
#line 280
        __nesc_atomic_end(__nesc_atomic); 
#line 280
        return __nesc_temp;
      }
    }
#line 282
    __nesc_atomic_end(__nesc_atomic); }
}

# 98 "/opt/tinyos-2.1.1/tos/chips/cc2420/packet/CC2420PacketP.nc"
static uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t *p_msg)
#line 98
{



  return __nesc_ntoh_leuint8(CC2420PacketP__CC2420PacketBody__getHeader(p_msg)->network.data);
}

# 131 "/opt/tinyos-2.1.1/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static am_addr_t CC2420ActiveMessageP__AMPacket__source(message_t *amsg)
#line 131
{
  cc2420_header_t *header = CC2420ActiveMessageP__CC2420PacketBody__getHeader(amsg);

#line 133
  return __nesc_ntoh_leuint16(header->src.data);
}

# 355 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static void LinkEstimatorP__updateNeighborEntryIdx(uint8_t idx, uint8_t seq)
#line 355
{
  uint8_t packetGap;

  if (LinkEstimatorP__NeighborTable[idx].flags & INIT_ENTRY) {
      ;
      LinkEstimatorP__NeighborTable[idx].lastseq = seq;
      LinkEstimatorP__NeighborTable[idx].flags &= ~INIT_ENTRY;
    }

  packetGap = seq - LinkEstimatorP__NeighborTable[idx].lastseq;
  ;

  LinkEstimatorP__NeighborTable[idx].lastseq = seq;
  LinkEstimatorP__NeighborTable[idx].rcvcnt++;
  if (packetGap > 0) {
      LinkEstimatorP__NeighborTable[idx].failcnt += packetGap - 1;
    }





  if (
#line 376
  LinkEstimatorP__NeighborTable[idx].rcvcnt + LinkEstimatorP__NeighborTable[idx].failcnt >= LinkEstimatorP__BLQ_PKT_WINDOW
   || packetGap >= LinkEstimatorP__BLQ_PKT_WINDOW) {
      LinkEstimatorP__updateNeighborTableEst(LinkEstimatorP__NeighborTable[idx].ll_addr);
    }

  if (packetGap > LinkEstimatorP__MAX_PKT_GAP) {
      LinkEstimatorP__initNeighborIdx(idx, LinkEstimatorP__NeighborTable[idx].ll_addr);
      LinkEstimatorP__NeighborTable[idx].lastseq = seq;
      LinkEstimatorP__NeighborTable[idx].rcvcnt = 1;
    }
}

#line 163
static void LinkEstimatorP__initNeighborIdx(uint8_t i, am_addr_t ll_addr)
#line 163
{
  neighbor_table_entry_t *ne;

#line 165
  ne = &LinkEstimatorP__NeighborTable[i];
  ne->ll_addr = ll_addr;
  ne->lastseq = 0;
  ne->rcvcnt = 0;
  ne->failcnt = 0;
  ne->flags = INIT_ENTRY | VALID_ENTRY;
  ne->inquality = 0;
  ne->etx = 0;
}

#line 189
static uint8_t LinkEstimatorP__findEmptyNeighborIdx(void )
#line 189
{
  uint8_t i;

#line 191
  for (i = 0; i < 10; i++) {
      if (LinkEstimatorP__NeighborTable[i].flags & VALID_ENTRY) {
        }
      else 
#line 193
        {
          return i;
        }
    }
  return LinkEstimatorP__INVALID_RVAL;
}



static uint8_t LinkEstimatorP__findWorstNeighborIdx(uint8_t thresholdETX)
#line 202
{
  uint8_t i;
#line 203
  uint8_t worstNeighborIdx;
  uint16_t worstETX;
#line 204
  uint16_t thisETX;

  worstNeighborIdx = LinkEstimatorP__INVALID_RVAL;
  worstETX = 0;
  for (i = 0; i < 10; i++) {
      if (!(LinkEstimatorP__NeighborTable[i].flags & VALID_ENTRY)) {
          ;
          continue;
        }
      if (!(LinkEstimatorP__NeighborTable[i].flags & MATURE_ENTRY)) {
          ;
          continue;
        }
      if (LinkEstimatorP__NeighborTable[i].flags & PINNED_ENTRY) {
          ;
          continue;
        }
      thisETX = LinkEstimatorP__NeighborTable[i].etx;
      if (thisETX >= worstETX) {
          worstNeighborIdx = i;
          worstETX = thisETX;
        }
    }
  if (worstETX >= thresholdETX) {
      return worstNeighborIdx;
    }
  else 
#line 229
    {
      return LinkEstimatorP__INVALID_RVAL;
    }
}

# 502 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static void /*CtpP.Router*/CtpRoutingEngineP__0__LinkEstimator__evicted(am_addr_t neighbor)
#line 502
{
  /*CtpP.Router*/CtpRoutingEngineP__0__routingTableEvict(neighbor);
  ;
  if (/*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent == neighbor) {
      routeInfoInit(&/*CtpP.Router*/CtpRoutingEngineP__0__routeInfo);
      /*CtpP.Router*/CtpRoutingEngineP__0__justEvicted = TRUE;
      /*CtpP.Router*/CtpRoutingEngineP__0__updateRouteTask__postTask();
    }
}

#line 687
static uint8_t /*CtpP.Router*/CtpRoutingEngineP__0__routingTableFind(am_addr_t neighbor)
#line 687
{
  uint8_t i;

#line 689
  if (neighbor == INVALID_ADDR) {
    return /*CtpP.Router*/CtpRoutingEngineP__0__routingTableActive;
    }
#line 691
  for (i = 0; i < /*CtpP.Router*/CtpRoutingEngineP__0__routingTableActive; i++) {
      if (/*CtpP.Router*/CtpRoutingEngineP__0__routingTable[i].neighbor == neighbor) {
        break;
        }
    }
#line 695
  return i;
}

# 679 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static uint8_t LinkEstimatorP__Packet__payloadLength(message_t *msg)
#line 679
{
  linkest_header_t *hdr;

#line 681
  hdr = LinkEstimatorP__getHeader(msg);
  return LinkEstimatorP__SubPacket__payloadLength(msg)
   - sizeof(linkest_header_t )
   - sizeof(linkest_footer_t ) * (NUM_ENTRIES_FLAG & __nesc_ntoh_uint8(hdr->flags.data));
}

# 775 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static bool /*CtpP.Router*/CtpRoutingEngineP__0__CtpRoutingPacket__getOption(message_t *msg, ctp_options_t opt)
#line 775
{
  return (__nesc_ntoh_uint8(/*CtpP.Router*/CtpRoutingEngineP__0__getHeader(msg)->options.data) & opt) == opt ? TRUE : FALSE;
}

# 698 "/opt/tinyos-2.1.1/tos/lib/net/4bitle/LinkEstimatorP.nc"
static uint8_t LinkEstimatorP__Packet__maxPayloadLength(void )
#line 698
{
  return LinkEstimatorP__SubPacket__maxPayloadLength() - sizeof(linkest_header_t );
}

# 556 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static void /*CtpP.Router*/CtpRoutingEngineP__0__CtpInfo__setNeighborCongested(am_addr_t n, bool congested)
#line 556
{
  uint8_t idx;

#line 558
  if (/*CtpP.Router*/CtpRoutingEngineP__0__ECNOff) {
    return;
    }
#line 560
  idx = /*CtpP.Router*/CtpRoutingEngineP__0__routingTableFind(n);
  if (idx < /*CtpP.Router*/CtpRoutingEngineP__0__routingTableActive) {
      /*CtpP.Router*/CtpRoutingEngineP__0__routingTable[idx].info.congested = congested;
    }
  if (/*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.congested && !congested) {
    /*CtpP.Router*/CtpRoutingEngineP__0__updateRouteTask__postTask();
    }
  else {
#line 566
    if (/*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent == n && congested) {
      /*CtpP.Router*/CtpRoutingEngineP__0__updateRouteTask__postTask();
      }
    }
}

# 97 "/opt/tinyos-2.1.1/tos/system/QueueC.nc"
static error_t /*CtpP.SendQueueP*/QueueC__0__Queue__enqueue(/*CtpP.SendQueueP*/QueueC__0__queue_t newVal)
#line 97
{
  if (/*CtpP.SendQueueP*/QueueC__0__Queue__size() < /*CtpP.SendQueueP*/QueueC__0__Queue__maxSize()) {
      ;
      /*CtpP.SendQueueP*/QueueC__0__queue[/*CtpP.SendQueueP*/QueueC__0__tail] = newVal;
      /*CtpP.SendQueueP*/QueueC__0__tail++;
      if (/*CtpP.SendQueueP*/QueueC__0__tail == 13) {
#line 102
        /*CtpP.SendQueueP*/QueueC__0__tail = 0;
        }
#line 103
      /*CtpP.SendQueueP*/QueueC__0__size++;
      /*CtpP.SendQueueP*/QueueC__0__printQueue();
      return SUCCESS;
    }
  else {
      return 0x0080;
    }
}

# 816 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpForwardingEngineP.nc"
static bool /*CtpP.Forwarder*/CtpForwardingEngineP__0__CtpPacket__option(message_t *msg, ctp_options_t opt)
#line 816
{
  return (__nesc_ntoh_uint8(/*CtpP.Forwarder*/CtpForwardingEngineP__0__getHeader(msg)->options.data) & opt) == opt ? TRUE : FALSE;
}

# 764 "/opt/tinyos-2.1.1/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static void CC2420ReceiveP__waitForNextPacket(void )
#line 764
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 765
    {
      if (CC2420ReceiveP__m_state == CC2420ReceiveP__S_STOPPED) {
          CC2420ReceiveP__SpiResource__release();
          {
#line 768
            __nesc_atomic_end(__nesc_atomic); 
#line 768
            return;
          }
        }
      CC2420ReceiveP__receivingPacket = FALSE;
#line 783
      if ((CC2420ReceiveP__m_missed_packets && CC2420ReceiveP__FIFO__get()) || !CC2420ReceiveP__FIFOP__get()) {

          if (CC2420ReceiveP__m_missed_packets) {
              CC2420ReceiveP__m_missed_packets--;
            }





          CC2420ReceiveP__beginReceive();
        }
      else 
        {

          CC2420ReceiveP__m_state = CC2420ReceiveP__S_STARTED;
          CC2420ReceiveP__m_missed_packets = 0;
          CC2420ReceiveP__SpiResource__release();
        }
    }
#line 802
    __nesc_atomic_end(__nesc_atomic); }
}

#line 711
static void CC2420ReceiveP__beginReceive(void )
#line 711
{
  CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_LENGTH;
  /* atomic removed: atomic calls only */
#line 713
  CC2420ReceiveP__receivingPacket = TRUE;
  if (CC2420ReceiveP__SpiResource__isOwner()) {
      CC2420ReceiveP__receive();
    }
  else {
#line 717
    if (CC2420ReceiveP__SpiResource__immediateRequest() == SUCCESS) {
        CC2420ReceiveP__receive();
      }
    else {
        CC2420ReceiveP__SpiResource__request();
      }
    }
}

#line 754
static void CC2420ReceiveP__receive(void )
#line 754
{
  CC2420ReceiveP__CSN__clr();
  CC2420ReceiveP__RXFIFO__beginRead((uint8_t *)CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf), 1);
}

# 189 "/opt/tinyos-2.1.1/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 190
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 194
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 196
            status;

            {
#line 196
              __nesc_atomic_end(__nesc_atomic); 
#line 196
              return __nesc_temp;
            }
          }
        }
    }
#line 200
    __nesc_atomic_end(__nesc_atomic); }
#line 200
  CC2420SpiP__m_addr = addr | 0x40;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__Fifo__continueRead(addr, data, len);

  return status;
}

#line 329
static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error)
#line 330
{
  if (CC2420SpiP__m_addr & 0x40) {
      CC2420SpiP__Fifo__readDone(CC2420SpiP__m_addr & ~0x40, rx_buf, len, error);
    }
  else 
#line 333
    {
      CC2420SpiP__Fifo__writeDone(CC2420SpiP__m_addr, tx_buf, len, error);
    }
}

# 728 "/opt/tinyos-2.1.1/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static void CC2420ReceiveP__flush(void )
#line 728
{








  CC2420ReceiveP__reset_state();

  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__CSN__clr();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__SpiResource__release();
  CC2420ReceiveP__waitForNextPacket();
}

#line 808
static void CC2420ReceiveP__reset_state(void )
#line 808
{
  CC2420ReceiveP__m_bytes_left = CC2420ReceiveP__RXFIFO_SIZE;
  /* atomic removed: atomic calls only */
#line 810
  CC2420ReceiveP__receivingPacket = FALSE;
  CC2420ReceiveP__m_timestamp_head = 0;
  CC2420ReceiveP__m_timestamp_size = 0;
  CC2420ReceiveP__m_missed_packets = 0;
}

# 456 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static void CC2420ControlP__writeFsctrl(void )
#line 456
{
  uint8_t channel;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 459
    {
      channel = CC2420ControlP__m_channel;
    }
#line 461
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP__FSCTRL__write((1 << CC2420_FSCTRL_LOCK_THR) | (((
  channel - 11) * 5 + 357) << CC2420_FSCTRL_FREQ));
}







static void CC2420ControlP__writeMdmctrl0(void )
#line 473
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 474
    {
      CC2420ControlP__MDMCTRL0__write((((((((1 << CC2420_MDMCTRL0_RESERVED_FRAME_MODE) | ((
      CC2420ControlP__addressRecognition && CC2420ControlP__hwAddressRecognition) << CC2420_MDMCTRL0_ADR_DECODE)) | (
      2 << CC2420_MDMCTRL0_CCA_HYST)) | (
      3 << CC2420_MDMCTRL0_CCA_MOD)) | (
      1 << CC2420_MDMCTRL0_AUTOCRC)) | ((
      CC2420ControlP__autoAckEnabled && CC2420ControlP__hwAutoAckDefault) << CC2420_MDMCTRL0_AUTOACK)) | (
      0 << CC2420_MDMCTRL0_AUTOACK)) | (
      2 << CC2420_MDMCTRL0_PREAMBLE_LENGTH));
    }
#line 483
    __nesc_atomic_end(__nesc_atomic); }
}







static void CC2420ControlP__writeId(void )
#line 492
{
  nxle_uint16_t id[2];

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 495
    {
      __nesc_hton_leuint16(id[0].data, CC2420ControlP__m_pan);
      __nesc_hton_leuint16(id[1].data, CC2420ControlP__m_short_addr);
    }
#line 498
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP__PANID__write(0, (uint8_t *)&id, sizeof id);
}

# 179 "/opt/tinyos-2.1.1/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static error_t CC2420TransmitP__StdControl__stop(void )
#line 179
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 180
    {
      CC2420TransmitP__m_state = CC2420TransmitP__S_STOPPED;
      CC2420TransmitP__BackoffTimer__stop();
      CC2420TransmitP__CaptureSFD__disable();
      CC2420TransmitP__SpiResource__release();
      CC2420TransmitP__CSN__set();
    }
#line 186
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 171 "/opt/tinyos-2.1.1/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static error_t CC2420ReceiveP__StdControl__stop(void )
#line 171
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 172
    {
      CC2420ReceiveP__m_state = CC2420ReceiveP__S_STOPPED;
      CC2420ReceiveP__reset_state();
      CC2420ReceiveP__CSN__set();
      CC2420ReceiveP__InterruptFIFOP__disable();
    }
#line 177
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 199 "/opt/tinyos-2.1.1/tos/chips/cc2420/control/CC2420ControlP.nc"
static error_t CC2420ControlP__CC2420Power__stopVReg(void )
#line 199
{
  CC2420ControlP__m_state = CC2420ControlP__S_VREG_STOPPED;
  CC2420ControlP__RSTN__clr();
  CC2420ControlP__VREN__clr();
  CC2420ControlP__RSTN__set();
  return SUCCESS;
}

# 56 "/opt/tinyos-2.1.1/tos/interfaces/State.nc"
static void TUnitP__TestState__toIdle(void ){
#line 56
  StateImplP__State__toIdle(3U);
#line 56
}
#line 56
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

# 301 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunit/TUnitP.nc"
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
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current < 2) {
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

# 109 "/opt/tinyos-2.x-contrib/tunit/tos/lib/tunitstats/StatisticsP.nc"
static void StatisticsP__AMSend__sendDone(message_t *msg, error_t error)
#line 109
{
  StatisticsMsg *focusedMsg;

  if (StatisticsP__FifoQueue__dequeue(&focusedMsg, 28) == SUCCESS) {
      memcpy((&StatisticsP__myMsg)->data, focusedMsg, sizeof(StatisticsMsg ));
      StatisticsP__AMSend__send(0, &StatisticsP__myMsg, sizeof(StatisticsMsg ));
    }
  else {
      StatisticsP__State__toIdle();
    }
}

# 147 "/opt/tinyos-2.1.1/tos/lib/serial/SerialActiveMessageP.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(message_t *amsg, am_addr_t addr)
#line 147
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 149
  __nesc_hton_uint16(header->dest.data, addr);
}

#line 166
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(message_t *amsg, am_id_t type)
#line 166
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 168
  __nesc_hton_uint8(header->type.data, type);
}

# 82 "/opt/tinyos-2.1.1/tos/system/AMQueueImplP.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len)
#line 83
{
  if (clientId >= 2) {
      return 0x0080;
    }
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg != (void *)0) {
      return EBUSY;
    }
  ;

  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = msg;
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(msg, len);

  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 2) {
      error_t err;
      am_id_t amId = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(msg);
      am_addr_t dest = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(msg);

      ;
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = clientId;

      err = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(amId, dest, msg, len);
      if (err != SUCCESS) {
          ;
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 2;
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = (void *)0;
        }

      return err;
    }
  else {
      ;
    }
  return SUCCESS;
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

# 81 "/opt/tinyos-2.1.1/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static error_t CC2420CsmaP__SplitControl__start(void )
#line 81
{
  if (CC2420CsmaP__SplitControlState__requestState(CC2420CsmaP__S_STARTING) == SUCCESS) {
      CC2420CsmaP__CC2420Power__startVReg();
      return SUCCESS;
    }
  else {
#line 86
    if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTED)) {
        return EALREADY;
      }
    else {
#line 89
      if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTING)) {
          return SUCCESS;
        }
      }
    }
#line 93
  return EBUSY;
}

# 46 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )49U &= ~(0x01 << 5);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 586 "/opt/tinyos-2.1.1/tos/lib/net/ctp/CtpRoutingEngineP.nc"
static error_t /*CtpP.Router*/CtpRoutingEngineP__0__RootControl__setRoot(void )
#line 586
{
  bool route_found = FALSE;

#line 588
  route_found = /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent == INVALID_ADDR;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 589
    {
      /*CtpP.Router*/CtpRoutingEngineP__0__state_is_root = 1;
      /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent = /*CtpP.Router*/CtpRoutingEngineP__0__my_ll_addr;
      /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.etx = 0;
    }
#line 593
    __nesc_atomic_end(__nesc_atomic); }
  if (route_found) {
    /*CtpP.Router*/CtpRoutingEngineP__0__Routing__routeFound();
    }
#line 596
  ;
  /*CtpP.Router*/CtpRoutingEngineP__0__CollectionDebug__logEventRoute(NET_C_TREE_NEW_PARENT, /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.parent, 0, /*CtpP.Router*/CtpRoutingEngineP__0__routeInfo.etx);
  return SUCCESS;
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

#line 150
static bool /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void )
#line 150
{
  /* atomic removed: atomic calls only */
#line 151
  {
    if (/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) 
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






static uint8_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void )
#line 163
{
  /* atomic removed: atomic calls only */
#line 164
  {
    if (/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__state != /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 166
        /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__NO_RES;

#line 166
        return __nesc_temp;
      }
#line 167
    {
      unsigned char __nesc_temp = 
#line 167
      /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__1__resId;

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

# 84 "/opt/tinyos-2.1.1/tos/chips/msp430X/pins/HplMsp430InterruptP.nc"
__attribute((wakeup)) __attribute((interrupt(36)))  void sig_PORT1_VECTOR(void )
{
  volatile int n = P1IFG & P1IE;

#line 87
  if (n & (1 << 0)) {
      HplMsp430InterruptP__Port10__fired();
      return;
    }
  if (n & (1 << 1)) {
      HplMsp430InterruptP__Port11__fired();
      return;
    }
  if (n & (1 << 2)) {
      HplMsp430InterruptP__Port12__fired();
      return;
    }
  if (n & (1 << 3)) {
      HplMsp430InterruptP__Port13__fired();
      return;
    }
  if (n & (1 << 4)) {
      HplMsp430InterruptP__Port14__fired();
      return;
    }
  if (n & (1 << 5)) {
      HplMsp430InterruptP__Port15__fired();
      return;
    }
  if (n & (1 << 6)) {
      HplMsp430InterruptP__Port16__fired();
      return;
    }
  if (n & (1 << 7)) {
      HplMsp430InterruptP__Port17__fired();
      return;
    }
}

#line 213
__attribute((wakeup)) __attribute((interrupt(38)))  void sig_PORT2_VECTOR(void )
{
  volatile int n = P2IFG & P2IE;

#line 216
  if (n & (1 << 0)) {
      HplMsp430InterruptP__Port20__fired();
      return;
    }
  if (n & (1 << 1)) {
      HplMsp430InterruptP__Port21__fired();
      return;
    }
  if (n & (1 << 2)) {
      HplMsp430InterruptP__Port22__fired();
      return;
    }
  if (n & (1 << 3)) {
      HplMsp430InterruptP__Port23__fired();
      return;
    }
  if (n & (1 << 4)) {
      HplMsp430InterruptP__Port24__fired();
      return;
    }
  if (n & (1 << 5)) {
      HplMsp430InterruptP__Port25__fired();
      return;
    }
  if (n & (1 << 6)) {
      HplMsp430InterruptP__Port26__fired();
      return;
    }
  if (n & (1 << 7)) {
      HplMsp430InterruptP__Port27__fired();
      return;
    }
}


/* -------------------------------------------------------------------------- */
/*                        Digital controller for Qspice                       */
/* -------------------------------------------------------------------------- */

/*
    Author: Rafael J. Scholtz
    Description: this is intended to be a template the for user to implement a
    digital control algorithm in a simulation. Because pwm is so relevant in
    this scenario, a pwm class was also implemented, but other forms of modula-
    tion (or lack of) can also be implemented.

    This code was adapted from physicsboy [1] and robdunn4 [2]:
    [1]. https://github.com/physicboy/QSPICE
    [2]. https://github.com/robdunn4/QSpice
*/

#include <cstdlib>
#include <math.h>
#include <cstring>

/* -------------------------------------------------------------------------- */
/*                             Version information                            */
/* -------------------------------------------------------------------------- */
#define PROGRAM_NAME    "digital_controller"
#define PROGRAM_VERSION "v0.1"
#define PROGRAM_INFO    PROGRAM_NAME " " PROGRAM_VERSION

// must follow above versioning information
#include "Cblock.h"



/*------------------------------------------------------------------------------
 * DbgLog - useful for debugging
 *----------------------------------------------------------------------------*/
// declare DbgLog instance for logging; instance name must be dbgLog; change
// file name and max line limit if desired (-1 = max).
#include "DbgLog.h"
DbgLog dbgLog("@qdebug.log", 500);


/* -------------------------------------------------------------------------- */
/*                         Custom types and defines                           */
/* -------------------------------------------------------------------------- */

// Pi
#define M_PI            3.14159265358979323846  /* pi */

// vline normalization factor (max value of vline_sense)
#define VLINE_SENSE_MAX 3.1113

// Struct with control output variables
struct cntrl_out_str{
    double duty;
    bool invert_pol;
};


// Struct with control input variables
struct inputData{
  double  v_ref;
  double  vline_sense;
  double  vo_sense;
  double  IL_sense;
};



/* -------------------------------------------------------------------------- */
/*                                Control class                               */
/* -------------------------------------------------------------------------- */
class control
{
  public:

  // General
  double fs;
  double duty_max_ss;

  // Some signals
  double vref;
  double vline_sense;
  double vin_norm;
  double vin_norm_n1;
  double vo_sense;
  double ilb_sense;

  // Current controller parameters
  double kpi;
  double kii;
  double ei_n1;
  double ui_n1;

  // Voltage controller parameters
  double kpv;
  double kiv;
  double ev_n1;
  double uv_n1;
  double uv_n2;

  // Control output for pwm module
  struct cntrl_out_str cntrl_out;

  // PLL
  double wn;        // Nominal grid freq.
  double wg;
  double wg_n1;
  double theta;     // PLL output angle
  double theta_n1;
  double vq;
  double vq_n1;
  double vq_n2;
  double vq_n3;
  double ma_out;
  double pi_out;
  double pi_in;
  double pi_in_n1;
  unsigned long long pll_count;
  unsigned long long pll_count_max;

  /*
    PLL function
  */
  void PLL_MA(){

    // MA only runs at 60*8 Hz - so requires decimation
    double vin = vline_sense / VLINE_SENSE_MAX;    // normalizes vin
    pll_count++;
    if(pll_count >= pll_count_max){

      // Updates past self values
      vq_n3 = vq_n2;
      vq_n2 = vq_n1;
      vq_n1 = vq;

      // Multiplies internal angle with input
      vq = -2*sin(theta) * vin;
      ma_out = (vq + vq_n1 + vq_n2 + vq_n3)/4;

      // Resets decimation counter
      pll_count = 0;

    }

    // PI filter
    pi_in = ma_out;
    pi_out = pi_out + (100*pi_in - 99.97*pi_in_n1);


    // Adds nominal freq
    wg = wn + pi_out;

    // Integrates freq.
    theta = theta_n1 + wg_n1*(1/fs);

    // Saves past values
    theta_n1 = theta;
    pi_in_n1 = pi_in;

    wg_n1 = wg;
  }


  /*
    main control routine - called at every valley of carrier
  */
  cntrl_out_str* cascade_pid(inputData *data){

    // Updates input data
    vref = data->v_ref;
    vo_sense = data->vo_sense;
    ilb_sense = data->IL_sense;
    vline_sense = data->vline_sense;

    // ---- PLL with moving average ---- //
    // Decimation by 16.2e3 for sampling freq. of 480 Hz
    PLL_MA();

    // Classic PFC cascade control
    double v_error = vref - vo_sense;
    //double u_v = uv_n1 + kpv*v_error + kiv*ev_n1;
    double u_v = 1.9904*uv_n1 - 0.9904*uv_n2 + 0.0030802*v_error - 0.0030802*0.9988*ev_n1;

    if(u_v >= 0.3)  u_v = 0.3;
    if(u_v < 0) u_v = 0;

    // store [n-1] sampling
    ev_n1  = v_error;
    uv_n2 = uv_n1;
    uv_n1 = u_v;

    // Gain for sinusoidal reference
    //double i_ref = u_v*fabs(vline_sense)/VLINE_SENSE_MAX;
    vin_norm = cos(theta);
    double i_ref = u_v*fabs(vin_norm);

    // Current loop
    double i_error = i_ref - fabs(ilb_sense);
    //double duty = ui_n1 + 5*i_error + (1-5)*ei_n1;
    double duty = ui_n1 + kpi*i_error + kii*ei_n1;

    // Min and max duty-values - no antiwindup as its after storage

    // --- Soft-start limiter of max duty --- //
    if(duty_max_ss < 0.9){
      duty_max_ss += 0.9/(1*fs);     // Requires 0,2 seconds to achieve max value
    }

    // Duty-cycle limiter
    if(duty > duty_max_ss)  duty = duty_max_ss;
    if(duty < 0)  duty = 0;

    // store [n-1] sampling
    ei_n1  = i_error;
    ui_n1 = duty;
    vin_norm_n1 = vin_norm;

    cntrl_out.duty = duty;
    //cntrl_out.duty = 0.3;
    if(vin_norm >= 0){
      cntrl_out.invert_pol = true;
    }
    else{
      if (vin_norm < 0) cntrl_out.invert_pol = false;
    }

    return &cntrl_out;

  };

  // Constructor
  control(double kpi_arg = 1, double kii_arg = 1, double kpv_arg = 1, \
  double kiv_arg = 1, double fs_arg = 40e3, double fline = 60){

    // General
    fs = fs_arg;
    vin_norm = 0;
    vin_norm_n1 = 0;
    duty_max_ss = 0;

    // Current
    kpi = kpi_arg;
    kii = kii_arg;

    // Voltage
    kpv = kpv_arg;
    kiv = kiv_arg;

    // Control signals
    ei_n1 = 0;
    ui_n1 = 0;
    ev_n1 = 0;
    uv_n1 = 0;

    // Other signals
    vref = 0;
    vline_sense = 0;
    vo_sense = 0;
    ilb_sense = 0;

    // PLL
    wn = 2*M_PI*fline;        // Nominal grid freq.
    wg = 0;
    theta = 0;     // PLL output angle
    theta_n1 = 0;
    vq = 0;
    vq_n1 = 0;
    vq_n2 = 0;
    vq_n3 = 0;
    ma_out = 0;
    pll_count = 0;
    pll_count_max = (unsigned long long)round(fs_arg/(fline*2*4));

    // Output structure
    cntrl_out.duty = 0;
    cntrl_out.invert_pol = false;

  }
};

/* -------------------------------------------------------------------------- */
/*                               Custom modules                               */
/* -------------------------------------------------------------------------- */
#include "pwm.h"


/* -------------------------------------------------------------------------- */
/*                              Per instance data                             */
/* -------------------------------------------------------------------------- */
struct InstData {

  // pwm structure holding its information
  pwm_data pwm1;
  control cntrl;
  double t;

  // Constructor to inialize non-zero members
  InstData(double fs, double fclk, double fline, double dt, CARRIER mode, \
  double kpi, double kii, double kpv, double kiv) {
    this->pwm1 = * (new pwm_data(fs, fclk, dt, mode));
    this->cntrl = * (new control(kpi, kii, kpv, kiv, fs, fline));
  };

};

/*------------------------------------------------------------------------------
 * UDATA() definition -- regenerate the template with QSpice and revise this
 * whenever ports/attributes change; make input/attribute parameters const&
 *----------------------------------------------------------------------------*/
#define UDATA(data)                                                           \
  double  v_ref        = data[ 0].d; /* input */                              \
  double  vline_sense  = data[ 1].d; /* input */                              \
  double  Vo_sense     = data[ 2].d; /* input */                              \
  double  IL_sense     = data[ 3].d; /* input */                              \
  double  fs           = data[ 4].d; /* input parameter */                    \
  double  fclk         = data[ 5].d; /* input parameter */                    \
  double  fline        = data[ 6].d; /* input parameter */                    \
  double  kpi          = data[ 7].d; /* input parameter */                    \
  double  kii          = data[ 8].d; /* input parameter */                    \
  double  kpv          = data[ 9].d; /* input parameter */                    \
  double  kiv          = data[10].d; /* input parameter */                    \
  double  tdead        = data[11].d; /* input parameter */                    \
  int    carrier_mode  = data[12].i; /* input parameter */                    \
  int     sample_mode  = data[13].i; /* input parameter */                    \
  double &duty         = data[14].d; /* input parameter */                    \
  double &Shigh        = data[15].d; /* output */                             \
  double &Slow         = data[16].d; /* output */                             \
  double &debug        = data[17].d; /* output */



/*------------------------------------------------------------------------------
 * Evaluation Function -- name must match DLL name, all lower case
 *----------------------------------------------------------------------------*/
extern "C" __declspec(dllexport) void digital_controller(
    InstData **opaque, double t, uData data[]) {

  UDATA(data);

  InstData *inst = *opaque;

  if (!inst) {
    // allocate instance data
    inst = *opaque = new InstData(fs, fclk, fline, tdead, (CARRIER)carrier_mode, \
    kpi, kii, kpv, kiv);

    if (!inst) {
      // terminate with prejudice
      msg("Memory allocation failure.  Terminating simulation.\n");
      exit(1);
    }

    // if important, output component parameters
    msg("DLL compiled with toolset: %s\n", TOOLSET);
  }

  // Stores time
  inst->t = t;

  // Packs data for control function
  inputData indata = {v_ref, vline_sense, Vo_sense, IL_sense};


  inst->pwm1.pwm_gen(&t, &inst->cntrl, &indata);
  //test(2, 3);
  Shigh = (double)(inst->pwm1.pwm_high);
  Slow = (double)(inst->pwm1.pwm_low);
  duty = inst->cntrl.cntrl_out.duty;
  debug = inst->cntrl.vin_norm;
  //debug = (double)inst->pwm1.trunc_flag;
  //LOGT("peak = %d", (int)inst->pwm1.peak);
}



/*------------------------------------------------------------------------------
 * MaxExtStepSize()
 *----------------------------------------------------------------------------*/
extern "C" __declspec(dllexport) double MaxExtStepSize(InstData *inst) {

  return inst->pwm1.pwm_max_step_handler(inst->t);

}


/*------------------------------------------------------------------------------
 * Trunc()
 *----------------------------------------------------------------------------*/
extern "C" __declspec(dllexport) void Trunc(
    InstData *inst, double t, union uData *data, double *timestep) {

  inst->pwm1.pwm_trunc_handler(timestep, t);

}


/*------------------------------------------------------------------------------
 * Destroy()
 *----------------------------------------------------------------------------*/
extern "C" __declspec(dllexport) void Destroy(InstData *inst) {
  // if important, output a final component message; for example:
  msg("Done, records processed, file closed, whatever.\n");

  // free allocated memory
  delete inst;
}
/*==============================================================================
 * End of Cblock.cpp
 *============================================================================*/

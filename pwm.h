#ifndef PWM_H
#define PWM_H

#include <cstdlib>
#include <math.h>

/* This class defines and implements stuff, which is bad practice, but I don't
    know how to do a multi-source project with dmc compiler                   */


/* -------------------------------------------------------------------------- */
/*                                 Prototypes                                 */
/* -------------------------------------------------------------------------- */

class control;
struct cntrl_out_str;
struct inputData;


/* -------------------------------------------------------------------------- */
/*                                Custom types                                */
/* -------------------------------------------------------------------------- */

typedef enum 
{
    SAWTOOTH = 0,
    TRIANGULAR = 1
} CARRIER;

/* -------------------------------------------------------------------------- */
/*                                   Classes                                  */
/* -------------------------------------------------------------------------- */

class pwm_data
{   

    public:
  
    unsigned long long peak;        // Peak counter value

    unsigned long long cmp_rise;    // Compare value with counter register - pwm
                                    // pulse is triggered when cmp_rise >= cntr
                                    // with rising carrier

    unsigned long long cmp_fall;    // Compare value with counter register - pwm
                                    // pulse is triggered when cmp_rise >= cntr
                                    // with falling carrier

    double cntr_prd;                // Time for pwm counter to tick

    double dead_time;               // Dead time in seconds

    double t_next_zero;             // Next time that carrier == 0
    double t_next_cmp_rise;         // Next time that cmp == carrier during rising period
    double t_next_cmp_fall;         // Next time that cmp == carrier during falling period
    double t_next_peak;             // Next time that carrier == peak
    double t_next_trigger_dead;     // Next time that a command is output after dead time

    double ttol;                    // Time tolerance when there is a switchin event
    double max_step;                // Max time step adjusted dynamically
    double t_n1;                    // Time of previous simulation step
    double t_trunc;

    bool pwm_init;                  // Initial pwm pulse
    bool pwm_init_n1;
    bool pwm_dead;                  // Pwm pulse with dead time
    bool pwm_trigger;               // Auxiliary bool - true when there is a switching event
                                    // it's both related to the true outputs and switching
                                    // within the software. This variable is used in the
                                    // Trunc function that the spice solver utilizes

    bool pwm_high;                  // Final output for high side - based on pwm bool
    bool pwm_high_n1;               
    bool pwm_low;                   // Final output for low side - based on  !pwm bool
    bool pwm_low_n1;
    bool pwm_invert_logic;          // Set to true to invert the pwm output logic
    bool pwm_invert_logic_n1;
    bool crossing;
    bool enable;                    // General enable for outputs
    bool trunc_flag;

    CARRIER mode;

    // ----------------------------------- PWM handler ----------------------------------- //
    void pwm_gen(double *t, class control *cntrl_ptr, inputData *indata)
    {

        // Writes false to trigger, code below writes true if needed
        pwm_trigger = false;

        // Depending of pwm mode:

        switch (mode)
        {
            default:
            case SAWTOOTH:
                // ------------- t crossing valley ------------- //
                if(t_n1 < t_next_zero && *t >= t_next_zero)
                {
                    // ======== PUT CODE THAT RUNS ON VALLEY BELOW ========= //
                    cntrl_out_str *cntrl_out = cntrl_ptr->cascade_pid(indata);

                    cmp_rise = (unsigned long long)((double)(peak+1) * cntrl_out->duty);
                    pwm_invert_logic = cntrl_out->invert_pol;

                    // Checks to see inversion logic
                    if(pwm_invert_logic == pwm_invert_logic_n1){
                        // Switching event
                        pwm_trigger = true;
                        pwm_init = true;
                    }
                    else{
                        pwm_trigger = true;
                        pwm_init = false;
                        crossing = true;
                    }

                    // Update next time stamps
                    t_next_cmp_rise = t_next_zero + (cmp_rise) * cntr_prd;
                    t_next_trigger_dead = t_next_zero + dead_time;
                    t_next_zero = t_next_zero + (peak+1) * cntr_prd;
                }
                
                // ------------- t < tpeak ------------- //
                if(*t < t_next_zero)
                {
                    if(t_n1 < t_next_cmp_rise && *t >= t_next_cmp_rise)
                    {
                        // Switching event
                        pwm_trigger = true;
                        pwm_init = false;

                        // Update next time stamps
                        t_next_trigger_dead = t_next_cmp_rise + dead_time;
                    }
                }

                // ------------- PWM after dead time ------------- //
                if(t_n1 < t_next_trigger_dead && *t >= t_next_trigger_dead)
                {
                    pwm_trigger = true;
                    pwm_dead = pwm_init;
                    crossing = false;
                }
                
                break;

            case TRIANGULAR:

                // ------------- t crossing valley ------------- //
                if(t_n1 < t_next_zero && *t >= t_next_zero)
                {
                    // ======== PUT CODE THAT RUNS ON VALLEY BELOW ========= //
                    cntrl_out_str *cntrl_out = cntrl_ptr->cascade_pid(indata);

                    cmp_rise = (unsigned long long)((double)peak * cntrl_out->duty);
                    cmp_fall = cmp_rise;
                    pwm_invert_logic = cntrl_out->invert_pol;

                    // Switching event
                    pwm_trigger = true;

                    // Update next time stamps
                    t_next_cmp_rise = t_next_zero + cmp_rise * cntr_prd;
                    t_next_peak = t_next_zero + peak * cntr_prd;
                    t_next_zero = t_next_zero + 2*peak * cntr_prd;
                    t_next_cmp_fall = t_next_zero - cmp_fall * cntr_prd;
                    max_step = peak*cntr_prd;

                }

                // ------------- t < tpeak ------------- //
                if(*t < t_next_peak)
                {
                    if(t_n1 < t_next_cmp_rise && *t >= t_next_cmp_rise)
                    {
                        // Switching event
                        pwm_trigger = true;
                        pwm_init = false;

                        // Time-stamp for pwm with dead time
                        t_next_trigger_dead = t_next_cmp_rise + dead_time;
                    }
                }
                // ------------- t >= tpeak ------------- //
                else
                {
                    if(t_n1 < t_next_peak && *t >= t_next_peak)
                    {
                        pwm_trigger = true; // Not sure if needed here but gonna maintain it
                    }
                    if(t_n1 < t_next_cmp_fall && *t >= t_next_cmp_fall)
                    {
                        pwm_trigger = true;
                        pwm_init = true;
                    }
                }

                // ------------- dead time pwm switching events ------------- //
                if(!pwm_init_n1 && pwm_init)
                {
                    // pwm is rising
                    pwm_trigger = true;
                    t_next_trigger_dead = *t + dead_time;
                }
                if(pwm_init_n1 && !pwm_init)
                {
                    // pwm is falling
                    pwm_trigger = true;
                    t_next_trigger_dead = *t + dead_time;
                }

                if(t_n1 < t_next_trigger_dead && *t >= t_next_trigger_dead)
                {
                    pwm_trigger = true;
                    pwm_dead = pwm_init;
                }

                break;
        }

        // Finally updates final outputs as needed
        if(*t < dead_time)  enable = false;
        else    enable = true;

        if(pwm_invert_logic){
            pwm_high = enable & !pwm_init  & !pwm_dead & !crossing;
            pwm_low  = enable & pwm_init & pwm_dead & !crossing; 
        }
        else{
            pwm_high = enable & pwm_init  & pwm_dead & !crossing;
            pwm_low  = enable & !pwm_init & !pwm_dead & !crossing; 
        }

        // Any weird exceptions should be caught below
        if(pwm_high_n1 != pwm_high || pwm_low_n1 != pwm_low) pwm_trigger = true;

        // Saves stuff for next iterations
        t_n1 = *t;
        pwm_invert_logic_n1 = pwm_invert_logic;

        pwm_high_n1 = pwm_high;
        pwm_low_n1 = pwm_low;
        pwm_init_n1 = pwm_init;
    };

    void pwm_trunc_handler(double *timestep, double t)
    {
  
        if(t < t_next_zero) if(*timestep > t_next_zero - t) *timestep = t_next_zero - t;
        if(t < t_next_cmp_rise)   if(*timestep > t_next_cmp_rise - t)   *timestep = t_next_cmp_rise - t;
        //if(t_n1 < t_next_peak)   if(*timestep > t_next_peak - t_n1)   *timestep = t_next_peak - t_n1;
        //if(t_n1 < t_next_cmp_fall)   if(*timestep > t_next_cmp_fall - t_n1)   *timestep = t_next_cmp_fall - t_n1;
        if(t < t_next_trigger_dead)   if(*timestep > t_next_trigger_dead - t)   *timestep = t_next_trigger_dead - t;  
        //if(pwm_trigger) if(*timestep > ttol)    *timestep = ttol;      
        //if(crossing) if(*timestep > ttol)   *timestep = ttol/4;
        
    }

    double pwm_max_step_handler(double t)
    {
        if(pwm_trigger){
            t_trunc = t;
            trunc_flag = true;
        }
        if(t - t_trunc >= dead_time)    trunc_flag = false;
        if(trunc_flag || crossing){
            return ttol;
        }

        // If here, then return max_step
        return max_step;
    }

    // --------------------- Constructor in .h file requires inline ---------------------- //
    inline pwm_data(double fs = 40e3, double fclk = 200e6, double dt = 10e-9, \
     CARRIER carr_mode = SAWTOOTH){
        
        // Initializes non-zero elements
        cmp_rise = 0;
        cmp_fall = 0;
        dead_time = dt;
        enable = true;
        mode = carr_mode;
        pwm_trigger = true;
        pwm_init = false;
        pwm_dead = !pwm_init;
        pwm_high = false;
        pwm_low = false;
        pwm_invert_logic = false;
        trunc_flag = false;

        // Initializes counter period
        cntr_prd = 1/fclk;
        ttol = dead_time/8;

        // Previous time at 0 = 0
        t_n1 = 0;

        t_trunc = 0;

        // Initializes peak value based on desired freq. and carrier mode
        switch (carr_mode)
        {
        default:
        case SAWTOOTH:
            // Initializes counter tick
            peak = (unsigned long long)(floor((fclk/fs)) - 1);
            max_step = peak * cntr_prd;
            t_next_zero = dead_time/10;
            t_next_trigger_dead = t_next_zero + dead_time;
            t_next_cmp_rise = t_next_zero + ((double)(cmp_rise)) * cntr_prd;
            t_next_peak = -1; // Not used, but could interfere in trunc
            //t_next_zero = t_next_zero + peak * cntr_prd;
            t_next_cmp_fall = -1; // Not used, but could interfere in trunc
            break;

        case TRIANGULAR:
            // Initializes peak counter value
            peak = (unsigned long long)round((fclk/fs)/2);
            max_step = peak * cntr_prd;
            t_next_zero = dead_time/10;
            t_next_cmp_rise = t_next_zero + cmp_rise * cntr_prd;
            t_next_peak = t_next_zero + peak * cntr_prd;
            t_next_cmp_fall = t_next_peak + (peak - cmp_fall) * cntr_prd;
            break;
        }
    };

};


/* -------------------------------------------------------------------------- */
/*                              Public functions                              */
/* -------------------------------------------------------------------------- */

//void pwm_trunc_handler(struct pwm_data a, double *t_prev, double *timestep, bool *pwm_trigger);


#endif
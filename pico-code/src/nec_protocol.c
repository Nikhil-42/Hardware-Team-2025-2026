#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "include/nec_protocol.h"

static uint carrier_level; // tracks the duty cycle of the carrier waveform 
int bit_index; // tracks the number of bits from the transmission frame that have been scheduled 
uint32_t nec_frame;  // stores the frame to be transmitted 
bool leading_pulse = false; // tracks if we have sent the leading pulse or not 
bool carrier_enabled = false; // tracks if the carrier waveform is active 

// use timer callback to schedule interrupt timings 
static int64_t nec_timer_callback (alarm_id_t id, void *user_data)
{
        // if leading pulse, schedule leading pulse or space 
        if (leading_pulse)
        {       
                // if carrier is enabled, leading pulse has already been sent 
                if (carrier_enabled)
                {
                        // disable carrier to pull lead space low 
                        disable_carrier(); 
                        leading_pulse = false; 
                        add_alarm_in_us(NEC_LEAD_SPACE_US, nec_timer_callback, NULL, false); 
                }
                // if carrier has not been enabled, we have yet to schedule the leading pulse 
                else 
                {
                        enable_carrier(); 
                        add_alarm_in_us(NEC_LEAD_PULSE_US, nec_timer_callback, NULL, false);
                }
                // return from ISR 
                return 0; 
                 
        }
        // if not leading pulse and have not exceeded the bit index, we are scheduling a data bit
        if (bit_index < 32)
        {
                // for data bits that are 1, enable the carrier for 562 us and disable for 562 * 3 us
                // for data bits that are 0, enable carrier for 562 us and disable for 562 us 
                uint32_t data_bit = (nec_frame >> bit_index) & 1;
                // if carrier is enabled, we sent a pulse previously, now need to schedule a space for the bit
                if (carrier_enabled)
                {       
                        disable_carrier();  
                        // if data_bit is 1, carrier waveform should be disabled for 562 * 3 us
                        // if data bit is 0, carrier waveform should be disabled for 562 us 
                        uint32_t space; 
                        if (data_bit)
                        {
                                space = NEC_1_SPACE_US;    
                        }
                        else 
                        {
                                space = NEC_0_SPACE_US; 
                        }
                        add_alarm_in_us(space, nec_timer_callback, NULL, false); 
                        bit_index++; // increment once space has been scheduled to move to next data pulse
                }
                // if carrier is disabled, we have not scheduled the pulse of the data bit 
                // pulse widths are the same for both data bits
                else 
                {
                        enable_carrier();  
                        add_alarm_in_us(NEC_PULSE_US, nec_timer_callback, NULL, false);
                }
                return 0; 
                
        }
        // if we have exceeded the bit index, need to send trailing pulse 
        else 
        {
                // if the carrier is enabled, callback fires and reaches this state, we need to disable the trailing pulse
                if (carrier_enabled)
                {
                        disable_carrier(); 
                        bit_index = 0; // reset global bit counter
                }
                // if the carrier is disabled, we have not sent the trailing pulse 
                else 
                {
                        enable_carrier(); 
                        add_alarm_in_us(NEC_PULSE_US, nec_timer_callback, NULL, false); 
                }
        }
        
        // for all pulses, configure for single-shot callback (i.e., timer callback for each scheduled timer is triggered only once)
        return 0; 
}

void pwm38k_init()
{
        // set GPIO for PWM hardware
        gpio_set_function(IR_TRANS_PIN, GPIO_FUNC_PWM); 
        
        // sets a slice (internal hardware generator) to the PWM GPIO
        uint slice_num = pwm_gpio_to_slice_num(IR_TRANS_PIN);
        
        // calculates top value to set for PWM wrapping 
        uint32_t top = (clock_get_hz(clk_sys) / 38000); 
        carrier_level = top/3; 
        
        // sets TOP of PWM
        pwm_set_wrap(slice_num, top); 

        // computes divider: left at 1 for highest resolution
        pwm_set_clkdiv(slice_num, 1.0f); 

        // sets 33% duty cycle for the PWM channel 
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(IR_TRANS_PIN), carrier_level);

        // initially disables PWM waveform 
        disable_carrier();
}

void enable_carrier() 
{
        // reset to PWM mode since we assume that disable_carrier was called to pull GPIO low 
        gpio_set_function(IR_TRANS_PIN, GPIO_FUNC_PWM); 
        uint slice = pwm_gpio_to_slice_num(IR_TRANS_PIN);
        pwm_set_chan_level(slice, pwm_gpio_to_channel(IR_TRANS_PIN), carrier_level);
        pwm_set_enabled(slice, true);
        carrier_enabled = true; 
    
}

void disable_carrier() 
{
        // disable PWM and pull low using GPIO (testing occasionally showed that signal was high during spaces; should be low when disabled)
        uint slice = pwm_gpio_to_slice_num(IR_TRANS_PIN);
        pwm_set_enabled(slice, false);
        gpio_set_function(IR_TRANS_PIN, GPIO_FUNC_SIO); 
        gpio_put(IR_TRANS_PIN, 0); 
        carrier_enabled = false; 
     
}

uint32_t nec_encode(uint8_t address, uint8_t command) 
{
        // assume frame is read from end of 2word so that it is LSb -> MSb of address onward
        uint32_t transmission_frame = ((uint8_t)(~command) << 24) | (command << 16) | ((uint8_t)(~address) << 8) | (address); 
        return transmission_frame; 
}

void nec_send(uint32_t data)
{
        // See NEC protocol information from more about protocol specifics 
        nec_frame = data; 
        bit_index = 0; 
        leading_pulse = true;

        // start series of timer schedulings with leading pulse 
        enable_carrier(); 
        add_alarm_in_us(NEC_LEAD_PULSE_US, nec_timer_callback, NULL, false);
}
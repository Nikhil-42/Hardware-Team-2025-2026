#include "include/ma.h"
#include <stdio.h>

void running_average_init(runningAverageFilter *filt)
{
        filt->val_count = 0; 
        filt->index = 0; 
        filt->sum = 0.0f;
        
        // fill buffer with zeros to create well-defined values 
        for (int i = 0; i < NUM_SAMPLES; i++)
        {
                filt->buffer[i] = 0.0f;
        }
}

void running_average_init_all(runningAverageFilter *all_filts, int N)
{
        for(int i = 0; i < N; i++)
        {
                running_average_init(&all_filts[i]); 
        }
}

float running_average_update(runningAverageFilter *filt, float new_sample)
{
        filt->sum -= filt->buffer[filt->index];       // remove old sample at the index from sum
        filt->buffer[filt->index] = new_sample;       // store new sample 
        filt->sum += new_sample;                      // add sample to sum
        filt->index = (filt->index + 1) % NUM_SAMPLES; // circularly update index

        // for the first N samples, use a counter to average over N values instead of whole buffer size
        if (filt->val_count < NUM_SAMPLES)
        {
                filt->val_count++;
        }

        return filt->sum/filt->val_count;
}

void running_average_update_all(runningAverageFilter *all_filts, float *new_samples, float *filtered_outputs, int N)
{
        for(int i = 0; i < N; i++)
        {
                filtered_outputs[i] = running_average_update(&all_filts[i], new_samples[i]); 
                printf("MA output for motor %d: %f\n", i + 1, filtered_outputs[i]);
        }
}
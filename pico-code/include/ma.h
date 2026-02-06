#pragma once

#define NUM_SAMPLES 4

// structure to store N-length running average information 
typedef struct runningAverageFilter 
{        
        float buffer[NUM_SAMPLES]; // buffer to store current values 
        int val_count; // counts number of nonzero elements currently in the buffer
        int index; // tracks the index position in the circular buffer
        float sum; // tracks the sum of the elements in the buffer
} runningAverageFilter; 

/*
        initializes running average structure
*/
void running_average_init(runningAverageFilter *filt);

/*
        runs N number of initializations for array of structures
*/
void running_average_init_all(runningAverageFilter *all_filts, int N); 

/*
        add new data point to running average filter and returns the average of the RA buffer
*/
float running_average_update(runningAverageFilter *filt, float new_sample);

/*
        runs N number of updates for all structures in the array of structures
*/
void running_average_update_all(runningAverageFilter *all_filts, float *new_samples, float *filtered_outputs, int N); 

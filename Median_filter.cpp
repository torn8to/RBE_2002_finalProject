#include "Median_filter.h"

void MedianFilter::Init(void)
{
    // no initialization required, but good practice
}

void MedianFilter::Sort(int index_a, int index_b)
{
    if(array[index_a] < array[index_b]){
        int temp = array[index_a];
        array[index_a] = array[index_b];
        array[index_b] = temp;
    }
}

int MedianFilter::Filter(int measurement)
{
    array[0] = measurement;
    for(int i = 4; i > 0; i--) array[i] = array[i-1];
    
    Sort(0,1);
    Sort(3,4);
    Sort(0,2);
    Sort(1,2);
    Sort(0,3);
    Sort(2,3);
    Sort(1,4);
    Sort(1,2);

    return array[2];
}
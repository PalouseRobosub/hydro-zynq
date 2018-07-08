#include "fft.h"

#include "abort.h"
#include "types.h"

#include <math.h>

#define SWAP(a,b) tempr=(a);(a)=(b);(b)=tempr

/**
 * Calculates the discrete fourier transform of the provided signal.
 *
 * @note This algorithm was taken from "C Numerical Recipes" pages 507-508.
 *
 * @param[inout] data The input signal. The calculated discrete fourier
 *        transform is stored within this array as output. Input data is stored
 *        [real, complex] as array indices.
 * @param len The length of input data in terms of complex pairs. Therefore,
 *        the number of elements pointed to by data should be len * 2.
 * @param isign 1 indicates a normal fourier transform whereas -1 indicates an
 *        inverse.
 *
 * @return Success or fail.
 */
result_t discrete_fourier_transform(float *data, int32_t len, int isign)
{
    /*
     * Ensure the input is valid and the data length is a power of two.
     */
    AbortIfNot(data, fail);
    AbortIfNot((len & (len - 1)) == 0, fail);

    unsigned long n, mmax, m, j, istep, i;

    /*
     * Double precision for the trigonometric recurrences.
     */
    double wtemp, wr, wpr, wpi, wi, theta;

    float tempr, tempi;

    n = len << 1;

    j = 1;

    // This is the bit-reversal section of the routine.
    for (i = 1; i < n; i += 2)
    {
        // Exchange the two complex numbers.
        if (j > i)
        {
            SWAP(data[j], data[i]);
            SWAP(data[j + 1], data[i + 1]);
        }

        m = len;

        while (m >= 2 && j > m)
        {
            j -= m;
            m >>= 1;
        }

        j += m;
    }

    mmax = 2;

    // Outer loop executed log2 len times.
    while (n > mmax)
    {
        istep = mmax << 1;

         //Initialize the trigonometric recurrence.
        theta = isign * (6.28318530717959 / mmax);
        wtemp = sin(0.5 * theta);
        wpr = -2.0 * wtemp * wtemp;
        wpi = sin(theta);
        wr = 1.0;
        wi = 0.0;

        // Here are the two nested inner loops.
        for (m = 1; m < mmax; m += 2)
        {
            for (i = m; i <= n; i += istep)
            {
                j = i + mmax;

                // This is the Danielson-Lanczos formula:
                tempr = wr * data[j] - wi * data[j + 1];
                tempi = wr * data[j + 1] + wi * data[j];
                data[j] = data[i] - tempr;
                data[j + 1] = data[i + 1] - tempi;
                data[i] += tempr;
                data[i + 1] += tempi;
            }

            // Trigonometric recurrence.
            wr = (wtemp = wr) * wpr - wi * wpi + wr;
            wi = wi * wpr + wtemp * wpi + wi;
        }

        mmax = istep;
    }

    return success;
}

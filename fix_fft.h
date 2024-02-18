/**
 * Fixed point FFT
 * 
 * https://forum.arduino.cc/t/16-bit-fft-on-arduino/72061/19
 */


#include <type_traits>
#include "fix.h"

class Fixed15FFT {
  public:
  constexpr static int N_WAVE = 1024;
  constexpr static int LOG2_N_WAVE = 10;
  /*
    Since we only use 3/4 of N_WAVE, we define only
    this many samples, in order to conserve data space.
  */
  constexpr static int SAMPLE_SIZE = N_WAVE-N_WAVE/4;
  
  private:
  static const short Sinewave[SAMPLE_SIZE];
  static const short Window[N_WAVE];

  public:

  template<typename T>
  static void apply_window(T v[])
  {
    static_assert(std::is_integral<T>::value, "Parameters must be integral.");
    int i;
    constexpr int n = N_WAVE;
    for (i = 0; i < n; ++i) {
      v[i] = FIX_MPY(v[i], Window[i]);
    }
  }

  /*
    fix_fft() - perform forward/inverse fast Fourier transform.
    fr[n],fi[n] are real and imaginary arrays, both INPUT AND
    RESULT (in-place FFT), with 0 <= n < 2**m; set inverse to
    0 for forward transform (FFT), or 1 for iFFT.
  */
  template<typename T>
  static void calc_fft(T fr[], T fi[])
  {
    static_assert(std::is_integral<T>::value, "Parameters must be integral.");
    int mr, i, j, l, k, m, istep;
    int qr, qi, tr, ti, wr, wi;

    constexpr int n = N_WAVE;
    constexpr int nn = n - 1;

    mr = 0;

    /* decimation in time - re-order data */
    for (m=1; m<=nn; ++m) {
      l = n;
      do 
      {
        l >>= 1;
      } while (mr+l > nn);
      mr = (mr & (l-1)) + l;
      if (mr <= m)
      continue;
    
      tr = fr[m];
      fr[m] = fr[mr];
      fr[mr] = tr;
      ti = fi[m];
      fi[m] = fi[mr];
      fi[mr] = ti;
    }
  
    l = 1;
    k = LOG2_N_WAVE-1;
    while (l < n) 
    {
      /*
        it may not be obvious, but the shift will be
        performed on each data point exactly once,
        during this pass.
      */
      istep = l << 1;
      for (m=0; m<l; ++m) 
      {
        j = m << k;
        /* 0 <= j < N_WAVE/2 */
        wr =  Sinewave[j+N_WAVE/4];
        wi = -Sinewave[j];
        wr >>= 1;
        wi >>= 1;
        for (i=m; i<n; i+=istep) 
        {
          j = i + l;
          //tr = ((long)wr*(long)fr[j] - (long)wi*(long)fi[j])>>15;
          //ti = ((long)wr*(long)fi[j] + (long)wi*(long)fr[j])>>15;
          tr = FIX_MPY(wr,fr[j]) - FIX_MPY(wi,fi[j]);
          ti = FIX_MPY(wr,fi[j]) + FIX_MPY(wi,fr[j]);
          qr = fr[i];
          qr >>= 1;
          fr[i] = qr + tr;
          fr[j] = qr - tr;
          qi = fi[i];
          qi >>= 1;
          fi[i] = qi + ti;
          fi[j] = qi - ti;
        }
      }
      --k;
      l = istep;
    }
  }
  
};

/* dywapitchtrack.c
 
 Dynamic Wavelet Algorithm Pitch Tracking library
 Released under the MIT open source licence
  
 Copyright (c) 2010 Antoine Schmitt
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
*/

/*
 * Modified by KIRA Ryouta on 2020.
 */

#include "dywapitchtrack.h"
#include <math.h>
#include <stdlib.h>
#include <string.h> // for memset
#include <limits.h>
#include <type_traits>

//**********************
//       Utils
//**********************

#ifndef max
#define max(x, y) ((x) > (y)) ? (x) : (y)
#endif
#ifndef min
#define min(x, y) ((x) < (y)) ? (x) : (y)
#endif

// returns 1 if power of 2
int _power2p(int value) {
	if (value == 0) return 1;
	if (value == 2) return 1;
	if (value & 0x1) return 0;
	return (_power2p(value >> 1));
}

// count number of bits
int _bitcount(int value) {
  float temp = (float)value + 0.5f;
  return (*reinterpret_cast<int*>(&temp) >> 23) - 126;
}

// closest power of 2 above or equal
int _ceil_power2(int value) {
	if (_power2p(value)) return value;
	
	if (value == 1) return 2;
	int i = _bitcount(value);
  return 1 << i;
}

// closest power of 2 below or equal
int _floor_power2(int value) {
	if (_power2p(value)) return value;
  if (value == 1) return 2;
  int i = _bitcount(value);
	return (1 << i) >> 1;
}

// abs value
int _iabs(int x) {
  return abs(x);
}

// 2 power
int _2power(int i) {
  return 1 << i;
}

//******************************
// the Wavelet algorithm itself
//******************************

int dywapitch_neededsamplecount(int minFreq) {
	int nbSam = 3*44100/minFreq; // 1017. for 130 Hz
	nbSam = _ceil_power2(nbSam); // 1024
	return nbSam;
}

typedef struct _minmax {
	int index;
	struct _minmax *next;
} minmax;

static int distances[DYWAPT_SAMPLESIZE];
static int mins[DYWAPT_SAMPLESIZE];
static int maxs[DYWAPT_SAMPLESIZE];

DYWAPT_FLOAT _dywapitch_computeWaveletPitch(DYWAPT_INPUT *sam) {
	DYWAPT_FLOAT pitchF = 0.0f;
	
	int i, j;
	DYWAPT_TEMP si, si1;
	
	int curSamNb = DYWAPT_SAMPLESIZE;
	int nbMins, nbMaxs;
	
	// algorithm parameters
#define	maxFLWTlevels 6
#define	maxF (DYWAPT_SAMPLERATE/2)
#define differenceLevelsN 3

#define maximaThresholdRatio 0.75f
#define maximaThresholdRatioMul 3
#define maximaThresholdRatioRSHIFT 2
	
	DYWAPT_TEMP ampltitudeThreshold;
	DYWAPT_TEMP theDC = 0;
	
	{ // compute ampltitudeThreshold and theDC
		//first compute the DC and maxAMplitude
		DYWAPT_TEMP maxValue = DYWAPT_TEMPMIN;
		DYWAPT_TEMP minValue = DYWAPT_TEMPMAX;
		for (i = 0; i < DYWAPT_SAMPLESIZE;i++) {
			si = sam[i];
			theDC = theDC + si;
			if (si > maxValue) maxValue = si;
			if (si < minValue) minValue = si;
		}
    if (std::is_integral<DYWAPT_TEMP>::value) {
      theDC = (int)theDC >> DYWAPT_SAMPLESIZELOG2;
    }
    else {
      theDC = theDC*(1./DYWAPT_SAMPLESIZE);
    }
		maxValue = maxValue - theDC;
		minValue = minValue - theDC;
		DYWAPT_TEMP amplitudeMax = (maxValue > -minValue ? maxValue : -minValue);
		
    if (std::is_integral<DYWAPT_TEMP>::value) {
      ampltitudeThreshold = (int)amplitudeMax * maximaThresholdRatioMul >> maximaThresholdRatioRSHIFT;
    }
    else {
      ampltitudeThreshold = amplitudeMax*maximaThresholdRatio;    
    }
		
	}
	
	// levels, start without downsampling..
	int curLevel = 0;
	DYWAPT_FLOAT curModeDistance = -1.f;
	int delta;
	
	while(1) {
		
		// delta
		delta = (DYWAPT_SAMPLERATE / maxF) >> curLevel;
		//("dywapitch doing level=%ld delta=%ld\n", curLevel, delta);
		
		if (curSamNb < 2) goto cleanup;
		
		// compute the first maximums and minumums after zero-crossing
		// store if greater than the min threshold
		// and if at a greater distance than delta
		DYWAPT_TEMP dv, previousDV = -1000;
		nbMins = nbMaxs = 0;   
		int lastMinIndex = -1000000;
		int lastmaxIndex = -1000000;
		int findMax = 0;
		int findMin = 0;
		for (i = 1; i < curSamNb; i++) {
			si = sam[i] - theDC;
			si1 = sam[i-1] - theDC;
			
			if (si1 <= 0 && si > 0) {findMax = 1; findMin = 0; }
			if (si1 >= 0 && si < 0) {findMin = 1; findMax = 0; }
			
			// min or max ?
			dv = si - si1;
			
			if (previousDV > -1000) {
				
				if (findMin && previousDV < 0 && dv >= 0) { 
					// minimum
					if (abs(si1) >= ampltitudeThreshold) {
						if (i - 1 > lastMinIndex + delta) {
							mins[nbMins++] = i - 1;
							lastMinIndex = i - 1;
							findMin = 0;
							//if DEBUGG then put "min ok"&&si
							//
						} else {
							//if DEBUGG then put "min too close to previous"&&(i - lastMinIndex)
							//
						}
					} else {
						// if DEBUGG then put "min "&abs(si)&" < thresh = "&ampltitudeThreshold
						//--
					}
				}
				
				if (findMax && previousDV > 0 && dv <= 0) {
					// maximum
					if (abs(si1) >= ampltitudeThreshold) {
						if (i -1 > lastmaxIndex + delta) {
							maxs[nbMaxs++] = i - 1;
							lastmaxIndex = i - 1;
							findMax = 0;
						} else {
							//if DEBUGG then put "max too close to previous"&&(i - lastmaxIndex)
							//--
						}
					} else {
						//if DEBUGG then put "max "&abs(si)&" < thresh = "&ampltitudeThreshold
						//--
					}
				}
			}
			
			previousDV = dv;
		}
		
		if (nbMins == 0 && nbMaxs == 0) {
			// no best distance !
			//asLog("dywapitch no mins nor maxs, exiting\n");
			
			// if DEBUGG then put "no mins nor maxs, exiting"
			goto cleanup;
		}
		//if DEBUGG then put count(maxs)&&"maxs &"&&count(mins)&&"mins"
		
		// maxs = [5, 20, 100,...]
		// compute distances
		int d;
		memset(distances, 0, DYWAPT_SAMPLESIZE*sizeof(int));
		for (i = 0 ; i < nbMins ; i++) {
			for (j = 1; j < differenceLevelsN; j++) {
				if (i+j < nbMins) {
					d = _iabs(mins[i] - mins[i+j]);
					//asLog("dywapitch i=%ld j=%ld d=%ld\n", i, j, d);
					++distances[d];
				}
			}
		}
		for (i = 0 ; i < nbMaxs ; i++) {
			for (j = 1; j < differenceLevelsN; j++) {
				if (i+j < nbMaxs) {
					d = _iabs(maxs[i] - maxs[i+j]);
					//asLog("dywapitch i=%ld j=%ld d=%ld\n", i, j, d);
					++distances[d];
				}
			}
		}
		
		// find best summed distance
		int bestDistance = -1;
		int bestValue = -1;
		for (i = 0; i< curSamNb; i++) {
			int summed = 0;
			for (j = -delta ; j <= delta ; j++) {
				if (i+j >=0 && i+j < curSamNb)
					summed += distances[i+j];
			}
			//asLog("dywapitch i=%ld summed=%ld bestDistance=%ld\n", i, summed, bestDistance);
			if (summed == bestValue) {
				if (i == 2*bestDistance)
					bestDistance = i;
				
			} else if (summed > bestValue) {
				bestValue = summed;
				bestDistance = i;
			}
		}
		//asLog("dywapitch bestDistance=%ld\n", bestDistance);
		
		// averaging
		DYWAPT_FLOAT distAvg = 0.0;
		DYWAPT_FLOAT nbDists = 0;
		for (j = -delta ; j <= delta ; j++) {
			if (bestDistance+j >=0 && bestDistance+j < DYWAPT_SAMPLESIZE) {
				int nbDist = distances[bestDistance+j];
				if (nbDist > 0) {
					nbDists += nbDist;
					distAvg += (bestDistance+j)*nbDist;
				}
			}
		}
		// this is our mode distance !
		distAvg /= nbDists;
		//asLog("dywapitch distAvg=%f\n", distAvg);
		
		// continue the levels ?
		if (curModeDistance > -1.) {
			DYWAPT_FLOAT similarity = fabs(distAvg*2 - curModeDistance);
			if (similarity <= 2*delta) {
				//if DEBUGG then put "similarity="&similarity&&"delta="&delta&&"ok"
 				//asLog("dywapitch similarity=%f OK !\n", similarity);
				// two consecutive similar mode distances : ok !
				pitchF = (DYWAPT_FLOAT)DYWAPT_SAMPLERATE/(_2power(curLevel-1)*curModeDistance);
				goto cleanup;
			}
			//if DEBUGG then put "similarity="&similarity&&"delta="&delta&&"not"
		}
		
		// not similar, continue next level
		curModeDistance = distAvg;
		
		curLevel = curLevel + 1;
		if (curLevel >= maxFLWTlevels) {
			// put "max levels reached, exiting"
 			//asLog("dywapitch max levels reached, exiting\n");
			goto cleanup;
		}
		
		// downsample
		if (curSamNb < 2) {
 			//asLog("dywapitch not enough samples, exiting\n");
			goto cleanup;
		}
		for (i = 0; i < curSamNb/2; i++) {
			sam[i] = (sam[2*i] + sam[2*i + 1])*0.5;
		}
		curSamNb >>= 1;
	}
	
	///
cleanup:
	
	return pitchF;
}

// ***********************************
// the dynamic postprocess
// ***********************************

/***
It states: 
 - a pitch cannot change much all of a sudden (20%) (impossible humanly,
 so if such a situation happens, consider that it is a mistake and drop it. 
 - a pitch cannot DYWAPT_FLOAT or be divided by 2 all of a sudden : it is an
 algorithm side-effect : divide it or DYWAPT_FLOAT it by 2. 
 - a lonely voiced pitch cannot happen, nor can a sudden drop in the middle
 of a voiced segment. Smooth the plot. 
***/

DYWAPT_FLOAT _dywapitch_dynamicprocess(dywapitchtracker *pitchtracker, DYWAPT_FLOAT pitch) {
	
	// equivalence
	if (pitch == 0.0) pitch = -1.0;
	
	//
	DYWAPT_FLOAT estimatedPitch = -1;
	DYWAPT_FLOAT acceptedError = 0.2f;
	int maxConfidence = 5;
	
	if (pitch != -1) {
		// I have a pitch here
		
		if (pitchtracker->_prevPitch == -1) {
			// no previous
			estimatedPitch = pitch;
			pitchtracker->_prevPitch = pitch;
			pitchtracker->_pitchConfidence = 1;
			
		} else if (abs(pitchtracker->_prevPitch - pitch)/pitch < acceptedError) {
			// similar : remember and increment pitch
			pitchtracker->_prevPitch = pitch;
			estimatedPitch = pitch;
			pitchtracker->_pitchConfidence = min(maxConfidence, pitchtracker->_pitchConfidence + 1); // maximum 3
			
		} else if ((pitchtracker->_pitchConfidence >= maxConfidence-2) && abs(pitchtracker->_prevPitch - 2.*pitch)/(2.*pitch) < acceptedError) {
			// close to half the last pitch, which is trusted
			estimatedPitch = 2.*pitch;
			pitchtracker->_prevPitch = estimatedPitch;
			
		} else if ((pitchtracker->_pitchConfidence >= maxConfidence-2) && abs(pitchtracker->_prevPitch - 0.5*pitch)/(0.5*pitch) < acceptedError) {
			// close to twice the last pitch, which is trusted
			estimatedPitch = 0.5*pitch;
			pitchtracker->_prevPitch = estimatedPitch;
			
		} else {
			// nothing like this : very different value
			if (pitchtracker->_pitchConfidence >= 1) {
				// previous trusted : keep previous
				estimatedPitch = pitchtracker->_prevPitch;
				pitchtracker->_pitchConfidence = max(0, pitchtracker->_pitchConfidence - 1);
			} else {
				// previous not trusted : take current
				estimatedPitch = pitch;
				pitchtracker->_prevPitch = pitch;
				pitchtracker->_pitchConfidence = 1;
			}
		}
		
	} else {
		// no pitch now
		if (pitchtracker->_prevPitch != -1) {
			// was pitch before
			if (pitchtracker->_pitchConfidence >= 1) {
				// continue previous
				estimatedPitch = pitchtracker->_prevPitch;
				pitchtracker->_pitchConfidence = max(0, pitchtracker->_pitchConfidence - 1);
			} else {
				pitchtracker->_prevPitch = -1;
				estimatedPitch = -1.;
				pitchtracker->_pitchConfidence = 0;
			}
		}
	}
	
	// put "_pitchConfidence="&pitchtracker->_pitchConfidence
	if (pitchtracker->_pitchConfidence >= 1) {
		// ok
		pitch = estimatedPitch;
	} else {
		pitch = -1;
	}
	
	// equivalence
	if (pitch == -1) pitch = 0.0;
	
	return pitch;
}


// ************************************
// the API main entry points
// ************************************

void dywapitch_inittracking(dywapitchtracker *pitchtracker) {
	pitchtracker->_prevPitch = -1.;
	pitchtracker->_pitchConfidence = -1;
}

DYWAPT_FLOAT dywapitch_computepitch(dywapitchtracker *pitchtracker, DYWAPT_INPUT *samples) {
	DYWAPT_FLOAT raw_pitch = _dywapitch_computeWaveletPitch(samples);
	return _dywapitch_dynamicprocess(pitchtracker, raw_pitch);
}

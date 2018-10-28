#include <Filters.h>
//    Copyright 2018, Robert L. Read 
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.

// This is an attempt the code downloaded from the Aruduino playground from this GitHub repo: https://github.com/JonHub/Filters
// The authorship of this is a little unclear, or I would attribute them directly.
// My goal here is to make a test signal composed of 3 signals: a low frequencey bias, some high-frequency (random) noise,
// and 60-80  Hz actual signal, and see if we can smoothly extract (plot) the 60 Hz signal.
// We are doing this for the purpose of building a pulse oximeter, so we are in particular interested in these
// kinds of signals.

float testFrequency = 1.0;                     // test signal frequency (Hz)
float testAmplitude = 100;                   // test signal amplitude
float testOffset = 100;

const int NUM_FREQS_IN_TEST_SIGNAL = 3;
float f[NUM_FREQS_IN_TEST_SIGNAL];
float a[NUM_FREQS_IN_TEST_SIGNAL];



int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

// Here I attempt to make a class rather like the Filter classes, to keep a 
// running implementation of a peak-to-peak measurement

// The sample structure is used to keep sets of samples.
struct sample {
  float v;
  long t;
};

const int NUM_SAMPLES = 100;
class PeakToPeak {
  float Xmax;        // position
  float Xmin;        // position
  long LastTimeUS;  // last time measured
  long OldestTime;  // the oldest time we consider in this window
  const float WindowDurationS = 10.0; // Length of the window in seconds
  float Fprev;

  
  sample maxs[NUM_SAMPLES];
  int max_i = 0; // index into the max ring, points to most recent if there is one
  int max_f = 0; // index into max ring, points to end of the ring (but not necessarily the oldest sample!)
  int SIZE_MAX_RING = 0; // The number of valid samples
  sample mins[NUM_SAMPLES];
  int min_i = 0; // index into the min ring
  int min_f = 0;

 public:


  PeakToPeak() {
    Xmax = 0.0;              // start it some arbitrary position
    Xmin = 0.0;
    LastTimeUS = micros();
  }
  int next_idx(int idx) {
    return (idx+1) % NUM_SAMPLES;
  }
  int prev_idx(int idx) {
    if (idx == 0) 
      return NUM_SAMPLES-1;
    return (idx-1) % NUM_SAMPLES;
  }
  // true if a dominates b (that is, a is greater and more recent.)
  bool dominates(bool compute_max,sample rec, sample old) {
    return (rec.t > old.t) && (compute_max ? (rec.v >= old.v) : (rec.v <= old.v)) ;
  }

  float input_to_ext(bool compute_max,float v,long now, int *ini, int *fin,sample* ss) {
    /* Serial.println("debug"); */
    /* Serial.println(OldestTime); */
    /* Serial.println(SIZE_MAX_RING); */

    // Now we push the structure into our rings...
    if (SIZE_MAX_RING != 0) {
      *ini = next_idx(*ini);
    }
    ss[*ini].v = v;
    ss[*ini].t = now;
    SIZE_MAX_RING++;
    if (SIZE_MAX_RING > NUM_SAMPLES) { // we have to drop one off the end, perhaps this should have a warning
      Serial.print( "HAD TO DROP A SAMPLE" );
      *fin = next_idx(*fin);
      SIZE_MAX_RING--;
    }

    // Now we clean the rings of any values that are completely dominated....
    // we will run over all values back ward from where we are to *fin,
    // removing all dominatred values...
    int cur_ext = *ini;
    bool reproc = false;

    for(int i = *ini; i != prev_idx(*fin); i = (reproc ? i : prev_idx(i)) ) {
      /* Serial.print("iter: "); */
      /* Serial.println(i); */
      /* Serial.println(*ini); */
      /* Serial.println(*fin); */
      /* Serial.println(cur_max); */

      reproc = false;
      // if ss[i] is expired, we will drop it...
      if (ss[i].t < OldestTime) {
        ss[i] = ss[*fin];
        *fin = next_idx(*fin);
        SIZE_MAX_RING--;
        if (ss[*fin].v > ss[cur_ext].v) {
          cur_ext = i;
        }
        reproc = true;
      } else {
        if ((compute_max ? (ss[i].v > ss[cur_ext].v) : (ss[i].v < ss[cur_ext].v))) {
          cur_ext = i;
        }
        // we only need to check the new value
        if ((*ini != i) && dominates(compute_max,ss[*ini],ss[i])) {
          // if i is dominated, we can drop it, so we add it to the drop list...
          // since we have the time in order, we move *fin into this slot and increment *fin
          /* Serial.println("dominated"); */
          ss[i] = ss[*fin];
          *fin = next_idx(*fin);
          SIZE_MAX_RING--;
          reproc = true;
        } 
      }
    }
  
    return ss[cur_ext].v;
  }
  void input( float v ) {
    long now = micros();                      // get current time
    float dt = 1e-6*float(now - LastTimeUS);  // find dt
    LastTimeUS = now;                         // save the last time
    OldestTime = now - ((long) (WindowDurationS * 1000.0 * 1000.0));
    
    Xmax = input_to_ext(true,v,now,&max_i,&max_f,maxs);
    Xmin = input_to_ext(false,v,now,&min_i,&min_f,mins);

  }

  float output_max() {
    return Xmax;          // the filtered value (position of oscillator)
  }
  float output_min() {
    return Xmin;          // the filtered value (position of oscillator)
  }
};


void initialize_test_signal() {
  // low frequency -- a large 1-Hz signal
  f[0] = 1.0;
  a[0] = 100.0;

  // mid frequency -- a small 70-Hz signal, modeling a pulse
  f[1] = 70.0;
  a[1] = 10.0;

  // high frequency -- a small 300, modeling noise.
  f[2] = 300.0;
  a[2] = 5.0;
}

int num_loops = 1000;

float signal(long x) {
     float s = 0.0;
     for(int i = 0; i < 3; i++ ) {
       s += a[i] + a[i]*sin( TWO_PI * f[i] * (float) x * 0.01 );   
     }
    return s;
}

float windowLength = 20.0/testFrequency;     // how long to average the signal, for statistist

float testSignalSigma = testAmplitude / sqrt(2.0);         // the RMS amplitude of the test signal
float testSignal3dbSigma = testSignalSigma / sqrt(2.0);    // the RMS amplitude of the test signal, down -3db

float printPeriod = 5.0;

// return the current time
float time() {
  return float( micros() ) * 1e-6;
}

float factor = sqrt(10);  // sqrt 10 will make the attenuation 
FilterTwoPole filterTwoLowpass;   

PeakToPeak peakToPeak;

void setup() {
  Serial.begin( 57600 );    // start the serial port
  initialize_test_signal();
  // create a two pole Lowpass filter
  filterTwoLowpass.setAsFilter( LOWPASS_BUTTERWORTH, testFrequency*factor );

}

void testOnePoleFilters() {
  // filters are test with a sine wave input, keep track of those values here for a sanity check
  RunningStatistics inputStats;                 // create statistics to look at the raw test signal
  inputStats.setWindowSecs( windowLength );
  
  FilterOnePole filterOneLowpass( LOWPASS, testFrequency );   // create a one pole (RC) lowpass filter
  RunningStatistics filterOneLowpassStats;                    // create running statistics to smooth these values
  filterOneLowpassStats.setWindowSecs( windowLength );
  
  FilterOnePole filterOneHighpass( HIGHPASS, testFrequency );  // create a one pole (RC) highpass filter
  RunningStatistics filterOneHighpassStats;                    // create running statistics to smooth these values
  filterOneHighpassStats.setWindowSecs( windowLength );
  
  float startTime = time();
  float nextPrintTime = time();
  
  while( true ) {
    // update all real time classes
    float inputValue = testAmplitude + testAmplitude*sin( TWO_PI * testFrequency * time() );

    // update the test value statistics
    inputStats.input( inputValue);
    
    // update the one pole lowpass filter, and statistics
    filterOneLowpass.input( inputValue );
    filterOneLowpassStats.input( filterOneLowpass.output() );
    
    // update the one pole highpass filter, and statistics
    filterOneHighpass.input( inputValue );
    filterOneHighpassStats.input( filterOneHighpass.output() );
    
    if( time() > nextPrintTime ) {
      // display current values to the screen
      nextPrintTime += printPeriod;   // update the next print time
      
      Serial.print( "\n" );
      Serial.print( "time: " ); Serial.print( time() );
      
      // output values associated with the inputValue itsel
      Serial.print( "\tin: " ); Serial.print( inputStats.mean() ); Serial.print( " +/- " ); Serial.print( inputStats.sigma() );
      Serial.print( " (" ); Serial.print( testOffset ); Serial.print( " +/- " ); Serial.print( testSignalSigma ); Serial.print( ")" );
      
      // output values associated with the lowpassed value
      Serial.print( "\tLP1: " ); Serial.print( filterOneLowpassStats.mean() ); Serial.print( " +/- " ); Serial.print( filterOneLowpassStats.sigma() );
      Serial.print( " (" ); Serial.print( testOffset ); Serial.print( " +/- " ); Serial.print( testSignal3dbSigma ); Serial.print( ")" );

      // output values associated with the highpass value
      Serial.print( "\tHP1: " ); Serial.print( filterOneHighpassStats.mean() ); Serial.print( " +/- " ); Serial.print( filterOneHighpassStats.sigma() );
      Serial.print( " (" ); Serial.print( "0.0" ); Serial.print( " +/- " ); Serial.print( testSignal3dbSigma ); Serial.print( ")" );
    }
  }
}



float testDirectLowPass(float v) {
     filterTwoLowpass.input( v);
     return filterTwoLowpass.output(); 
}

void testTwoPoleFilters() {
  float factor = sqrt(10);  // sqrt 10 will make the attenuation 10x
  
  // filters are test with a sine wave input, keep track of those values here for a sanity check
  RunningStatistics inputStats;                 // create statistics to look at the raw test signal
  inputStats.setWindowSecs( windowLength );

  // standard Lowpass, set to the corner frequency
  FilterTwoPole filterTwoLowpass;                                       // create a two pole Lowpass filter
  filterTwoLowpass.setAsFilter( LOWPASS_BUTTERWORTH, testFrequency );

  RunningStatistics filterTwoLowpassStats;
  filterTwoLowpassStats.setWindowSecs( windowLength );

  // Lowpass, set above corner frequency
  FilterTwoPole filterTwoLowpassAbove;                                       // create a two pole Lowpass filter
  filterTwoLowpassAbove.setAsFilter( LOWPASS_BUTTERWORTH, testFrequency*factor );

  RunningStatistics filterTwoLowpassAboveStats;
  filterTwoLowpassAboveStats.setWindowSecs( windowLength );

  // Lowpass, set below corner frequency
  FilterTwoPole filterTwoLowpassBelow;                                       // create a two pole Lowpass filter
  filterTwoLowpassBelow.setAsFilter( LOWPASS_BUTTERWORTH, testFrequency/factor );

  RunningStatistics filterTwoLowpassBelowStats;
  filterTwoLowpassBelowStats.setWindowSecs( windowLength );

  float startTime = time();
  float nextPrintTime = time();
  
  while( true ) {
    // update all real time classes
    float inputValue = testAmplitude + testAmplitude*sin( TWO_PI * testFrequency * time() );
        
    // update the test value statistics
    inputStats.input( inputValue);
        
    // update the two pole Lowpass filter, and statistics
    filterTwoLowpass.input( inputValue );
    filterTwoLowpassStats.input( filterTwoLowpass.output() );
    
    // update the two pole Lowpass filter, and statistics
    filterTwoLowpassAbove.input( inputValue );
    filterTwoLowpassAboveStats.input( filterTwoLowpassAbove.output() );

    // update the two pole Lowpass filter, and statistics
    filterTwoLowpassBelow.input( inputValue );
    filterTwoLowpassBelowStats.input( filterTwoLowpassBelow.output() );
    
    if( time() > nextPrintTime ) {
      // display current values to the screen
      nextPrintTime += printPeriod;   // update the next print time
      
      Serial.print( "\n" );
      Serial.print( "time: " ); Serial.print( time() );
      
      // output values associated with the inputValue itsel
      Serial.print( "\tin: " ); Serial.print( inputStats.mean() ); Serial.print( " +/- " ); Serial.print( inputStats.sigma() );
      Serial.print( " (" ); Serial.print( testOffset ); Serial.print( " +/- " ); Serial.print( testSignalSigma ); Serial.print( ")" );
            
      // output values associated with the Lowpass value
      Serial.print( "\tLP2: " ); Serial.print( filterTwoLowpassStats.mean() ); Serial.print( " +/- " ); Serial.print( filterTwoLowpassStats.sigma() );
      Serial.print( " (" ); Serial.print( "0.0" ); Serial.print( " +/- " ); Serial.print( testSignal3dbSigma ); Serial.print( ")" );
      
      // output values associated with the Lowpass value, set above corner frequency (should be almost full)
      Serial.print( "\tLP2A: " ); Serial.print( filterTwoLowpassAboveStats.mean() ); Serial.print( " +/- " ); Serial.print( filterTwoLowpassAboveStats.sigma() );

      // output values associated with the Lowpass value, set above corner frequency (should be lower amplitude)
      Serial.print( "\tLP2B: " ); Serial.print( filterTwoLowpassBelowStats.mean() ); Serial.print( " +/- " ); Serial.print( filterTwoLowpassBelowStats.sigma() );
    }
  }
}

long calls = 0;
void plot_signal() {
  float s = signal(calls);

    sensorValue = analogRead(sensorPin);
    s = sensorValue * 5.0 / 1000.0;


  Serial.print(s);
  /* Low-pass-signal */
  float sl = testDirectLowPass(s);
  Serial.print(" ");
  Serial.print(sl);
  /* De-biased signal */
  float sh = s - sl;
  Serial.print(" ");
  Serial.print(sh);
  /* Maximum of signal */
  peakToPeak.input(sh);
  float max = peakToPeak.output_max();

  Serial.print(" ");
  Serial.print(max);

  float min = peakToPeak.output_min();
  Serial.print(" ");
  Serial.print(min);

  // Now this is quite important...R0 is the ratio of the peak-to-peak value to the DC value...
  float R0 = (max - min) / sl;
  Serial.print(" ");
  Serial.print(R0);

  Serial.println();
}


void loop() {
  //testOnePoleFilters();
 // testTwoPoleFilters();
 
 plot_signal();
 calls++;
 // num_loops--;
 if (num_loops < 0) {
  abort();
 }
 delay(100);
}

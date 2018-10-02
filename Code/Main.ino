/* Code for EWB pulse oximeter project
 * 
 * 
 * 
 * Last Updated: 10/2/18
 */

//pin defines
#define redPin 3
#define irPin 5
#define diodePin A0

//data declararions and defines
#define numReadings 30
int valNum = 0;
int redValues[numReadings];
int irValues[numReadings];
int blankValues[numReadings];

//declare variable for reads
int redVal = 0;
int irVal = 0;
int blankVal = 0;

//floats for the spO2 claculations
float rRed;
float rIR;
float rBlank;
float realR;
float spO2;

//one-time initialization
void setup() {
  //clear memory
  for(int i = 0; i < numReadings; i++){
    redValues[i] = 0;
    irValues[i] = 0;
    blankValues[i] = 0;
  }
  //go to start of arrays
  valNum = 0;
  //start serial for debugging
  Serial.begin(9600);
  //set up pins
  pinMode(redPin, OUTPUT);
  pinMode(irPin, OUTPUT);
  pinMode(diodePin, INPUT);
}

//main program loop
void loop()
{
  //COLLECT DATA
  
  //set LEDs to red
  digitalWrite(redPin, HIGH);
  digitalWrite(irPin, LOW);
  delay(100);
  //read red
  redVal = analogRead(diodePin);
  Serial.print("RED: ");
  Serial.println(redVal);
  //temporary delay
  delay(250);
  //set LEDs to ir
  digitalWrite(redPin, LOW);
  digitalWrite(irPin, HIGH);
  delay(100);
  //read ir
  irVal = analogRead(diodePin);
  Serial.print("IR: ");
  Serial.println(irVal);
  //temporary delay
  delay(250);
  //set LEDs off
  digitalWrite(redPin, LOW);
  digitalWrite(irPin, LOW);
  delay(100);
  //read background
  blankVal = analogRead(diodePin);
  Serial.print("BG: ");
  Serial.println(blankVal);
  //temporary delay
  delay(250);

  //ADD DATA

  redValues[valNum] = redVal;
  irValues[valNum] = irVal;
  blankValues[valNum] = blankVal;
  //go to next index, wrap if at end
  valNum = (valNum % numReadings) + 1;

  //CALCULATE
  
  //get the partial R values
  rRed = getR(redValues);
  rIR = getR(irValues);
  rBlank = getR(irValues);

  //get the actual R values
  realR = rRed/rIR;
  //calculate spO2 based on realR
  //needs to be calibrated
  spO2 = 110 - (25 * realR);
  //print spO2
  Serial.print(realR, 2);
  Serial.print('\t');
  Serial.println(spO2, 2);
}

//returns the AC/DC for the passed set of values
float getR(int arr[numReadings]){
  unsigned long sum = 0;  //sum of squares
  int high = 0;     //max value
  int low = 1023;   //min value
  int val;
  //find min and max values and add square to sum
  for(int i = 0; i < numReadings; i++){
    val = arr[i];
    high = val > high ? val : high;
    low = val < low ? val : low;
    sum += val * val;
  }
  //get DC and AC values
  float dc = sum / numReadings;
  dc = sqrt(dc);
  int ac = high - low;
  return (ac / dc);
}



// compass functions

// HMC5883L
#include <HMC5883L.h>
HMC5883L compass;

//for calibration
int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;

void compassSetup()
{
  // Initialize Initialize HMC5883L
  while (!compass.begin())
  {
    delay(500);
  }
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  //compass.setOffset(150, -203, 0); // 16.06.2021
  compass.setOffset(150, -460, 0); // 23.04.2024

  Serial.println(" compass setup successful");
  
  }

void compassCalibrate()
{
  while (2>1){
  compass.setOffset(0, 0, 0);
  Vector mag = compass.readRaw();

  // Determine Min / Max values
  if (mag.XAxis < minX) minX = mag.XAxis;
  if (mag.XAxis > maxX) maxX = mag.XAxis;
  if (mag.YAxis < minY) minY = mag.YAxis;
  if (mag.YAxis > maxY) maxY = mag.YAxis;

  // Calculate offsets
  offX = (maxX + minX) / 2;
  offY = (maxY + minY) / 2;

  Serial.print(offX);
  Serial.print(":");
  Serial.print(offY);
  Serial.print("\n");

  }
}
 

float getCompassHeading()
{
  
  Vector norm = compass.readNormalize();

  // Calculate heading
  float compassHeading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  // Berlin: 4'28
  float declinationAngle = (4.0 + (28.0 / 60.0)) / (180 / M_PI);
  compassHeading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (compassHeading < 0)
  {
    compassHeading += 2 * PI;
  }

  if (compassHeading > 2 * PI)
  {
    compassHeading -= 2 * PI;
  }

  // Convert to degrees
  float compassHeadingDegrees = compassHeading * 180/M_PI; 

  //float error_compensation = 8.0914e-13 * pow(compassHeadingDegrees, 6) - 1.0224e-9 * pow(compassHeadingDegrees, 5) + 4.4334e-7 * pow(compassHeadingDegrees, 4) - 
  //           7.152e-5 * pow(compassHeadingDegrees, 3) + 2.2338e-3 * pow(compassHeadingDegrees, 2) + 5.3333e-2 * compassHeadingDegrees + 1.2191;
  float error_compensation = -2.9315e-13 * pow(compassHeadingDegrees, 6) + 6.8868e-10 * pow(compassHeadingDegrees, 5) - 4.4659e-7 * pow(compassHeadingDegrees, 4) +
             1.081e-4 * pow(compassHeadingDegrees, 3) - 8.0276e-3 * pow(compassHeadingDegrees, 2) - 5.9696e-2 * compassHeadingDegrees - 5.7446;
  float calibrated_compassHeadingDegrees = compassHeadingDegrees + error_compensation;
  return(calibrated_compassHeadingDegrees);
}

# Driving 2 Stepper Motors Using SD CArd and STM32F746Zg

Objective of this code is to simultaneously Drive 2 Stepper Motors Based on the Data Uploaded to a SD Card. SD Card is Connected to STM32F746Zg using an Adaptor.
Data Uploaded to The SD card is in format Of CODE.txt file Which Contains a serie of four zero and one (Based on the Motors Moving Angle in each Time Sample)
Each set of numbers are defined as below:

0000 ( Motor1 Stop Motor2 Stop)
1000 ( Motor1 CW Motor2 Stop)
0100 ( Motor1 CCW Motor2 Stop)
0010 ( Motor1 Stop Motor2 CW)
0001 ( Motor1 Stop Motor2 CCW)
1010 ( Motor1 CW Motor2 CW)
1001 ( Motor1 CW Motor2 CCW)
0110 ( Motor1 CCW Motor2 CW)
0101 ( Motor1 CCW Motor2 CCW)

For Further Information You Are Welcomed To cantact me : "mehrdad_sadeghiye@yahoo.com"




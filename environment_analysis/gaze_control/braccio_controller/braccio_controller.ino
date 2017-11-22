#include <FileIO.h>
#include <Console.h>
#include <Bridge.h>
#include <Servo.h>
#include <Braccio.h>
#include <string.h>

// TODO: Find out what these are for!
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

#define CHAR_LENGTH 4
char bridge_flag[CHAR_LENGTH];

String line;
short angles[100][5];
int c = 0;
char delimiter[] = ","; // Used to split a command to get each joint's angle
int start_index;        // Used with comma_index for parsing the joint's angle
int delimiter_index;    // Index of the first comma found in the string

void setup()
{
  // Zero out the memory used for the bridge
  memset(bridge_flag, 0, CHAR_LENGTH);

  // Initialize the bridge and serial
  Bridge.begin();
  Serial.begin(9600);
  FileSystem.begin();
  Console.begin();

  // Start using the bridge // TODO: This was already initialized! Why has he done it again?
  Bridge.begin();

  Braccio.begin();
  Bridge.put("new_gaze_point", "F");
}

void loop()
{
  // Check if a new gaze point has been sent
  Bridge.get("new_gaze_point", bridge_flag, CHAR_LENGTH);

  // If a new gaze point is available, read the file it was written and act upon it
  if (bridge_flag[0] == 'P')
  {
    // Open the file
    File file = FileSystem.open("/mnt/sda1/gaze_point_angles.csv", FILE_READ);
    if (file)
    {
      while (file.available())
      {
        line = String(file.readStringUntil(10)); // newline in ASCII
        if (line != "" && c != 100)
        {
          start_index = 0;

          for (int i = 0; i < 5; i++)
          {
            comma_index = strcspn(line.c_str(), key);
            angles[c][z] = (short)line.substring(start_index, comma_index).toInt();
            line[comma_index] = '.';
            start_index = comma_index + 1;
          }

          //angles[c][5] = (short)line.substring(comma_index+1, comma_index+3).toInt(); // 10 < gripper < 73
          c++;
        }
        else if (c == 100)
        {
          Console.println("\nERROR: Angles limit (100 positions) has been reached");
        }
      }
    }
    else
    {
      Console.println("\nERROR: File not found");
      delay(1000);
    }

    file.close();

    for (int i = 0; i < c; i++)
    {
      Braccio.ServoMovement(10, angles[i][0], angles[i][1], angles[i][2], angles[i][3], angles[i][4], 10);
      if (i == 0)
      {
        delay(50); // TODO: Apparently this is debugging related - serves as visual indicator
      }
    }

    Bridge.put("RESULT", "complete");
    c = 0;
    Bridge.put("new_gaze_point", "F");
  }
  else
  {
    Console.println("... Waiting for a command ...");
  }

  delay(20);
}


#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <stdio.h>

int main()
{

  printf("Pico is online! Reading CSV...\n");

  std::string filePath = "/Users/gabriellabest/RocketryCode/BLiMS/sim/L3_Launch4_2025.csv";
  std::ifstream file(filePath);

  if (!file.is_open())
  {
    std::cerr << "Could not open the file!" << std::endl;
    return 1;
  }

  std::string line;
  int targetIndex = 1; // Change this to the index you want to print
  int alt_index = 4;

  while (std::getline(file, line))
  {
    std::stringstream ss(line);
    std::string value;
    int currentIndex = 0;

    // Split the line by commas
    while (std::getline(ss, value, ','))
    {
      // if (currentIndex == targetIndex)
      // {
      //   printf("Value: %s\n", value.c_str());
      //   break; // Found the value, move to the next line
      // }

      if (currentIndex == alt_index)
      {
        printf("Alt Value: %s\n", value.c_str());
        break; // Found the value, move to the next line
      }

      currentIndex++;
    }
  }

  file.close();
  return 0;
}
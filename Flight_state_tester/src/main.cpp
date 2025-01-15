#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>


#include "FlightStateDetector.h"

AHRS_t AHRS_d;
FlightState_t FlightState_d;
long long prevTime;

int main() {
    std::ifstream file("data/meas_parsed.csv");
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file." << std::endl;
        return 1;
    }

    std::ofstream output_file("data/output.csv");
    if (!output_file.is_open()) {
        std::cerr << "Error: Could not create the output file." << std::endl;
        return 1;
    }


    std::string line;
    // Read the header line
    std::getline(file, line);

    FSD_init(&AHRS_d);
    FSD_arming();
	

    // Parse the data lines
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        
        double sys_time = 0.0, accX = 0.0, Alt_press = 0.0;
        int column = 0;

        // Split the line by semicolons
        while (std::getline(ss, token, ';')) {
            if (column == 0) {
                sys_time = std::stod(token) * 1000;
            } else if (column == 1) {
                accX = std::stod(token);
            } else if (column == 19) {
                Alt_press = std::stod(token);
            }
            column++;
        }

        AHRS_d.accX = accX*9.81f;
        AHRS_d.acc_axis_lowpass = accX*9.81f;
        AHRS_d.velocityP = (Alt_press - AHRS_d.altitudeP) / ((float)(sys_time - prevTime) / 1000.0f);
        prevTime = sys_time;
        AHRS_d.altitudeP = Alt_press;
        
        if(AHRS_d.altitudeP > AHRS_d.max_altitude){
            AHRS_d.max_altitude = AHRS_d.altitudeP;
        }

       

        FSD_detect((long long)sys_time);

        output_file << sys_time << "," << accX << "," << Alt_press << "," << FSD_getState() << "\n";
        // Pass the extracted values to the function
        //processData(sys_time, accX, Alt_press);
    }

    file.close();
    return 0;
}

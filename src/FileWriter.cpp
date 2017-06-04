#include "FileWriter.h"

FileWriter::FileWriter(std::string filename) {
  this->filename_ = filename;
  this->writeHeader();
}

void FileWriter::writeHeader()
{
  this->writeLine("timestamp,cte,speed,angle,steering_angle,throttle,total_cte,avg_cte");
}

void FileWriter::writePidParameters(std::string title, double kp, double ki, double kd )
{
  std::stringstream string_s;
  string_s << title << ",Kp," << kp << ",Ki," << ki << ",Kd," << kd;
  this->writeLine(string_s.str());
}

void FileWriter::writeParameter(std::string name, double value)
{
  std::stringstream string_s;
  string_s << "," << name << "," << value;
  this->writeLine(string_s.str());
}

void FileWriter::writeLine(double timestamp, double cte, double speed, double angle, double steering_angle,
    double throttle, double total_cte, double avg_cte )
{
  std::stringstream sstring_s
  sstring_s<< timestamp << "," << cte << "," << speed << "," << angle << "," << steering_angle << "," << throttle << "," << total_cte << "," << avg_cte;
  this->writeLine(string_s.str());
}

void FileWriter::writeLine(std::string line) {
  this->write(line + "\n");
}

void FileWriter::write(std::string data) {
  std::ofstream dataFile;
  dataFile.open(this->filename_, std::ios::app);
  dataFile << data;
  dataFile.close();
}

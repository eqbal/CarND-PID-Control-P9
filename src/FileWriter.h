#ifndef SRC_FILEWRITER_H_
#define SRC_FILEWRITER_H_

#include <fstream>
#include <sstream>

class FileWriter {
 public:
  FileWriter(std::string filename);

  void writeLine(double timestamp, double cte, double speed, double angle, double steering_angle,
                 double throttle, double total_cte, double avg_cte );

  void writePidParameters(std::string title, double kp, double ki, double kd );
  void writeParameter(std::string name, double value);

  void writeLine(std::string line);
  void write(std::string data);

 private:
  std::string filename_;
  void writeHeader();

};

#endif /* SRC_FILEWRITER_H_ */

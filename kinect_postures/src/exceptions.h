#ifndef EXCEPTIONS_H
#define	EXCEPTIONS_H

class NoTrackerDataException: public std::exception
{
  public:
  virtual const char* what() const throw()
  {
    return "Insufficient data from OpenNI tracker. Sensor is not ready or user1 is not calibrated yet.";
  }
};

class LoadModelFileException: public std::exception
{
  public:
  virtual const char* what() const throw()
  {
        return "Error loading classifier from file. Check if 'models' folder is in the program running directory (usually ROS home).";
  }
};

#endif	/* EXCEPTIONS_H */

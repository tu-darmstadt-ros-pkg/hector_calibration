#include <lidar_calibration/lidar_calibration.h>
#include <boost/program_options.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_calibration_node");

  ros::NodeHandle nh;
  google::InitGoogleLogging(argv[0]);
  hector_calibration::lidar_calibration::LidarCalibration calibration(nh);

  boost::program_options::options_description desc("Allowed options");

  desc.add_options()
      ("help", "produce help message")
      ("n", "Enable normal visualization")
      ("m", "Enable manual mode")
  ;

  boost::program_options::variables_map vmap;
  boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).allow_unregistered().run(), vmap);
  boost::program_options::notify(vmap);

  if (vmap.count("help")) {
    std::cout << desc << std::endl;
    return 0;
  }
  if (vmap.count("n")) {
    calibration.enableNormalVisualization(true);
  }
  if (vmap.count("m")) {
    calibration.setManualMode(true);
  }

  calibration.setPeriodicPublishing(true, 5);
  calibration.loadOptionsFromParamServer();

  calibration.calibrate();

  return 0;
}

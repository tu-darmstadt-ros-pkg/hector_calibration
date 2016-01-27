#include <ros/ros.h>
#include <boost/date_time.hpp>
#include <omp.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_node");

  int i;
  int threadID = 0;
  #pragma omp parallel for private(i, threadID)
  for(i = 0; i < 16; i++ )
  {
      threadID = omp_get_thread_num();
      #pragma omp critical
      {
          printf("Thread %d reporting\n", threadID);
      }
  }
  return 0;
}

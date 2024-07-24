#include <iostream>
#include <fabrics_controllers/pure_controller.c>
#include <chrono>

void print_array(const char* intro, double* array, unsigned int length) {
  std::cout << intro;
  for(unsigned int i = 0; i < length; ++i){
    std::cout << array[i] << " ";
  }
  std::cout << std::endl;
}

void print_array(const char* intro, const double* array, unsigned int length) {
  std::cout << intro;
  for(unsigned int i = 0; i < length; ++i){
    std::cout << array[i] << " ";
  }
  std::cout << std::endl;
}

int main(int argc, char *argv[])
{
  // Some settings
  // Unclear what they are used for
  long long int* setting_0 = new long long int[2];
  double* setting_1 = new double[2];
  int setting_2 = 0;

  // Define inputs and outputs
  // Potential Segmentation Faults can be caused by the wrong number of arguments.
  unsigned int nb_inputs = 4;
  unsigned int nb_outputs = 1;

  double joint_positions[] = {0.1, 0.5, 0.1, 0.1, 0.1, 0.1, 0.1};
  double joint_velocities[] = {0.4, 1.0, 0.1, 0.1, 0.1, 0.1, 0.1};
  double goal_position_0[] = {0.3, 0.2, 0.1, 0.2, 0.2, 0.3, 0.1};
  double weight_goal_0[] = {0.1};

  const double** input = new const double*[nb_inputs];
  input[0] = joint_positions;
  input[1] = joint_velocities;
  input[2] = weight_goal_0;
  input[3] = goal_position_0;
  double** output = new double*[nb_outputs];
  output[0] = new double[7];

  // Timing 
  auto start = std::chrono::high_resolution_clock::now();

  unsigned int N = 10;
  for (unsigned int i = 0; i < N; i++) {
    casadi_f0(input, output, setting_0, setting_1, setting_2);
  }
  // Stop the timer
  auto end = std::chrono::high_resolution_clock::now();

  // Calculate the elapsed time in microseconds
  std::chrono::nanoseconds duration = std::chrono::duration_cast<std::chrono::nanoseconds>((end - start)/N);

  // Print the elapsed time in microseconds, milliseconds, or seconds
  std::cout << "Average compute time: " << duration.count() << " ns" << std::endl;
  print_array("Output of motion planning : ", output[0], 7);




  return EXIT_SUCCESS;
}

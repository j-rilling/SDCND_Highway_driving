#include "PTG.h"

PTG::PTG() {

}

PTG::~PTG() {

}

std::vector<double> PTG::JMT(std::vector<double> &start, std::vector<double> &end, double T) {
      /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */

  double a0, a1, a2, a3, a4, a5;
   
   a0 = start[0];
   a1 = start[1];
   a2 = start[2]/2;

   double sp = end[0] - (a0 + a1*T + a2*T*T);
   double sv = end[1] - (a1 + 2.0*a2*T);
   double sa = end[2] - 2.0*a2;

   double det = 1/(2.0*pow(T,9.0));

   a3 = det*(20.0*sp*pow(T,6.0) - 8.0*sv*pow(T,7.0) + sa*pow(T,8.0));
   a4 = det*(-30.0*sp*pow(T,5.0) + 14.0*sv*pow(T,6.0) - 2.0*sa*pow(T,7.0));
   a5 = det*(12.0*sp*pow(T,4.0) - 6.0*sv*pow(T,5.0) + sa*pow(T,6.0));

   return {a0, a1, a2, a3, a4, a5};
}
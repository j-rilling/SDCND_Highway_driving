#ifndef PTG_H
#define PTG_H

// This class is responsible of generating polynomial trajectories

class PTG {
  public:
    PTG();
    ~PTG();

  private:
    const double N_SAMPLES = 10;
    const double SIGMA_S[3] = {10.0, 4.0, 2.0};
    const double SIGMA_D[3] = {1.0, 1.0, 1.0};
    const double SIGMA_T = 2.0;
    
    const double MAX_JERK = 10;     // m/(s^3)
    const double MAX_ACCEL = 10;    // m/(s^2)

    const double EXPECTED_JERK_IN_ONE_SEC = 2; // m/(s^2)
    const double EXPECTED_ACC_IN_ONE_SEC = 1;  // m/s

    const double SPEED_LIMIT = 30.0;
    const double VEHICLE_RADIUS = 1.5;  // model vehicle as circle to simplify collision detection (change later)

    const double ROAD_D[2] = {0.0, 5.0}; // Change later to {0.0, 12.0}

};


#endif //PTG_H
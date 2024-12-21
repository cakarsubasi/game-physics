//
// Created by Jasmina Vulovic on 19.12.24.
//

#ifndef SINGLESTEP_H
#define SINGLESTEP_H
#include "Scene.h"
#include <iostream>
#include <vector>

 class SingleStep :public Scene {
    const double nu = 0.1; // thermal diffusivity
    const double dt = 0.1; //time step
    const double x_min = 0.0;
    const double x_max = 2.0; // domain in x-direction
    const double y_min = 0.0;
    const double y_max = 4.0; // domain in y-direction

     void explicitEulerStep() {
         int m = 3;
         int n = 6;
         double dx = (x_max - x_min) / (m + 1);
         double dy = (y_max - y_min) / (n + 1);

         // given in problem statement
         std::vector<std::vector<double>> T0 = {
             {6, 5, 1, -1, -2, -1},
             {4, 3, 0, -1, -3, -1},
             {3, 2, -1, -2, -4, -2}
         };

         std::vector<std::vector<double>> T1(m, std::vector<double>(n, 0.0));

         // FTCS formula step by step:

         for (int i = 0; i < m; ++i) {
             for (int j = 0; j < n; ++j) {
                 double Txx = 0.0;
                 double Tyy = 0.0;

                 // Compute second derivatives in x and y directions (handling boundaries)
                 if (i > 0) Txx += T0[i - 1][j];
                 if (i < m - 1) Txx += T0[i + 1][j];
                 Txx -= 2 * T0[i][j];
                 Txx /= (dx * dx);

                 if (j > 0) Tyy += T0[i][j - 1];
                 if (j < n - 1) Tyy += T0[i][j + 1];
                 Tyy -= 2 * T0[i][j];
                 Tyy /= (dy * dy);

                 // Update temperature
                 T1[i][j] = T0[i][j] + nu * dt * (Txx + Tyy);
                 std::cout << "Scene Single-Step: Single-step FTCS Results:" << std::endl;
                 std::cout << "T[1][1,3] = " << T1[1][3] << std::endl;
                 std::cout << "T[1][0,3] = " << T1[0][3] << std::endl;
                 std::cout << "T[1][0,5] = " << T1[0][5] << std::endl;
             }
         }
     }

     void init() override {
         explicitEulerStep();
     }

};



#endif //SINGLESTEP_H

/**
 * My Solution to the AImotive planning assignement 
 *
 * Copyright 2018 by Ziad ZAMZAMI <zamzami@isir.upmc.fr>
 *
 * This file is part of my Solution to the AImotive planning assignement.
 * 
 * You can redistribute it and/or modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation, either version 3 of the License.
 * 
 * It is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 * See theGNU General Public License for more details.
 *
 * @license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>
 */

#pragma once
#include <vector>

class AimotiveMpc {
 public:
  AimotiveMpc();

  virtual ~AimotiveMpc();
  // Solve the Model Predictive Control (MPC) problem
  // given the current state (position, velocity, acceleration and jerk).
  // Return only the first state from the calculated plan.
  std::vector<double> Solve(
      double pos,
      double vel,
      double acc,
      double jerk,
      double ref_vel);
};

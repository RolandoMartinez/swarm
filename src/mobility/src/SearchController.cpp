#include "SearchController.h"

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
}

/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation) {
  geometry_msgs::Pose2D goalLocation;

  //select new heading from Gaussian distribution around current heading
<<<<<<< HEAD
  goalLocation.theta = currentLocation.theta + rng->gaussian(0.14, 0.21) ;// original rng->gaussian(currentLocation.theta, 0.25); - Abe
=======
<<<<<<< HEAD
  goalLocation.theta = currentLocation.theta + rng->gaussian(0.14, 0.21) ;// original rng->gaussian(currentLocation.theta, 0.25); - Abe
=======
  goalLocation.theta = currentLocation.theta + 0.175;// original rng->gaussian(currentLocation.theta, 0.25); - Abe
                                                   //i believe that its the value to decide a new angle change between 9-10 degrees
  //select new position 50 cm from current location
>>>>>>> 7a709764b24be41d4d752890422b841c27c9578d
>>>>>>> 55feff949fc86396d6dd5775d6ef517625128407
  goalLocation.x = currentLocation.x + (0.4 * cos(goalLocation.theta));//0.5 original Abe
  goalLocation.y = currentLocation.y + (0.4 * sin(goalLocation.theta));//0.5 original Abe


  return goalLocation;
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation) {
  geometry_msgs::Pose2D newGoalLocation;

  //remainingGoalDist avoids magic numbers by calculating the dist
  double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

  //this of course assumes random walk continuation. Change for diffrent search methods.
  newGoalLocation.theta = oldGoalLocation.theta;
  newGoalLocation.x = currentLocation.x + (0.50 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
  newGoalLocation.y = currentLocation.y + (0.50 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));

  return newGoalLocation;
}

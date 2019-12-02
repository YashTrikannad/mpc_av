//
// Created by yash on 12/2/19.
//

/// Get an appropriate steering speed based on input steering angle
/// @param steering_angle
/// @return steering speed
double get_steering_speed(double steering_angle)
{
    // Calculated Using https://www.desmos.com/calculator/wdb45brrj8
    // Polynomial Regression Tool
    if(steering_angle < 0.05)
        return 5;
    if(steering_angle < 0.1)
        return 3;
    if(steering_angle < 0.15)
        return 2;
    if(steering_angle < 0.23)
        return 1;
}

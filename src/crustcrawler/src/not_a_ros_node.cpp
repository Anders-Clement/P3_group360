
////////////////////////////////////////////////////////////////
// Used by Clement for testing stuff, just leave me bere
////////////////////////////////////////////////////////////////

#include "math.h"
#include <iostream>



struct Vector3
{
  float x;
  float y;
  float z;
};


Vector3 f_kin(Vector3 thetas);
Vector3 inv_kin_closest(Vector3 position, Vector3 angles);
void check_for_zero(Vector3 &input);


int main(int argc, char const *argv[]) {

  std::cout << std::endl << std::endl << std::endl << std::endl;

  Vector3 input;
  input.x = -2.0;
  input.y = 1.20;
  input.z = 1.0;

  std::cout << "Input angles, theta1: " << input.x << ", theta2: " << input.y << ", theta3: " << input.z << std::endl;

  Vector3 result = f_kin(input);
  std::cout <<"f_kin, x: "<< result.x << ", y: " << result.y << ", z: " << result.z << std::endl;

  result = inv_kin_closest(result, input);
  std::cout <<"inv_kin_closest, x: "<< result.x << ", y: " << result.y << ", z: " << result.z << std::endl;

  return 0;
}


void check_for_zero(Vector3 &input)
{
  float zero_value = 0.0001;
  if (input.x == 0.0)
    input.x = zero_value;
  if (input.y == 0.0)
    input.y = zero_value;
  if (input.z == 0.0)
    input.z = zero_value;
}

Vector3 f_kin(Vector3 thetas)
{
  Vector3 result;
  float pi = 3.1416;
  result.x = (11*cos(thetas.x)*cos(thetas.y + pi/2))/50 - (3*cos(thetas.x)*sin(thetas.z)*sin(thetas.y + pi/2))/20 + (3*cos(thetas.x)*cos(thetas.z)*cos(thetas.y + pi/2))/20;
  result.y = (11*cos(thetas.y + pi/2)*sin(thetas.x))/50 + (3*cos(thetas.z)*cos(thetas.y + pi/2)*sin(thetas.x))/20 - (3*sin(thetas.x)*sin(thetas.z)*sin(thetas.y + pi/2))/20;
  result.z = (11*sin(thetas.y + pi/2))/50 + (3*cos(thetas.z)*sin(thetas.y + pi/2))/20 + (3*cos(thetas.y + pi/2)*sin(thetas.z))/20 + 11.0/200.0;

  return result;
}

Vector3 inv_kin_closest(Vector3 pos, Vector3 angles)
{
  check_for_zero(angles);
  check_for_zero(pos);

  std::cout << "angles vector: ";
  std::cout << angles.x << ", "<< angles.y << ", "<< angles.z << ", " << std::endl;

  std::cout << "pos vector: ";
  std::cout << pos.x << ", "<< pos.y << ", "<< pos.z << ", " << std::endl;

  //constants
  float pi = 3.14159;
  float L2 = 0.150;
  float L1 = 0.220;
  float z1 = 0.055;

  //extra angles and lenghs
  float a1 = sqrt(pos.x*pos.x + pos.y*pos.y);
  float d1 = sqrt((pos.z-z1)*(pos.z-z1) + a1*a1);

  //more angles to calculate a solution
  float alpha = acos((d1*d1 + L1*L1 - L2*L2)/(2*L1*d1));
  float tmp = (L1*L1 + L2*L2 - d1*d1)/(2*L1*L2);
  if(tmp < -1.0)
    if(tmp + 1.0 < -0.00001)
      std::cout << "ACOS FAILURE, LESS THAN -1, OUT OF REACH? MAYBE?" << std::endl;
    else
      tmp = -1.0;
  if(tmp > 1.0)
    if(tmp - 1.0 > 0.00001)
      std::cout << "ACOS FAILURE, GREATER THAN 1, OUT OF REACH? MAYBE?" << std::endl;
    else
      tmp = 1.0;


  float beta = acos(tmp);
  float delta = atan2((pos.z - z1),a1);

  std::cout << "a1: " << a1 << std::endl;
  std::cout << "d1: " << d1 << std::endl;
  std::cout << "alpha: " << alpha << std::endl;
  std::cout << "beta: " << beta << std::endl;
  std::cout << "delta: " << delta << std::endl;


  //calculate the four solutions
  Vector3 solutions[4];

  solutions[0].x = atan2(pos.y,pos.x) + pi;
  solutions[0].y = (pi/2)-(alpha+delta);
  solutions[0].z = pi-beta;

  solutions[1].x = atan2(pos.y,pos.x);
  solutions[1].y = (-pi/2)+(alpha+delta);
  solutions[1].z = -pi+beta;

  solutions[2].x = atan2(pos.y,pos.x);
  solutions[2].y = (-pi/2)-(alpha-delta);
  solutions[2].z = pi-beta;

  solutions[3].x = atan2(pos.y,pos.x) + pi;
  solutions[3].y = -((-pi/2)-(alpha-delta));
  solutions[3].z = -pi+beta;

  //print solutions
  std::cout << std::endl << "four solutions for inv_kin: " << std::endl;
  for (size_t i = 0; i < 4; i++) {
    std::cout <<"x: "<< solutions[i].x << ", y: " << solutions[i].y << ", z: " << solutions[i].z << std::endl;
  }
  std::cout << std::endl;



  //find the closest solution:

  float distance[4];

  for (size_t i = 0; i < 4; i++) {
    distance[i] = 0;
    distance[i] += std::abs(solutions[i].x - angles.x);
    distance[i] += std::abs(solutions[i].y - angles.y);
    distance[i] += std::abs(solutions[i].z - angles.z);
    std::cout <<"distance " << i << ": " << distance[i] << ", ";

  }
  std::cout << std::endl;

  int lowest = 0;

  for (size_t i = 0; i < 4; i++) {
    if(distance[i] < distance[lowest])
      lowest = i;
  }

  return solutions[lowest];
}

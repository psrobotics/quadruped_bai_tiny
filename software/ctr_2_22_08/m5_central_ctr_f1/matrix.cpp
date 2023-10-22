#include "matrix.h"
#include <math.h>
#include "Arduino.h"

matrix3d::matrix3d()
{
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      mat[i][j] = 0;
}

void matrix3d::operator=(const matrix3d &in)
{
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      mat[i][j] = in.mat[i][j];
}

matrix3d matrix3d::operator+(const matrix3d &in)
{
  matrix3d result;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      result.mat[i][j] = mat[i][j] + in.mat[i][j];
  return result;
}

matrix3d matrix3d::operator-(const matrix3d &in)
{
  matrix3d result;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      result.mat[i][j] = mat[i][j] - in.mat[i][j];
  return result;
}

matrix3d matrix3d::operator*(const matrix3d &in)
{
  matrix3d result;
  for (int c = 0; c < 3; c++)
    for (int r = 0; r < 3; r++)
      result.mat[r][c] = mat[r][0] * in.mat[0][c] + mat[r][1] * in.mat[1][c] + mat[r][2] * in.mat[2][c];
  return result;
}

vec3d matrix3d::operator*(const vec3d &in)
{
  vec3d result;
  for (int n = 0; n < 3; n++)
    for (int s = 0; s < 3; s++)
      result.vec[n] += mat[n][s] * in.vec[s];
  return result;
}

void matrix3d::set_indentity()
{
  for (int s = 0; s < 3; s++)
    mat[s][s] = 1;
}

void matrix3d::gen_rotx(float anglex)
{
  set_indentity();
  mat[1][1] = cos(anglex);
  mat[1][2] = -1 * sin(anglex);
  mat[2][1] = sin(anglex);
  mat[2][2] = cos(anglex);
}

void matrix3d::gen_roty(float angley)
{
  set_indentity();
  mat[0][0] = cos(angley);
  mat[0][2] = -1 * sin(angley);
  mat[2][0] = sin(angley);
  mat[2][2] = cos(angley);
}

void matrix3d::gen_rotz(float anglez)
{
  set_indentity();
  mat[0][0] = cos(anglez);
  mat[0][1] = -1 * sin(anglez);
  mat[1][0] = sin(anglez);
  mat[1][1] = cos(anglez);
}

void matrix3d::gen_rotation(float *angle_in)
{
  matrix3d rotx; rotx.gen_rotx(angle_in[0]);
  matrix3d roty; roty.gen_roty(angle_in[1]);
  matrix3d rotz; rotz.gen_rotz(angle_in[2]);

  matrix3d result; result = rotx;
  result = result * roty;
  result = result * rotz;

  //print_mat(result);

  for (int n = 0; n < 3; n++)
    for (int s = 0; s < 3; s++)
      mat[n][s] = result.mat[n][s];
}

vec3d::vec3d()
{
  for (int i = 0; i < 3; i++)
    vec[i] = 0;
}

void vec3d::operator=(const vec3d &in)
{
  for (int s = 0; s < 3; s++)
    vec[s] = in.vec[s];
}

vec3d vec3d::operator+(const vec3d &in)
{
  vec3d result;
  for (int s = 0; s < 3; s++)
    result.vec[s] = vec[s] + in.vec[s];
  return result;
}

vec3d vec3d::operator-(const vec3d &in)
{
  vec3d result;
  for (int s = 0; s < 3; s++)
    result.vec[s] = vec[s] - in.vec[s];
  return result;
}

void vec3d::gen_trans(float *pos_in)
{
  for (int s = 0; s < 3; s++)
    vec[s] = pos_in[s];
}

#ifndef _MATRIX__
#define _MATRIX__

class matrix3d;
class vec3d;

class matrix3d
{
  public:
    float mat[3][3];
    matrix3d();
    void operator=(const matrix3d &in);
    matrix3d operator+(const matrix3d &in);
    matrix3d operator-(const matrix3d &in);
    matrix3d operator*(const matrix3d &in);
    void set_indentity();

    void gen_rotx(float anglex);
    void gen_roty(float angley);
    void gen_rotz(float anglez);
    void gen_rotation(float *angle_in);

    vec3d operator*(const vec3d &in);
};

class vec3d
{
  public:
    float vec[3];
    vec3d();
    void operator=(const vec3d &in);
    vec3d operator+(const vec3d &in);
    vec3d operator-(const vec3d &in);

    void gen_trans(float *pos_in);
};

#endif

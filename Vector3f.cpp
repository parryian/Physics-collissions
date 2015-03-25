#include "Vector3f.h"


//Vector3f::Vector3f(float x, float y, float z) : X(x), Y(y), Z(z){};
/*Vector3f Vector3f::add(Vector3f a, Vector3f b) {
         float x, y, z;
         x = a.getX() + b.getX();
         y = a.getY() + b.getY();
         z = a.getZ() + b.getZ();
         return Vector3f(x,y,z);
};
*/

// OUTPUT THE VECTOR IN THE FORM [i,j,k]
void Vector3f::print()
{
    std::cout << "[" <<  X << "," << Y << "," << Z << "]" <<std::endl;
};

// ADD TWO VECTORS TOGETHER
Vector3f Vector3f::operator + (Vector3f param)
{
    Vector3f c;
    c.X=X+param.X;
    c.Y=Y+param.Y;
    c.Z=Z+param.Z;
    return (c);
};
Point Vector3f::vToP()
{
      Point p;
      p.x = X;
      p.y = Y;
      p.z = Z;
      return p;
      
};
void Vector3f::operator += (Vector3f param)
{
    Vector3f c;
    c.X=X+param.X;
    c.Y=Y+param.Y;
    c.Z=Z+param.Z;
    *this = c;
};

// SUBTRACT ONE VECTOR FROM ANOTHER
Vector3f Vector3f::operator - (Vector3f param)
{
    Vector3f c;
    c.X=X-param.X;
    c.Y=Y-param.Y;
    c.Z=Z-param.Z;
    return (c);
};

// SET A VECTOR TO ANOTHER VECTOR
Vector3f Vector3f::operator = (Vector3f param)
{
    X=param.X;
    Y=param.Y;
    Z=param.Z;
    return *this;
};

// CROSS PRODUCT
Vector3f Vector3f::operator / (Vector3f param)
{
    Vector3f c;
    c.X=Y*param.Z-Z*param.Y;
    c.Y=Z*param.X-X*param.Z;
    c.Z=X*param.Y-Y*param.X;
    return (c);
};

// DOT PRODUCT
float Vector3f::operator % (Vector3f param)
{
    return (X*param.X+Y*param.Y+Z*param.Z);
};

// ANGLE BETWEEN IN RADIANS
float Vector3f::operator ^ (Vector3f b)
{
    Vector3f a=*this;
    return (acos((a%b)/(a.mod()*b.mod())));
};

// MULTIPLY BY REAL
Vector3f Vector3f::operator * (float b)
{
    Vector3f c;
    c.X=X*b;
    c.Y=Y*b;
    c.Z=Z*b;
    return (c);
};

// DIVIDE BY REAL
Vector3f Vector3f::operator / (float b)
{
    Vector3f c;
    c.X=X/b;
    c.Y=Y/b;
    c.Z=Z/b;
    return (c);
};

// STRETCH VECTORS
Vector3f Vector3f::operator * (Vector3f b)
{
    Vector3f c;
    c.X=X*b.X;
    c.Y=Y*b.Y;
    c.Z=Z*b.Z;
    return (c);
};

Vector3f Vector3f::compProd(Vector3f b)
{
    Vector3f c;
    c.X=X*b.X;
    c.Y=Y*b.Y;
    c.Z=Z*b.Z;
    return (c);
};

// MODULUS -- magnitude of vector
float Vector3f::mod()
{
    return sqrt(pow(X,2)+pow(Y,2)+pow(Z,2));
};



float Vector3f::mod2()
{
    return (pow(X,2)+pow(Y,2)+pow(Z,2));
};

// A.B = ||B||*cos(angle between A and B)
float Vector3f::dot(Vector3f param)
{
      Vector3f A = *this;
      //return param.mod()*cos(A ^ param);
      return A%param;
};

// UNIT VECTOR -- normalized vector
Vector3f Vector3f::unit()
{
    Vector3f c;
    c.X=X/this->mod();
    c.Y=Y/this->mod();
    c.Z=Z/this->mod();
    return c;
};

///>Matrix4Inverse
float Matrix4::getDeterminant() const
{
    return data[8]*data[5]*data[2]+
        data[4]*data[9]*data[2]+
        data[8]*data[1]*data[6]-
        data[0]*data[9]*data[6]-
        data[4]*data[1]*data[10]+
        data[0]*data[5]*data[10];
}

void Matrix4::setInverse(const Matrix4 &m)
{
    // Make sure the determinant is non-zero.
    float det = getDeterminant();
    if (det == 0) return;
    det = ((float)1.0)/det;

    data[0] = (-m.data[9]*m.data[6]+m.data[5]*m.data[10])*det;
    data[4] = (m.data[8]*m.data[6]-m.data[4]*m.data[10])*det;
    data[8] = (-m.data[8]*m.data[5]+m.data[4]*m.data[9]* m.data[15])*det;

    data[1] = (m.data[9]*m.data[2]-m.data[1]*m.data[10])*det;
    data[5] = (-m.data[8]*m.data[2]+m.data[0]*m.data[10])*det;
    data[9] = (m.data[8]*m.data[1]-m.data[0]*m.data[9]* m.data[15])*det;

    data[2] = (-m.data[5]*m.data[2]+m.data[1]*m.data[6]* m.data[15])*det;
    data[6] = (+m.data[4]*m.data[2]-m.data[0]*m.data[6]* m.data[15])*det;
    data[10] = (-m.data[4]*m.data[1]+m.data[0]*m.data[5]* m.data[15])*det;

    data[3] = (m.data[9]*m.data[6]*m.data[3]
               -m.data[5]*m.data[10]*m.data[3]
               -m.data[9]*m.data[2]*m.data[7]
               +m.data[1]*m.data[10]*m.data[7]
               +m.data[5]*m.data[2]*m.data[11]
               -m.data[1]*m.data[6]*m.data[11])*det;
    data[7] = (-m.data[8]*m.data[6]*m.data[3]
               +m.data[4]*m.data[10]*m.data[3]
               +m.data[8]*m.data[2]*m.data[7]
               -m.data[0]*m.data[10]*m.data[7]
               -m.data[4]*m.data[2]*m.data[11]
               +m.data[0]*m.data[6]*m.data[11])*det;
    data[11] =(m.data[8]*m.data[5]*m.data[3]
               -m.data[4]*m.data[9]*m.data[3]
               -m.data[8]*m.data[1]*m.data[7]
               +m.data[0]*m.data[9]*m.data[7]
               +m.data[4]*m.data[1]*m.data[11]
               -m.data[0]*m.data[5]*m.data[11])*det;
}

Matrix3 Matrix3::linearInterpolate(const Matrix3& a, const Matrix3& b, float prop)
{
	Matrix3 result;
	for (unsigned i = 0; i < 9; i++) {
		result.data[i] = a.data[i] * (1-prop) + b.data[i] * prop;
	}
	return result;
}




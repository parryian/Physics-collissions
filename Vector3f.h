#include <math.h>
#include <sstream>
#include <iostream>
#include <string>
#include "AABB.h"

class Vector3f
{
private:
  
public:
       float X, Y, Z;
  Vector3f() : X(0.0f), Y(0.0f), Z(0.0f) {}
  Vector3f(float x, float y, float z) : X(x), Y(y), Z(z){}      
  float getX() const { return X; }
  float getY() const { return Y; }
  float getZ() const { return Z; }
  
  Vector3f operator + (Vector3f);
  void operator += (Vector3f);
  Vector3f operator - (Vector3f);
  Vector3f operator = (Vector3f);
 // Vector3f operator = (string);
  Vector3f operator / (Vector3f);
  Vector3f operator * (float);
  Vector3f operator / (float);
  Vector3f operator * (Vector3f);
  Vector3f unit();
  float dot(Vector3f);
  float operator % (Vector3f);
  float operator ^ (Vector3f);
  Vector3f compProd(Vector3f);
  float mod();
  float mod2();
  void print();
  void zero() { X = Y = Z =0; }
  Point vToP();
  
  void setX(const float& value) { X = value; }
  void setY(const float& value) { Y = value; }
  void setZ(const float& value) { Z = value; } 
 // Vector3f add(Vector3f from, Vector3f transVec);
};

class Quaternion
    {
///<Quaternion
    public:
        union {
            struct {
                /**
                 * Holds the float component of the quaternion.
                 */
                float r;

                /**
                 * Holds the first complex component of the
                 * quaternion.
                 */
                float i;

                /**
                 * Holds the second complex component of the
                 * quaternion.
                 */
                float j;

                /**
                 * Holds the third complex component of the
                 * quaternion.
                 */
                float k;
            };

            /**
             * Holds the quaternion data in array form.
             */
            float data[4];
        };
///<QuaternionIntro

///>Omit;Quaternion
        // ... other Quaternion code as before ...

///<Omit;Quaternion
        /**
         * The default constructor creates a quaternion representing
         * a zero rotation.
         */
        Quaternion() : r(1), i(0), j(0), k(0) {}

        /**
         * The explicit constructor creates a quaternion with the given
         * components.         
         *
         * @param r The float component of the rigid body's orientation
         * quaternion.
         *
         * @param i The first complex component of the rigid body's
         * orientation quaternion.
         *
         * @param j The second complex component of the rigid body's
         * orientation quaternion.
         *
         * @param k The third complex component of the rigid body's
         * orientation quaternion.
         *
         * @note The given orientation does not need to be normalised,
         * and can be zero. This function will not alter the given 
         * values, or normalise the quaternion. To normalise the 
         * quaternion (and make a zero quaternion a legal rotation),
         * use the normalise function.
         *
         * @see normalise
         */
        Quaternion(const float r, const float i, const float j, const float k) 
            : r(r), i(i), j(j), k(k) 
        {
        }
        
///>QuatNormalise
        /**
         * Normalises the quaternion to unit length, making it a valid
         * orientation quaternion.
         */
        void normalise()
        {
            float d = r*r+i*i+j*j+k*k;

            // Check for zero length quaternion, and use the no-rotation
            // quaternion in that case.
            if (d == 0) { 
                r = 1; 
                return;
            }

            d = ((float)1.0)/sqrt(d);
            r *= d;
            i *= d;
            j *= d;
            k *= d;
        }
///<QuatNormalise

///>QuatCombine
        /**
         * Multiplies the quaternion by the given quaternion.
         *
         * @param multiplier The quaternion by which to multiply.
         */
        void operator *=(const Quaternion &multiplier)
        {
            Quaternion q = *this;
            r = q.r*multiplier.r - q.i*multiplier.i - 
                q.j*multiplier.j - q.k*multiplier.k;
            i = q.r*multiplier.i + q.i*multiplier.r + 
                q.j*multiplier.k - q.k*multiplier.j;
            j = q.r*multiplier.j + q.j*multiplier.r + 
                q.k*multiplier.i - q.i*multiplier.k;
            k = q.r*multiplier.k + q.k*multiplier.r + 
                q.i*multiplier.j - q.j*multiplier.i;
        }
///<QuatCombine

///>QuatUpdateByVel
        /**
         * Adds the given vector to this, scaled by the given amount. 
         * This is used to update the orientation quaternion by a rotation
         * and time. 
         *
         * @param vector The vector to add.
         *
         * @param scale The amount of the vector to add.
         */
        void addScaledVector(const Vector3f& vector, float scale)
        {
            Quaternion q(0,
                vector.X* scale,
                vector.Y* scale,
                vector.Z* scale);
            q *= *this;
            r += q.r * ((float)0.5);
            i += q.i * ((float)0.5);
            j += q.j * ((float)0.5);
            k += q.k * ((float)0.5);
        }
///<QuatUpdateByVel

///>QuatRotByVector
        void rotateByVector(const Vector3f& vector)
        {
            Quaternion q(0, vector.X, vector.Y, vector.Z);
            (*this) *= q;
        }
///<QuatRotByVector
///>Quaternion;QuaternionIntro
    };
///<Quaternion;QuaternionIntro

///>Matrix4;Matrix4Intro
    /** 
     * Holds a transform matrix, consisting of a rotation matrix and
     * a position. The matrix has 12 elements, it is assumed that the
     * remaining four are (0,0,0,1); producing a homogenous matrix.
     */
    class Matrix4
    {
///<Matrix4
    public:
        /**
         * Holds the transform matrix data in array form.
         */
        float data[12];
///<Matrix4Intro

///>Omit;Matrix4
        // ... Other Matrix4 code as before ...
        
///<Omit;Matrix4

        /**
         * Creates an identity matrix.
         */
        Matrix4()
        {
            data[1] = data[2] = data[3] = data[4] = data[6] = 
                data[7] = data[8] = data[9] = data[11] = 0;
            data[0] = data[5] = data[10] = 1;
        }

        /**
         * Sets the matrix to be a diagonal matrix with the given coefficients.
         */
        void setDiagonal(float a, float b, float c)
        {
            data[0] = a;
            data[5] = b;
            data[10] = c;
        }

///>Matrix4MatrixMultiply
        /** 
         * Returns a matrix which is this matrix multiplied by the given 
         * other matrix. 
         */
        Matrix4 operator*(const Matrix4 &o) const
        {
            Matrix4 result;
            result.data[0] = (o.data[0]*data[0]) + (o.data[4]*data[1]) + (o.data[8]*data[2]);
            result.data[4] = (o.data[0]*data[4]) + (o.data[4]*data[5]) + (o.data[8]*data[6]);
            result.data[8] = (o.data[0]*data[8]) + (o.data[4]*data[9]) + (o.data[8]*data[10]);

            result.data[1] = (o.data[1]*data[0]) + (o.data[5]*data[1]) + (o.data[9]*data[2]);
            result.data[5] = (o.data[1]*data[4]) + (o.data[5]*data[5]) + (o.data[9]*data[6]);
            result.data[9] = (o.data[1]*data[8]) + (o.data[5]*data[9]) + (o.data[9]*data[10]);

            result.data[2] = (o.data[2]*data[0]) + (o.data[6]*data[1]) + (o.data[10]*data[2]);
            result.data[6] = (o.data[2]*data[4]) + (o.data[6]*data[5]) + (o.data[10]*data[6]);
            result.data[10] = (o.data[2]*data[8]) + (o.data[6]*data[9]) + (o.data[10]*data[10]);

            result.data[3] = (o.data[3]*data[0]) + (o.data[7]*data[1]) + (o.data[11]*data[2]) + data[3];
            result.data[7] = (o.data[3]*data[4]) + (o.data[7]*data[5]) + (o.data[11]*data[6]) + data[7];
            result.data[11] = (o.data[3]*data[8]) + (o.data[7]*data[9]) + (o.data[11]*data[10]) + data[11];

            return result;
        }
///<Matrix4MatrixMultiply

///>Matrix4VectorMultiply
        /**
         * Transform the given vector by this matrix.
         *
         * @param vector The vector to transform.
         */
        Vector3f operator*(const Vector3f&vector) const
        {
            return Vector3f(
                vector.X * data[0] + 
                vector.Y * data[1] + 
                vector.Z * data[2] + data[3],

                vector.X * data[4] + 
                vector.Y * data[5] + 
                vector.Z * data[6] + data[7],

                vector.X * data[8] + 
                vector.Y * data[9] + 
                vector.Z * data[10] + data[11]
            );
        }
///<Matrix4VectorMultiply

        /**
         * Transform the given vector by this matrix.
         *
         * @param vector The vector to transform.
         */
        Vector3f transform(const Vector3f &vector) const
        {
            return (*this) * vector;
        }

///>Matrix4Inverse
        /**
         * Returns the determinant of the matrix.
         */
        float getDeterminant() const;

        /**
         * Sets the matrix to be the inverse of the given matrix.
         * 
         * @param m The matrix to invert and use to set this.
         */
        void setInverse(const Matrix4 &m);

        /** Returns a new matrix containing the inverse of this matrix. */
        Matrix4 inverse() const
        {
            Matrix4 result;
            result.setInverse(*this);
            return result;
        }

        /**
         * Inverts the matrix.
         */
        void invert()
        {
            setInverse(*this);
        }
///<Matrix4Inverse

///>TransformDirection        
        /**
         * Transform the given direction vector by this matrix.
///<TransformDirection
         *
         * @note When a direction is converted between frames of
         * reference, there is no translation required.
         *
         * @param vector The vector to transform.
///>TransformDirection         
         */
        Vector3f transformDirection(const Vector3f &vector) const
        {
            return Vector3f(
                vector.X * data[0] + 
                vector.Y * data[1] + 
                vector.Z * data[2],

                vector.X * data[4] + 
                vector.Y * data[5] + 
                vector.Z * data[6],

                vector.X * data[8] + 
                vector.Y * data[9] + 
                vector.Z * data[10]
            );
        }

        /**
         * Transform the given direction vector by the 
         * transformational inverse of this matrix.
///<TransformDirection
         *
         * @note This function relies on the fact that the inverse of
         * a pure rotation matrix is its transpose. It separates the
         * translational and rotation components, transposes the 
         * rotation, and multiplies out. If the matrix is not a
         * scale and shear free transform matrix, then this function
         * will not give correct results. 
         *
         * @note When a direction is converted between frames of
         * reference, there is no translation required.
         *
         * @param vector The vector to transform.
///>TransformDirection
         */
        Vector3f transformInverseDirection(const Vector3f &vector) const
        {
            return Vector3f(
                vector.X * data[0] + 
                vector.Y * data[4] + 
                vector.Z * data[8],

                vector.X * data[1] + 
                vector.Y * data[5] + 
                vector.Z * data[9],

                vector.X * data[2] + 
                vector.Y * data[6] + 
                vector.Z * data[10]
            );
        }        
///<TransformDirection

///>TransformInverse
        /**
         * Transform the given vector by the transformational inverse
         * of this matrix.
///<TransformInverse         
         *
         * @note This function relies on the fact that the inverse of
         * a pure rotation matrix is its transpose. It separates the
         * translational and rotation components, transposes the 
         * rotation, and multiplies out. If the matrix is not a
         * scale and shear free transform matrix, then this function
         * will not give correct results.
         *
         * @param vector The vector to transform.
///>TransformInverse         
         */
        Vector3f transformInverse(const Vector3f &vector) const
        {
            Vector3f tmp = vector;
            tmp.X -= data[3];
            tmp.Y -= data[7];
            tmp.Z -= data[11];
            return Vector3f(
                tmp.X * data[0] + 
                tmp.Y * data[4] + 
                tmp.Z * data[8],

                tmp.X * data[1] + 
                tmp.Y * data[5] + 
                tmp.Z * data[9],

                tmp.X * data[2] + 
                tmp.Y * data[6] + 
                tmp.Z * data[10]
            );
        }
///<TransformInverse        

        /**
         * Gets a vector representing one axis (i.e. one column) in the matrix.
         *
         * @param i The row to return. Row 3 corresponds to the position
         * of the transform matrix.
         *
         * @return The vector.
         */
        Vector3f getAxisVector(int i) const
        {
            return Vector3f(data[i], data[i+4], data[i+8]);
        }

///>OrientationAndPosToMatrix4
        /**
         * Sets this matrix to be the rotation matrix corresponding to 
         * the given quaternion.
         */
        void setOrientationAndPos(const Quaternion &q, const Vector3f &pos)
        {
            data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
            data[1] = 2*q.i*q.j + 2*q.k*q.r;
            data[2] = 2*q.i*q.k - 2*q.j*q.r;
            data[3] = pos.X;

            data[4] = 2*q.i*q.j - 2*q.k*q.r;
            data[5] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
            data[6] = 2*q.j*q.k + 2*q.i*q.r;
            data[7] = pos.Y;

            data[8] = 2*q.i*q.k + 2*q.j*q.r;
            data[9] = 2*q.j*q.k - 2*q.i*q.r;
            data[10] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
            data[11] = pos.Z;
        }
///<OrientationAndPosToMatrix4

		/**
		 * Fills the given array with this transform matrix, so it is 
		 * usable as an open-gl transform matrix. OpenGL uses a column
		 * major format, so that the values are transposed as they are
		 * written.
		 */
		void fillGLArray(float array[16]) const
		{
			array[0] = (float)data[0];
			array[1] = (float)data[4];
			array[2] = (float)data[8];
			array[3] = (float)0;

			array[4] = (float)data[1];
			array[5] = (float)data[5];
			array[6] = (float)data[9];
			array[7] = (float)0;

			array[8] = (float)data[2];
			array[9] = (float)data[6];
			array[10] = (float)data[10];
			array[11] = (float)0;

			array[12] = (float)data[3];
			array[13] = (float)data[7];
			array[14] = (float)data[11];
			array[15] = (float)1;
		}
///>Matrix4;Matrix4Intro
    };
///<Matrix4;Matrix4Intro

///>Matrix3;Matrix3Intro
    /**
     * Holds an inertia tensor, consisting of a 3x3 row-major matrix.
     * This matrix is not padding to produce an aligned structure, since
     * it is most commonly used with a mass (single float) and two 
     * damping coefficients to make the 12-element characteristics array
     * of a rigid body.
     */
    class Matrix3
///<Matrix3;Matrix3Intro
    {
    public:
        /**
         * Holds the tensor matrix data in array form.
         */
        float data[9];
///<Matrix3Intro

///>Omit;Matrix3
        // ... Other Matrix3 code as before ...
        
///<Omit;Matrix3
        /**
         * Creates a new matrix.
         */
        Matrix3()
        {
            data[0] = data[1] = data[2] = data[3] = data[4] = data[5] =
                data[6] = data[7] = data[8] = 0;
        }

        /**
         * Creates a new matrix with the given three vectors making
         * up its columns.
         */
        Matrix3(const Vector3f &compOne, const Vector3f &compTwo,
            const Vector3f &compThree)
        {
            setComponents(compOne, compTwo, compThree);
        }

        /**
         * Creates a new matrix with explicit coefficients.
         */
        Matrix3(float c0, float c1, float c2, float c3, float c4, float c5, 
            float c6, float c7, float c8)
        {
            data[0] = c0; data[1] = c1; data[2] = c2;
            data[3] = c3; data[4] = c4; data[5] = c5;
            data[6] = c6; data[7] = c7; data[8] = c8;
        }

        /**
         * Sets the matrix to be a diagonal matrix with the given
         * values along the leading diagonal.
         */
        void setDiagonal(float a, float b, float c)
        {
            setInertiaTensorCoeffs(a, b, c);
        }

        /** 
         * Sets the value of the matrix from inertia tensor values.
         */
        void setInertiaTensorCoeffs(float ix, float iy, float iz,
            float ixy=0, float ixz=0, float iyz=0)
        {
            data[0] = ix;
            data[1] = data[3] = -ixy;
            data[2] = data[6] = -ixz;
            data[4] = iy;
            data[5] = data[7] = -iyz;
            data[8] = iz;
        }

        /**
         * Sets the value of the matrix as an inertia tensor of
         * a rectangular block aligned with the body's coordinate 
         * system with the given axis half-sizes and mass.
         */
        void setBlockInertiaTensor( Vector3f &halfSizes, float mass)
        {
            Vector3f squares = halfSizes.compProd(halfSizes);
            setInertiaTensorCoeffs(0.3f*mass*(squares.Y + squares.Z),
                0.3f*mass*(squares.X + squares.Z),
                0.3f*mass*(squares.X + squares.Y));
        }

///>Matrix3SetSkewSymmetric
        /**
         * Sets the matrix to be a skew symmetric matrix based on
         * the given vector. The skew symmetric matrix is the equivalent
         * of the vector product. So if a,b are vectors. a x b = A_s b
         * where A_s is the skew symmetric form of a.
         */
        void setSkewSymmetric(const Vector3f vector)
        {
            data[0] = data[4] = data[8] = 0;
            data[1] = -vector.Z;
            data[2] = vector.Y;
            data[3] = vector.Z;
            data[5] = -vector.X;
            data[6] = -vector.Y;
            data[7] = vector.X;
        }
///<Matrix3SetSkewSymmetric

///>Matrix3SetComponents
        /**
         * Sets the matrix values from the given three vector components.
         * These are arranged as the three columns of the vector.
         */
        void setComponents(const Vector3f &compOne, const Vector3f &compTwo,
            const Vector3f &compThree)
        {
            data[0] = compOne.X;
            data[1] = compTwo.X;
            data[2] = compThree.X;
            data[3] = compOne.Y;
            data[4] = compTwo.Y;
            data[5] = compThree.Y;
            data[6] = compOne.Z;
            data[7] = compTwo.Z;
            data[8] = compThree.Z;

        }
///<Matrix3SetComponents
        
///>Matrix3VectorMultiply
        /**
         * Transform the given vector by this matrix.
         *
         * @param vector The vector to transform.
         */
        Vector3f operator*(const Vector3f &vector) const
        {
            return Vector3f(
                vector.X * data[0] + vector.Y * data[1] + vector.Z * data[2],
                vector.X * data[3] + vector.Y * data[4] + vector.Z * data[5],
                vector.X * data[6] + vector.Y * data[7] + vector.Z * data[8]
            );
        }
///>Matrix3VectorMultiply

        /**
         * Transform the given vector by this matrix.
         *
         * @param vector The vector to transform.
         */
        Vector3f transform(const Vector3f &vector) const
        {
            return (*this) * vector;
        }

        /**
         * Transform the given vector by the transpose of this matrix.
         *
         * @param vector The vector to transform.
         */
        Vector3f transformTranspose(const Vector3f &vector) const
        {
            return Vector3f(
                vector.X * data[0] + vector.Y * data[3] + vector.Z * data[6],
                vector.X * data[1] + vector.Y * data[4] + vector.Z * data[7],
                vector.X * data[2] + vector.Y * data[5] + vector.Z * data[8]
            );
        }

        /**
         * Gets a vector representing one row in the matrix.
         *
         * @param i The row to return.
         */
        Vector3f getRowVector(int i) const
        {
            return Vector3f(data[i*3], data[i*3+1], data[i*3+2]);
        }

        /**
         * Gets a vector representing one axis (i.e. one column) in the matrix.
         *
         * @param i The row to return.
         *
         * @return The vector.
         */
        Vector3f getAxisVector(int i) const
        {
            return Vector3f(data[i], data[i+3], data[i+6]);
        }

///>Matrix3Inverse
        /**
         * Sets the matrix to be the inverse of the given matrix.
         * 
         * @param m The matrix to invert and use to set this.
         */
        void setInverse(const Matrix3 &m)
        {
            float t4 = m.data[0]*m.data[4];
            float t6 = m.data[0]*m.data[5];
            float t8 = m.data[1]*m.data[3];
            float t10 = m.data[2]*m.data[3];
            float t12 = m.data[1]*m.data[6];
            float t14 = m.data[2]*m.data[6];

            // Calculate the determinant
            float t16 = (t4*m.data[8] - t6*m.data[7] - t8*m.data[8]+
                        t10*m.data[7] + t12*m.data[5] - t14*m.data[4]);

            // Make sure the determinant is non-zero.
            if (t16 == (float)0.0f) return;
            float t17 = 1/t16;

            data[0] = (m.data[4]*m.data[8]-m.data[5]*m.data[7])*t17;
            data[1] = -(m.data[1]*m.data[8]-m.data[2]*m.data[7])*t17;
            data[2] = (m.data[1]*m.data[5]-m.data[2]*m.data[4])*t17;
            data[3] = -(m.data[3]*m.data[8]-m.data[5]*m.data[6])*t17;
            data[4] = (m.data[0]*m.data[8]-t14)*t17;
            data[5] = -(t6-t10)*t17;
            data[6] = (m.data[3]*m.data[7]-m.data[4]*m.data[6])*t17;
            data[7] = -(m.data[0]*m.data[7]-t12)*t17;
            data[8] = (t4-t8)*t17;
        }

        /** Returns a new matrix containing the inverse of this matrix. */
        Matrix3 inverse() const
        {
            Matrix3 result;
            result.setInverse(*this);
            return result;
        }

        /**
         * Inverts the matrix.
         */
        void invert()
        {
            setInverse(*this);
        }
///<Matrix3Inverse

///>Matrix3Transpose
        /**
         * Sets the matrix to be the transpose of the given matrix.
         * 
         * @param m The matrix to transpose and use to set this.
         */
        void setTranspose(const Matrix3 &m)
        {
            data[0] = m.data[0];
            data[1] = m.data[3];
            data[2] = m.data[6];
            data[3] = m.data[1];
            data[4] = m.data[4];
            data[5] = m.data[7];
            data[6] = m.data[2];
            data[7] = m.data[5];
            data[8] = m.data[8];
        }

        /** Returns a new matrix containing the transpose of this matrix. */
        Matrix3 transpose() const
        {
            Matrix3 result;
            result.setTranspose(*this);
            return result;
        }
///<Matrix3Transpose

///>Matrix3MatrixMultiply
        /** 
         * Returns a matrix which is this matrix multiplied by the given 
         * other matrix. 
         */
        Matrix3 operator*(const Matrix3 &o) const
        {
            return Matrix3(
                data[0]*o.data[0] + data[1]*o.data[3] + data[2]*o.data[6],
                data[0]*o.data[1] + data[1]*o.data[4] + data[2]*o.data[7],
                data[0]*o.data[2] + data[1]*o.data[5] + data[2]*o.data[8],

                data[3]*o.data[0] + data[4]*o.data[3] + data[5]*o.data[6],
                data[3]*o.data[1] + data[4]*o.data[4] + data[5]*o.data[7],
                data[3]*o.data[2] + data[4]*o.data[5] + data[5]*o.data[8],

                data[6]*o.data[0] + data[7]*o.data[3] + data[8]*o.data[6],
                data[6]*o.data[1] + data[7]*o.data[4] + data[8]*o.data[7],
                data[6]*o.data[2] + data[7]*o.data[5] + data[8]*o.data[8]
                );
        }
///<Matrix3MatrixMultiply

        /** 
         * Multiplies this matrix in place by the given other matrix.
         */
        void operator*=(const Matrix3 &o)
        {
            float t1;
            float t2;
            float t3;

            t1 = data[0]*o.data[0] + data[1]*o.data[3] + data[2]*o.data[6];
            t2 = data[0]*o.data[1] + data[1]*o.data[4] + data[2]*o.data[7];
            t3 = data[0]*o.data[2] + data[1]*o.data[5] + data[2]*o.data[8];
            data[0] = t1;
            data[1] = t2;
            data[2] = t3;

            t1 = data[3]*o.data[0] + data[4]*o.data[3] + data[5]*o.data[6];
            t2 = data[3]*o.data[1] + data[4]*o.data[4] + data[5]*o.data[7];
            t3 = data[3]*o.data[2] + data[4]*o.data[5] + data[5]*o.data[8];
            data[3] = t1;
            data[4] = t2;
            data[5] = t3;

            t1 = data[6]*o.data[0] + data[7]*o.data[3] + data[8]*o.data[6];
            t2 = data[6]*o.data[1] + data[7]*o.data[4] + data[8]*o.data[7];
            t3 = data[6]*o.data[2] + data[7]*o.data[5] + data[8]*o.data[8];
            data[6] = t1;
            data[7] = t2;
            data[8] = t3;
        }

        /** 
         * Multiplies this matrix in place by the given scalar.
         */
        void operator*=(const float scalar)
        {
            data[0] *= scalar; data[1] *= scalar; data[2] *= scalar;
            data[3] *= scalar; data[4] *= scalar; data[5] *= scalar;
            data[6] *= scalar; data[7] *= scalar; data[8] *= scalar;
        }

        /**
         * Does a component-wise addition of this matrix and the given
         * matrix.
         */
        void operator+=(const Matrix3 &o)
        {
            data[0] += o.data[0]; data[1] += o.data[1]; data[2] += o.data[2];
            data[3] += o.data[3]; data[4] += o.data[4]; data[5] += o.data[5];
            data[6] += o.data[6]; data[7] += o.data[7]; data[8] += o.data[8];
        }

///>OrientationToMatrix3
        /**
         * Sets this matrix to be the rotation matrix corresponding to 
         * the given quaternion.
         */
        void setOrientation(const Quaternion &q)
        {
            data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
            data[1] = 2*q.i*q.j + 2*q.k*q.r;
            data[2] = 2*q.i*q.k - 2*q.j*q.r;
            data[3] = 2*q.i*q.j - 2*q.k*q.r;
            data[4] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
            data[5] = 2*q.j*q.k + 2*q.i*q.r;
            data[6] = 2*q.i*q.k + 2*q.j*q.r;
            data[7] = 2*q.j*q.k - 2*q.i*q.r;
            data[8] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
        }

		/**
		 * Interpolates a couple of matrices.
		 */
		static Matrix3 linearInterpolate(const Matrix3& a, const Matrix3& b, float prop);
    };



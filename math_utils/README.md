# math_utils


This package contains a clooection of python scripts to support the implementation of the other packages on this stack.




## math_utils (python)

The code `math_utils.py` contains a list of simple mathemathical functions. In general, it includes algebraic operations, quaternion operations, spatial rotaion converssions and other simple functions.

It can be imported as:
```python
import math_utils.math_utils as MU
#or
from math_utils.math_utils import *
```

The available functions are listed below.



### Methods

#### `get_norm(arr):`

Compute the Euclidean norm of a vector.

Inputs:
- `arr`: python list with floats representing a vector<br/>

Returns:
- norm of the vector


#### `normalize(list_in)`:

Normalize a vector to the unit norm.

Inputs:
- `list_in`: python list with floats representing a vector

Returns:
- python list with floats representing a normalized vector


#### `quat_conj(q)`:

Return the conjugate of a quaternion.

Inputs:
- `q`: python list of floats representing a quatrnion. Sequency [w, x, y, z].

Returns:
- python list of floats representing the normalize quatrnion. Sequency [w, x, y, z].


#### `quat_mult(q1,q2)`:

Perform the quaternion multipliction.

Inputs:
- `q1`: python list of floats representing the first quatrnion. Sequency [w, x, y, z].
- `q2`: python list of floats representing the second quatrnion. Sequency [w, x, y, z].

Returns:
- python list of floats representing the quaternion product . Sequency [w, x, y, z].


#### `quat_apply_rot(q,u)`:

Apply the rotation represented by a quaternion to a vector.

Inputs:
- `q`: python list of floats representing quatrnion. Sequency [w, x, y, z].
- `v`: python list of representing the 3 dimensional vector.

Returns:
- vector `v` rotated by the quaternion `q`.



#### `qpos(q)`:

Force the quaternion to have a positve real part.

Inputs:
- `q`: python list of floats representing quatrnion. Sequency [w, x, y, z].

Returns:
- Equivalent quaternion with the positve real part.


#### `d_quat_from_omega(q,w)`:

Compute the derivative of a quaternion giev an angular speed.

Inputs:
- `q`: python list of floats representing quatrnion. Sequency [w, x, y, z].
- `w`: Angular velocity written on the (body/world) frame. ?????

Returns:
- Time Derivative of the quaternion `q`, given the (body/world) angular speed `w`. ?????


#### `mat_times_v(M,v)`:

Multiplication of a matrix and a vector.

Inputs:
- `M`: python list of lists representing a matrix.
- `w`: python list of floats representing a vector.

Returns:
- Linear algebraic product `M*v`


#### `mat_times_mat(M1,M2)`:

Multiplication of two matrices.

Inputs:
- `M1`: python list of lists representing the first matrix.
- `M2`: python list of lists representing the second matrix.

Returns:
- Linear algebraic product `M1*M2`


#### `mat_plus_mat(M1,M2)`:

Addition of two matrices.

Inputs:
- `M1`: python list of lists representing the first matrix.
- `M2`: python list of lists representing the second matrix.

Returns:
- Linear algebraic sum `M1+M2`


#### `mat_minus_mat(M1,M2)`:

Subtraction of two matrices.

Inputs:
- `M1`: python list of lists representing the first matrix.
- `M2`: python list of lists representing the second matrix.

Returns:
- Linear algebraic difference `M1-M2`


#### `vec_plus_vec(v1,v2)`:

Addition of two vectors.

Inputs:
- `v1`: python list representing the first vector.
- `v2`: python list representing the second vector.

Returns:
- Linear algebraic sum `v1+v2`


#### `vec_minus_vec(v1,v2)`:

Subtraction of two vectors.

Inputs:
- `v1`: python list representing the first vector.
- `v2`: python list representing the second vector.

Returns:
- Linear algebraic difference `v1-v2`


#### `mat_transpose(Min)`:

Transpose of a matrix.

Inputs:
- `Min`: python list of list representing a matrix.

Returns:
- Python list of list representing the transpose of the input matrix.


#### `mat_inv(M)`:

Inverse of a matrix.

Inputs:
- `M`: python list of list representing a matrix.

Returns:
- Python list of list representing the inverse of the input matrix.




#### `mat_identity(n)`:

Creates an identty matrix.

Inputs:
- `n`: integer representing the dimension of the matrix.

Returns:
- Identidy matrix with dimension `n`


#### `mat_zeros(n)`:

Creates an matrix of zeros.

Inputs:
- `n`: integer representing the dimension of the matrix.

Returns:
- Zeros matrix with dimension `n`


#### `eul2quat(a)`:

Convert Euler angles to quaternion.

Inputs:
- `a`: python list of floats with the roll, pitch and yaw angles in radians. Sequency ZYX in the body frame.

Returns:
- python list of floats representing quatrnion. Sequency [w, x, y, z].


#### `euler2rotm(rpy)`:

Convert Euler angles to rotation matrix.

Inputs:
- `a`: python list of floats with the roll, pitch and yaw angles in radians. Sequency ZYX in the body frame.

Returns:
- python list of lists representing a rotation matrix.


#### `quat2rotm(q)`:

Convert a quaternion to rotation matrix.

Inputs:
- `q`: python list of floats representing quatrnion. Sequency [w, x, y, z].

Returns:
- python list of lists representing a rotation matrix.



#### `quat2euler(q)`:

Convert a quaternion to Euler angles.

Inputs:
- `q`: python list of floats representing quatrnion. Sequency [w, x, y, z].

Returns:
- python list of floats with the roll, pitch and yaw angles in radians. Sequency ZYX in the body frame.


#### `rotm2quat(R)`:

Convert a rotation matrix to a quaternion.

Inputs:
- `R`: python list of lists representing a rtation matrix.

Returns:
- python list of floats representing quatrnion. Sequency [w, x, y, z].


#### `quat2axang(q)`:

Convert a quaternion to the axis/angle representation.

Inputs:
- `q`: python list of floats representing quatrnion. Sequency [w, x, y, z].

Returns:
- python list of floats representing the unit vector with the direction of rotation.
- float with the angle of rotatio in radians


#### `rotm2axang(R)`:

Convert a rotation matrix to the axis/angle representation.

Inputs:
- `R`: python list of lists representing a rtation matrix.

Returns:
- python list of floats representing the unit vector with the direction of rotation.
- float with the angle of rotatio in radians

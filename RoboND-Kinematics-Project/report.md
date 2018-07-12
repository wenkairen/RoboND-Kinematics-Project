## Project: Kinematics Pick & Place



[//]: # (Image References)

[image1]: ./misc_images/fk.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/3.jpg
[image4]: ./misc_images/4.jpg
[image5]: ./misc_images/5.jpg
[image5]: ./misc_images/6.jpg
### Kinematic Analysis

This is the diagram of the Kuka KR210 robot:

![alt text][image1]

### DH parameter Kuka KR210 robot

Links | i | alpha(i-1) | a(i-1) | d(i) | theta(i) |
:---: | :---: | :---: | :---: | :---: | :---: |
0->1 | 1 | 0 | 0 | 0.75 | q1 |
1->2 | 2 | -90 | 0.35 | 0 | -90+q2 |
2->3 | 3 | 0 |  | 1.25 | q3 |
3->4 | 4 | -90 | -0.05 | 1.5 | q4 |
4->5 | 5 | 90 | 0 | 0 | q5 |
5->6 | 6 | -90 | 0 | 0 | q6 |
6->EE | 7 | 0 | 0 | 0.303 | q7 |

Python code to represent DH parameters table is:

```python
		DH_table = {
				alpha0:      0, a0:      0, d1:  0.75, q1:        q1,
				alpha1: -pi/2., a1:   0.35, d2:     0, q2: -pi/2.+q2,
				alpha2:      0, a2:   1.25, d3:     0, q3:        q3,
				alpha3: -pi/2., a3: -0.054, d4:   1.5, q4:        q4,
				alpha4:  pi/2., a4:      0, d5:     0, q5:        q5,
				alpha5: -pi/2., a5:      0, d6:     0, q6:        q6,
				alpha6:      0, a6:      0, d7: 0.303, q7:         0}
```

### Creating the individual transformation matrices about each joint:

Python code for a function that will return the individual frame transformation matrix is as following:

```python
# Function to return homogeneous transform matrix
def TF_Matrix(alpha, a, d, q):
     TF = Matrix([
			  [            cos(q),           -sin(q),           0,             a],
			  [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
			  [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
			  [                 0,                 0,           0,             1]])
	return TF
```
Then using the following code to substitute the DH parameters into the transformation matrix: 

```python

T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_table)
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_table)
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_table)
3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_table)
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_table)
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_table)
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_table)

```

Then we mutiply each transform matrix to the get the forward kinenmatic matrix :

```python
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

```

### The wrist center WC is composed of  three joint axises , which allows us to kinematically decouple the IK problem into position and orentation

By receiving from the request, 

```python
# Extract end-effector position and orientation from request
# px,py,pz = end-effector positio.
px = req.poses[x].position.x
py = req.poses[x].position.y
pz = req.poses[x].position.z
			
# store EE position in a matrix
EE = Matrix([[px],
	     [py],
	     [pz]])

(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
			req.poses[x].orientation.x, req.poses[x].orientation.y,
			req.poses[x].orientation.z, req.poses[x].orientation.w])
```

#### The end-effector can also be represented by roll pitch yall angle:
The Rotation matrix error is becase the notation difference between the DH table and the urdf.

```python
		r,p,y = symbols('r p y')
		# Roll
		ROT_x = Matrix([[       1,       0,       0],
						[       0,  cos(r), -sin(r)],
						[       0,  sin(r),  cos(r)]])

		# Pitch
		ROT_y = Matrix([[  cos(p),       0,  sin(p)],
						[       0,       1,       0],
						[ -sin(p),       0,  cos(p)]])

		# Yaw
		ROT_z = Matrix([[  cos(y), -sin(y),       0],
						[  sin(y),  cos(y),       0],
						[       0,       0,       1]])

		ROT_EE = ROT_z * ROT_y * ROT_x

		Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
		ROT_EE = ROT_EE * Rot_Error
```
After we have Ending effatcor posion and distance of the wrist joint to gripper, also alone with the rotation matrix, we can get the wrist postion.

```python
    # Calculate Wrest Center
    WC = EE - (0.303) * ROT_EE[:,2]
```

WC is now having position of wrist center (Wx, Wy, Wz).

To find 𝜃1,  then we use Wz to project onto the ground plane, we get theta1:

```python
    theta1 = atan2(WC[1],WC[0])
```
Using trigonometry,  we can calculate 𝜃2 and 𝜃3, a is the right side of the triangle, c is the left and b is the bottom.

shown in below image:

![alt text][image2]


```python
			side_a = 1.501
			side_b = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
			side_c = 1.25

			angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a)/ (2*side_b*side_c))
			angle_b = acos((side_a*side_a + side_c*side_c - side_b*side_b)/ (2*side_a*side_c))
			angle_c = acos((side_a*side_a + side_b*side_b - side_c*side_c)/ (2*side_b*side_a))

			theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35)
			theta3 = pi/2 - (angle_b + 0.036)
```
we can move on to  𝜃4, 𝜃5 and 𝜃6.

due to the relation of the matrix 
R0_6 = R_EE
so R0_3 * R3_6 = R_EE

We can substitute the values we calculated for 𝜃1, 𝜃2 and 𝜃3 to get R0_3, then, we get inverse of R0_3, thus,

R3_6 = inv(R0_3) * R_EE

```python
    # Extract rotation matrix R0_3 from transformation matrix T0_3 the substitute angles q1-3
    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3:theta3})

    # Get rotation matrix R3_6 from (inverse of R0_3 * R_EE)
    R3_6 = R0_3.inv(method="LU") * ROT_EE
```
thus, we can get the rest angles:

```python
			# Euler angles from rotation matrix
			theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
			# select best solution based on theta5
			if (theta5 > pi) :
				theta4 = atan2(-R3_6[2,2], R3_6[0,2])
				theta6 = atan2(R3_6[1,1],-R3_6[1,0])
			else:
				theta4 = atan2(R3_6[2,2], -R3_6[0,2])
				theta6 = atan2(-R3_6[1,1],R3_6[1,0])
```

**After finish setting up the code, I use the IK_debug.py to test the error and the time for code.**

### Result 

the final result of the testing IK_server.py is shown below:

![alt text][image6]

![alt text][image5]

![alt text][image4]

![alt text][image3]

I have tested 8/10 for most of the tests, but this is still lots of improvement in this project,
1. the timing issue of the project, reponses didn't give enough time for the gripper to work with
2. the Gazebo can eassilly crush for some reason, maybe running too long?
3. this method using too much geometry of calculation, which I think is not efficient and can't be generalized for other robot arms, we should use numerically next time, which is more reasonable.


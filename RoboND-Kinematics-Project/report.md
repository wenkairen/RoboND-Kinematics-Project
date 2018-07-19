## Project: Kinematics Pick & Place



[//]: # (Image References)

[image1]: ./misc_images/fk.png
[image2]: ./misc_images/misc3.png

[image6]: ./misc_images/dh-transform-matrix.png

[image7]: ./misc_images/7.png

[image8]: ./misc_images/urdf.png

[image9]: ./misc_images/8.png
### Kinematic Analysis

This is the diagram of the Kuka KR210 robot:

![alt text][image1]

The following images for reference.   
![alt text][image7]

the link frames(coordinate systems) choosen according to Modified DH convention. `O(i)` is the origin for link i frame, and `X(i)`, `Z(i)` are the X and Z axis correspondingly, and Z represents the axis of rotation(translation in case of prismatic joints). Since we are using a right handed coordinate system, `Y(i)` can be calculated accordingly.   


![alt text][image8]

the reference frames as specified in the URDF file. Here, `Ojointi` represents ith frame origin(also represented by a black triangle). It also shows various distance between these joint positions as specified in URDF. We will be refering to these values in our description for DH parameter table.

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

Here,   
`alpha(i-1)` : Angle between Z(i-1) and Z(i) measured along X(i-1)   
`a(i-1)` : Link length, distance between Z(i-1) and Z(i), measured along X(i-1)   
`d(i)`  : Link offset, distance between X(i-1) and X(i), measured along Z(i-1), variable in prismatic joints(there are no prismatic joints in the given problem)   
`q(i)` : Joint angle, Angle between X(i-1), X(i) measured along Z(i), variable in revolute joints.   
Also,   
`Link 0` represents the base link, and   
`Link 7` represents the gripper link, which is fixed. It contains left and right grippers(not controlled by IK code.)   

**Link 1 :** `Z0`(0 0 1) is *collinear* to `Z1`(0 0 1), `alpha0 = 0`, `a0 = 0`, `d1 = 0.33(joint1.z) + 0.42(joint2.z) = 0.75`, and `q1` is *unknown*.   

**Link 2 :** `Z1`(0 0 1) is *perpendicular* to `Z2`(0 1 0), so, `alpha1 = -pi/2`, `a1 = 0.35(joint2.x)`, and `d2 = 0` since `X1` intersects `X2` at `O2`. Also, we can see that when joint2 is in *zero* configuration, there is an offset of `-pi/2` from `X1` to `X2`, measured along `Z2`. So, we also need to substitute `q2` with `q2 - pi/2` in the parameter table.   

**Link 3 :**, since `Z2`(0 1 0) is *parallel* to `Z3`(0 1 0), `alpha2 = 0`, `a2 = 1.25(joint3.z)` along `X2`. Also, `X2` and `X3` are collinear, so `d3 = 0`.   

**Link 4 :**, `Z3`(0 1 0) and `Z4`(1 0 0) are *perpendicular*, so `alpha3 = -pi/2` and `a3 = -0.054(joint4.z)`, and `d4 = 0.96(joint4.x) + 0.54(joint5.x) = 1.50`.   
*Note:* We have choosen O4, O5 and O6 to be co-incident with the Wrist Center(WC). This helps in separating the IK problem into computation of the Wrist Center and then Wrist Orientation.   

**Link 5 :**, `Z4`(1 0 0) and `Z5`(0 1 0) are *perpendicular* and *intersect* at `O5`, so `alpha4 = pi/2` and `a4 = 0`. Also, `d5 = 0`, since `X4` and `X4` are *collinear*.   

**Link 6 :**, `Z5`(0 1 0) and `Z6`(1, 0, 0) are *perpendicular* and *intersect* at `O5`, so `alpha5 = -pi/2` and `a5 = 0`, `d6 = 0`, since `X5` and `X6` are *collinear*.   

**Link 7(Gripper Link) :**, this is a fixed link, with a translation along `Z6`. So, `Z6` and `Zg` are *collinear*, so `alpha6 = 0`, `a6 = 0` and `d6 = 0.193(joint6.x) + 0.11(gripper_joint.x)`. Also, since this is fixed(w.r.t link 6), `q7 = 0`.    


*Modified DH convention axes assignment and parameters.

From the above image it is clear thet the total transform between `link(i-1)` and `link(i)` can be thought of as a *rotation* by `alpha(i-1)` along `X(i-1)`, *translation* by `a(i-1)` along `X(i-1)`, *rotation* by `q(i)` along `Z(i)`, and finally *translation* by `d(i)` along `Z(i)`. 



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


![alt text][image6]
```
T0_1 = Matrix([[cos(q1), -sin(q1), 0, a0],
               [sin(q1) * cos(alpha0), cos(q1) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d1],
               [sin(q1) * sin(alpha0), cos(q1) * sin(alpha0), cos(alpha0), cos(alpha0) * d1],
               [0, 0, 0, 1]])

T1_2 = Matrix([[cos(q2), -sin(q2), 0, a1],
               [sin(q2) * cos(alpha1), cos(q2) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d2],
               [sin(q2) * sin(alpha1), cos(q2) * sin(alpha1), cos(alpha1), cos(alpha1) * d2],
               [0, 0, 0, 1]])

T2_3 = Matrix([[cos(q3), -sin(q3), 0, a2],
               [sin(q3) * cos(alpha2), cos(q3) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d3],
               [sin(q3) * sin(alpha2), cos(q3) * sin(alpha2), cos(alpha2), cos(alpha2) * d3],
               [0, 0, 0, 1]])

T3_4 = Matrix([[cos(q4), -sin(q4), 0, a3],
               [sin(q4) * cos(alpha3), cos(q4) * cos(alpha3), -sin(alpha3), -sin(alpha3) * d4],
               [sin(q4) * sin(alpha3), cos(q4) * sin(alpha3), cos(alpha3), cos(alpha3) * d4],
               [0, 0, 0, 1]])

T4_5 = Matrix([[cos(q5), -sin(q5), 0, a4],
               [sin(q5) * cos(alpha4), cos(q5) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d5],
               [sin(q5) * sin(alpha4), cos(q5) * sin(alpha4), cos(alpha4), cos(alpha4) * d5],
               [0, 0, 0, 1]])

T5_6 = Matrix([[cos(q6), -sin(q6), 0, a5],
               [sin(q6) * cos(alpha5), cos(q6) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d6],
               [sin(q6) * sin(alpha5), cos(q6) * sin(alpha5), cos(alpha5), cos(alpha5) * d6],
               [0, 0, 0, 1]])

T6_EE = Matrix([[cos(q7), -sin(q7), 0, a6],
                [sin(q7) * cos(alpha6), cos(q7) * cos(alpha6), -sin(alpha6), -sin(alpha6) * d7],
                [sin(q7) * sin(alpha6), cos(q7) * sin(alpha6), cos(alpha6), cos(alpha6) * d7],
                [0, 0, 0, 1]])
```

Of course, the total homogeneous transform between base and gripper is the product of the matrices above, in the order of the joints:

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

###### To find 𝜃1,  then we use Wz to project onto the ground plane, we get theta1:

```python
    theta1 = atan2(WC[1],WC[0])
```


###### q2, q3

Using trigonometry,  we can calculate 𝜃2 and 𝜃3, a is the right side of the triangle, c is the left and b is the bottom.

shown in below image:

![alt text][image2]

For the second and the third angles, there were also multiple solutions -- I could choose between two positions for the third joint: above the WC or below the WC. I decided to go with the above one, because I was worried that the arm might hit the floor when reaching for objects on the lowest shelf.

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
![alt text][image9]

Find q4, q5, q6.
```
q4 = atan2(r33, -r13)
q5 = atan2(sqrt(r21**2 + r22**2), r23)
q6 = atan2(-r22, r21)

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



I have tested 8/10 for most of the tests, but this is still lots of improvement in this project,
1. the timing issue of the project, reponses didn't give enough time for the gripper to work with
2. the Gazebo can eassilly crush for some reason, maybe running too long?
3. this method using too much geometry of calculation, which I think is not efficient and can't be generalized for other robot arms, we should use numerically next time, which is more reasonable.


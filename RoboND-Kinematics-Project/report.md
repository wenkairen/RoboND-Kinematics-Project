## Project: Kinematics Pick & Place



[//]: # (Image References)

[image1]: ./misc_images/fk.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

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

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

Since the last three joints in KUKA KR210 robot (Joint_4, Joint_5, and Joint_6) are revolute and their joint axes intersect at a single point (Joint_5), we have a case of spherical wrist with joint_5 being the common intersection point; the wrist center (**WC**). This allows us to kinematically decouple the IK problem into **Inverse Position** and **Inverse Orientation** problems.

### Inverse Position

First step is to get the end-effector position(**Px, Py, Pz**) and orientation (**Roll, Pitch, Yaw**) from the test cases data class as shown in below code:

```python
    # Requested end-effector (EE) position
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z
    
    # store EE position in a matrix
    EE = Matrix([[px],
                 [py],
                 [pz]])
    
    # Requested end-effector (EE) orientation
    (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x,
         req.poses[x].orientation.y,
         req.poses[x].orientation.z,
         req.poses[x].orientation.w])
```

We will need rotation matrix for the end-effector:

**R_rpy = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll)**

and orientation difference correction matrix (Rot_corr) as earlier discussed in FK section.

**R_EE = R_rpy * R_corr**

We substitute the obtained roll, pitch and yaw in the final rotation matrix. Python Code is as following:

```python
 # Find EE rotation matrix RPY (Roll, Pitch, Yaw)
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

    # Correction Needed to Account for Orientation Difference Between
    # Definition of Gripper Link_G in URDF versus DH Convention

    ROT_corr = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
    
    ROT_EE = ROT_EE * ROT_corr
    ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
```

The obtained matrix will be the rotation part of the full homogeneous transform matrix as yellow highlighted in the following:

<p align="center"> <img src="./misc_images/homo-xform-2.png"> </p>

<p align="center"> <img src="./misc_images/R_EE.png"> </p>

where **l**, **m** and **n** are orthonormal vectors representing the end-effector orientation along X, Y, Z axes of the local coordinate frame.

Since **n** is the vector along the **z-axis** of the **gripper_link**, we can say the following:

<p align="center"> <img src="./misc_images/ik_equ.png"> </p>

Where,

**Px, Py, Pz** = end-effector positions obtained from test case data

**Xwc, Ywc, Zwc** = wrist center positions that we are trying to find.

**d6** = link_6 length obtained from DH table (d6=0)

**d7** = end-effector length obtained from DH table (d7=0.303)

The same equation in vectorized version (d is the displacement):

<p align="center"> <img src="./misc_images/WC.png"> </p>

In Python code:

```python
    # Calculate Wrest Center
    WC = EE - (0.303) * ROT_EE[:,2]
```
WC is now having position of wrist center (Wx, Wy, Wz).

To find 𝜃1, we need to project Wz onto the ground plane Thus,

**Theta1=atan2(Wy,Wx)**

```python
    # Calculate theat1
    theta1 = atan2(WC[1],WC[0])
```
Using trigonometry, we can calculate **𝜃2 and 𝜃3**. 

We have a triangle (the green color in below figure) with two sides known to us (**A** = d4 = 1.5) and (**C** = a2 = 1.25), the 3rd side (**B**) can be calculated as following:

<p align="center"> <img src="./misc_images/B.png"> </p>

Below is the same in Python code:

```python
    #SSS triangle for theta2 and theta3
    A = 1.501
    C = 1.25
    B = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
``` 
Now since we have all three sides of the triangle known to us we can calculate all of the three inner angles of the triangle from the known three sides Using trigonometry (specifically the **Cosine Laws** SSS type).

<p align="center"> <img src="./misc_images/coslaw.png"> </p>

The same in Python code:

 ```python
    a = acos((B*B + C*C - A*A) / (2*B*C))
    b = acos((A*A + C*C - B*B) / (2*A*C))
    c = acos((A*A + B*B - C*C) / (2*A*B))
```
Finally we calculate **𝜃2** and **𝜃3**

```python
    theta2 = pi/2 - a - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35)
    theta3 = pi/2 - (b+0.036) # 0.036 accounts for sag in link4 of -0.054m
```
<p align="center"> <img src="./misc_images/ik_analysis.jpg"> </p>

### Inverse Orientation

For the **Inverse Orientation** problem, we need to find values of the final three joint variables **𝜃4, 𝜃5 and 𝜃6**.

Using the individual DH transforms we can obtain the resultant transform and hence resultant rotation by:

**R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6**

Since the overall RPY (Roll Pitch Yaw) rotation between base_link and gripper_link must be equal to the product of individual rotations between respective links, following holds true:

**R0_6 = R_EE**

where,

**R_EE** = Homogeneous RPY rotation between base_link and gripper_link as calculated above.

We can substitute the values we calculated for **𝜃1, 𝜃2 and 𝜃3**. in their respective individual rotation matrices and pre-multiply both sides of the above equation by **inv(R0_3)** which leads to:

**R3_6 = inv(R0_3) * R_EE**

<p align="center"> <img src="./misc_images/R3_6.png"> </p>

The resultant matrix on the RHS (Right Hand Side of the equation) does not have any variables after substituting the joint angle values, and hence comparing LHS (Left Hand Side of the equation) with RHS will result in equations for **𝜃4, 𝜃5 and 𝜃6**.

```python
    # Extract rotation matrix R0_3 from transformation matrix T0_3 the substitute angles q1-3
    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3:theta3})

    # Get rotation matrix R3_6 from (inverse of R0_3 * R_EE)
    R3_6 = R0_3.inv(method="LU") * ROT_EE
```

I have added if/else to select the best solution for **𝜃4, 𝜃5 and 𝜃6**.

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

Also I have added to the forward kinematics code to help in checking for errors.

```python
FK = T0_7.evalf(subs={q1:theta1,q2:theta2,q3:theta3,q4:theta4,q5:theta5,q6:theta6})
```

The rest of the code will utilize wrist center position **WC** and the **thetas** to calculate the corresponding errors. Using these error values as a basis, We can gauge how well our current IK performs.

I have added one line of code to print out the test case number. rest of the code is as provided.

```python
    # Print test case number
    print ("Using Test Case Number %d" %test_case_number)
```

The output of all 3 provided test cases are as following:

### Test Case 1 output:

```
Total run time to calculate joint angles from pose is 0.6331 seconds
Using Test Case Number 1

Wrist error for x position is: 0.00000046
Wrist error for y position is: 0.00000032
Wrist error for z position is: 0.00000545
Overall wrist offset is: 0.00000548 units

Theta 1 error is: 0.00093770
Theta 2 error is: 0.00181024
Theta 3 error is: 0.00205031
Theta 4 error is: 0.00172067
Theta 5 error is: 0.00197873
Theta 6 error is: 0.00251871

**These theta errors may not be a correct representation of your code, due to the fact            
that the arm can have multiple positions. It is best to add your forward kinematics to            
confirm whether your code is working or not**
 

End effector error for x position is: 0.00002010
End effector error for y position is: 0.00001531
End effector error for z position is: 0.00002660
Overall end effector offset is: 0.00003668 units 
```


### Test Case 2 output:

```
Total run time to calculate joint angles from pose is 0.6635 seconds
Using Test Case Number 2

Wrist error for x position is: 0.00002426
Wrist error for y position is: 0.00000562
Wrist error for z position is: 0.00006521
Overall wrist offset is: 0.00006980 units

Theta 1 error is: 3.14309971
Theta 2 error is: 0.27930449
Theta 3 error is: 1.86835102
Theta 4 error is: 3.08639294
Theta 5 error is: 0.06340564
Theta 6 error is: 6.13524247

**These theta errors may not be a correct representation of your code, due to the fact            
that the arm can have multiple positions. It is best to add your forward kinematics to            
confirm whether your code is working or not**
 

End effector error for x position is: 0.00002566
End effector error for y position is: 0.00002581
End effector error for z position is: 0.00000461
Overall end effector offset is: 0.00003668 units 
```

### Test Case 3 output:

```
Total run time to calculate joint angles from pose is 0.6569 seconds
Using Test Case Number 3

Wrist error for x position is: 0.00000503
Wrist error for y position is: 0.00000512
Wrist error for z position is: 0.00000585
Overall wrist offset is: 0.00000926 units

Theta 1 error is: 0.00136747
Theta 2 error is: 0.00325738
Theta 3 error is: 0.00339563
Theta 4 error is: 6.28212730
Theta 5 error is: 0.00284405
Theta 6 error is: 6.28223850

**These theta errors may not be a correct representation of your code, due to the fact            
that the arm can have multiple positions. It is best to add your forward kinematics to            
confirm whether your code is working or not**
 

End effector error for x position is: 0.00000069
End effector error for y position is: 0.00000011
End effector error for z position is: 0.00003668
Overall end effector offset is: 0.00003668 units 
```

[Python code for forward kinematics test `IK_debug.py` is located on this link](./src/IK_debug.py)


And just for fun, another example image:
![alt text][image3]



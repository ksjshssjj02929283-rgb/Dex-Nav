import math

qx = 0.00939883954913446
qy = 0.00504051010240261
qz = -0.22737747513412024
qw = 0.9737482933872952

# Conversion to Euler angles
phi = math.atan2(2 * (qw*qx + qy*qz), 1 - 2 * (qx**2 + qy**2))
theta = math.asin(2 * (qw*qy - qz*qx))
psi = math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy**2 + qz**2))

# Convert from radians to degrees
roll = math.degrees(phi)
pitch = math.degrees(theta)
yaw = math.degrees(psi)

result = [roll,pitch,yaw]

print(result)
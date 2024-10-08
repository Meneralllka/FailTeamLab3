import math

def inverse_kinematics(x, y, theta, l1, l2, l3):

  # Calculate wrist position
  xw = x - l3 * math.cos(theta)
  yw = y - l3 * math.sin(theta)

  # Calculate elbow joint angle
  theta2 = math.acos((xw**2 + yw**2 - l1**2 - l2**2) / (2 * l1 * l2))

  # Calculate shoulder joint angle
  theta1 = math.atan2(yw, xw) - math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))

  # Calculate wrist joint angle
  theta3 = theta - theta1 - theta2

  return theta1, theta2, theta3
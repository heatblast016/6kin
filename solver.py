import numpy as np
import math
import transformations as tfm


class solver:
    def __init__(self, l1: float, l2: float, l3: float, l4: float):
        '''Initializes class, storing variables representing joint lengths'''
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4

    def solve_forward(
            self,
            q1: float,
            q2: float,
            q3: float,
            q4: float,
            q5: float,
            q6: float):
        '''Returns end effector and simplified link positions for all links'''
        # Calculates first link transform
        disp1 = np.array([0, 0, self.l1])
        r1 = tfm.rotation_z(q1)
        t1 = tfm.generate_transform(r1, disp1)

        # Calculates second link transform
        r2 = tfm.rotation_y(q2)
        t2 = np.matmul(t1, tfm.generate_transform(r2, np.array([0, 0, 0])))

        # Calculates third link transform

        r3 = tfm.rotation_y(q3)
        t3 = np.matmul(t2, tfm.generate_transform(
            r3, np.array([self.l2, 0, 0])))

        # Calculates fourth link transform
        r4 = tfm.rotation_x(q4)
        t4 = np.matmul(t3, tfm.generate_transform(
            r4, np.array([self.l3, 0, 0])))

        # Calculates fifth link transform
        r5 = tfm.rotation_y(q5)
        t5 = np.matmul(t4, tfm.generate_transform(r5, np.array([0, 0, 0])))

        # Calculates sixth link transform
        r6 = tfm.rotation_x(q6)
        t6 = np.matmul(t5, tfm.generate_transform(r6, np.array([0, 0, 0])))

        # Gets position of each joint ("Shoulder", "Elbow", "Wrist", and End
        # effector position)
        x1 = np.matmul(t1, np.array([0, 0, 0, 1]))
        x2 = np.matmul(t3, np.array([0, 0, 0, 1]))
        x3 = np.matmul(t4, np.array([0, 0, 0, 1]))
        x4 = np.matmul(t6, np.array([self.l4, 0, 0, 1]))
        return [x1, x2, x3, x4]

    def solve_inverse_2dof(self, x, z):
        """Solves 2 degree of freedom inverse kinematics as a utility function for 3d inverse kinematics"""
        # Calculates cosine of joint 2 using lengths
        c2 = (x**2 + z**2 - self.l2**2 - self.l3**2) / (2 * self.l3 * self.l2)

        # Checks edge cases and returns special outputs
        if abs(c2) > 1:
            return []
        elif c2 == 1:
            return [(math.atan2(z, x), 0)]
        elif c2 == -1 and (x**2 + z**2) == 0:
            return [(0, math.pi)]
        else:
            # Solves the general case rotation
            q21 = math.acos(c2)
            q22 = -math.acos(c2)
            solutions = []
            solutions2 = []
            if q21 == q22:
                solutions2.append(q21)
            else:
                solutions2.append(q21)
                solutions2.append(q22)
            theta = math.atan2(z, x)
            for q2 in solutions2:
                q1 = theta - \
                    math.atan2(self.l3 * math.sin(q2), self.l2 + self.l3 * math.cos(q2))
                solutions.append(np.array([q1, q2]))
            return solutions

    def solve_inverse_3dof(self, x, y, z):
        """Solves 3 degrees-of-freedom inverse kinematics problems"""
        solutions = []
        baserotations = [math.atan2(y, x), math.atan2(y, x) + math.pi]
        for i, q1 in enumerate(baserotations):
            sols = self.solve_inverse_2dof(
                (-1)**i * math.sqrt(x**2 + y**2), -1 * z + self.l1)
            for sol in sols:
                solutions.append(np.insert(sol, 0, q1))
        return solutions

    def solve_inverse(self, x, y, z, roll, pitch, yaw):
        """Solves inverse kinematics analytically for a 6-dof arm"""
        solutions = []
        fingertip = np.array([x, y, z])
        end_rotation = np.matmul(
            np.matmul(
                tfm.rotation_z(yaw),
                tfm.rotation_y(pitch)),
            tfm.rotation_x(roll))

        end_offset = np.array([self.l4, 0, 0])
        wrist_pos = fingertip - np.matmul(end_rotation, end_offset)
        sols_wrist = self.solve_inverse_3dof(
            wrist_pos[0], wrist_pos[1], wrist_pos[2])
        for i in sols_wrist:
            # Get R3
            r3 = np.matmul(
                np.matmul(
                    tfm.rotation_z(
                        i[0]), tfm.rotation_y(
                        i[1])), tfm.rotation_y(
                    i[2]))
            xyx = np.matmul(np.linalg.inv(r3), end_rotation)
            q5 = [math.acos(xyx[0, 0]), -1 * math.acos(xyx[0, 0])]
            for q in q5:
                if xyx[0, 0] == 1:
                    # Singularity
                    q4 = 0
                    q6 = math.atan2(xyx[2, 1], xyx[1, 1])
                    solutions.append([i[0], i[1], i[2], q4, q, q6])
                else:
                    sinq = math.sin(q)
                    q4 = math.atan2(xyx[1, 0] / sinq, -1 * xyx[2, 0] / sinq)
                    q6 = math.atan2(xyx[0, 1] / sinq, xyx[0, 2] / sinq)
                    solutions.append([i[0], i[1], i[2], q4, q, q6])
        return solutions

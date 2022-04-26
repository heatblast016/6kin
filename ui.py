import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from solver import solver


class ui():
    def __init__(self, config):
        """Initializes User Interface Object"""
        l1 = 0.0
        l2 = 0.0
        l3 = 0.0
        l4 = 0.0
        with open(config, encoding='utf-8') as f:
            # Reads joint lengths from config file and instantiates solver
            l1 = float(f.readline())
            l2 = float(f.readline())
            l3 = float(f.readline())
            l4 = float(f.readline())
            self.solver = solver(l1, l2, l3, l4)

    def run(self):
        """Runs program"""
        q1 = float(input("Please enter the value of joint 1 in radians: "))
        q2 = float(input("Please enter the value of joint 2 in radians: "))
        q3 = float(input("Please enter the value of joint 3 in radians: "))
        q4 = float(input("Please enter the value of joint 4 in radians: "))
        q5 = float(input("Please enter the value of joint 5 in radians: "))
        q6 = float(input("Please enter the value of joint 6 in radians: "))
        coords = self.solver.solve_forward(q1, q2, q3, q4, q5, q6)
        self.plot(coords)

    def plot(self, coords, labels):
        '''Plots arm as a series of lines'''
        plt.style.use('dark_background')
        fig = plt.figure(figsize=plt.figaspect(0.5))
        fig.suptitle("6Kin Inverse Kinematics Solutions")
        for j, coord in enumerate(coords):
            ax = fig.add_subplot(2, 4, j + 1, projection='3d')
            ax.set_title("Solution " + str(j), fontsize=12)
            print(
                "Joint Values (q1,q2,q3,q4,q5,q6) for Solution " +
                str(j) +
                ": " +
                labels[j])
            ax.set_xlim3d(-4, 4)
            ax.set_ylim3d(-4, 4)
            ax.set_zlim3d(0, 4)
            for i in range(0, 3):
                start = coord[i]
                end = coord[i + 1]
                x, y, z = [
                    start[0], end[0]], [
                    start[1], end[1]], [
                    start[2], end[2]]

                if(i < 2):
                    ax.scatter(x, y, z, c='#00ffa3', s=50)
                    ax.plot(x, y, z, color='#000000', linewidth=2)
                else:
                    ax.scatter(x, y, z, c='#00ffa3', s=50)
                    ax.plot(x, y, z, color='#00ffa3', linewidth=2)

            start = [0, 0, 0]
            end = coord[0]
            x, y, z = [
                start[0], end[0]], [
                start[1], end[1]], [
                start[2], end[2]]
            ax.scatter(x, y, z, c='#00ffa3', s=50)
            ax.plot(x, y, z, color='#000000', linewidth=2)
        plt.show()

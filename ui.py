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

        #Gets user inputs
        x = float(input("Please enter your desired x-position: "))
        y = float(input("Please enter your desired y-position: "))
        z = float(input("Please enter your desired z-poisition: "))
        roll = float(input("Please enter the roll of your desired orientation in radians: "))
        pitch = float(input("Please enter the pitch of your desired orientation in radians: "))
        yaw = float(input("Please enter the yaw of your desired orientation in radians: "))

        #Solves Inverse Kinematics
        lcoords = self.solver.solve_inverse(x, y, z, roll, pitch, yaw)
        
        #Checks that a solution exists
        if len(lcoords) == 0:
            print("The manipulator cannot reach the desired position")

        #Plots solution
        else:
            plotting = []
            labels = []
            for coords in lcoords:
                q1 = coords[0]
                q2 = coords[1]
                q3 = coords[2]
                q4 = coords[3]
                q5 = coords[4]
                q6 = coords[5]
                labels.append(str((round(q1,2),round(q2,2),round(q3,2),round(q4,2),round(q5,2),round(q6,2))))
                plotting.append(self.solver.solve_forward(q1,q2,q3,q4,q5,q6))
            self.plot(plotting,labels)

    def plot(self, coords, labels):
        '''Plots arm as a series of lines'''

        #Plot options
        plt.style.use('dark_background')
        fig = plt.figure(figsize=plt.figaspect(0.5))
        fig.suptitle("6Kin Inverse Kinematics Solutions", color="#00ffa3")


        #Plots each coordinate and draws lines between them
        for j, coord in enumerate(coords):
            ax = fig.add_subplot(2, 4, j + 1, projection='3d')
            ax.set_title("Solution " + str(j), fontsize=12,color="#00ffa3")
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
                #Plots tool/end effector in a different color
                if(i < 2):
                    ax.scatter(x, y, z, c='#00ffa3', s=50)
                    ax.plot(x, y, z, color='#000000', linewidth=2)
                else:
                    ax.scatter(x, y, z, c='#00ffa3', s=50)
                    ax.plot(x, y, z, color='#00ffa3', linewidth=2)
            #Plots base of arm
            start = [0, 0, 0]
            end = coord[0]
            x, y, z = [
                start[0], end[0]], [
                start[1], end[1]], [
                start[2], end[2]]
            ax.scatter(x, y, z, c='#00ffa3', s=50)
            ax.plot(x, y, z, color='#000000', linewidth=2)
        
        #Shows plot
        plt.show()

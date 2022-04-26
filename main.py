from ui import ui
from solver import solver
import math

solver = solver(1, 1, 1, 0.5)
ui = ui("config.txt")

l = solver.solve_inverse(1.375, 0.55, 2, 0, math.pi / 4, math.pi / 2)
plotting = []
labels = []
for coords in l:
    q1 = coords[0]
    q2 = coords[1]
    q3 = coords[2]
    q4 = coords[3]
    q5 = coords[4]
    q6 = coords[5]
    labels.append(
        str(
            (round(
                q1, 2), round(
                q2, 2), round(
                    q3, 2), round(
                        q4, 2), round(
                            q5, 2), round(
                                q6, 2))))
    plotting.append(solver.solve_forward(q1, q2, q3, q4, q5, q6))
ui.plot(plotting, labels)

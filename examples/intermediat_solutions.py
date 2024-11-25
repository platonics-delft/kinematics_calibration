from casadi import *

# Define the optimization problem
x = SX.sym("x")
y = SX.sym("y")
obj = (1 - x)**2 + 100 * (y - x**2)**2  # Rosenbrock function
nlp = {'x': vertcat(x, y), 'f': obj}

# Callback class to track intermediate solutions
class MyCallback(Callback):
    def __init__(self, name, nx, opts={}):
        Callback.__init__(self)
        self.nx = nx
        self.solutions = []  # List to store intermediate solutions
        self.construct(name, opts)

    def get_n_in(self): 
        return nlpsol_n_out()  # Number of inputs expected by the solver

    def get_n_out(self): 
        return 1  # Number of outputs from the callback

    def get_name_in(self, i): 
        return nlpsol_out(i)  # Names of inputs

    def get_name_out(self, i): 
        return "ret"  # Single output

    def get_sparsity_in(self, i):
        name = nlpsol_out(i)
        if name == "x":  # Decision variable
            return Sparsity.dense(self.nx)
        elif name in ("f"):  # Cost
            return Sparsity.scalar()
        elif name == "g":  # Constraints (empty in this case)
            return Sparsity(0, 0)
        else:
            return Sparsity(0, 0)

    def eval(self, arg):
        # Retrieve the intermediate solution for the decision variable `x`
        x_solution = arg[nlpsol_out().index("x")]
        self.solutions.append([float(v) for v in np.array(x_solution)[:, 0].tolist()])
        print("Intermediate solution:", x_solution)
        return [0]

# Instantiate the callback
mycallback = MyCallback("mycallback", nx=2)

# Solver options
opts = {
    'iteration_callback': mycallback,
    'ipopt.tol': 1e-8,
    'ipopt.max_iter': 50,
}

# Create the solver
solver = nlpsol('solver', 'ipopt', nlp, opts)

# Solve the problem
sol = solver(x0=[-1.5, 1.5])  # Initial guess

# Print final solution
print("Final solution:", sol['x'])

# Print all intermediate solutions
print("All intermediate solutions:")
for s in mycallback.solutions:
    print(s)


# plot intermediate solutions

import matplotlib.pyplot as plt

x_, y_ = np.mgrid[-1:1.5:0.01, -1:1.5:0.01]
z_ = np.zeros(x_.shape)

for i in range(x_.shape[0]):
    for j in range(x_.shape[1]):
        z_[i, j] = (1 - x_[i, j])**2 + 100 * (y_[i, j] - x_[i, j]**2)**2

plt.figure()
for color_index, s in enumerate(mycallback.solutions):
    color = plt.cm.viridis(color_index / len(mycallback.solutions))
    plt.plot(s[0], s[1], 'ro-', color=color, label=f"Iteration {color_index}")

plt.plot(sol['x'][0], sol['x'][1], 'ro-', color='red', label="Final solution")



plt.contourf(x_, y_, z_)
plt.colorbar()
plt.title('Iterations of Rosenbrock')
plt.show()


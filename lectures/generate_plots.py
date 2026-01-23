import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

# Generate sample data
np.random.seed(42)
x = np.linspace(0, 10, 15)
y = 2 * x + 1 + np.random.normal(0, 1.5, 15)

# Perform least squares fit
slope, intercept, r_value, p_value, std_err = stats.linregress(x, y)
line = slope * x + intercept

# Create the plot with LaTeX-style fonts
plt.style.use("seaborn")
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Computer Modern Roman"],
})

# Create figure
plt.figure(figsize=(10, 6))

# Plot data points
plt.scatter(x, y, color="red", label="Data points", zorder=3)

# Plot fitted line
plt.plot(x, line, "b-", label=f"$\\hat{{y}} = {slope:.2f}x + {intercept:.2f}$", zorder=2)

# Plot residuals
for i in range(len(x)):
    plt.plot([x[i], x[i]], [y[i], line[i]], "k--", alpha=0.3, zorder=1)

# Add labels and title
plt.xlabel("$x$")
plt.ylabel("$y$")
plt.title("Least Squares Fitting Example")
plt.legend()
plt.grid(True, alpha=0.3)

# Save plot
plt.savefig("least_squares.pdf", bbox_inches="tight", dpi=300) 
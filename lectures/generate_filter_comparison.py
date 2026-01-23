import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
import seaborn as sns

# Use LaTeX fonts
rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
rc('text', usetex=True)
plt.style.use('seaborn')

# Generate data
np.random.seed(42)
t = np.linspace(0, 10, 100)
true_state = np.sin(t) + 0.5 * np.sin(2*t)
measurements = true_state + np.random.normal(0, 0.2, len(t))

# Simulate Kalman Filter estimates (simplified)
kf_estimates = np.zeros_like(t)
kf_estimates[0] = measurements[0]
kf_var = 0.1
for i in range(1, len(t)):
    # Prediction
    kf_estimates[i] = kf_estimates[i-1]
    # Update
    kf_estimates[i] = kf_estimates[i] + kf_var * (measurements[i] - kf_estimates[i])

# Simulate Particle Filter estimates (simplified)
pf_estimates = np.zeros_like(t)
n_particles = 100
particles = np.random.normal(measurements[0], 0.1, n_particles)
for i in range(1, len(t)):
    # Propagate particles
    particles = particles + np.random.normal(0, 0.1, n_particles)
    # Weight update (simplified)
    weights = np.exp(-0.5 * ((particles - measurements[i])/0.2)**2)
    weights = weights / np.sum(weights)
    # Resample
    indices = np.random.choice(n_particles, n_particles, p=weights)
    particles = particles[indices]
    pf_estimates[i] = np.mean(particles)

# Create comparison plot
plt.figure(figsize=(10, 6))
plt.plot(t, true_state, 'k-', label='True State', linewidth=2)
plt.plot(t, measurements, 'g.', label='Measurements', markersize=4, alpha=0.5)
plt.plot(t, kf_estimates, 'b--', label='Kalman Filter', linewidth=1.5)
plt.plot(t, pf_estimates, 'r--', label='Particle Filter', linewidth=1.5)
plt.xlabel('Time')
plt.ylabel('State')
plt.title('Comparison of Kalman Filter and Particle Filter Performance')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('filter_comparison.pdf', dpi=300, bbox_inches='tight')
plt.close()

# Create error plot
plt.figure(figsize=(10, 4))
plt.plot(t, np.abs(kf_estimates - true_state), 'b-', label='Kalman Filter Error', linewidth=1.5)
plt.plot(t, np.abs(pf_estimates - true_state), 'r-', label='Particle Filter Error', linewidth=1.5)
plt.xlabel('Time')
plt.ylabel('Absolute Error')
plt.title('Estimation Error Comparison')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('error_comparison.pdf', dpi=300, bbox_inches='tight') 
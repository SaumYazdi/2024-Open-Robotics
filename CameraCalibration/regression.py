import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

def func(x, k, a):
    return k / pow(x, a)

x_data = [178, 106, 67, 53.5, 44.6, 42]
y_data = [10, 20, 30, 40, 50, 60]

# Fit the data to the tanh function
# Set initial guess to apparent inflection point
initial_guess = [2000.0, -1.0]
params, covariance = curve_fit(func, x_data, y_data, p0=initial_guess)
# Extract the parameters
k, a = params

# Create a range of x values for the curve change value of "127" to max number or data points i didnt know how to get max size of the data sheet
x_fit = np.linspace(min(x_data), max(x_data), 137)
# Calculate the y values for the fitted curve
y_fit = func(x_fit, k, a)

# Plot of the data and the fitted curve
plt.figure(figsize=(16, 12)) 
plt.scatter(x_data, y_data, label="Data")
plt.plot(x_fit, y_fit, label="Power Series Fit", color= "red")
plt.xticks(fontsize=15) 
plt.yticks(fontsize=15)
plt.xlabel("Temperature",fontsize=17)
plt.ylabel("Energy",fontsize=17)
plt.title("Temperature vs Energy",fontsize=22)
# Display the equation
equation = f"y = {k:.2f}/x^{a:.2f}"
print("Equation:", equation)


text_x = 8  # x-coordinate
text_y = 16  # y-coordinate

plt.text(text_x, text_y, equation, fontsize=19, color='red',
         bbox={'facecolor': 'white', 'alpha': 0.7, 'edgecolor': 'gray'})
plt.show()
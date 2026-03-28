import os
import sys

import numpy as np
from scipy.optimize import curve_fit

# Append parent directory to sys.path so we can import constants.py
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from constants import TARGET_SHOOTER_DATA

# TODO: Replace this placeholder array with your actual array of inputs.
# The format expected here is a list of tuples or lists: (x, y)
# For example, if fitting distance to flywheel speed: (distance, speed)
# TARGET_SHOOTER_DATA = [
#     (7.4, -0.485),
#     (9.7, -0.65),
#     (13.0, -0.85),
#     (13.4, -1.0)
# ]

def quadratic_function(x, a, b, c):
    """Quadratic equation: y = a*x^2 + b*x + c"""
    return a * x**2 + b * x + c

def main():
    # Parse the input data into independent (x) and dependent (y) variables
    x_data = np.array([val[0] for val in TARGET_SHOOTER_DATA])
    y_data = np.array([val[1] for val in TARGET_SHOOTER_DATA])

    # curve_fit returns the optimal parameters (popt) and the estimated covariance (pcov)
    try:
        popt, pcov = curve_fit(quadratic_function, x_data, y_data)
        a, b, c = popt
        
        print("====== Quadratic Fit Results ======")
        print(f"a = {a}")
        print(f"b = {b}")
        print(f"c = {c}")
        
        print("\n====== Generated Python Formula ======")
        print("def calculate_value(x: float) -> float:")
        print("    \"\"\"")
        print("    Calculates the target value using a quadratic fit:")
        print("    y = a*x^2 + b*x + c")
        print("    \"\"\"")
        print(f"    a = {a}")
        print(f"    b = {b}")
        print(f"    c = {c}")
        print("    return a * (x ** 2) + b * x + c")
        print("======================================")

        print("\n====== Validation ======")
        for x, y in TARGET_SHOOTER_DATA:
            calc_y = quadratic_function(x, a, b, c)
            print(f"Mismatch at x={x}: expected {y}, got {calc_y:.3f}")
            assert abs(calc_y - y) <= 0.2, f"Mismatch at x={x}: expected {y}, got {calc_y:.3f} (difference: {abs(calc_y - y):.3f})"
        print("All inputs validated successfully!")
        
        # Update constants.py
        import os, re
        constants_path = os.path.join(os.path.dirname(__file__), '..', 'constants.py')
        with open(constants_path, 'r') as f:
            content = f.read()
            
        new_block = (
            "# --- BEGIN AUTO-GENERATED CONSTANTS ---\n"
            f"SHOOTER_QUADRCOEF_A = {a}\n"
            f"SHOOTER_QUADRCOEF_B = {b}\n"
            f"SHOOTER_QUADRCOEF_C = {c}\n"
            "# --- END AUTO-GENERATED CONSTANTS ---"
        )
        
        updated_content = re.sub(
            r"# --- BEGIN AUTO-GENERATED CONSTANTS ---.*?# --- END AUTO-GENERATED CONSTANTS ---",
            new_block,
            content,
            flags=re.DOTALL
        )
        
        with open(constants_path, 'w') as f:
            f.write(updated_content)
            
        print(f"Updated constants.py with new coefficients.")
        
    except Exception as e:
        print(f"Failed to fit the curve. Error: {e}")

if __name__ == "__main__":
    main()
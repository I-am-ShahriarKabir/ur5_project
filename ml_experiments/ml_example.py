import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
from sklearn.neural_network import MLPRegressor
import matplotlib.pyplot as plt

# Load the joint data CSV (update the file path if necessary)
data = pd.read_csv('../catkin_ws/src/ur5_controller/src/joint_data.csv')

# Display the first few rows to understand the data structure
print("Data sample:")
print(data.head())

# For demonstration, 'time' is used as the predictor and 'shoulder_pan_joint' as the target.
# Ensure your CSV contains columns with these names.
X = data['time'].values.reshape(-1, 1)
y = data['shoulder_pan_joint'].values

# Split data into training and testing sets if needed (optional)
# For simplicity, we will use the entire dataset for training in this example.
# sklearn.model_selection.train_test_split can be used for a more robust approach.

# Used a polynomial regression model. Adjust the degree to capture the sine wave behavior.
# Model 1: Polynomial Regression with a higher degree (e.g., degree = 6)
degree = 6
poly_model = make_pipeline(PolynomialFeatures(degree), LinearRegression())
poly_model.fit(X, y)

# Perform inference: predict joint angles using the trained model
predictions_poly = poly_model.predict(X)

# Calculate regression metrics
mse = mean_squared_error(y, predictions_poly)
mae = mean_absolute_error(y, predictions_poly)
r2 = r2_score(y, predictions_poly)

print("\nPolynomial Regression Model (Degree 6):")
print(poly_model)
print(f"Mean Squared Error: {mse:.4f}")
print(f"Mean Absolute Error: {mae:.4f}")
print(f"R² Score: {r2:.4f}")

# Model 2: MLP Regressor (a simple neural network)
mlp_model = MLPRegressor(hidden_layer_sizes=(1000, 1000), max_iter=1000, random_state=42, verbose=True)
mlp_model.fit(X, y)
predictions_mlp = mlp_model.predict(X)

# Calculate regression metrics for MLP regressor
mse_mlp = mean_squared_error(y, predictions_mlp)
mae_mlp = mean_absolute_error(y, predictions_mlp)
r2_mlp = r2_score(y, predictions_mlp)

print("\nMLP Regressor Model:")
print(mlp_model)
print(f"Mean Squared Error: {mse_mlp:.4f}")
print(f"Mean Absolute Error: {mae_mlp:.4f}")
print(f"R² Score: {r2_mlp:.4f}")

# Visualize the results for the better performing model (choose one based on your metrics)
plt.figure(figsize=(10, 6))
plt.plot(data['time'].to_numpy(), y, label='Actual', color='blue')

# Here, models' predictions are visualized:
plt.plot(data['time'].to_numpy(), predictions_poly, label='Poly Regression (Degree 6)', linestyle='--', color='red')
plt.plot(data['time'].to_numpy(), predictions_mlp, label='MLP Regressor', linestyle=':', color='green')

plt.xlabel('Time (s)')
plt.ylabel('Shoulder Pan Joint Angle (rad)')
plt.title('Joint Angle Prediction Comparison')
plt.legend()
plt.show()

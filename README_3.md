# Expanding the UR5 Data Pipeline with ML/AI Capabilities

This task extends the UR5 simulation data pipeline by integrating a Machine Learning component to predict joint angles based on historical data. The primary goal is to demonstrate how to load stored data, prepare a dataset, train a predictive model, and perform inference while evaluating the model's performance.


## 1. Overview and Architecture

### ML/AI Use Case
For this example, we predict the `shoulder_pan_joint` angle of the UR5 robot as a function of time. Although the underlying data follows a sine wave pattern, we experiment with two models:
- **Polynomial Regression:** A non-linear model that uses polynomial features (degree 6) to capture the sine wave behavior.
- **MLP Regressor:** A neural network model (with two hidden layers, each containing 1000 neurons) to capture more complex non-linear relationships.

### Software Architecture Diagram

     +-------------------------+
     |   UR5 Simulation in     |  
     |        Gazebo           |
     |                         |
     |  Publishes Topics:      |
     |   - /joint_states       |  
     |   - /camera/image_raw   |
     +------------+------------+
                  |
                  v
     +-------------------------+
     |   Data Storage          |
     |  (joint_data.csv)       |
     |                         |
     | (Collected via data     |
     |  collector node)        |
     +------------+------------+
                  |
                  v
     +-------------------------+
     |   ML/AI Module          |
     | (ml_example.py)         |
     |                         |
     |  Steps:                 |
     |   - Load CSV Data       |
     |   - Prepare Dataset     |
     |   - Train Models        |
     |      (Polynomial &      |
     |       MLP Regressor)    |
     |   - Perform Inference    |
     |   - Evaluate Performance |
     |     (MSE, MAE, R²)      |
     |   - Visualize Results    |
     +-------------------------+


**Explanation:**
- **Data Storage:** The CSV file `joint_data.csv` (generated in Task 2) contains the recorded joint state data.
- **ML/AI Module:** The `ml_example.py` script (or notebook) loads the CSV, prepares the data, trains models, infers predictions, computes evaluation metrics, and visualizes the results.


## 2. Files and Their Roles

### a. ml_example.py
- **Location:**  
  You can place `ml_example.py` in a dedicated ML experiments folder (e.g., `~/Neura_task/ml_experiments`) or within your ROS package if preferred.
- **Purpose:**  
  This script demonstrates the entire ML workflow:
  1. **Load Storage Data:**  
     Uses `pandas.read_csv` to load the CSV data (joint_data.csv) containing joint configurations.
  2. **Prepare Dataset:**  
     Extracts the `time` column as the predictor (X) and `shoulder_pan_joint` as the target (y), reshaping as needed.
  3. **Train the Model:**  
     Trains two models:
     - **Polynomial Regression:** Uses a pipeline with `PolynomialFeatures` (degree 6) and `LinearRegression`.
     - **MLP Regressor:** Uses a neural network with two hidden layers (1000 neurons each) and is configured with `verbose=True` to display training progress.
  4. **Inference:**  
     Predicts joint angles using the trained models.
  5. **Evaluation:**  
     Computes performance metrics including Mean Squared Error (MSE), Mean Absolute Error (MAE), and R² Score.
  6. **Visualization:**  
     Plots actual vs. predicted joint angles for both models, enabling visual comparison.
- **Additional Capability:**  
  The script prints the model pipelines (or parameters) and evaluation metrics, offering insight into the model’s configuration and performance.


## 3. Installation and Environment Setup

### Prerequisites
- **Python Libraries:**  
  Ensure the following libraries are installed:
  - pandas, numpy
  - scikit-learn
  - matplotlib

    Install using:
    ```bash
    python3 -m pip install pandas numpy scikit-learn matplotlib


- **Data File:**
    Ensure that joint_data.csv exists and is accessible. For instance, if it is generated in ~/Neura_task/catkin_ws/src/ur5_controller/src, update the path in ml_example.py accordingly.


## 4. Running the ML/AI Module

### Steps to Run the Script
- **Navigate to the ML Experiments Folder:**  
    ```bash
    cd ~/Neura_task/ml_experiments

- **Run the Script:**
    ```bash
    python3 ml_example.py

### Expected Output
- The script will print a sample of the loaded data.
- It will then train both models, printing progress for the MLP (since verbose=True is set).
- Evaluation metrics (MSE, MAE, R² Score) for each model will be printed to the console.
- A plot window will open showing the actual joint angle data versus the predictions from the polynomial regression and MLP models.


## 5. Discussion of Model Choices and Evaluation

### Model Choices
- **Polynomial Regression:**
    A non-linear model that uses polynomial features to capture the periodic nature of sine wave data. Degree 6 is used as a starting point; this can be tuned.

- **MLP Regressor:**
    - A simple neural network model configured with two hidden layers (1000 neurons each). It is trained for 1000 iterations. The verbose option is enabled to display epoch-by-epoch progress.
    - This model can capture more complex non-linear relationships if the data exhibits higher complexity.

### Evaluation Metrics
- **Mean Squared Error (MSE):**
    Provides an average of the squared differences between predictions and actual values.
- **Mean Absolute Error (MAE):**
    Provides an average of the absolute differences between predictions and actual values.
- **R² Score:**
    Indicates the proportion of variance in the dependent variable that is predictable from the independent variable(s); closer to 1 is better.
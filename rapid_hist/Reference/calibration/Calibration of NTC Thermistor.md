# Calibration of NTC Thermistor

### Methodology

To achieve a more accurate temperature-resistance relationship, we placed a thermocouple thermometer alongside the NTC thermistor at the same location and conducted calibration within the device's operating temperature range. The recorded data is shown in the following table.
 We then applied four fitting models: first-order, third-order, fifth-order logarithmic polynomials, and a log-linear model. For comparison, we also evaluated the standard **β-parameter model** provided in the thermistor datasheet.

### Fitting Curve and Parameters

#### 1st-order Logarithmic Polynomial

Fitting form:
$$
T=a⋅ln(R)+b
$$
Fitted parameters:
$$
a = -19.377573,b = 245.901173
$$

#### 3rd-order Logarithmic Polynomial

Fitting form:
$$
T=a⋅ln(R)^3+b⋅ln(R)^2+c⋅ln(R)+d
$$
Fitted parameters:
$$
𝑎=1.260494,𝑏=−37.085894,𝑐=341.676840,d=−916.389987
$$

#### 5th-order Logarithm Polynomial

Fitting form:
$$
T=a⋅ln(R)^5+b⋅ln(R)^4+c⋅ln(R)^3+d⋅ln(R)^2+e⋅ln(R)+f
$$
Fitted parameters:
$$
a=-0.410963,b=23.007500,c=-511.787183,d=5660.766498,e=-31182.521670,f=68609.215924
$$

#### Log-linear Model

Fitting form:
$$
T=\frac{ln(R)-a}{b}
$$
Fitted parameters:
$$
𝑎=12.674620,𝑏=−0.051245
$$

#### NTC Beta-parameter Equation

This is the classic thermistor model based on the β-parameter found in datasheets:

$$
\frac{1}{T}- \frac{1}{T_{\text{nominal}}}={\frac{1}{B} }\ln\left(\frac{R_{\text{thermistor}}}{R_{\text{nominal}}}\right)
$$
Parameters used:
$$
R = 100K\Omega,B = 3950 K, T_{nominal}=275.15K-25K=250.15K
$$

### Evaluation

Based on the measured data and the fitting models above, we generated the following R-T curve and temperature error curve:

| R-T Curve                                                    | Error-T Curve                                                |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![r-t curve](C:\Users\mosyu\Desktop\rapid_hist_prep\heatonly\rapid_hist\Reference\calibration\r-t curve.png) | ![error](C:\Users\mosyu\Desktop\rapid_hist_prep\heatonly\rapid_hist\Reference\calibration\error.png) |

Additionally, we obtained the following error metrics table:

|            | Max Error (°C) | Mean Error (°C) | RMSE (°C)    |
| ---------- | -------------- | --------------- | ------------ |
| 1st-order  | 10.231937      | 1.802454        | 2.595068     |
| 3rd-order  | **1.577701**   | 0.639457        | 0.730019     |
| 5th-order  | 1.795768       | **0.597279**    | **0.691203** |
| Log-Linear | 10.774909      | 1.823146        | 2.686234     |
| Beta Model | 23.218667      | 17.419537       | 18.280827    |

### Final Decision

Given the fitting accuracy and model complexity, we selected the **fifth-order logarithmic polynomial** as the final formula for temperature calculation.
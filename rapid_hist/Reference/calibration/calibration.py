import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.optimize import curve_fit

# 加载数据
df = pd.read_csv("ntc.csv")
df.columns = ["Resistance", "Temp_C"]
df["lnR"] = np.log(df["Resistance"])
df["Temp_K"] = df["Temp_C"] + 273.15

# 多项式拟合
poly1 = np.poly1d(np.polyfit(df["lnR"], df["Temp_C"], deg=1))
poly3 = np.poly1d(np.polyfit(df["lnR"], df["Temp_C"], deg=3))
poly5 = np.poly1d(np.polyfit(df["lnR"], df["Temp_C"], deg=5))
df["T_fit_poly1_C"] = poly1(df["lnR"])
df["T_fit_poly3_C"] = poly3(df["lnR"])
df["T_fit_poly5_C"] = poly5(df["lnR"])

# log-linear 拟合
def log_linear_model(T, a, b):
    return a + b * T
popt_ln, _ = curve_fit(log_linear_model, df["Temp_C"], df["lnR"])
a_ln, b_ln = popt_ln
df["T_fit_ln_C"] = (df["lnR"] - a_ln) / b_ln

# Beta 模型参数
nominalR = 100000.0  # 25°C 电阻
nominalT_K = 25.0 + 273.15
B = 3950.0

# Beta 模型拟合函数
def beta_temp(R):
    return 1.0 / ((np.log(R / nominalR) / B) + (1.0 / nominalT_K)) - 273.15

df["T_fit_beta_C"] = beta_temp(df["Resistance"])

# ----------- 画 RT 曲线图 -----------
plt.figure(figsize=(10, 6))
plt.scatter(df["Temp_C"], df["Resistance"], label="Measured", color="black", s=25)
plt.plot(df["T_fit_poly1_C"], df["Resistance"], label="1st-order", linestyle="--")
plt.plot(df["T_fit_poly3_C"], df["Resistance"], label="3rd-order", linestyle="-.")
plt.plot(df["T_fit_poly5_C"], df["Resistance"], label="5th-order", linewidth=2)
plt.plot(df["T_fit_ln_C"], df["Resistance"], label="Log-Linear", linestyle="-")
plt.plot(df["T_fit_beta_C"], df["Resistance"], label="Beta Model", linestyle=":")

plt.xlim(df["Temp_C"].min(), df["Temp_C"].max())
plt.ylim(df["Resistance"].max(), df["Resistance"].min())
plt.xlabel("Temperature (°C)")
plt.ylabel("Resistance (Ohm)")
plt.title("R-T Curve of NTC Thermistor (All Models)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# ----------- 画温度误差图（相对误差） -----------
df["err_poly1"] = 100*(df["T_fit_poly1_C"] - df["Temp_C"])/df["Temp_C"]
df["err_poly3"] = 100*(df["T_fit_poly3_C"] - df["Temp_C"])/df["Temp_C"]
df["err_poly5"] = 100*(df["T_fit_poly5_C"] - df["Temp_C"])/df["Temp_C"]
df["err_ln"] = 100*(df["T_fit_ln_C"] - df["Temp_C"])/df["Temp_C"]
df["err_beta"] = 100*(df["T_fit_beta_C"] - df["Temp_C"])/df["Temp_C"]

plt.figure(figsize=(10, 6))
plt.axhline(0, color='black', linestyle='--', linewidth=1)
plt.plot(df["Temp_C"], df["err_poly1"], label="1st-order", linestyle="--")
plt.plot(df["Temp_C"], df["err_poly3"], label="3rd-order", linestyle="-.")
plt.plot(df["Temp_C"], df["err_poly5"], label="5th-order", linewidth=2)
plt.plot(df["Temp_C"], df["err_ln"], label="Log-Linear", linestyle="-")
plt.plot(df["Temp_C"], df["err_beta"], label="Beta Model", linestyle=":")

plt.xlabel("Temperature (°C)")
plt.ylabel("Relative Error (%)")
plt.title("Temperature Error vs Temperature")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# ----------- 输出误差指标表格（最大/平均/RMSE） -----------
def evaluate_errors(err):
    return {
        "Max Error (°C)": np.max(np.abs(err)),
        "Mean Error (°C)": np.mean(np.abs(err)),
        "RMSE (°C)": np.sqrt(np.mean(err**2))
    }

metrics = {
    "1st-order": evaluate_errors(df["err_poly1"]),
    "3rd-order": evaluate_errors(df["err_poly3"]),
    "5th-order": evaluate_errors(df["err_poly5"]),
    "Log-Linear": evaluate_errors(df["err_ln"]),
    "Beta Model": evaluate_errors(df["err_beta"])
}

df_metrics = pd.DataFrame(metrics).T
print("所有模型绝对误差评估：")
print(df_metrics)
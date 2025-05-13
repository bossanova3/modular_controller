# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt
# from sklearn.preprocessing import MinMaxScaler
# from sklearn.model_selection import train_test_split
# from tensorflow.keras.models import Sequential
# from tensorflow.keras.layers import SimpleRNN, Dense, Dropout


# df = pd.read_csv("hand_pose_data_RNN.csv")             

# # Columnas de predicción
# df["roll_next"] = df["roll"].shift(-1)
# df["pitch_next"] = df["pitch"].shift(-1)
# df["yaw_next"] = df["yaw"].shift(-1)
# df.dropna(inplace=True)

# # Guardar el nuevo CSV
# df.to_csv("hand_pose_data_with_labels.csv", index=False)

# print("CSV actualizado con columnas 'roll_next', 'pitch_next', 'yaw_next'")


# # xd_(k)   xd -> X_deseada
# X = df[["roll", "pitch", "yaw"]].values
# # xda(k + 1)  -> X_deseada_aprox
# y = df[["roll_next", "pitch_next", "yaw_next"]].values


# scaler_X = MinMaxScaler()
# scaler_y = MinMaxScaler()

# X_scaled = scaler_X.fit_transform(X)
# y_scaled = scaler_y.fit_transform(y)

# sequence_length = 10
# X_seq, y_seq = [], []

# for i in range(len(X_scaled) - sequence_length):
#     X_seq.append(X_scaled[i:i+sequence_length])
#     y_seq.append(y_scaled[i+sequence_length-1])

# X_seq = np.array(X_seq)
# y_seq = np.array(y_seq)
# X_train, X_test, y_train, y_test = train_test_split(X_seq, y_seq, test_size=0.2, random_state=42)
# model = Sequential()
# model.add(SimpleRNN(128, activation='tanh', input_shape=(sequence_length, 3)))
# model.add(Dropout(0.3))  # Regularización
# model.add(Dense(3))  # Para predecir roll, pitch, yaw

# model.compile(optimizer='adam', loss='mse', metrics=['mae'])
# model.fit(X_train, y_train, epochs=200, batch_size=64, validation_split=0.2)
# loss, mae = model.evaluate(X_test, y_test)
# print(f"MAE en test: {mae:.4f}")

# # xd_a(k + 1)
# y_pred = model.predict(X_test)
# y_pred_real = scaler_y.inverse_transform(y_pred)
# y_test_real = scaler_y.inverse_transform(y_test)

# # Error(k) = xd(k) - xd_aprox(k)
# e = y_test_real - y_pred_real

# fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

# labels = ['Roll', 'Pitch', 'Yaw']
# for i in range(3):
#     axs[i].plot(y_test_real[:, i], label=f'{labels[i]} Real', linewidth=1.5)
#     axs[i].plot(y_pred_real[:, i], label=f'{labels[i]} Predicho', linestyle='--')
#     axs[i].set_ylabel(labels[i])
#     axs[i].legend()
#     axs[i].grid(True)

# axs[2].set_xlabel("Tiempos")
# plt.suptitle("Comparación entre valores reales y predichos")
# plt.tight_layout()
# plt.show()


import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import SimpleRNN, Dense, Dropout


df = pd.read_csv("hand_pose_data_RNN.csv")             

# Columnas de predicción
df["roll_next"] = df["roll"].shift(-1)
df["pitch_next"] = df["pitch"].shift(-1)
df["yaw_next"] = df["yaw"].shift(-1)
df.dropna(inplace=True)

# Guardar el nuevo CSV
df.to_csv("hand_pose_data_with_labels.csv", index=False)

print("CSV actualizado con columnas 'roll_next', 'pitch_next', 'yaw_next'")


# xd_(k)   xd -> X_deseada
X = df[["roll", "pitch", "yaw"]].values
# xda(k + 1)  -> X_deseada_aprox
y = df[["roll_next", "pitch_next", "yaw_next"]].values


scaler_X = MinMaxScaler()
scaler_y = MinMaxScaler()

X_scaled = scaler_X.fit_transform(X)
y_scaled = scaler_y.fit_transform(y)

sequence_length = 1
X_seq, y_seq = [], []

for i in range(len(X_scaled) - sequence_length):
    X_seq.append(X_scaled[i:i+sequence_length])
    y_seq.append(y_scaled[i+sequence_length-1])

X_seq = np.array(X_seq)
y_seq = np.array(y_seq)
X_train, X_test, y_train, y_test = train_test_split(X_seq, y_seq, test_size=0.2, random_state=42)
model = Sequential()
model.add(SimpleRNN(128, activation='tanh', input_shape=(sequence_length, 3)))
model.add(Dropout(0.3))  # Regularización
model.add(Dense(3))  # Para predecir roll, pitch, yaw

model.compile(optimizer='adam', loss='mse', metrics=['mae'])
model.fit(X_train, y_train, epochs=200, batch_size=64, validation_split=0.2)
loss, mae = model.evaluate(X_test, y_test)
print(f"MAE en test: {mae:.4f}")

# xd_a(k + 1)
y_pred = model.predict(X_test)
y_pred_real = scaler_y.inverse_transform(y_pred)
y_test_real = scaler_y.inverse_transform(y_test)

# Error(k) = xd(k) - xd_aprox(k)
e = y_test_real - y_pred_real

fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

labels = ['Roll', 'Pitch', 'Yaw']
for i in range(3):
    axs[i].plot(y_test_real[:, i], label=f'{labels[i]} Real', linewidth=1.5)
    axs[i].plot(y_pred_real[:, i], label=f'{labels[i]} Predicho', linestyle='--')
    axs[i].set_ylabel(labels[i])
    axs[i].legend()
    axs[i].grid(True)

axs[2].set_xlabel("Tiempos")
plt.suptitle("Comparación entre valores reales y predichos")
plt.tight_layout()
plt.show()

model.save("mi_modelo_rnn.h5")
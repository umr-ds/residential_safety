import matplotlib.pyplot as plt
import pandas as pd

file_names_sensors = ['hall_sensor.csv', 'temp_sensor.csv', 'odor_sensor.csv', 'co_sensor.csv', 'accelerometer.csv',
                      'pir_sensor.csv', 'water_level_sensor.csv']
file_names_boards = ['10sek_deepsleep_normalized.csv', '30sek_deepsleep_normalized.csv',
                     '60sek_deepsleep_normalized.csv', '10sek_deepsleep_without_any_sensor.csv',
                     '10sek_deepsleep_with_event']

plt.figure(figsize=(10, 6))

for file_name in file_names_sensors:
    df = pd.read_csv(file_name)
    time = df['Time(ms)']
    df['Power(mW)'] = df['Main(mA)'] * df['USB Voltage(V)']
    neuer_df = df[['Time(ms)', 'Power(mW)']]
    neuer_df.to_csv(file_name + '_power_consumption.csv', index=False, columns=['Time(ms)', 'Power(mW)'])
    power_consumption = df['Power(mW)']
    mean_power_consumption = power_consumption.mean()

    print(mean_power_consumption)

    plt.plot(time, power_consumption, label=file_name)

for file_name in file_names_boards:
    df = pd.read_csv(file_name)
    df['Main(mA)']
    time = df['Time(ms)']
    df['Power(mW)'] = df['USB(mA)'] * df['USB Voltage(V)']
    neuer_df = df[['Time(ms)', 'Power(mW)']]
    neuer_df.to_csv(file_name + '_power_consumption.csv', index=False, columns=['Time(ms)', 'Power(mW)'])
    power_consumption = df['Power(mW)']
    mean_power_consumption = power_consumption.mean()
    print(mean_power_consumption)
    plt.plot(time, power_consumption, label=file_name)

plt.xticks(fontsize=16)
plt.yticks(fontsize=16)
plt.xlabel('Time(ms)', fontsize=18)
plt.ylabel('Power(mW)', fontsize=18)
plt.show()

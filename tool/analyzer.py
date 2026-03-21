import pandas as pd
import math
import matplotlib.pyplot as plt

file_path = 'variable_double.csv'
data = pd.read_csv(file_path)

column_stats = {}
TOTAL_BITS = 16

def calculate_fixed_point_data_type(value):
    if math.isnan(value):  # Check if the value is NaN
        return 'NaN'
    if value==0:
        int_bits=1
    else:
        int_bits = math.ceil(math.log2(math.ceil(abs(value))))+1
    frac_bits = TOTAL_BITS - int_bits -1
    data_type = f'int_{int_bits}_{frac_bits}'
    
    return data_type

for column in data.columns:
    max_value = data[column].max()
    min_value = data[column].min()
    std_dev = data[column].std()
    num_cal = data[column].count()
    column_stats[column] = {'max_value': max_value, 'min_value': min_value, 'std_dev': std_dev, 'num_cal': num_cal}

# Find the maximum number of rows among all columns
max_num_cal = max(stats['num_cal'] for stats in column_stats.values())

for column, stats in column_stats.items():
    print(f"variable： {column}:")
    print("std_dev:", stats['std_dev'])
    print("num_cal:", stats['num_cal'])
    print("num_cal/max(num_call[i]):", stats['num_cal']/max_num_cal*100)
    print("max_value:", stats['max_value'])
    print("min_value:", stats['min_value'])
    print("data_type:",calculate_fixed_point_data_type(max(abs(stats['max_value']), abs(stats['min_value']))))
    print("\n")
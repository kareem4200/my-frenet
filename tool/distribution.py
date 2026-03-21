import pandas as pd
import math
import matplotlib.pyplot as plt

file_path = 'variable_float.csv'
data = pd.read_csv(file_path)

column_stats = {}
TOTAL_BITS = 16

def calculate_fixed_point_data_type(value):
    if value==0:
        int_bits=1
    else:
        int_bits = math.ceil(math.log2(math.ceil(abs(value))))+1
    frac_bits = TOTAL_BITS - int_bits -1
    data_type = f'int_{int_bits}_{frac_bits}'
    
    return data_type

# delta value
# delta = 0.1

for column in data.columns:
    max_value = data[column].max()
    min_value = data[column].min()
 
    plt.hist(data[column], bins=100)
    plt.xlabel('Value Range')
    plt.ylabel('Frequency')
    plt.title(f'Distribution of {column}')
    plt.show()

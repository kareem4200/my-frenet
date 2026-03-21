import pandas as pd
from sklearn.model_selection import train_test_split

# Load your full dataset
c_df = pd.read_csv("CVAE/data/conditioned_vars_with_img.csv")
x_df = pd.read_csv("CVAE/data/sampled_vars.csv")

train_size = x_df.shape[0] * 80 // 100
val_size = x_df.shape[0] * 90 // 100
# test_size = x_df.shape[0] * 10 // 100
print(f"Train size: {train_size}, Val size: {val_size}")
# Split into train + temp (val + test)
# c_train_df, c_temp_df = train_test_split(c_df, test_size=0.2, random_state=42)

# # Split temp into validation and test
# c_val_df, c_test_df = train_test_split(c_temp_df, test_size=0.5, random_state=42)

# # Split into train + temp (val + test)
# x_train_df, x_temp_df = train_test_split(x_df, test_size=0.2, random_state=42)

# # Split temp into validation and test
# x_val_df, x_test_df = train_test_split(x_temp_df, test_size=0.5, random_state=42)

x_train_df = x_df.iloc[:train_size, 2:]
c_train_df = c_df.iloc[:train_size, 2:]

x_val_df = x_df.iloc[train_size:val_size, 2:]
c_val_df = c_df.iloc[train_size:val_size, 2:]

x_test_df = x_df.iloc[val_size:, 2:]
c_test_df = c_df.iloc[val_size:, 2:]

# Optionally save them back to disk
x_train_df.to_csv("CVAE/data/x_train.csv", index=False)
x_val_df.to_csv("CVAE/data/x_validation.csv", index=False)
x_test_df.to_csv("CVAE/data/x_test.csv", index=False)

c_train_df.to_csv("CVAE/data/c_train.csv", index=False)
c_val_df.to_csv("CVAE/data/c_validation.csv", index=False)
c_test_df.to_csv("CVAE/data/c_test.csv", index=False)

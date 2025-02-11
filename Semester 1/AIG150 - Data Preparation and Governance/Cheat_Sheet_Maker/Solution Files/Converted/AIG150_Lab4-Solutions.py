import pandas as pd
from numpy import nan
import numpy as np

np.__version__

ebola = pd.read_csv('country_timeseries.csv')
ebola.dtypes
ebola.shape

#print(ebola.count())
num_rows = ebola.shape[0]
num_missing = num_rows - ebola.count()
print(num_missing)

num_rows = ebola.shape[0]
num_missing = num_rows - ebola.count()

missing_val_count_by_column = ebola.isnull().sum()
missing_percentages = missing_val_count_by_column / num_rows * 100
missing_percentages

ebola.columns # is a list of column names

np.count_nonzero(ebola['Deaths_Mali'].isnull())

for col in ebola.columns:
    print("% of data missing in {} column = {}%".format(col, missing_percentages[col]))

print("Rows missing more than 90% of Data:")
for i in range(len(missing_val_count_by_column)):
  if missing_val_count_by_column[i] > 0.9 * num_rows:
    print(missing_val_count_by_column.index[i], missing_val_count_by_column[i]/num_rows*100)

columns_to_drop = []
# add columns with more than 90% missing values to list
for i in range(len(missing_val_count_by_column)):
  if missing_val_count_by_column[i] > 0.9 * num_rows:
    columns_to_drop.append(missing_val_count_by_column.index[i])

print(columns_to_drop)

ebola_droppedCol = ebola.drop(columns=columns_to_drop)
ebola_droppedCol.shape

ebola_droppedCol.isnull().sum()

ebola.bfill().isnull().sum().sum()

ebola.iloc[0:10, 0:5].bfill()

ebola.ffill().isnull().sum().sum()

ebola.iloc[0:10, 0:5].ffill()

double_fill = ebola.bfill().ffill()
double_fill.isnull().sum().sum()

interpolated = ebola.interpolate(method='linear')
interpolated.head()

interpolated.isnull().sum()




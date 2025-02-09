import pandas as pd
import numpy as np

ebola = pd.read_csv('country_timeseries.csv')
ebola

ebola.dtypes

ebola.shape

ebola.count()

num_rows = ebola.shape[0]
num_missing = num_rows - ebola.count()
print(num_missing)

# Alternatively, you can use the isnull method to find missing values.
missing_val_count_by_column = ebola.isnull().sum()
missing_val_count_by_column

# Modify the code to display the percentage of missing values for each of the columns in ebola dataset.
num_missing = missing_val_count_by_column.copy()
num_rows, num_columns = ebola.shape
percentage_missing = (num_missing / num_rows)*100
percentage_missing

ebola.columns # is a list of column names

np.count_nonzero(ebola['Deaths_Mali'].isnull())

# Percentage of missing values for each column
for col in ebola.columns:
    num_missing = np.count_nonzero(ebola[col].isnull())
    if num_missing > 0:
        percentage_missing = (num_missing / num_rows)*100
        print(col, f"{percentage_missing:.2f}%")

missing_val_count_by_column

missing_val_count_by_column.index

missing_val_count_by_column['Date']

print("Columns Having more than 90% missing values = ")
for i in missing_val_count_by_column.index: # for col, val in missing_val_count_by_column.items():
    percentage_missing = (missing_val_count_by_column[i]/num_rows)*100 # (val / num_rows)*100
    if percentage_missing > 90:
        print(i) # print(col, f"{percentage_missing:.2f}%")

columns_to_drop = [col for col, val in missing_val_count_by_column.items() if (val/num_rows)*100 > 90]
columns_to_drop

if all(column in ebola.columns for column in columns_to_drop):
    ebola_droppedCol = ebola.copy().drop(columns_to_drop, axis=1)
ebola_droppedCol

ebola_droppedCol.shape

ebola_droppedCol.isnull().sum()

ebola.iloc[0:10, 0:6]

ebola_fillType1 = ebola.fillna(method='bfill')
ebola_fillType1

missing_values_after_bfill = ebola_fillType1.isnull().sum()
print(f"Indivdual number of missing values after fill backward imputation:")
missing_values_after_bfill # Breakdown

print("Total number of missing values after fill backward imputation:")
int(missing_values_after_bfill.sum()) # Total

ebola_fillType1.iloc[:10, :5]

# The Changes made after the backward fill imputation havn't removed all the missing values from the DataFrame & is evident from the end of the DataFrame.

ebola.iloc[0:10, 0:6]

ebola_fillType1 = ebola_fillType1.fillna(method='ffill')
ebola_fillType1

missing_values_after_ffill = ebola_fillType1.isnull().sum()
print(f"Indivdual number of missing values after fill forward imputation:")
missing_values_after_ffill # Breakdown

print("Total number of missing values after fill forward imputation:")
int(missing_values_after_ffill.sum()) # Total

ebola_fillType1.iloc[:10, :5]

# The Changes made after the fill forward imputation have removed all the missing values from the DataFrame (since a Backward fill was performed first).

ebola_fillType3 = ebola.fillna(method='bfill').fillna(method='ffill')
ebola_fillType3

current_missing_values = ebola_fillType3.isnull().sum()
current_missing_values

int(current_missing_values.sum()) # TOTAL

ebola_fillType4 = ebola.interpolate()
ebola_fillType4.head()

# The fill forward and fill backward methods are simple imputation techniques that replace missing values with the last known value (forward fill) or the next known value (backward fill). These methods are particularly useful for time series data where the assumption is that the previous or next value is a reasonable estimate for the missing value. However, these methods can introduce bias if the data has a trend or seasonality, as they do not account for changes over time.
# On the other hand, linear interpolation estimates missing values by assuming a linear relationship between the known data points. This method calculates the missing value based on the slope between the two nearest known values, providing a more accurate estimate when the data follows a linear trend. Linear interpolation is more sophisticated than fill forward/backward as it considers the overall data pattern, but it may not be suitable for data with non-linear trends or significant fluctuations.
# In summary, fill forward/backward methods are simpler and faster but may introduce bias in trending data, while linear interpolation provides a more accurate estimate by considering the data trend but may not handle non-linear patterns well.

# ebola['Date'] = pd.to_datetime(ebola['Date'])
# ebola.set_index('Date', inplace=True)
# ebola_fillType5 = ebola.interpolate(method='time')
# ebola_fillType5.head()

# -> Time series interpolation looks at the actual times between data points to fill in gaps, so it's really good for data that changes over time. Linear interpolation just draws a straight line between points, ignoring the time gaps.

# -> They give the same results when the data is evenly spaced in time and follows a straight line trend, because then both methods end up doing the same calculation.


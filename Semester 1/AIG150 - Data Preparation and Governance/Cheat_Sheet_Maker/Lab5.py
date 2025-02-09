import pandas as pd
housing = pd.read_csv('housing.csv')
housing

housing.shape

housing.columns

def zscore(series):
    return (series - series.mean()) / series.std()

housing['stand_Gross.SqFt'] = zscore(housing['Gross.SqFt'])

housing['stand_Gross.SqFt'].head(1) # housing.loc[1, 'stand_Gross.SqFt']

def outlierDetectionZ(x):
    if x < -3:
        return 'low'
    elif x > 3:
        return 'high'
    return None

import numpy as np

# Vectorize the function
outlierDetectionZ_vec = np.vectorize(outlierDetectionZ)

# Apply vectorized function to the standardized column
lowhigh = outlierDetectionZ_vec(housing['stand_Gross.SqFt'])

# Loop through and print outliers
for idx, label in enumerate(lowhigh):
    if label is not None:  # Only print if it's an outlier (high or low)
        print(f"\nLabel: {label}")
        print(housing.iloc[idx])
        print("------------------------------------------\n")

# Apply outlierDetectionZ function and filter outliers
result = housing[housing['stand_Gross.SqFt'].apply(outlierDetectionZ).notna()]

# Display the outliers
for _, row in result.iterrows():
    print(f"\nLabel: {outlierDetectionZ(row['stand_Gross.SqFt'])}")
    print(row)
    print("------------------------------------------\n")

housing['stand_Gross.SqFt'].apply(lambda x: 'outlier' if x < -3 or x > 3 else None)

print("6.1. Average full market value per boro:")
housing.groupby('Boro')[['Full.Market.Value']].mean()

print("6.2. Average full market value per boro and Neighborhood:")
housing.groupby(['Boro', 'Neighborhood'])[['Full.Market.Value']].mean()

print("6.3. Statistics per boro and Neighborhood:") # pd.set_option('display.float_format', lambda x: '{:,.2f}'.format(x))
# Caluclate
stats_by_boro_neigh_aggregated_df = housing.groupby(['Boro', 'Neighborhood'])['Full.Market.Value'].agg([ # .aggregate([
    'min', 'max', 'mean'
])
# Rename
stats_by_boro_neigh_aggregated_df.rename(columns={
    'mean': 'avg market val',
    'min': 'min Market val',
    'max': 'max market val'
}, inplace=True) # stats_by_boro_neigh_aggregated_df.columns = ['min Market val', 'max market val', 'avg market val']
# Flatten
stats_by_boro_neigh_aggregated_df.reset_index(inplace=True)
# Print in Python_Notebook
stats_by_boro_neigh_aggregated_df

grouped_zscore = housing.groupby('Boro')['Full.Market.Value'].transform(zscore) # 7.1
print("Grouped zscore:")
grouped_zscore.head() # 7.3.a

non_grouped_zscore = zscore(housing['Full.Market.Value']) # 7.2
print("Non-grouped zscore:")
non_grouped_zscore.head() # 7.3.b

boro_counts = housing['Boro'].value_counts()
print("8.1 Frequency count per Boro:")
boro_counts

print("8.2 & 8.3 Filtered houses based on Boro frequency:")
# 8.2
boro_mask = housing['Boro'].map(boro_counts) >= 100
housing_filtered = housing[boro_mask]
# 8.3
filtered_counts = housing_filtered.value_counts('Boro') # housing_filtered['Boro'].value_counts()
filtered_counts.plot(kind='bar', title='Frequency count per Boro') # Not Nessary just thought it would be fun.
filtered_counts

groups = housing.groupby(['Boro', 'Neighborhood'])[['Gross.SqFt', 'Estimated.Gross.Income']] # housing[['Boro', 'Neighborhood', 'Gross.SqFt','Estimated.Gross.Income']].groupby(['Boro', 'Neighborhood'])
groups.mean() # .reset_index()

for (boro, neighborhood), group in groups:
    if boro == 'Bronx' and neighborhood == 'BEDFORD PARK/NORWOOD':
        for index, row in group.iterrows():
            # if row['Gross.SqFt'] == 79920:
                print(f"\tthe total gross square Feet for {boro} in {neighborhood} is: {row['Gross.SqFt']}")


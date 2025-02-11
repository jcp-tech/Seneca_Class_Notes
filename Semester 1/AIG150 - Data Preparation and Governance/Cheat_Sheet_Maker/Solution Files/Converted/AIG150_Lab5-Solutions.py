from google.colab import drive
drive.mount('/content/drive')

import pandas as pd
housing = pd.read_csv('/content/drive/MyDrive/Colab Notebooks/Seneca Notebooks/Week 5/Assignment/housing.csv')
housing

housing['stand_Gross.SqFt'] = (housing['Gross.SqFt'] - housing['Gross.SqFt'].mean())/housing['Gross.SqFt'].std()
housing['stand_Gross.SqFt'].head(1)

def outlierDetectionZ(x):
  if x < -3:
    return 'low'
  elif x > 3:
    return 'high'
  else:
    return 'none'

import numpy as np
outlierDetectionZ_vec = np.vectorize(outlierDetectionZ)
lowhigh = outlierDetectionZ_vec(housing['stand_Gross.SqFt'])

for i in range(len(lowhigh)):
  if lowhigh[i] == 'low' or lowhigh[i] == 'high':
    print(housing.loc[i], lowhigh[i])

apply_df = housing['stand_Gross.SqFt'].apply(outlierDetectionZ)

for i in range(len(apply_df)):
  if apply_df[i] == 'low' or apply_df[i] == 'high':
    print(housing.loc[i], apply_df[i])

housing['stand_Gross.SqFt'].apply(lambda x: 'outlier' if x < -3 or x > 3 else None)

housing.groupby('Boro')['Full.Market.Value'].mean()

housing.groupby(['Boro', 'Neighborhood'])['Full.Market.Value'].mean()

# fun fact - since it's such a long line of code, I like to write it on separate lines
# If I just wrote on separate lines without including \ at the end of each line, I would
# get an error. The \ just tells Python my line of code isn't done, and i want to continue
# on the next line.
housing.groupby(['Boro', 'Neighborhood'])['Full.Market.Value']\
        .agg(['min', 'max', 'mean'])\
        .rename(columns={'min': 'min Market val', \
                         'max': 'max market val', \
                         'mean': 'avg market val'})\
        .reset_index()

housing['grouped_zscore'] = housing.groupby('Boro')['Full.Market.Value'].transform(lambda x: (x - x.mean())/x.std())

housing['non_grouped_zscore'] = (housing['Full.Market.Value'] - housing['Full.Market.Value'].mean())/housing['Full.Market.Value'].std()

housing[['grouped_zscore', 'non_grouped_zscore']].head()

frequency_counts = housing['Boro'].value_counts()
frequency_counts

frequency_counts = frequency_counts[frequency_counts >= 100]
housing_filtered = housing[housing['Boro'].isin(frequency_counts.index)]

housing_filtered['Boro'].value_counts()

groups = housing.groupby(['Boro', 'Neighborhood'])[['Gross.SqFt', 'Estimated.Gross.Income']]
avg_groups = groups.mean()
avg_groups

'''
You would either need to do a new grouping, and get the sum instead of the mean (PREFERRED)
'''
sum_groups = groups.sum()
for group, data in sum_groups.iterrows():
  if group[0] == 'Bronx' and group[1] == 'BEDFORD PARK/NORWOOD':
    print('the gross square Feet for Bronx in BEDFORD PARK/NORWOOD is:', data['Gross.SqFt'])

'''
Or, you iterate through the original housing df, check the Boro and Neighbourhood, and
print the total Gross.SqFt
TECHNICALLY, this is INCORRECT - you were supposed to iterate through the GROUPS.
I will let it slide for now.. :)
'''
total_sqft = 0
for i in range(len(housing)):
  if housing['Boro'][i] == 'Bronx' and housing['Neighborhood'][i] == 'BEDFORD PARK/NORWOOD':
    total_sqft += housing['Gross.SqFt'][i]
print('the gross square Feet for Bronx in BEDFORD PARK/NORWOOD is:', total_sqft)


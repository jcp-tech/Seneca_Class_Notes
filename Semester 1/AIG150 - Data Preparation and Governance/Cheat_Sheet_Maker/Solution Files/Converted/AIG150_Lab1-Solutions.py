from sklearn.datasets import load_diabetes
import pandas as pd

diabetes=load_diabetes()
print(diabetes.keys())
print(diabetes.DESCR)
print(diabetes.feature_names) # these are the column names

df=pd.DataFrame(diabetes.data, columns=diabetes.feature_names)

print(df.shape)

print(df.head())

print(df.columns)

print(df.dtypes)

df.loc[1]

df.loc[[1, 99, 149]]

df.iloc[[1, 99]]

df.loc[:, 'bmi']

df.loc[:, ['bmi', 'bp']]

df.iloc[-1]

df.iloc[-2:]

subset_df = df.loc[:9, ['age', 'bmi', 'bp']]

subset_df.head()

sliced_df = df.iloc[:, :6]
sliced_df.head()

subset_df = df.iloc[:, 2:5]
subset_df.head()

tmp = df.iloc[:, :6:2]
tmp.head()

df.iloc[:, :5]

df.loc[9, 'age']

df.loc[[100, 200, 300], 'bmi']

# URL of the data
url = "https://www4.stat.ncsu.edu/~boos/var.select/diabetes.tab.txt"

# Import the data into a DataFrame
new_df = pd.read_csv(url, sep='\t')

new_df.loc[new_df['AGE'] > 50]

new_df.loc[new_df['SEX'] == 2, ['BMI', 'BP']]

age = new_df['AGE']
age

print(age.min())
print(age.max())
print(age.mean())

age.info()

age.loc[age < age.mean()]

age.sort_values()

age.unique()



